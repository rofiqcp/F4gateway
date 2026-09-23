#if defined(F103_BUILD_BOOTLOADER)
#include "stm32f1xx_hal.h"
#include "boot_usb.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define APP_BASE 0x08002000UL
#define APP_LIMIT 0x0800F7F0UL
#define APP_META_BASE 0x0800F7F0UL
#define PERSIST_BASE 0x0800F800UL
#define GATEWAY_BOARD_ID 0xF103C801UL
#define BOOT_PROTOCOL_VERSION 4UL
#define BOOT_LAYOUT_STRING "AGVBL4-C8-02000-F7F0"
#define SRAM_BASE_ADDR 0x20000000UL
#define SRAM_END_ADDR 0x20005000UL
#define FLASH_PAGE_C8_BYTES 0x400UL
#define FLASH_PAGE_CLONE_414_BYTES 0x800UL
#define BOOT_REQ_LO 0x5544U
#define BOOT_REQ_HI 0x4246U
#define APP_META_MAGIC 0x34475641UL
#define BOOT_IDLE_TIMEOUT_MS 20000UL
#define BOOT_WATCHDOG_TIMEOUT_MS 10000UL
#define BOOT_UPDATE_TIMEOUT_MS 30000UL
#define BOOT_USB_LOSS_RECOVERY_MS 5000UL
#define BOOT_USB_INITIAL_RECOVERY_MS 2500UL
#define BOOT_USB_RECOVERY_COOLDOWN_MS 3000UL
#define BOOT_USB_MAX_RECOVERIES 3U
#define BOOT_LINE_MAX 600U
#define BOOT_DATA_MAX 240U

typedef struct {
  uint32_t magic;
  uint32_t app_size;
  uint32_t app_crc32;
  uint32_t header_crc32;
} app_metadata_t;
static char line_buf[BOOT_LINE_MAX];
static uint32_t line_len;
static bool line_discard_until_newline;
static uint32_t rx_drop_seen;
static bool update_started;
static uint32_t expected_size, expected_crc, write_offset;
static uint32_t update_last_activity_ms;
static uint32_t boot_started_ms;
static bool boot_usb_started;
static volatile bool watchdog_armed;
static volatile uint32_t watchdog_last_pat_ms;

static void watchdog_pat(void) { watchdog_last_pat_ms = HAL_GetTick(); }

/*
 * Assert a physical USB disconnect before HAL or the clock tree is touched.
 * Bluepill-class boards use an external D+ pull-up, so a plain core reset would
 * otherwise expose a short reconnect pulse before firmware can take PA12 low.
 */
static void early_usb_detach_hold(void) {
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  __DSB();
  uint32_t crh = GPIOA->CRH;
  crh &= ~(0xFUL << 16U);     /* PA12 configuration nibble. */
  crh |=  (0x2UL << 16U);     /* 2 MHz push-pull output. */
  GPIOA->CRH = crh;
  GPIOA->BRR = GPIO_PIN_12;
  __DSB();
}

static void early_usb_detach_release(void) {
  RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
  __DSB();
  uint32_t crh = GPIOA->CRH;
  crh &= ~(0xFUL << 16U);
  crh |=  (0x4UL << 16U);     /* Reset-equivalent floating input: release D+. */
  GPIOA->CRH = crh;
  __DSB();
}

/*
 * Some F103-compatible clone silicon can preserve more core/peripheral state
 * across SYSRESETREQ than the resident boot path should rely on. Make the
 * bootloader vector/IRQ/USB reset state explicit before HAL enables SysTick.
 */
static void early_reset_sanitize(void) {
  __disable_irq();
  SCB->VTOR = 0x08000000UL;
  SysTick->CTRL = 0U;
  SysTick->LOAD = 0U;
  SysTick->VAL = 0U;
  for (uint32_t i = 0U; i < 8U; ++i) {
    NVIC->ICER[i] = 0xFFFFFFFFUL;
    NVIC->ICPR[i] = 0xFFFFFFFFUL;
  }
  RCC->APB1RSTR |= RCC_APB1RSTR_USBRST;
  __DSB();
  RCC->APB1RSTR &= ~RCC_APB1RSTR_USBRST;
  __DSB();
  __ISB();
  __enable_irq();
}

static void fatal_reset(void) { NVIC_SystemReset(); while (1) {} }

static void system_clock_config(void) {
  RCC_OscInitTypeDef osc = {0};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) fatal_reset();

  RCC_ClkInitTypeDef clk = {0};
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK) fatal_reset();

  RCC_PeriphCLKInitTypeDef periph = {0};
  periph.PeriphClockSelection = RCC_PERIPHCLK_USB;
  periph.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK) fatal_reset();
}

static uint32_t crc32_bytes(const uint8_t *data, uint32_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  uint32_t serviced = 0U;
  while (len-- != 0U) {
    crc ^= *data++;
    for (uint32_t bit = 0U; bit < 8U; ++bit)
      crc = (crc >> 1U) ^ (0xEDB88320UL & (0U - (crc & 1U)));
    if ((++serviced & 0x0FFFU) == 0U) watchdog_pat();
  }
  return crc ^ 0xFFFFFFFFUL;
}

static void backup_access_enable(void) {
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_BKP_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
}

static bool read_boot_request(void) {
  backup_access_enable();
  const uint16_t lo = (uint16_t)BKP->DR1;
  const uint16_t hi = (uint16_t)BKP->DR10;
  return lo == BOOT_REQ_LO && hi == BOOT_REQ_HI;
}

static void clear_boot_request(void) {
  backup_access_enable();
  BKP->DR1 = 0U;
  BKP->DR10 = 0U;
  __DSB();
}

static bool vector_valid(uint32_t base, uint32_t limit) {
  const uint32_t sp = *(volatile const uint32_t *)base;
  const uint32_t reset = *(volatile const uint32_t *)(base + 4U);
  if (sp < SRAM_BASE_ADDR || sp > SRAM_END_ADDR || (sp & 3U) != 0U) return false;
  if ((reset & 1U) == 0U) return false;
  const uint32_t pc = reset & ~1UL;
  return pc >= base && pc < limit;
}

static bool metadata_valid(const app_metadata_t *m) {
  if (m->magic != APP_META_MAGIC) return false;
  if (m->app_size < 8U || m->app_size > (APP_LIMIT - APP_BASE)) return false;
  return crc32_bytes((const uint8_t *)m, 12U) == m->header_crc32;
}

static bool application_valid(void) {
  const app_metadata_t *m = (const app_metadata_t *)APP_META_BASE;
  if (!metadata_valid(m)) return false;
  if (!vector_valid(APP_BASE, APP_BASE + m->app_size)) return false;
  return crc32_bytes((const uint8_t *)APP_BASE, m->app_size) == m->app_crc32;
}

static void quiesce(void) {
  __disable_irq();
  SysTick->CTRL = SysTick->LOAD = SysTick->VAL = 0U;
  for (uint32_t i = 0U; i < 8U; ++i) {
    NVIC->ICER[i] = 0xFFFFFFFFUL;
    NVIC->ICPR[i] = 0xFFFFFFFFUL;
  }
  __DSB(); __ISB();
}
__attribute__((noreturn)) static void reset_to_app_after_usb(void) {
  if (boot_usb_started) {
    boot_usb_disconnect_hold();
    boot_usb_started = false;
  } else {
    early_usb_detach_hold();
  }
  watchdog_armed = false;
  clear_boot_request();
  /* Give the host a real detach window before the core/peripherals reset. */
  HAL_Delay(350U);
  NVIC_SystemReset();
  while (1) {}
}

__attribute__((noreturn)) static void jump_app(void) {
  /*
   * Direct jump is allowed only from a freshly reset boot path. Once the USB
   * stack has ever been active, perform a hardware reset first so the app never
   * inherits endpoint/PCD state from the resident bootloader.
   */
  if (boot_usb_started)
    reset_to_app_after_usb();

  const uint32_t sp = *(volatile const uint32_t *)APP_BASE;
  const uint32_t reset = *(volatile const uint32_t *)(APP_BASE + 4U);
  watchdog_armed = false;
  clear_boot_request();
  /*
   * early_usb_detach_hold() intentionally owns PA12 from reset. A direct jump
   * does not reset GPIO registers, so release D+ before entering the app or the
   * external pull-up can never enumerate the runtime CDC device.
   */
  early_usb_detach_release();
  if (HAL_RCC_DeInit() != HAL_OK) fatal_reset();
  quiesce();
  SCB->VTOR = APP_BASE;
  __set_CONTROL(0U);
  __set_PSP(0U);
  __set_MSP(sp);
  __DSB(); __ISB();
  __enable_irq();
  ((void (*)(void))reset)();
  while (1) {}
}

static uint32_t flash_page_bytes(void) {
  const uint32_t dev = DBGMCU->IDCODE & 0xFFFU;
  return dev == 0x414U ? FLASH_PAGE_CLONE_414_BYTES : FLASH_PAGE_C8_BYTES;
}

__attribute__((section(".RamFunc"), noinline, used))
static bool erase_flash_pages_ram(uint32_t start, uint32_t end) {
  const uint32_t page_bytes = flash_page_bytes();
  if ((start & (page_bytes - 1U)) != 0U || (end & (page_bytes - 1U)) != 0U ||
      end <= start || end > PERSIST_BASE)
    return false;

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  if ((FLASH->CR & FLASH_CR_LOCK) != 0U) {
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }
  if ((FLASH->CR & FLASH_CR_LOCK) != 0U) {
    if (primask == 0U) __enable_irq();
    return false;
  }

  for (uint32_t guard = 0U; (FLASH->SR & FLASH_SR_BSY) != 0U; ++guard) {
    if (guard >= 8000000U) {
      FLASH->CR = FLASH_CR_LOCK;
      if (primask == 0U) __enable_irq();
      return false;
    }
  }

  FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;

  for (uint32_t address = start; address < end; address += page_bytes) {
    FLASH->CR = FLASH_CR_PER;
    FLASH->AR = address;
    FLASH->CR = FLASH_CR_PER | FLASH_CR_STRT;

    uint32_t guard = 0U;
    while ((FLASH->SR & FLASH_SR_BSY) != 0U && guard < 8000000U)
      ++guard;

    const uint32_t sr = FLASH->SR;
    if (guard >= 8000000U ||
        (sr & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) != 0U) {
      FLASH->CR = FLASH_CR_LOCK;
      if (primask == 0U) __enable_irq();
      return false;
    }

    FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
  }

  FLASH->CR = FLASH_CR_LOCK;
  __DSB();
  if (primask == 0U) __enable_irq();
  return true;
}

static bool begin_update(uint32_t size, uint32_t crc) {
  if (size < 8U || size > (APP_LIMIT - APP_BASE)) {
    return false;
  }

  /*
   * Invalidate the old image first by erasing the complete application area,
   * including the fixed metadata footer. Persistent configuration starts at
   * PERSIST_BASE and is never touched by the bootloader update path.
   */
  if (!erase_flash_pages_ram(APP_BASE, PERSIST_BASE)) {
    return false;
  }

  expected_size = size;
  expected_crc = crc;
  write_offset = 0U;
  update_last_activity_ms = HAL_GetTick();
  update_started = true;
  return true;
}

__attribute__((section(".RamFunc"), noinline, used))
static bool flash_wait_ready_atomic(void) {
  for (uint32_t guard = 0U; guard < 8000000U; ++guard)
    if ((FLASH->SR & FLASH_SR_BSY) == 0U) return true;
  return false;
}

/*
 * Match OpenOCD's proven STM32F1 block-writer sequence:
 * unlock once, keep PG asserted for the entire halfword stream, wait for BSY
 * after every halfword, then lock once.  Do not call HAL_FLASH_Program() for
 * each halfword because HAL toggles PG between writes.
 */
__attribute__((section(".RamFunc"), noinline, used))
static bool program_halfword_stream(uint32_t address, const uint8_t *data,
                                    uint32_t len) {
  if (data == NULL || len == 0U || (address & 1U) != 0U) return false;
  if (HAL_FLASH_Unlock() != HAL_OK) return false;

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();

  bool ok = flash_wait_ready_atomic();
  FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
  FLASH->CR = FLASH_CR_PG;

  for (uint32_t pos = 0U; ok && pos < len; pos += 2U) {
    uint16_t half = data[pos];
    if ((pos + 1U) < len)
      half |= (uint16_t)((uint16_t)data[pos + 1U] << 8U);
    else
      half |= 0xFF00U;

    *(__IO uint16_t *)(address + pos) = half;
    __DSB();

    ok = flash_wait_ready_atomic();
    const uint32_t sr = FLASH->SR;
    if ((sr & (FLASH_SR_PGERR | FLASH_SR_WRPRTERR)) != 0U)
      ok = false;
    if (*(volatile const uint16_t *)(address + pos) != half)
      ok = false;
  }

  FLASH->CR = FLASH_CR_LOCK;
  FLASH->SR = FLASH_SR_EOP | FLASH_SR_PGERR | FLASH_SR_WRPRTERR;
  __DSB();
  if (primask == 0U) __enable_irq();

  return ok && memcmp((const void *)address, data, len) == 0;
}

static bool program_chunk(uint32_t offset, const uint8_t *data, uint32_t len) {
  if (!update_started || data == NULL || len == 0U || len > BOOT_DATA_MAX)
    return false;
  if (offset != write_offset || offset + len > expected_size) return false;
  if ((offset & 1U) != 0U) return false;

  const bool ok = program_halfword_stream(APP_BASE + offset, data, len);
  if (ok) {
    write_offset += len;
    update_last_activity_ms = HAL_GetTick();
  }
  watchdog_pat();
  return ok;
}

static bool program_bytes_stream(uint32_t address, const void *src, uint32_t len) {
  if (src == NULL || (address & 1U) != 0U || (len & 1U) != 0U) return false;
  return program_halfword_stream(address, (const uint8_t *)src, len);
}

static bool commit_metadata(void) {
  if (!update_started || write_offset != expected_size) return false;
  if (crc32_bytes((const uint8_t *)APP_BASE, expected_size) != expected_crc)
    return false;
  if (!vector_valid(APP_BASE, APP_BASE + expected_size)) return false;

  app_metadata_t m;
  memset(&m, 0, sizeof(m));
  m.magic = APP_META_MAGIC;
  m.app_size = expected_size;
  m.app_crc32 = expected_crc;
  m.header_crc32 = crc32_bytes((const uint8_t *)&m, 12U);
  return program_bytes_stream(APP_META_BASE, &m, sizeof(m)) && application_valid();
}

static int hex_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static bool parse_u32(const char *s, uint32_t base, uint32_t *out) {
  if (s == NULL || out == NULL || *s == '\0') return false;
  uint32_t value = 0U;
  while (*s != '\0') {
    const int d = hex_nibble(*s++);
    if (d < 0 || (uint32_t)d >= base) return false;
    if (value > (0xFFFFFFFFUL - (uint32_t)d) / base) return false;
    value = value * base + (uint32_t)d;
  }
  *out = value;
  return true;
}

static bool decode_hex(const char *hex, uint8_t *out, uint32_t *len) {
  const size_t n = strlen(hex);
  if (n == 0U || (n & 1U) != 0U || n > BOOT_DATA_MAX * 2U) return false;
  *len = (uint32_t)(n / 2U);
  for (uint32_t i = 0U; i < *len; ++i) {
    const int hi = hex_nibble(hex[2U * i]);
    const int lo = hex_nibble(hex[2U * i + 1U]);
    if (hi < 0 || lo < 0) return false;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  return true;
}

static char *append_text(char *p, char *end, const char *s) {
  while (*s != '\0' && p < end) *p++ = *s++;
  return p;
}
static char *append_dec(char *p, char *end, uint32_t value) {
  char rev[10]; uint32_t n = 0U;
  do { rev[n++] = (char)('0' + value % 10U); value /= 10U; }
  while (value != 0U && n < sizeof(rev));
  while (n != 0U && p < end) *p++ = rev[--n];
  return p;
}
static char *append_hex8(char *p, char *end, uint32_t value) {
  static const char digits[] = "0123456789ABCDEF";
  for (int shift = 28; shift >= 0 && p < end; shift -= 4)
    *p++ = digits[(value >> shift) & 0xFU];
  return p;
}

static void reply_info(void) {
  char out[200];
  char *p = out, *end = out + sizeof(out) - 1U;
  p = append_text(p, end, "BOOT:INFO:proto=");
  p = append_dec(p, end, BOOT_PROTOCOL_VERSION);
  p = append_text(p, end, ":board=");
  p = append_hex8(p, end, GATEWAY_BOARD_ID);
  p = append_text(p, end, ":layout=");
  p = append_text(p, end, BOOT_LAYOUT_STRING);
  p = append_text(p, end, ":valid=");
  p = append_dec(p, end, application_valid() ? 1U : 0U);
  *p = '\0';
  (void)boot_usb_write_line(out, 250U);
}

static void handle_line(char *line) {
  if (!strcmp(line, "PING")) {
    (void)boot_usb_write_line("BOOT:PONG", 250U);
    return;
  }
  if (!strcmp(line, "INFO")) {
    reply_info();
    return;
  }
  if (!strncmp(line, "BEGIN:", 6U)) {
    char *sep = strchr(line + 6U, ':');
    if (sep == NULL) {
      (void)boot_usb_write_line("ERR:BEGIN:FORMAT", 250U);
      return;
    }
    *sep = '\0';
    uint32_t size = 0U, crc = 0U;
    if (!parse_u32(line + 6U, 10U, &size) ||
        !parse_u32(sep + 1U, 16U, &crc)) {
      (void)boot_usb_write_line("ERR:BEGIN:FORMAT", 250U);
      return;
    }
    (void)boot_usb_write_line("BOOT:ERASING", 250U);
    boot_usb_flush(250U);
    if (!begin_update(size, crc)) {
      (void)boot_usb_write_line("ERR:BEGIN", 250U);
      return;
    }
    (void)boot_usb_write_line("ACK:BEGIN:0", 250U);
    return;
  }
  if (!strncmp(line, "DATA2:", 6U)) {
    char *sep1 = strchr(line + 6U, ':');
    if (sep1 == NULL) { (void)boot_usb_write_line("ERR:DATA2:FORMAT",250U); return; }
    *sep1 = '\0';
    char *sep2 = strchr(sep1 + 1U, ':');
    if (sep2 == NULL) { (void)boot_usb_write_line("ERR:DATA2:FORMAT",250U); return; }
    *sep2 = '\0';
    uint32_t off = 0U, chunk_crc = 0U, len = 0U;
    uint8_t data[BOOT_DATA_MAX];
    if (!parse_u32(line + 6U, 10U, &off) ||
        !parse_u32(sep1 + 1U, 16U, &chunk_crc) ||
        !decode_hex(sep2 + 1U, data, &len)) {
      (void)boot_usb_write_line("ERR:DATA2:FORMAT",250U);
      return;
    }
    if (crc32_bytes(data, len) != chunk_crc) {
      (void)boot_usb_write_line("ERR:DATA2:CRC",250U);
      return;
    }
    if (!program_chunk(off, data, len)) {
      char out[48], *p = out, *end = out + sizeof(out) - 1U;
      p = append_text(p,end,"ERR:DATA2:OFFSET:");
      p = append_dec(p,end,write_offset); *p='\0';
      (void)boot_usb_write_line(out,250U);
      return;
    }
    char out[48], *p = out, *end = out + sizeof(out) - 1U;
    p = append_text(p,end,"ACK:DATA2:");
    p = append_dec(p,end,write_offset);
    p = append_text(p,end,":");
    p = append_hex8(p,end,chunk_crc);
    *p='\0';
    (void)boot_usb_write_line(out,250U);
    return;
  }
  if (!strcmp(line, "END")) {
    if (!commit_metadata()) {
      (void)boot_usb_write_line("ERR:END:VERIFY",250U);
      return;
    }
    update_started = false;
    update_last_activity_ms = 0U;
    (void)boot_usb_write_line("ACK:END:OK",250U);
    boot_usb_flush(300U);
    HAL_Delay(30U);
    /*
     * After a committed image, leave the active USB stack through a real core
     * reset. PA12 is held low before reset and asserted low again at the very
     * start of bootloader main(), so the host sees one clean detach while the
     * app starts from reset-clean peripheral state.
     */
    reset_to_app_after_usb();
  }
  (void)boot_usb_write_line("ERR:UNKNOWN",250U);
}

static void maintenance_loop(bool allow_timeout) {
  update_started = false;
  expected_size = expected_crc = write_offset = 0U;
  line_len = 0U;
  line_discard_until_newline = false;
  if (!boot_usb_begin()) fatal_reset();
  boot_usb_started = true;
  rx_drop_seen = boot_usb_rx_dropped();
  boot_started_ms = HAL_GetTick();
  const uint32_t maintenance_started_ms = boot_started_ms;
  bool usb_was_configured = false;
  uint32_t usb_loss_started_ms = 0U;
  uint32_t usb_last_recovery_ms = 0U;
  uint8_t usb_recovery_count = 0U;
  while (1) {
    watchdog_pat();

    const uint32_t dropped_now = boot_usb_rx_dropped();
    if (dropped_now != rx_drop_seen) {
      rx_drop_seen = dropped_now;
      line_len = 0U;
      line_discard_until_newline = true;
      (void)boot_usb_write_line("ERR:RX:OVERFLOW", 250U);
    }

    while (boot_usb_available() > 0) {
      const int v = boot_usb_read();
      if (v < 0) break;
      const char c = (char)v;
      if (c == '\r') continue;

      if (line_discard_until_newline) {
        if (c == '\n') {
          line_discard_until_newline = false;
          line_len = 0U;
          boot_started_ms = HAL_GetTick();
        }
        continue;
      }

      if (c == '\n') {
        line_buf[line_len] = '\0';
        if (line_len != 0U) handle_line(line_buf);
        line_len = 0U;
        boot_started_ms = HAL_GetTick();
      } else if (line_len + 1U < sizeof(line_buf)) {
        line_buf[line_len++] = c;
      } else {
        line_len = 0U;
        line_discard_until_newline = true;
        (void)boot_usb_write_line("ERR:LINE:TOO_LONG", 250U);
      }
    }
    const uint32_t now = HAL_GetTick();

    /*
     * A host-side close must not matter, but a hub reset/endpoint fault can
     * de-configure USB without resetting the MCU. Once this boot session has
     * been configured at least once, recover a sustained link loss locally
     * instead of requiring NRST/ST-Link or a physical reconnect.
     */
    if (boot_usb_connected()) {
      usb_was_configured = true;
      usb_loss_started_ms = 0U;
    } else {
      bool recover_usb = false;
      if (usb_was_configured) {
        if (usb_loss_started_ms == 0U)
          usb_loss_started_ms = now;
        else if ((uint32_t)(now - usb_loss_started_ms) >=
                 BOOT_USB_LOSS_RECOVERY_MS)
          recover_usb = true;
      } else if ((uint32_t)(now - boot_started_ms) >=
                 BOOT_USB_INITIAL_RECOVERY_MS) {
        recover_usb = true;
      }

      if (recover_usb && usb_recovery_count < BOOT_USB_MAX_RECOVERIES &&
          (usb_last_recovery_ms == 0U ||
           (uint32_t)(now - usb_last_recovery_ms) >=
               BOOT_USB_RECOVERY_COOLDOWN_MS)) {
        /*
         * Recover both a lost configured link and a first-enumeration stall.
         * DATA2 remains resumable at write_offset if this occurs mid-update.
         */
        ++usb_recovery_count;
        usb_last_recovery_ms = now;
        boot_usb_disconnect_hold();
        boot_usb_started = false;
        HAL_Delay(2000U);
        if (!boot_usb_begin()) fatal_reset();
        boot_usb_started = true;
        usb_was_configured = false;
        usb_loss_started_ms = 0U;
        line_len = 0U;
        line_discard_until_newline = false;
        rx_drop_seen = boot_usb_rx_dropped();
        boot_started_ms = HAL_GetTick();
      }
    }

    if (update_started &&
        (uint32_t)(now - update_last_activity_ms) >= BOOT_UPDATE_TIMEOUT_MS) {
      update_started = false;
      expected_size = expected_crc = write_offset = 0U;
      update_last_activity_ms = 0U;
      line_len = 0U;
      line_discard_until_newline = false;
      boot_started_ms = now;
      (void)boot_usb_write_line("ERR:UPDATE:TIMEOUT", 250U);
    }

    if (allow_timeout && !update_started &&
        (uint32_t)(now - maintenance_started_ms) >= BOOT_IDLE_TIMEOUT_MS &&
        application_valid()) {
      jump_app();
    }
  }
}

int main(void) {
  early_usb_detach_hold();
  early_reset_sanitize();
  HAL_Init();
  system_clock_config();
  backup_access_enable();
  watchdog_last_pat_ms = HAL_GetTick();
  watchdog_armed = true;
  const bool request = read_boot_request();
  const bool valid = application_valid();
  if (request) {
    /* One-shot maintenance request: never trap a valid application across resets. */
    clear_boot_request();
    maintenance_loop(valid);
  }
  if (valid) jump_app();
  maintenance_loop(false);
}

void SysTick_Handler(void) {
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  if (watchdog_armed &&
      (uint32_t)(HAL_GetTick() - watchdog_last_pat_ms) >
          BOOT_WATCHDOG_TIMEOUT_MS) {
    NVIC_SystemReset();
  }
}

void Error_Handler(void) { fatal_reset(); }

#endif
