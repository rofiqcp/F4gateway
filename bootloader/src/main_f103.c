#include "stm32f1xx_hal.h"
#include "boot_usb.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#define APP_BASE 0x08004000UL
#define APP_LIMIT 0x0803E000UL
#define MANIFEST_BASE 0x0803E000UL
#define MANIFEST_LIMIT 0x0803E800UL
#define GATEWAY_BOARD_ID 0xF1030101UL
#define BOOT_PROTOCOL_VERSION 3UL
#define BOOT_LAYOUT_STRING "AGVBL3-04000-3E000"
#define SRAM_BASE_ADDR 0x20000000UL
#define SRAM_END_ADDR 0x20005000UL
#define FLASH_PAGE_BYTES 0x800UL
#define BOOT_REQ_LO 0x5544U
#define BOOT_REQ_HI 0x4246U
#define MANIFEST_MAGIC 0x31564741UL
#define MANIFEST_FORMAT 2UL
#define BOOT_IDLE_TIMEOUT_MS 20000UL
#define BOOT_WATCHDOG_TIMEOUT_MS 10000UL
#define BOOT_LINE_MAX 600U
#define BOOT_DATA_MAX 240U

typedef struct {
  uint32_t magic, format, app_base, app_size;
  uint32_t app_crc32, header_crc32, generation, reserved;
} app_manifest_t;
static char line_buf[BOOT_LINE_MAX];
static uint32_t line_len;
static bool update_started;
static uint32_t expected_size, expected_crc, write_offset;
static uint32_t boot_started_ms;
static bool boot_usb_started;
static volatile bool watchdog_armed;
static volatile uint32_t watchdog_last_pat_ms;

static void watchdog_pat(void) { watchdog_last_pat_ms = HAL_GetTick(); }
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
  BKP->DR1 = 0U;
  BKP->DR10 = 0U;
  __DSB();
  return lo == BOOT_REQ_LO && hi == BOOT_REQ_HI;
}

static bool vector_valid(uint32_t base, uint32_t limit) {
  const uint32_t sp = *(volatile const uint32_t *)base;
  const uint32_t reset = *(volatile const uint32_t *)(base + 4U);
  if (sp < SRAM_BASE_ADDR || sp > SRAM_END_ADDR || (sp & 3U) != 0U) return false;
  if ((reset & 1U) == 0U) return false;
  const uint32_t pc = reset & ~1UL;
  return pc >= base && pc < limit;
}

static bool manifest_erased(const app_manifest_t *m) {
  const uint32_t *w = (const uint32_t *)m;
  for (uint32_t i = 0U; i < sizeof(*m) / sizeof(*w); ++i)
    if (w[i] != 0xFFFFFFFFUL) return false;
  return true;
}

static bool manifest_record_valid(const app_manifest_t *m) {
  if (m->magic != MANIFEST_MAGIC || m->format != MANIFEST_FORMAT) return false;
  if (m->reserved != GATEWAY_BOARD_ID || m->app_base != APP_BASE) return false;
  if (m->app_size < 8U || m->app_size > (APP_LIMIT - APP_BASE)) return false;
  return crc32_bytes((const uint8_t *)m, 20U) == m->header_crc32;
}
static const app_manifest_t *latest_manifest(void) {
  const app_manifest_t *latest = NULL;
  for (uint32_t a = MANIFEST_BASE;
       a + sizeof(app_manifest_t) <= MANIFEST_LIMIT;
       a += sizeof(app_manifest_t)) {
    const app_manifest_t *m = (const app_manifest_t *)a;
    if (manifest_erased(m)) break;
    if (manifest_record_valid(m)) latest = m;
  }
  return latest;
}

static uint32_t manifest_next_address(void) {
  for (uint32_t a = MANIFEST_BASE;
       a + sizeof(app_manifest_t) <= MANIFEST_LIMIT;
       a += sizeof(app_manifest_t))
    if (manifest_erased((const app_manifest_t *)a)) return a;
  return 0U;
}

static bool application_valid(void) {
  const app_manifest_t *m = latest_manifest();
  if (m == NULL) return false;
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
__attribute__((noreturn)) static void jump_app(void) {
  const uint32_t sp = *(volatile const uint32_t *)APP_BASE;
  const uint32_t reset = *(volatile const uint32_t *)(APP_BASE + 4U);
  if (boot_usb_started) { boot_usb_end(); boot_usb_started = false; }
  watchdog_armed = false;
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

static bool erase_page(uint32_t address) {
  watchdog_pat();
  FLASH_EraseInitTypeDef erase = {0};
  uint32_t error = 0U;
  erase.TypeErase = FLASH_TYPEERASE_PAGES;
  erase.PageAddress = address;
  erase.NbPages = 1U;
  const bool ok = HAL_FLASHEx_Erase(&erase, &error) == HAL_OK;
  watchdog_pat();
  return ok;
}

static bool begin_update(uint32_t size, uint32_t crc) {
  if (size < 8U || size > (APP_LIMIT - APP_BASE)) return false;
  if (manifest_next_address() == 0U) return false;
  if (HAL_FLASH_Unlock() != HAL_OK) return false;
  bool ok = true;
  for (uint32_t a = APP_BASE; ok && a < APP_LIMIT; a += FLASH_PAGE_BYTES)
    ok = erase_page(a);
  (void)HAL_FLASH_Lock();
  if (!ok) return false;
  expected_size = size;
  expected_crc = crc;
  write_offset = 0U;
  update_started = true;
  return true;
}

static bool program_chunk(uint32_t offset, const uint8_t *data, uint32_t len) {
  if (!update_started || data == NULL || len == 0U || len > BOOT_DATA_MAX)
    return false;
  if (offset != write_offset || offset + len > expected_size) return false;
  if (HAL_FLASH_Unlock() != HAL_OK) return false;
  bool ok = true;
  uint32_t pos = 0U;
  while (pos < len) {
    uint16_t half = 0xFFFFU;
    const uint32_t take = (len - pos) >= 2U ? 2U : 1U;
    memcpy(&half, data + pos, take);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                          APP_BASE + offset + pos, half) != HAL_OK) {
      ok = false;
      break;
    }
    pos += take;
    if ((pos & 0x3FU) == 0U) watchdog_pat();
  }
  (void)HAL_FLASH_Lock();
  if (ok) ok = memcmp((const void *)(APP_BASE + offset), data, len) == 0;
  if (ok) write_offset += len;
  watchdog_pat();
  return ok;
}
static bool program_bytes_halfword(uint32_t address, const void *src, uint32_t len) {
  const uint8_t *data = (const uint8_t *)src;
  if ((address & 1U) != 0U || (len & 1U) != 0U) return false;
  if (HAL_FLASH_Unlock() != HAL_OK) return false;
  bool ok = true;
  for (uint32_t pos = 0U; pos < len; pos += 2U) {
    uint16_t half;
    memcpy(&half, data + pos, 2U);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_HALFWORD,
                          address + pos, half) != HAL_OK) {
      ok = false;
      break;
    }
  }
  (void)HAL_FLASH_Lock();
  return ok && memcmp((const void *)address, src, len) == 0;
}

static bool commit_manifest(void) {
  if (!update_started || write_offset != expected_size) return false;
  if (crc32_bytes((const uint8_t *)APP_BASE, expected_size) != expected_crc)
    return false;
  if (!vector_valid(APP_BASE, APP_BASE + expected_size)) return false;
  const uint32_t target = manifest_next_address();
  if (target == 0U) return false;
  const app_manifest_t *previous = latest_manifest();
  app_manifest_t m;
  memset(&m, 0, sizeof(m));
  m.magic = MANIFEST_MAGIC;
  m.format = MANIFEST_FORMAT;
  m.app_base = APP_BASE;
  m.app_size = expected_size;
  m.app_crc32 = expected_crc;
  m.header_crc32 = crc32_bytes((const uint8_t *)&m, 20U);
  m.generation = previous == NULL ? 1U : previous->generation + 1U;
  m.reserved = GATEWAY_BOARD_ID;
  return program_bytes_halfword(target, &m, sizeof(m)) && application_valid();
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
      (void)boot_usb_write_line("ERR:BEGIN:FLASH", 250U);
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
    if (!commit_manifest()) {
      (void)boot_usb_write_line("ERR:END:VERIFY",250U);
      return;
    }
    update_started = false;
    (void)boot_usb_write_line("ACK:END:OK",250U);
    boot_usb_flush(300U);
    HAL_Delay(30U);
    NVIC_SystemReset();
    while (1) {}
  }
  (void)boot_usb_write_line("ERR:UNKNOWN",250U);
}

static void maintenance_loop(bool allow_timeout) {
  update_started = false;
  expected_size = expected_crc = write_offset = 0U;
  if (!boot_usb_begin()) fatal_reset();
  boot_usb_started = true;
  boot_started_ms = HAL_GetTick();
  while (1) {
    watchdog_pat();
    while (boot_usb_available() > 0) {
      const int v = boot_usb_read();
      if (v < 0) break;
      const char c = (char)v;
      if (c == '\r') continue;
      if (c == '\n') {
        line_buf[line_len] = '\0';
        if (line_len != 0U) handle_line(line_buf);
        line_len = 0U;
        boot_started_ms = HAL_GetTick();
      } else if (line_len + 1U < sizeof(line_buf)) {
        line_buf[line_len++] = c;
      } else {
        line_len = 0U;
      }
    }
    if (allow_timeout && !update_started &&
        (uint32_t)(HAL_GetTick() - boot_started_ms) >= BOOT_IDLE_TIMEOUT_MS &&
        application_valid()) {
      jump_app();
    }
  }
}

int main(void) {
  HAL_Init();
  system_clock_config();
  backup_access_enable();
  watchdog_last_pat_ms = HAL_GetTick();
  watchdog_armed = true;
  const bool request = read_boot_request();
  const bool valid = application_valid();
  if (request) maintenance_loop(valid);
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
