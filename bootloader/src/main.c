#include "stm32f4xx_hal.h"
#include "boot_usb.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

#define MANIFEST_MAGIC 0x31564741UL
#define MANIFEST_FORMAT 1UL
#define SRAM_BASE_ADDR 0x20000000UL
#define SRAM_END_ADDR  0x20020000UL
#define APP_CRASH_MAGIC 0x48535243UL
#define APP_CRASH_LIMIT 3UL
#define BOOT_IDLE_TIMEOUT_MS 20000UL
#define BOOT_LINE_MAX 600U
#define BOOT_DATA_MAX 240U

typedef struct {
  uint32_t magic, format, app_base, app_size, app_crc32, header_crc32, generation, reserved;
} app_manifest_t;

static char line_buf[BOOT_LINE_MAX];
static uint32_t line_len;
static bool update_started;
static uint32_t expected_size, expected_crc, write_offset;
static bool timeout_to_app;
static uint32_t boot_started_ms;
static bool boot_usb_started;

static void fatal_reset(void) { NVIC_SystemReset(); while (1) {} }

static void system_clock_config(void) {
  RCC_OscInitTypeDef osc = {0};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc.PLL.PLLM = 8U; osc.PLL.PLLN = 96U; osc.PLL.PLLP = RCC_PLLP_DIV2; osc.PLL.PLLQ = 4U;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) fatal_reset();
  RCC_ClkInitTypeDef clk = {0};
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) fatal_reset();
}

static uint32_t crc32_bytes(const uint8_t *data, uint32_t len) {
  uint32_t crc = 0xFFFFFFFFUL;
  while (len--) {
    crc ^= *data++;
    for (uint32_t bit = 0; bit < 8U; ++bit) crc = (crc >> 1U) ^ (0xEDB88320UL & (0U - (crc & 1U)));
  }
  return crc ^ 0xFFFFFFFFUL;
}

static void backup_access_enable(void) {
  __HAL_RCC_PWR_CLK_ENABLE(); HAL_PWR_EnableBkUpAccess();
  for (volatile uint32_t i = 0; i < 1000U; ++i) __NOP();
}

static uint32_t read_boot_request(void) {
  backup_access_enable();
  const uint32_t v = RTC->BKP0R; RTC->BKP0R = 0U; __DSB(); return v;
}

static bool repeated_app_crash(void) {
  backup_access_enable();
  const uint32_t marker = RTC->BKP1R; RTC->BKP1R = 0U;
  if (marker != APP_CRASH_MAGIC) return false;
  uint32_t count = RTC->BKP2R;
  if (count < APP_CRASH_LIMIT) ++count;
  RTC->BKP2R = count; __DSB();
  return count >= APP_CRASH_LIMIT;
}

static bool vector_valid(uint32_t base, uint32_t limit) {
  const uint32_t sp = *(volatile const uint32_t *)base;
  const uint32_t reset = *(volatile const uint32_t *)(base + 4U);
  if (sp < SRAM_BASE_ADDR || sp > SRAM_END_ADDR || (sp & 3U) != 0U) return false;
  if ((reset & 1U) == 0U) return false;
  const uint32_t pc = reset & ~1UL;
  return pc >= base && pc < limit;
}

static bool application_valid(void) {
  const app_manifest_t *m = (const app_manifest_t *)MANIFEST_ADDR;
  if (m->magic != MANIFEST_MAGIC || m->format != MANIFEST_FORMAT) return false;
  if (m->app_base != APP_BASE || m->app_size < 8U || m->app_size > (APP_LIMIT - APP_BASE)) return false;
  if (crc32_bytes((const uint8_t *)m, 20U) != m->header_crc32) return false;
  if (!vector_valid(APP_BASE, APP_LIMIT)) return false;
  return crc32_bytes((const uint8_t *)APP_BASE, m->app_size) == m->app_crc32;
}

static void quiesce(void) {
  __disable_irq(); SysTick->CTRL = SysTick->LOAD = SysTick->VAL = 0U;
  for (uint32_t i = 0; i < 8U; ++i) { NVIC->ICER[i] = 0xFFFFFFFFUL; NVIC->ICPR[i] = 0xFFFFFFFFUL; }
  __DSB(); __ISB();
}

__attribute__((noreturn)) static void jump_vector(uint32_t base) {
  const uint32_t sp = *(volatile const uint32_t *)base;
  const uint32_t reset = *(volatile const uint32_t *)(base + 4U);
  if (boot_usb_started) { boot_usb_end(); boot_usb_started = false; }
  quiesce(); SCB->VTOR = base; __set_CONTROL(0U); __set_PSP(0U); __set_MSP(sp);
  __DSB(); __ISB(); __enable_irq(); ((void (*)(void))reset)(); while (1) {}
}

__attribute__((noreturn)) static void jump_system_dfu(void) {
  if (!vector_valid(SYSTEM_MEMORY, 0x20000000UL)) fatal_reset();
  if (boot_usb_started) { boot_usb_end(); boot_usb_started = false; }
  __HAL_RCC_SYSCFG_CLK_ENABLE(); SYSCFG->MEMRMP = 0x01U;
  const uint32_t sp = *(volatile const uint32_t *)SYSTEM_MEMORY;
  const uint32_t reset = *(volatile const uint32_t *)(SYSTEM_MEMORY + 4U);
  quiesce(); SCB->VTOR = SYSTEM_MEMORY; __set_CONTROL(0U); __set_PSP(0U); __set_MSP(sp);
  __DSB(); __ISB(); __enable_irq(); ((void (*)(void))reset)(); while (1) {}
}

static bool erase_sector(uint32_t sector) {
  FLASH_EraseInitTypeDef erase = {0}; uint32_t error = 0U;
  erase.TypeErase = FLASH_TYPEERASE_SECTORS; erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
  erase.Sector = sector; erase.NbSectors = 1U;
  return HAL_FLASHEx_Erase(&erase, &error) == HAL_OK;
}

static bool begin_update(uint32_t size, uint32_t crc) {
  if (size < 8U || size > (APP_LIMIT - APP_BASE)) return false;
  if (HAL_FLASH_Unlock() != HAL_OK) return false;
  bool ok = erase_sector(FLASH_SECTOR_7); /* invalidate manifest FIRST */
  for (uint32_t s = FLASH_SECTOR_2; ok && s <= FLASH_SECTOR_6; ++s) ok = erase_sector(s);
  (void)HAL_FLASH_Lock();
  if (!ok) return false;
  expected_size = size; expected_crc = crc; write_offset = 0U; update_started = true; timeout_to_app = false;
  return true;
}

static bool program_chunk(uint32_t offset, const uint8_t *data, uint32_t len) {
  if (!update_started || data == NULL || len == 0U || len > BOOT_DATA_MAX || offset != write_offset) return false;
  if (offset + len > expected_size) return false;
  if (HAL_FLASH_Unlock() != HAL_OK) return false;
  bool ok = true;
  uint32_t pos = 0U;
  while (pos < len) {
    uint32_t word = 0xFFFFFFFFUL;
    const uint32_t take = (len - pos) >= 4U ? 4U : (len - pos);
    memcpy(&word, data + pos, take);
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, APP_BASE + offset + pos, word) != HAL_OK) { ok = false; break; }
    pos += take;
  }
  (void)HAL_FLASH_Lock();
  if (ok) write_offset += len;
  return ok;
}

static bool commit_manifest(void) {
  if (!update_started || write_offset != expected_size) return false;
  if (crc32_bytes((const uint8_t *)APP_BASE, expected_size) != expected_crc) return false;
  if (!vector_valid(APP_BASE, APP_BASE + expected_size)) return false;
  app_manifest_t m = {0};
  m.magic = MANIFEST_MAGIC; m.format = MANIFEST_FORMAT; m.app_base = APP_BASE;
  m.app_size = expected_size; m.app_crc32 = expected_crc; m.header_crc32 = crc32_bytes((const uint8_t *)&m, 20U);
  m.generation = HAL_GetTick(); m.reserved = 0U;
  if (HAL_FLASH_Unlock() != HAL_OK) return false;
  bool ok = true;
  const uint32_t *words = (const uint32_t *)&m;
  for (uint32_t i = 0U; i < sizeof(m)/4U; ++i) {
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, MANIFEST_ADDR + 4U*i, words[i]) != HAL_OK) { ok = false; break; }
  }
  (void)HAL_FLASH_Lock();
  return ok && application_valid();
}

static int hex_nibble(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

static bool decode_hex(const char *hex, uint8_t *out, uint32_t *len) {
  const size_t n = strlen(hex);
  if ((n & 1U) != 0U || n == 0U || n > BOOT_DATA_MAX*2U) return false;
  *len = (uint32_t)(n/2U);
  for (uint32_t i = 0U; i < *len; ++i) {
    int hi = hex_nibble(hex[2U*i]), lo = hex_nibble(hex[2U*i+1U]);
    if (hi < 0 || lo < 0) return false;
    out[i] = (uint8_t)((hi << 4) | lo);
  }
  return true;
}

static void reply_info(void) {
  char out[160];
  const app_manifest_t *m = (const app_manifest_t *)MANIFEST_ADDR;
  snprintf(out, sizeof(out), "BOOT:INFO:valid=%u:update=%u:offset=%lu:size=%lu:manifest=%08lX",
           application_valid()?1U:0U, update_started?1U:0U, (unsigned long)write_offset,
           (unsigned long)expected_size, (unsigned long)m->magic);
  (void)boot_usb_write_line(out, 250U);
}

static void handle_line(char *line) {
  if (!strcmp(line, "PING")) { (void)boot_usb_write_line("BOOT:PONG", 250U); return; }
  if (!strcmp(line, "INFO")) { reply_info(); return; }
  if (!strcmp(line, "BOOT")) {
    if (!update_started && application_valid()) { (void)boot_usb_write_line("ACK:BOOT", 250U); boot_usb_flush(250U); HAL_Delay(20U); jump_vector(APP_BASE); }
    (void)boot_usb_write_line("ERR:BOOT:APP_INVALID", 250U); return;
  }
  if (!strcmp(line, "ROMDFU")) { (void)boot_usb_write_line("ACK:ROMDFU", 250U); boot_usb_flush(250U); HAL_Delay(20U); jump_system_dfu(); }
  if (!strncmp(line, "BEGIN:", 6U)) {
    char *sep = strchr(line + 6U, ':');
    if (!sep) { (void)boot_usb_write_line("ERR:BEGIN:FORMAT", 250U); return; }
    *sep = '\0'; char *end1=NULL,*end2=NULL;
    uint32_t size = (uint32_t)strtoul(line+6U,&end1,0); uint32_t crc=(uint32_t)strtoul(sep+1U,&end2,16);
    if (!end1 || *end1!='\0' || !end2 || *end2!='\0') { (void)boot_usb_write_line("ERR:BEGIN:FORMAT",250U); return; }
    (void)boot_usb_write_line("BOOT:ERASING", 250U); boot_usb_flush(250U);
    if (!begin_update(size,crc)) { (void)boot_usb_write_line("ERR:BEGIN:FLASH",250U); return; }
    (void)boot_usb_write_line("ACK:BEGIN:0",250U); return;
  }
  if (!strncmp(line,"DATA:",5U)) {
    char *sep=strchr(line+5U,':'); if(!sep){(void)boot_usb_write_line("ERR:DATA:FORMAT",250U);return;}
    *sep='\0'; char *end=NULL; uint32_t off=(uint32_t)strtoul(line+5U,&end,0);
    uint8_t data[BOOT_DATA_MAX]; uint32_t len=0U;
    if(!end||*end!='\0'||!decode_hex(sep+1U,data,&len)){(void)boot_usb_write_line("ERR:DATA:FORMAT",250U);return;}
    if(!program_chunk(off,data,len)){ char out[64]; snprintf(out,sizeof(out),"ERR:DATA:OFFSET:%lu",(unsigned long)write_offset); (void)boot_usb_write_line(out,250U); return; }
    char out[64]; snprintf(out,sizeof(out),"ACK:DATA:%lu",(unsigned long)write_offset); (void)boot_usb_write_line(out,250U); return;
  }
  if (!strcmp(line,"END")) {
    if(!commit_manifest()){(void)boot_usb_write_line("ERR:END:VERIFY",250U);return;}
    update_started=false; (void)boot_usb_write_line("ACK:END:OK",250U); boot_usb_flush(300U); HAL_Delay(30U); NVIC_SystemReset(); while(1){}
  }
  if (!strcmp(line,"ABORT")) {
    if(update_started || !application_valid()) {(void)boot_usb_write_line("ACK:ABORT:HOLD",250U); timeout_to_app=false; return;}
    (void)boot_usb_write_line("ACK:ABORT:BOOT",250U); boot_usb_flush(250U); HAL_Delay(20U); jump_vector(APP_BASE);
  }
  (void)boot_usb_write_line("ERR:UNKNOWN",250U);
}

static void maintenance_loop(bool allow_timeout) {
  timeout_to_app = allow_timeout && application_valid(); update_started=false; expected_size=expected_crc=write_offset=0U;
  if (!boot_usb_begin()) fatal_reset();
  boot_usb_started = true;
  boot_started_ms=HAL_GetTick();
  while(1) {
    while(boot_usb_available()>0) {
      int v=boot_usb_read(); if(v<0) break; char c=(char)v; if(c=='\r')continue;
      if(c=='\n') { line_buf[line_len]='\0'; if(line_len)handle_line(line_buf); line_len=0U; boot_started_ms=HAL_GetTick(); }
      else if(line_len+1U<sizeof(line_buf)) line_buf[line_len++]=c; else line_len=0U;
    }
    if(timeout_to_app && !update_started && (uint32_t)(HAL_GetTick()-boot_started_ms)>=BOOT_IDLE_TIMEOUT_MS && application_valid()) jump_vector(APP_BASE);
  }
}

int main(void) {
  HAL_Init(); __HAL_RCC_PWR_CLK_ENABLE(); __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1); system_clock_config();
  const uint32_t request=read_boot_request(); const bool crash=repeated_app_crash(); const bool valid=application_valid();
  if(request==BOOT_REQUEST_MAGIC) maintenance_loop(valid);
  if(crash) maintenance_loop(false);
  if(valid) jump_vector(APP_BASE);
  maintenance_loop(false);
}

void SysTick_Handler(void){ HAL_IncTick(); HAL_SYSTICK_IRQHandler(); }
void Error_Handler(void){ fatal_reset(); }
