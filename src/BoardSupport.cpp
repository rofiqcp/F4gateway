#if !defined(F103_BUILD_BOOTLOADER)
#include "BoardSupport.h"

#include <algorithm>
#include <cstring>

SPI_HandleTypeDef hspi1{};
TIM_HandleTypeDef htim1{};
TIM_HandleTypeDef htim3{};
#if BTS_WINCH_ENABLED
TIM_HandleTypeDef htim2{};
TIM_HandleTypeDef htim4{};
#endif


extern "C" uint8_t _end;
extern "C" __attribute__((used, noinline, externally_visible, noreturn)) void Board_FaultReset(uint32_t *stack, uint32_t reason);

namespace {
constexpr uint16_t kTftResetPin = GPIO_PIN_10;

void (*g_watchdog_callback)() = nullptr;
void (*g_realtime_service_callback)() = nullptr;
bool g_realtime_service_active = false;
uint32_t g_realtime_service_last_ms = 0U;
uint32_t g_board_last_service_ms = 0U;
uint32_t g_board_max_service_gap_ms = 0U;
uint32_t g_board_steady_max_service_gap_ms = 0U;
static constexpr uint8_t kServiceGapSampleCount = 64U;
uint16_t g_board_service_gap_samples[kServiceGapSampleCount]{};
uint8_t g_board_service_gap_head = 0U;
uint8_t g_board_service_gap_count = 0U;
bool g_board_service_gap_stats_enabled = false;
uint32_t g_board_min_stack_headroom_bytes = 0xFFFFFFFFUL;
uint32_t g_buzzer_deadline_ms = 0U;
bool g_buzzer_active = false;
#ifdef HMI_TEST_HOOKS
bool g_test_spi_init_fail_once = false;
#endif
BoardSpiOwner g_spi_owner = BoardSpiOwner::NONE;
uint32_t g_spi_contention_count = 0U;
uint32_t g_spi_recovery_count = 0U;

#if BTS_WINCH_ENABLED
volatile uint16_t g_bts_pwm_ticks = 0U;
volatile int8_t g_bts_pwm_direction = 0;

// Hardware-timer PWM with DMA-driven GPIO edges. This keeps pulse generation
// out of CPU/ISR code while avoiding the unreliable native AF output observed
// on this installed F103-compatible device.
//   DOWN: TIM2 period + CH4 events -> DMA -> PA3 (LPWM)
//   UP:   TIM4 period + CH3 events -> DMA -> PB8 (RPWM)
static uint32_t g_dma_pa3_mask = GPIO_PIN_3;
static uint32_t g_dma_pb8_mask = GPIO_PIN_8;

inline void BtsPinsConfigureOutput() {
  GPIO_InitTypeDef gpio{};
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  gpio.Pin = GPIO_PIN_3;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOB, &gpio);
}

inline void BtsPinsLowRaw() {
  GPIOA->BRR = GPIO_PIN_3;
  GPIOB->BRR = GPIO_PIN_8;
}

inline void BtsPinsForceLow() {
  BtsPinsConfigureOutput();
  BtsPinsLowRaw();
}

inline void DmaChannelDisable(DMA_Channel_TypeDef *ch) {
  ch->CCR &= ~DMA_CCR_EN;
  while ((ch->CCR & DMA_CCR_EN) != 0U) {}
}

inline void ConfigureDmaWrite(DMA_Channel_TypeDef *ch,
                              volatile uint32_t *peripheral,
                              uint32_t *memory) {
  DmaChannelDisable(ch);
  ch->CCR = 0U;
  ch->CNDTR = 1U;
  ch->CPAR = reinterpret_cast<uint32_t>(peripheral);
  ch->CMAR = reinterpret_cast<uint32_t>(memory);
  ch->CCR = DMA_CCR_DIR | DMA_CCR_CIRC |
            DMA_CCR_PSIZE_1 | DMA_CCR_MSIZE_1 |
            DMA_CCR_PL_1;
  ch->CCR |= DMA_CCR_EN;
}

inline void StopBtsDmaHardware() {
  TIM2->DIER &= ~(TIM_DIER_UDE | TIM_DIER_CC4DE);
  TIM4->DIER &= ~(TIM_DIER_UDE | TIM_DIER_CC3DE);
  // F103 DMA1 routing used here:
  // TIM2_UP=Ch2, TIM2_CH4=Ch7, TIM4_CH3=Ch5, TIM4_UP=Ch7.
  DmaChannelDisable(DMA1_Channel2);
  DmaChannelDisable(DMA1_Channel5);
  DmaChannelDisable(DMA1_Channel7);
}

inline void StartDownDmaPwm(uint16_t ticks) {
  StopBtsDmaHardware();
  BtsPinsConfigureOutput();
  BtsPinsLowRaw();
  TIM2->CCR4 = ticks;
  TIM2->CNT = 0U;
  TIM2->SR = 0U;
  ConfigureDmaWrite(DMA1_Channel2, &GPIOA->BSRR, &g_dma_pa3_mask); // period HIGH
  ConfigureDmaWrite(DMA1_Channel7, &GPIOA->BRR,  &g_dma_pa3_mask); // compare LOW
  GPIOA->BSRR = GPIO_PIN_3; // seed first period
  TIM2->DIER = TIM_DIER_UDE | TIM_DIER_CC4DE;
  TIM2->CR1 |= TIM_CR1_CEN;
}

inline void StartUpDmaPwm(uint16_t ticks) {
  StopBtsDmaHardware();
  BtsPinsConfigureOutput();
  BtsPinsLowRaw();
  TIM4->CCR3 = ticks;
  TIM4->CNT = 0U;
  TIM4->SR = 0U;
  ConfigureDmaWrite(DMA1_Channel7, &GPIOB->BSRR, &g_dma_pb8_mask); // period HIGH
  ConfigureDmaWrite(DMA1_Channel5, &GPIOB->BRR,  &g_dma_pb8_mask); // compare LOW
  GPIOB->BSRR = GPIO_PIN_8; // seed first period
  TIM4->DIER = TIM_DIER_UDE | TIM_DIER_CC3DE;
  TIM4->CR1 |= TIM_CR1_CEN;
}
#endif

static constexpr uint32_t kAppCrashMagic = 0x48535243UL; // CRSH, shared with recovery bootloader

[[noreturn]] void FatalError() {
  // Generic HAL/init failure. The common reset path records reset diagnostics
  // and marks the application crash for the resident recovery bootloader.
  Board_FaultReset(nullptr, 5U);
}

void SystemClock_Config() {
  RCC_OscInitTypeDef osc{};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  osc.HSEState = RCC_HSE_ON;
  osc.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  osc.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK)
    FatalError();

  RCC_ClkInitTypeDef clk{};
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2) != HAL_OK)
    FatalError();

  RCC_PeriphCLKInitTypeDef periph{};
  periph.PeriphClockSelection = RCC_PERIPHCLK_USB;
  periph.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&periph) != HAL_OK)
    FatalError();
}

void Gpio_Init() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | kTftResetPin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
#if BTS_WINCH_ENABLED
  // BTS7960 outputs must be LOW before timers take ownership.
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);
#endif

  GPIO_InitTypeDef gpio{};
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | kTftResetPin;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOA, &gpio);

#if BTS_WINCH_ENABLED
  // Limit switches use the active-low electrical contract.
  gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);
#endif




  gpio.Pin = GPIO_PIN_13;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &gpio);
}

bool Spi1_Configure() {
#ifdef HMI_TEST_HOOKS
  if (g_test_spi_init_fail_once) {
    g_test_spi_init_fail_once = false;
    return false;
  }
#endif
  __HAL_RCC_SPI1_CLK_ENABLE();
  GPIO_InitTypeDef gpio{};
  // F1 SPI1: SCK/MOSI are AF push-pull; MISO is floating input.
  gpio.Pin = GPIO_PIN_5 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &gpio);
  gpio.Pin = GPIO_PIN_6;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &gpio);

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7U;
  return HAL_SPI_Init(&hspi1) == HAL_OK;
}

void Spi1_InitBootBestEffort() {
  if (Spi1_Configure())
    return;

  // HMI is recoverable and must never keep the gateway in a reset loop before
  // USB diagnostics become available. Leave SPI1 quiescent; HmiDisplay will
  // perform its bounded Board_ReinitSpi1() recovery when initialization starts.
  (void)HAL_SPI_DeInit(&hspi1);
  __HAL_RCC_SPI1_FORCE_RESET();
  __NOP();
  __NOP();
  __HAL_RCC_SPI1_RELEASE_RESET();
}


void Timers_Init() {
  constexpr uint32_t kTimer1MHzPrescaler = 71U;
  constexpr uint32_t kWatchdogPrescaler = 7199U;
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM3_CLK_ENABLE();

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = kTimer1MHzPrescaler;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999U;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0U;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    FatalError();
  TIM_OC_InitTypeDef oc{};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0U;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK)
    FatalError();

  GPIO_InitTypeDef gpio{};
  gpio.Pin = GPIO_PIN_8;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &gpio);

#if BTS_WINCH_ENABLED
  // Hardware-only winch PWM: timer timing + DMA GPIO edges.
  // PA3=LPWM/DOWN uses TIM2 period+CH4 events.
  // PB8=RPWM/UP uses TIM4 period+CH3 events.
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_TIM2_CLK_ENABLE();
  __HAL_RCC_TIM4_CLK_ENABLE();
  BtsPinsForceLow();

  TIM_OC_InitTypeDef btsOc{};
  btsOc.OCMode = TIM_OCMODE_TIMING;
  btsOc.Pulse = 0U;
  btsOc.OCPolarity = TIM_OCPOLARITY_HIGH;
  btsOc.OCFastMode = TIM_OCFAST_DISABLE;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = kTimer1MHzPrescaler;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999U;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim2) != HAL_OK ||
      HAL_TIM_OC_ConfigChannel(&htim2, &btsOc, TIM_CHANNEL_4) != HAL_OK)
    FatalError();

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = kTimer1MHzPrescaler;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999U;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_OC_Init(&htim4) != HAL_OK ||
      HAL_TIM_OC_ConfigChannel(&htim4, &btsOc, TIM_CHANNEL_3) != HAL_OK)
    FatalError();

  TIM2->CCR4 = 0U;
  TIM4->CCR3 = 0U;
  TIM2->DIER = 0U;
  TIM4->DIER = 0U;
  TIM2->CR1 |= TIM_CR1_CEN;
  TIM4->CR1 |= TIM_CR1_CEN;
#endif

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = kWatchdogPrescaler;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999U;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
    FatalError();
  HAL_NVIC_SetPriority(TIM3_IRQn, 1U, 0U);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
}

} // namespace

void Board_BackupWrite(uint8_t slot, uint32_t value) {
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_RCC_BKP_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  // F103 medium-density parts expose 16-bit backup registers. Diagnostic
  // storage is best-effort only; safety never depends on these values.
  volatile uint32_t *reg = nullptr;
  switch (slot) {
    case 0: reg = &BKP->DR1; break; case 1: reg = &BKP->DR2; break;
    case 2: reg = &BKP->DR3; break; case 3: reg = &BKP->DR4; break;
    case 4: reg = &BKP->DR5; break; case 5: reg = &BKP->DR6; break;
    case 6: reg = &BKP->DR7; break; case 7: reg = &BKP->DR8; break;
    case 8: reg = &BKP->DR9; break; case 9: reg = &BKP->DR10; break;
    default: break;
  }
  if (reg != nullptr) *reg = value & 0xFFFFU;
}

uint32_t Board_BackupRead(uint8_t slot) {
  volatile uint32_t *reg = nullptr;
  switch (slot) {
    case 0: reg = &BKP->DR1; break; case 1: reg = &BKP->DR2; break;
    case 2: reg = &BKP->DR3; break; case 3: reg = &BKP->DR4; break;
    case 4: reg = &BKP->DR5; break; case 5: reg = &BKP->DR6; break;
    case 6: reg = &BKP->DR7; break; case 7: reg = &BKP->DR8; break;
    case 8: reg = &BKP->DR9; break; case 9: reg = &BKP->DR10; break;
    default: break;
  }
  return reg != nullptr ? (*reg & 0xFFFFU) : 0U;
}

extern "C" __attribute__((used, noinline, externally_visible, noreturn)) void Board_FaultReset(uint32_t *stack, uint32_t reason) {
  __disable_irq();
#if BTS_WINCH_ENABLED
  g_bts_pwm_ticks = 0U;
  g_bts_pwm_direction = 0;
  StopBtsDmaHardware();
  TIM2->CCR4 = 0U;
  TIM4->CCR3 = 0U;
  BtsPinsForceLow();
  __DSB();
#endif
  Board_BackupWrite(1U, kAppCrashMagic);
  Board_BackupWrite(3U, reason);
  Board_BackupWrite(4U, SCB->CFSR);
  Board_BackupWrite(5U, SCB->HFSR);
  Board_BackupWrite(6U, stack ? stack[6] : 0U);
  Board_BackupWrite(7U, stack ? stack[5] : 0U);
  Board_BackupWrite(8U, SCB->MMFAR);
  Board_BackupWrite(9U, SCB->BFAR);
  __DSB();
  NVIC_SystemReset();
  while (true) { __NOP(); }
}

void Board_Init() {
  HAL_Init();
  __HAL_RCC_PWR_CLK_ENABLE();
  SystemClock_Config();
  Gpio_Init();
  Spi1_InitBootBestEffort();
  Timers_Init();
  Board_SpiDeselectAll();
  // DWT powers deterministic microsecond delays used by TFT/touch timing.
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL &= ~DWT_CTRL_CYCCNTENA_Msk;
  DWT->CYCCNT = 0U;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

void Board_Service() {
  const uint32_t now = HAL_GetTick();
  uint32_t sp = 0U;
  __asm volatile("mov %0, sp" : "=r"(sp));
  const uint32_t ramFloor = reinterpret_cast<uint32_t>(&_end);
  const uint32_t stackHeadroom = sp > ramFloor ? sp - ramFloor : 0U;
  if (stackHeadroom < g_board_min_stack_headroom_bytes)
    g_board_min_stack_headroom_bytes = stackHeadroom;
  if (g_board_last_service_ms != 0U) {
    const uint32_t gap = static_cast<uint32_t>(now - g_board_last_service_ms);
    if (gap > g_board_max_service_gap_ms)
      g_board_max_service_gap_ms = gap;
    if (g_board_service_gap_stats_enabled && gap > 0U) {
      if (gap > g_board_steady_max_service_gap_ms)
        g_board_steady_max_service_gap_ms = gap;
      g_board_service_gap_samples[g_board_service_gap_head] =
          static_cast<uint16_t>(std::min<uint32_t>(gap, 0xFFFFU));
      g_board_service_gap_head = static_cast<uint8_t>(
          (g_board_service_gap_head + 1U) % kServiceGapSampleCount);
      if (g_board_service_gap_count < kServiceGapSampleCount)
        ++g_board_service_gap_count;
    }
  }
  g_board_last_service_ms = now;
  if (g_buzzer_active &&
      static_cast<int32_t>(HAL_GetTick() - g_buzzer_deadline_ms) >= 0) {
    Board_BuzzerStop();
  }
}

uint32_t Board_MaxServiceGapMs() { return g_board_max_service_gap_ms; }
uint32_t Board_SteadyMaxServiceGapMs() {
  return g_board_steady_max_service_gap_ms;
}

static uint32_t ServiceGapPercentile(uint8_t percentile) {
  if (g_board_service_gap_count == 0U)
    return 0U;
  uint16_t values[kServiceGapSampleCount];
  for (uint8_t i = 0U; i < g_board_service_gap_count; ++i)
    values[i] = g_board_service_gap_samples[i];
  for (uint8_t i = 1U; i < g_board_service_gap_count; ++i) {
    const uint16_t value = values[i];
    uint8_t j = i;
    while (j > 0U && values[j - 1U] > value) {
      values[j] = values[j - 1U];
      --j;
    }
    values[j] = value;
  }
  const uint32_t numerator =
      static_cast<uint32_t>(g_board_service_gap_count - 1U) * percentile;
  const uint8_t index = static_cast<uint8_t>((numerator + 99U) / 100U);
  return values[std::min<uint8_t>(
      index, static_cast<uint8_t>(g_board_service_gap_count - 1U))];
}

uint32_t Board_ServiceGapP95Ms() { return ServiceGapPercentile(95U); }
uint32_t Board_ServiceGapP99Ms() { return ServiceGapPercentile(99U); }

void Board_ResetServiceGapStats() {
  g_board_steady_max_service_gap_ms = 0U;
  g_board_service_gap_head = 0U;
  g_board_service_gap_count = 0U;
  std::fill_n(g_board_service_gap_samples, kServiceGapSampleCount, 0U);
  g_board_last_service_ms = HAL_GetTick();
  g_board_service_gap_stats_enabled = true;
}

uint32_t Board_StackHeadroomBytes() {
  uint32_t sp = 0U;
  __asm volatile("mov %0, sp" : "=r"(sp));
  const uint32_t ramFloor = reinterpret_cast<uint32_t>(&_end);
  return sp > ramFloor ? sp - ramFloor : 0U;
}

uint32_t Board_MinStackHeadroomBytes() {
  return g_board_min_stack_headroom_bytes == 0xFFFFFFFFUL
             ? Board_StackHeadroomBytes()
             : g_board_min_stack_headroom_bytes;
}

void Board_SetRealtimeServiceCallback(void (*callback)()) {
  g_realtime_service_callback = callback;
}

void Board_RealtimeService() {
  // Long TFT draws yield at most once per millisecond so USB and safety
  // processing stay responsive without re-entering this callback recursively.
  const uint32_t now = HAL_GetTick();
  if (g_realtime_service_active || now == g_realtime_service_last_ms)
    return;
  g_realtime_service_last_ms = now;
  g_realtime_service_active = true;
  Board_Service();
  if (g_realtime_service_callback != nullptr)
    g_realtime_service_callback();
  g_realtime_service_active = false;
}

void Board_DelayUs(uint32_t microseconds) {
  const uint32_t start = DWT->CYCCNT;
  const uint32_t cycles = microseconds * (HAL_RCC_GetHCLKFreq() / 1000000U);
  while (static_cast<uint32_t>(DWT->CYCCNT - start) < cycles) {
    __NOP();
  }
}

#ifdef HMI_TEST_HOOKS
void Board_TestInjectSpiInitFailureOnce() { g_test_spi_init_fail_once = true; }
#endif

void Board_SpiDeselectAll() {
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);  // TFT CS
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);  // XPT2046 CS
}

BoardSpiOwner Board_SpiOwner() { return g_spi_owner; }
uint32_t Board_SpiContentionCount() { return g_spi_contention_count; }
uint32_t Board_SpiRecoveryCount() { return g_spi_recovery_count; }

bool Board_SpiAcquire(BoardSpiOwner owner, uint32_t prescaler) {
  if (owner == BoardSpiOwner::NONE)
    return false;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (g_spi_owner != BoardSpiOwner::NONE) {
    ++g_spi_contention_count;
    if (primask == 0U) __enable_irq();
    return false;
  }
  g_spi_owner = owner;
  if (primask == 0U) __enable_irq();

  Board_SpiDeselectAll();
  const uint32_t cycles_per_us = std::max<uint32_t>(1U, HAL_RCC_GetHCLKFreq() / 1000000U);
  const uint32_t start = DWT->CYCCNT;
  while ((SPI1->SR & SPI_SR_BSY) != 0U) {
    if (static_cast<uint32_t>(DWT->CYCCNT - start) >= 2500U * cycles_per_us) {
      Board_SpiRelease(owner);
      return false;
    }
  }
  while ((SPI1->SR & SPI_SR_RXNE) != 0U)
    (void)*reinterpret_cast<volatile uint8_t *>(&SPI1->DR);
  if ((SPI1->SR & SPI_SR_OVR) != 0U) {
    (void)SPI1->DR;
    (void)SPI1->SR;
  }
  CLEAR_BIT(SPI1->CR1, SPI_CR1_SPE);
  // ILI9341 and XPT2046 share SPI1 mode 0; reassert it on each owner handoff.
  CLEAR_BIT(SPI1->CR1, SPI_CR1_CPOL | SPI_CR1_CPHA);
  MODIFY_REG(SPI1->CR1, SPI_CR1_BR, prescaler);
  SET_BIT(SPI1->CR1, SPI_CR1_SPE);
  return true;
}

void Board_SpiRelease(BoardSpiOwner owner) {
  Board_SpiDeselectAll();
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (g_spi_owner == owner || owner == BoardSpiOwner::NONE)
    g_spi_owner = BoardSpiOwner::NONE;
  if (primask == 0U) __enable_irq();
}

bool Board_ReinitSpi1() {
  // SPI1 is dedicated to TFT/touch in the production v2 build.
  Board_SpiDeselectAll();
  g_spi_owner = BoardSpiOwner::NONE;
  ++g_spi_recovery_count;
  (void)HAL_SPI_DeInit(&hspi1);
  __HAL_RCC_SPI1_FORCE_RESET();
  __NOP();
  __NOP();
  __HAL_RCC_SPI1_RELEASE_RESET();
  return Spi1_Configure() && hspi1.State == HAL_SPI_STATE_READY;
}

void Board_RealtimeDelayMs(uint32_t duration_ms) {
  const uint32_t deadline = HAL_GetTick() + duration_ms;
  while (static_cast<int32_t>(deadline - HAL_GetTick()) > 0) {
    Board_RealtimeService();
    __WFI();
  }
}


void Board_SetWatchdogCallback(void (*callback)()) {
  g_watchdog_callback = callback;
}
void Board_WatchdogStart() {
  __HAL_TIM_SET_COUNTER(&htim3, 0U);
  (void)HAL_TIM_Base_Start_IT(&htim3);
}
void Board_WatchdogStop() { (void)HAL_TIM_Base_Stop_IT(&htim3); }

#if BTS_WINCH_ENABLED
void Board_BtsEmergencyCut() {
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_bts_pwm_ticks = 0U;
  g_bts_pwm_direction = 0;
  StopBtsDmaHardware();
  TIM2->CCR4 = 0U;
  TIM4->CCR3 = 0U;
  BtsPinsForceLow();
  __DSB();
  if (primask == 0U) __enable_irq();
}

void Board_BtsSetPwm(uint16_t rpwm, uint16_t lpwm) {
  rpwm = std::min<uint16_t>(rpwm, 1023U);
  lpwm = std::min<uint16_t>(lpwm, 1023U);
  if (rpwm != 0U && lpwm != 0U) {
    Board_BtsEmergencyCut();
    return;
  }
  if (rpwm == 0U && lpwm == 0U) {
    Board_BtsEmergencyCut();
    return;
  }

  const uint16_t rTicks = rpwm >= 1023U ? 1000U : static_cast<uint16_t>(
      (static_cast<uint32_t>(rpwm) * 999U + 511U) / 1023U);
  const uint16_t lTicks = lpwm >= 1023U ? 1000U : static_cast<uint16_t>(
      (static_cast<uint32_t>(lpwm) * 999U + 511U) / 1023U);

  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (rpwm != 0U) {
    TIM2->CCR4 = 0U;
    StartUpDmaPwm(rTicks);
    g_bts_pwm_ticks = rTicks;
    g_bts_pwm_direction = 1;
  } else {
    TIM4->CCR3 = 0U;
    StartDownDmaPwm(lTicks);
    g_bts_pwm_ticks = lTicks;
    g_bts_pwm_direction = -1;
  }
  __DSB();
  if (primask == 0U) __enable_irq();
}

uint16_t Board_BtsDutyTicks() { return g_bts_pwm_ticks; }
int8_t Board_BtsDirection() { return g_bts_pwm_direction; }
#endif

void Board_BuzzerStart(uint16_t frequency_hz, uint16_t duration_ms) {
  if (frequency_hz < 20U)
    return;
  const uint32_t period = std::max<uint32_t>(2U, 1000000U / frequency_hz);
  __HAL_TIM_DISABLE(&htim1);
  __HAL_TIM_SET_AUTORELOAD(&htim1, period - 1U);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, period / 2U);
  __HAL_TIM_SET_COUNTER(&htim1, 0U);
  __HAL_TIM_ENABLE(&htim1);
  (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  g_buzzer_active = true;
  g_buzzer_deadline_ms = HAL_GetTick() + duration_ms;
}

void Board_BuzzerStop() {
  (void)HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
  g_buzzer_active = false;
  g_buzzer_deadline_ms = 0U;
}

// HAL_Init() enables the Cortex-M SysTick timebase. The STM32Cube startup file
// weak-aliases SysTick_Handler to Default_Handler, so a native application must
// provide this ISR explicitly. Without it the first 1 ms tick traps the MCU in
// the default infinite loop before USB/HMI/sensor startup can complete.
extern "C" void SysTick_Handler() {
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}



extern "C" void TIM3_IRQHandler() { HAL_TIM_IRQHandler(&htim3); }

// cppcheck-suppress constParameter -- STM32 HAL callback ABI requires mutable
// handle pointer.
extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim3 && g_watchdog_callback != nullptr)
    g_watchdog_callback();
}

// Fault wrappers capture the active exception stack before resetting. Keep the
// handlers minimal: no printf, USB, SPI, heap, or HAL calls on a corrupt stack.
extern "C" __attribute__((naked)) void HardFault_Handler() {
  __asm volatile("tst lr,#4\n ite eq\n mrseq r0,msp\n mrsne r0,psp\n movs r1,#1\n b Board_FaultReset");
}
extern "C" __attribute__((naked)) void MemManage_Handler() {
  __asm volatile("tst lr,#4\n ite eq\n mrseq r0,msp\n mrsne r0,psp\n movs r1,#2\n b Board_FaultReset");
}
extern "C" __attribute__((naked)) void BusFault_Handler() {
  __asm volatile("tst lr,#4\n ite eq\n mrseq r0,msp\n mrsne r0,psp\n movs r1,#3\n b Board_FaultReset");
}
extern "C" __attribute__((naked)) void UsageFault_Handler() {
  __asm volatile("tst lr,#4\n ite eq\n mrseq r0,msp\n mrsne r0,psp\n movs r1,#4\n b Board_FaultReset");
}
extern "C" void Error_Handler() { FatalError(); }

#endif
