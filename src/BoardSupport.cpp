#include "BoardSupport.h"

#include <algorithm>
#include <cstring>

SPI_HandleTypeDef hspi1{};
I2C_HandleTypeDef hi2c1{};
UART_HandleTypeDef huart1{};
UART_HandleTypeDef huart2{};
TIM_HandleTypeDef htim1{};
TIM_HandleTypeDef htim11{};

HalUartPort gVescUart(&huart1, USART1);
HalUartPort gGnssUart(&huart2, USART2);

namespace {
void (*g_watchdog_callback)() = nullptr;
uint32_t g_buzzer_deadline_ms = 0U;
bool g_buzzer_active = false;

[[noreturn]] void FatalError() {
  __disable_irq();
  while (true) { __NOP(); }
}

void SystemClock_Config() {
  RCC_OscInitTypeDef osc{};
  osc.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  osc.HSIState = RCC_HSI_ON;
  osc.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  osc.PLL.PLLState = RCC_PLL_ON;
  osc.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  osc.PLL.PLLM = 8U;
  osc.PLL.PLLN = 96U;
  osc.PLL.PLLP = RCC_PLLP_DIV2;
  osc.PLL.PLLQ = 4U;
  if (HAL_RCC_OscConfig(&osc) != HAL_OK) FatalError();

  RCC_ClkInitTypeDef clk{};
  clk.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                  RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  clk.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clk.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clk.APB1CLKDivider = RCC_HCLK_DIV2;
  clk.APB2CLKDivider = RCC_HCLK_DIV1;
  if (HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_3) != HAL_OK) FatalError();
}

void Gpio_Init() {
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0 | GPIO_PIN_2, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);

  GPIO_InitTypeDef gpio{};
  gpio.Pin = GPIO_PIN_0 | GPIO_PIN_1 | GPIO_PIN_2;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = GPIO_PIN_4;
  HAL_GPIO_Init(GPIOA, &gpio);

  gpio.Pin = GPIO_PIN_13;
  gpio.Mode = GPIO_MODE_OUTPUT_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = GPIO_PIN_12;
  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &gpio);

  gpio.Pin = GPIO_PIN_13;
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &gpio);
}

void Spi1_Init() {
  __HAL_RCC_SPI1_CLK_ENABLE();
  GPIO_InitTypeDef gpio{};
  gpio.Pin = GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF5_SPI1;
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
  if (HAL_SPI_Init(&hspi1) != HAL_OK) FatalError();
}

void I2c1_Init() {
  __HAL_RCC_I2C1_CLK_ENABLE();
  GPIO_InitTypeDef gpio{};
  gpio.Pin = GPIO_PIN_8 | GPIO_PIN_9;
  gpio.Mode = GPIO_MODE_AF_OD;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  gpio.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &gpio);

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000U;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0U;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0U;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK) FatalError();
}

void Timers_Init() {
  __HAL_RCC_TIM1_CLK_ENABLE();
  __HAL_RCC_TIM11_CLK_ENABLE();

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 95U;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999U;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0U;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK) FatalError();
  TIM_OC_InitTypeDef oc{};
  oc.OCMode = TIM_OCMODE_PWM1;
  oc.Pulse = 0U;
  oc.OCPolarity = TIM_OCPOLARITY_HIGH;
  oc.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  oc.OCFastMode = TIM_OCFAST_DISABLE;
  oc.OCIdleState = TIM_OCIDLESTATE_RESET;
  oc.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &oc, TIM_CHANNEL_1) != HAL_OK) FatalError();

  GPIO_InitTypeDef gpio{};
  gpio.Pin = GPIO_PIN_8;
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;
  gpio.Alternate = GPIO_AF1_TIM1;
  HAL_GPIO_Init(GPIOA, &gpio);

  htim11.Instance = TIM11;
  htim11.Init.Prescaler = 9599U;
  htim11.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim11.Init.Period = 999U;
  htim11.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim11.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim11) != HAL_OK) FatalError();
  HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 2U, 0U);
  HAL_NVIC_EnableIRQ(TIM1_TRG_COM_TIM11_IRQn);
}

void UartPinsInit(UART_HandleTypeDef *huart) {
  GPIO_InitTypeDef gpio{};
  gpio.Mode = GPIO_MODE_AF_PP;
  gpio.Pull = GPIO_PULLUP;
  gpio.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  if (huart->Instance == USART1) {
    __HAL_RCC_USART1_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    gpio.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOB, &gpio);
    HAL_NVIC_SetPriority(USART1_IRQn, 0U, 0U);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  } else if (huart->Instance == USART2) {
    __HAL_RCC_USART2_CLK_ENABLE();
    gpio.Pin = GPIO_PIN_2 | GPIO_PIN_3;
    gpio.Alternate = GPIO_AF7_USART2;
    HAL_GPIO_Init(GPIOA, &gpio);
    HAL_NVIC_SetPriority(USART2_IRQn, 1U, 0U);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  }
}
}

void Board_Init() {
  HAL_Init();
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  SystemClock_Config();
  Gpio_Init();
  Spi1_Init();
  I2c1_Init();
  Timers_Init();
  if ((DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) == 0U) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
  }
}

void Board_Service() {
  (void)gVescUart.service();
  (void)gGnssUart.service();
  if (g_buzzer_active && static_cast<int32_t>(HAL_GetTick() - g_buzzer_deadline_ms) >= 0) {
    Board_BuzzerStop();
  }
}

void Board_DelayUs(uint32_t microseconds) {
  const uint32_t start = DWT->CYCCNT;
  const uint32_t cycles = microseconds * (HAL_RCC_GetHCLKFreq() / 1000000U);
  while (static_cast<uint32_t>(DWT->CYCCNT - start) < cycles) { __NOP(); }
}

void Board_ReinitI2c1() {
  (void)HAL_I2C_DeInit(&hi2c1);
  I2c1_Init();
}

void Board_SetWatchdogCallback(void (*callback)()) { g_watchdog_callback = callback; }
void Board_WatchdogStart() { __HAL_TIM_SET_COUNTER(&htim11, 0U); (void)HAL_TIM_Base_Start_IT(&htim11); }
void Board_WatchdogStop() { (void)HAL_TIM_Base_Stop_IT(&htim11); }

void Board_BuzzerStart(uint16_t frequency_hz, uint16_t duration_ms) {
  if (frequency_hz < 20U) return;
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

bool HalUartPort::begin(uint32_t baudrate) {
  end();
  handle_->Instance = instance_;
  handle_->Init.BaudRate = baudrate;
  handle_->Init.WordLength = UART_WORDLENGTH_8B;
  handle_->Init.StopBits = UART_STOPBITS_1;
  handle_->Init.Parity = UART_PARITY_NONE;
  handle_->Init.Mode = UART_MODE_TX_RX;
  handle_->Init.HwFlowCtl = UART_HWCONTROL_NONE;
  handle_->Init.OverSampling = UART_OVERSAMPLING_16;
  UartPinsInit(handle_);
  if (HAL_UART_Init(handle_) != HAL_OK) return false;
  rx_head_ = rx_tail_ = 0U;
  tx_head_ = tx_tail_ = tx_pending_ = 0U;
  tx_busy_ = false;
  tx_dropped_ = 0U;
  overflow_count_ = 0U;
  error_count_ = 0U;
  rx_restart_required_ = false;
  const bool started = HAL_UART_Receive_IT(handle_, &rx_byte_, 1U) == HAL_OK;
  rx_restart_required_ = !started;
  return started;
}

void HalUartPort::end() {
  if (handle_->Instance != nullptr && handle_->gState != HAL_UART_STATE_RESET) {
    (void)HAL_UART_Abort_IT(handle_);
    (void)HAL_UART_DeInit(handle_);
  }
  handle_->Instance = instance_;
  rx_head_ = rx_tail_ = 0U;
  tx_head_ = tx_tail_ = tx_pending_ = 0U;
  tx_busy_ = false;
  rx_restart_required_ = false;
}

int HalUartPort::available() const {
  const uint16_t head = rx_head_;
  const uint16_t tail = rx_tail_;
  return head >= tail ? static_cast<int>(head - tail)
                      : static_cast<int>(kRxSize - tail + head);
}

int HalUartPort::read() {
  if (rx_tail_ == rx_head_) return -1;
  const uint8_t value = rx_buffer_[rx_tail_];
  rx_tail_ = static_cast<uint16_t>((rx_tail_ + 1U) % kRxSize);
  return value;
}

int HalUartPort::availableForWrite() const {
  const uint16_t head = tx_head_;
  const uint16_t tail = tx_tail_;
  const uint16_t used = head >= tail ? static_cast<uint16_t>(head - tail)
                                     : static_cast<uint16_t>(kTxSize - tail + head);
  return static_cast<int>(kTxSize - used - 1U);
}

std::size_t HalUartPort::queuedForWrite() const {
  const uint16_t head = tx_head_;
  const uint16_t tail = tx_tail_;
  return head >= tail ? static_cast<std::size_t>(head - tail)
                      : static_cast<std::size_t>(kTxSize - tail + head);
}

void HalUartPort::discardPendingTx() {
  // Safety-critical preemption: no stale motor command may remain ahead of an
  // E-stop. Abort only the TX side; USART RX and its ring stay alive.
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  (void)HAL_UART_AbortTransmit(handle_);
  tx_head_ = tx_tail_ = tx_pending_ = 0U;
  tx_busy_ = false;
  if (primask == 0U) __enable_irq();
}

std::size_t HalUartPort::write(const uint8_t *data, std::size_t length) {
  if (data == nullptr || length == 0U || length >= kTxSize) return 0U;
  const uint32_t primask = __get_PRIMASK();
  __disable_irq();
  const uint16_t head = tx_head_;
  const uint16_t tail = tx_tail_;
  const uint16_t used = head >= tail ? static_cast<uint16_t>(head - tail)
                                     : static_cast<uint16_t>(kTxSize - tail + head);
  const uint16_t free = static_cast<uint16_t>(kTxSize - used - 1U);
  if (free < length) {
    ++tx_dropped_;
    if (primask == 0U) __enable_irq();
    return 0U;
  }
  for (std::size_t i = 0U; i < length; ++i) {
    tx_buffer_[tx_head_] = data[i];
    tx_head_ = static_cast<uint16_t>((tx_head_ + 1U) % kTxSize);
  }
  if (primask == 0U) __enable_irq();
  (void)service();
  return length;
}

void HalUartPort::flush() {
  const uint32_t start = HAL_GetTick();
  while ((tx_head_ != tx_tail_ || tx_busy_ ||
          __HAL_UART_GET_FLAG(handle_, UART_FLAG_TC) == RESET) &&
         static_cast<uint32_t>(HAL_GetTick() - start) < 150U) {
    (void)service();
  }
}

bool HalUartPort::service() {
  bool ok = true;
  if (rx_restart_required_) {
    (void)HAL_UART_AbortReceive(handle_);
    __HAL_UART_CLEAR_OREFLAG(handle_);
    __HAL_UART_CLEAR_NEFLAG(handle_);
    __HAL_UART_CLEAR_FEFLAG(handle_);
    const bool restarted = HAL_UART_Receive_IT(handle_, &rx_byte_, 1U) == HAL_OK;
    rx_restart_required_ = !restarted;
    ok = restarted;
  }

  if (!tx_busy_ && tx_head_ != tx_tail_) {
    uint16_t count = 0U;
    uint16_t tail = 0U;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (!tx_busy_ && tx_head_ != tx_tail_) {
      tail = tx_tail_;
      count = tx_head_ > tx_tail_ ? static_cast<uint16_t>(tx_head_ - tx_tail_)
                                  : static_cast<uint16_t>(kTxSize - tx_tail_);
      tx_pending_ = count;
      tx_busy_ = true;
    }
    if (primask == 0U) __enable_irq();
    if (count > 0U && HAL_UART_Transmit_IT(handle_, &tx_buffer_[tail], count) != HAL_OK) {
      const uint32_t retry_primask = __get_PRIMASK();
      __disable_irq();
      tx_pending_ = 0U;
      tx_busy_ = false;
      ++error_count_;
      if (retry_primask == 0U) __enable_irq();
      ok = false;
    }
  }
  return ok;
}

void HalUartPort::irqRxComplete() {
  const uint16_t next = static_cast<uint16_t>((rx_head_ + 1U) % kRxSize);
  if (next != rx_tail_) {
    rx_buffer_[rx_head_] = rx_byte_;
    rx_head_ = next;
  } else {
    ++overflow_count_;
  }
  if (HAL_UART_Receive_IT(handle_, &rx_byte_, 1U) != HAL_OK) rx_restart_required_ = true;
}

void HalUartPort::irqTxComplete() {
  if (!tx_busy_) return;
  tx_tail_ = static_cast<uint16_t>((tx_tail_ + tx_pending_) % kTxSize);
  tx_pending_ = 0U;
  tx_busy_ = false;
}

void HalUartPort::irqError() {
  ++error_count_;
  __HAL_UART_CLEAR_OREFLAG(handle_);
  __HAL_UART_CLEAR_NEFLAG(handle_);
  __HAL_UART_CLEAR_FEFLAG(handle_);
  if (HAL_UART_Receive_IT(handle_, &rx_byte_, 1U) != HAL_OK) rx_restart_required_ = true;
}

// HAL_Init() enables the Cortex-M SysTick timebase. The STM32Cube startup file
// weak-aliases SysTick_Handler to Default_Handler, so a native application must
// provide this ISR explicitly. Without it the first 1 ms tick traps the MCU in
// the default infinite loop before USB/HMI/sensor startup can complete.
extern "C" void SysTick_Handler() {
  HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
}

extern "C" void USART1_IRQHandler() { HAL_UART_IRQHandler(&huart1); }
extern "C" void USART2_IRQHandler() { HAL_UART_IRQHandler(&huart2); }
extern "C" void TIM1_TRG_COM_TIM11_IRQHandler() { HAL_TIM_IRQHandler(&htim11); }

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) gVescUart.irqRxComplete();
  else if (huart == &huart2) gGnssUart.irqRxComplete();
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) gVescUart.irqTxComplete();
  else if (huart == &huart2) gGnssUart.irqTxComplete();
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart == &huart1) gVescUart.irqError();
  else if (huart == &huart2) gGnssUart.irqError();
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim == &htim11 && g_watchdog_callback != nullptr) g_watchdog_callback();
}

extern "C" void Error_Handler() { FatalError(); }
