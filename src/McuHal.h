#pragma once

#if !defined(BOARD_F103_FAMILY) || !defined(BOARD_F103C8)
#error "F4gateway now targets STM32F103C8T6 only"
#endif

#include "stm32f1xx_hal.h"

#ifndef GPIO_SPEED_FREQ_VERY_HIGH
#define GPIO_SPEED_FREQ_VERY_HIGH GPIO_SPEED_FREQ_HIGH
#endif
