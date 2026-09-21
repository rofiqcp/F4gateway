#pragma once

#if defined(BOARD_F103C8)
#include "stm32f1xx_hal.h"
#ifndef GPIO_SPEED_FREQ_VERY_HIGH
#define GPIO_SPEED_FREQ_VERY_HIGH GPIO_SPEED_FREQ_HIGH
#endif
#else
#include "stm32f4xx_hal.h"
#endif
