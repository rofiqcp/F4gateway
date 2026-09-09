// User_Setup.h for TFT_eSPI with ILI9341 on Black Pill F411CE (STM32F411CEU6)
// Configuration for ILI9341 240x320 display with user's specific wiring:
// TDO/MISO = PA6, TDIN/MOSI = PA7, TCLK/SCK = PA5
// CS LCD = PB0, CS TS = PA4 (XPT2046 touch)
// VCC = 5V (module has onboard regulator), GND = GND
// RESET = PB2, DC = PB1

#ifndef USER_SETUP_H
#define USER_SETUP_H

// ============================================================================================
// Driver for ILI9341
// ============================================================================================
#define ILI9341_DRIVER

// ============================================================================================
// STM32F411CE SPI1 pinout (PA5=SCK, PA6=MISO, PA7=MOSI)
// ============================================================================================
#define TFT_SPI_PORT SPI1

#define TFT_SPI_MODE 0  // SPI mode 0

#define TFT_SCLK   PA5
#define TFT_MISO   PA6
#define TFT_MOSI   PA7

// User's wiring:
// CS LCD  = PB0
// DC      = PB1
// RESET   = PB2
// (Touch CS = PA4, shared SPI)
#define TFT_CS     PB0   // Chip select control pin (LCD CS)
#define TFT_DC     PB1   // Data/Command control pin
#define TFT_RST    PB2   // Reset pin
#define TOUCH_CS   PA4   // XPT2046 chip select

// ============================================================================================
// ILI9341 Display settings (240x320)
// ============================================================================================
#define TFT_WIDTH  240
#define TFT_HEIGHT 320

// ============================================================================================
// SPI frequency settings
// ============================================================================================
#define SPI_FREQUENCY  10000000   // conservative shared-SPI display clock
#define SPI_READ_FREQUENCY  6000000   // conservative read frequency
#define SPI_TOUCH_FREQUENCY  2500000  // 2.5MHz for touch (if touch supported)

// ============================================================================================
// Fonts and font loading
// ============================================================================================
#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF
#define SMOOTH_FONT

// ============================================================================================
// SPI settings
// ============================================================================================
#define SUPPORT_TRANSACTIONS

#endif // USER_SETUP_H