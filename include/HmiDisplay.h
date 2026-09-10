#pragma once

#include "stm32f4xx_hal.h"
#include <cstddef>
#include <cstdint>

struct GFXglyph {
  uint32_t bitmapOffset;
  uint8_t width;
  uint8_t height;
  uint8_t xAdvance;
  int8_t xOffset;
  int8_t yOffset;
};

struct GFXfont {
  uint8_t *bitmap;
  GFXglyph *glyph;
  uint16_t first;
  uint16_t last;
  uint8_t yAdvance;
};

extern const GFXfont FreeSans9pt7b;
extern const GFXfont FreeSansBold9pt7b;
extern const GFXfont FreeSansBold12pt7b;
extern const GFXfont FreeSansBold24pt7b;

enum TextDatum : uint8_t {
  TL_DATUM = 0U, TC_DATUM, TR_DATUM,
  ML_DATUM, MC_DATUM, MR_DATUM,
  BL_DATUM, BC_DATUM, BR_DATUM
};

class HmiDisplay {
 public:
  void init();
  uint8_t readRegister8(uint8_t command, uint8_t index = 0U);
  uint32_t readId();
  void setRotation(uint8_t rotation);
  void setSwapBytes(bool swap) { swap_bytes_ = swap; }
  void fillScreen(uint16_t color);
  void fillRect(int32_t x, int32_t y, int32_t w, int32_t h, uint16_t color);
  void drawFastHLine(int32_t x, int32_t y, int32_t w, uint16_t color);
  void drawFastVLine(int32_t x, int32_t y, int32_t h, uint16_t color);
  void drawLine(int32_t x0, int32_t y0, int32_t x1, int32_t y1, uint16_t color);
  void drawCircle(int32_t x0, int32_t y0, int32_t r, uint16_t color);
  void fillCircle(int32_t x0, int32_t y0, int32_t r, uint16_t color);
  void drawRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint16_t color);
  void fillRoundRect(int32_t x, int32_t y, int32_t w, int32_t h, int32_t r, uint16_t color);
  void fillTriangle(int32_t x0, int32_t y0, int32_t x1, int32_t y1,
                    int32_t x2, int32_t y2, uint16_t color);
  void pushImage(int32_t x, int32_t y, int32_t w, int32_t h, const uint16_t *pixels);

  void setFreeFont(const GFXfont *font) { font_ = font; }
  void setTextFont(uint8_t font) { builtin_font_ = font; }
  void setTextSize(uint8_t size) { text_size_ = size == 0U ? 1U : size; }
  void setTextDatum(uint8_t datum) { datum_ = datum; }
  void setTextColor(uint16_t fg, uint16_t bg) { text_fg_ = fg; text_bg_ = bg; }
  void setTextPadding(uint16_t padding) { padding_ = padding; }
  int16_t drawString(const char *text, int32_t x, int32_t y);
  int16_t textWidth(const char *text) const;

  void setTouch(const uint16_t *parameters);
  bool getTouch(uint16_t *x, uint16_t *y, uint16_t threshold = 600U);

  void beginFrame();
  void endFrame();
  void setDisplayReady(bool ready) { display_ready_ = ready; if (ready) display_faulted_ = false; }
  bool displayReady() const { return display_ready_; }
  bool displayFaulted() const { return display_faulted_; }
  uint32_t spiTransactions() const { return spi_transactions_; }
  uint32_t spiBytesTx() const { return spi_bytes_tx_; }
  uint32_t spiTimeoutCount() const { return spi_timeout_count_; }
  uint32_t spiHalErrorCount() const { return spi_hal_error_count_; }
  uint32_t spiRecoveryCount() const { return spi_recovery_count_; }
  uint32_t spiBusConflictCount() const { return spi_bus_conflict_count_; }
  uint32_t touchReadCount() const { return touch_read_count_; }
  uint32_t touchRejectFastCount() const { return touch_reject_fast_count_; }
  uint32_t lastFrameBytes() const { return last_frame_bytes_; }
  uint32_t maxFrameBytes() const { return max_frame_bytes_; }

 private:
  enum class SpiOwner : uint8_t { IDLE = 0U, TFT_WRITE, TFT_READ, TOUCH };
  static constexpr uint32_t kSpiWaitTimeoutUs = 2500U;
  static constexpr uint32_t kSpiHalTimeoutMs = 8U;
  static constexpr uint32_t kTftPrescaler = SPI_BAUDRATEPRESCALER_16;
  static constexpr uint32_t kTouchPrescaler = SPI_BAUDRATEPRESCALER_64;
  static constexpr uint8_t kTextTileRows = 8U;

  bool waitSpiIdle(uint32_t timeoutUs = kSpiWaitTimeoutUs);
  void drainSpiRx();
  bool recoverSpi();
  bool beginTransaction(SpiOwner owner, uint32_t prescaler);
  bool endTransaction();
  bool tx(const uint8_t *bytes, uint16_t length);
  bool txrx(const uint8_t *txBytes, uint8_t *rxBytes, uint16_t length);
  bool writeRegister(uint8_t command, const uint8_t *bytes = nullptr, uint16_t length = 0U);
  bool commandTx(uint8_t value);
  bool dataTx(const uint8_t *bytes, uint16_t length);
  bool setWindowTx(int32_t x, int32_t y, int32_t w, int32_t h);
  bool writeColorTx(uint16_t color, uint32_t count);
  bool pushImageTx(const uint16_t *pixels, uint32_t count);
  uint8_t transferTx(uint8_t value);
  uint16_t transfer16Tx(uint16_t value);
  uint16_t readTouchZTx();
  void readTouchRawTx(uint16_t *x, uint16_t *y);
  void textBounds(const char *text, int32_t &min_x, int32_t &min_y,
                  int32_t &max_x, int32_t &max_y, int32_t &advance) const;

  int32_t width_{320};
  int32_t height_{240};
  uint8_t rotation_{1U};
  bool swap_bytes_{false};
  const GFXfont *font_{nullptr};
  uint8_t builtin_font_{1U};
  uint8_t text_size_{1U};
  uint8_t datum_{TL_DATUM};
  uint16_t text_fg_{0xFFFFU};
  uint16_t text_bg_{0x0000U};
  uint16_t padding_{0U};
  uint16_t touch_x0_{300U};
  uint16_t touch_x1_{3600U};
  uint16_t touch_y0_{300U};
  uint16_t touch_y1_{3600U};
  bool touch_rotate_{true};
  bool touch_invert_x_{false};
  bool touch_invert_y_{false};
  uint32_t press_time_ms_{0U};

  SpiOwner spi_owner_{SpiOwner::IDLE};
  bool display_ready_{false};
  bool display_faulted_{false};
  uint32_t spi_transactions_{0U};
  uint32_t spi_bytes_tx_{0U};
  uint32_t spi_timeout_count_{0U};
  uint32_t spi_hal_error_count_{0U};
  uint32_t spi_recovery_count_{0U};
  uint32_t spi_bus_conflict_count_{0U};
  uint32_t touch_read_count_{0U};
  uint32_t touch_reject_fast_count_{0U};
  bool frame_active_{false};
  uint32_t frame_start_bytes_{0U};
  uint32_t last_frame_bytes_{0U};
  uint32_t max_frame_bytes_{0U};
  uint16_t text_tile_[320U * kTextTileRows]{};
};
