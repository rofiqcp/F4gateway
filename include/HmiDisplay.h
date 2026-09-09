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

 private:
  void command(uint8_t value);
  void data8(uint8_t value);
  void data(const uint8_t *bytes, uint16_t length);
  void setWindow(int32_t x, int32_t y, int32_t w, int32_t h);
  void drawPixel(int32_t x, int32_t y, uint16_t color);
  void writeColor(uint16_t color, uint32_t count);
  void setSpiPrescaler(uint32_t prescaler);
  uint8_t transfer(uint8_t value);
  uint16_t transfer16(uint16_t value);
  uint16_t readTouchZ();
  void readTouchRaw(uint16_t *x, uint16_t *y);
  bool validTouch(uint16_t *x, uint16_t *y, uint16_t threshold);
  void textBounds(const char *text, int32_t &min_x, int32_t &min_y,
                  int32_t &max_x, int32_t &max_y, int32_t &advance) const;
  void drawBuiltinChar(char c, int32_t x, int32_t y, uint8_t scale);
  void drawFont2Char(char c, int32_t x, int32_t y, uint8_t scale);
  void drawGfxGlyph(uint8_t c, int32_t baseline_x, int32_t baseline_y);

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
};
