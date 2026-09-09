// ============================================================================
// Theme.h — typography and drawing helpers matching the approved ADV visual
// ============================================================================
#pragma once

#include "HmiDisplay.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
// IMPORTANT: Do not include individual GFXFF font headers here.
// With -DLOAD_GFXFF, TFT_eSPI.h -> gfxfont.h already includes the full
// FreeFont set (including FreeSans/FreeSansBold). Including them again
// causes redefinition errors on PlatformIO/TFT_eSPI 2.5.x.
#include "Config.h"

extern HmiDisplay tft;

inline void useFontMicro() {
  tft.setFreeFont(nullptr);
  tft.setTextFont(1);
  tft.setTextSize(1);
}


inline void useFontCompact() {
  tft.setFreeFont(nullptr);
  tft.setTextFont(2);
  tft.setTextSize(1);
}

inline void useFontSmall() {
  tft.setTextFont(1);
  tft.setFreeFont(&FreeSans9pt7b);
  tft.setTextSize(1);
}

inline void useFontUi() {
  tft.setTextFont(1);
  tft.setFreeFont(&FreeSansBold9pt7b);
  tft.setTextSize(1);
}

inline void useFontValue() {
  tft.setTextFont(1);
  tft.setFreeFont(&FreeSansBold12pt7b);
  tft.setTextSize(1);
}

inline void useFontHero() {
  tft.setTextFont(1);
  tft.setFreeFont(&FreeSansBold24pt7b);
  tft.setTextSize(1);
}

inline void setTextStyle(uint16_t fg, uint16_t bg, uint8_t datum) {
  tft.setTextDatum(datum);
  tft.setTextColor(fg, bg);
}

inline void drawCard(int x, int y, int w, int h, uint16_t fill, uint16_t border = C_BORDER,
                     bool shadow = true) {
  if (shadow) tft.fillRoundRect(x + 1, y + 2, w, h, CARD_RADIUS, C_SHADOW);
  tft.fillRoundRect(x, y, w, h, CARD_RADIUS, fill);
  tft.drawRoundRect(x, y, w, h, CARD_RADIUS, border);
}

inline void drawStatusDot(int x, int y, uint16_t color, int r = 3) {
  tft.fillCircle(x, y, r + 1, C_SHADOW);
  tft.fillCircle(x, y, r, color);
}

inline void drawMicroText(const char* text, int x, int y, uint16_t color, uint16_t bg,
                          uint8_t datum = TL_DATUM) {
  useFontMicro();
  setTextStyle(color, bg, datum);
  tft.drawString(text, x, y);
}


inline void drawCompactText(const char* text, int x, int y, uint16_t color, uint16_t bg,
                            uint8_t datum = TL_DATUM) {
  useFontCompact();
  setTextStyle(color, bg, datum);
  tft.drawString(text, x, y);
}

inline void drawSmallText(const char* text, int x, int y, uint16_t color, uint16_t bg,
                          uint8_t datum = TL_DATUM) {
  useFontSmall();
  setTextStyle(color, bg, datum);
  tft.drawString(text, x, y);
}

inline void drawUiText(const char* text, int x, int y, uint16_t color, uint16_t bg,
                       uint8_t datum = TL_DATUM) {
  useFontUi();
  setTextStyle(color, bg, datum);
  tft.drawString(text, x, y);
}

inline void drawValueText(const char* text, int x, int y, uint16_t color, uint16_t bg,
                          uint8_t datum = TL_DATUM) {
  useFontValue();
  setTextStyle(color, bg, datum);
  tft.drawString(text, x, y);
}

inline void drawHeroText(const char* text, int x, int y, uint16_t color, uint16_t bg,
                         uint8_t datum = TL_DATUM) {
  useFontHero();
  setTextStyle(color, bg, datum);
  tft.drawString(text, x, y);
}

// Dynamic-value helpers: TFT_eSPI clears only the text padding as part of the
// same drawString operation. This removes the visible blank frame produced by
// a separate fillRect() followed by text rendering.
inline void drawMicroTextPadded(const char* text, int x, int y, uint16_t color, uint16_t bg,
                                uint16_t padding, uint8_t datum = TL_DATUM) {
  useFontMicro(); setTextStyle(color, bg, datum); tft.setTextPadding(padding);
  tft.drawString(text, x, y); tft.setTextPadding(0);
}
inline void drawCompactTextPadded(const char* text, int x, int y, uint16_t color, uint16_t bg,
                                  uint16_t padding, uint8_t datum = TL_DATUM) {
  useFontCompact(); setTextStyle(color, bg, datum); tft.setTextPadding(padding);
  tft.drawString(text, x, y); tft.setTextPadding(0);
}
inline void drawSmallTextPadded(const char* text, int x, int y, uint16_t color, uint16_t bg,
                                uint16_t padding, uint8_t datum = TL_DATUM) {
  useFontSmall(); setTextStyle(color, bg, datum); tft.setTextPadding(padding);
  tft.drawString(text, x, y); tft.setTextPadding(0);
}
inline void drawUiTextPadded(const char* text, int x, int y, uint16_t color, uint16_t bg,
                             uint16_t padding, uint8_t datum = TL_DATUM) {
  useFontUi(); setTextStyle(color, bg, datum); tft.setTextPadding(padding);
  tft.drawString(text, x, y); tft.setTextPadding(0);
}
inline void drawValueTextPadded(const char* text, int x, int y, uint16_t color, uint16_t bg,
                                uint16_t padding, uint8_t datum = TL_DATUM) {
  useFontValue(); setTextStyle(color, bg, datum); tft.setTextPadding(padding);
  tft.drawString(text, x, y); tft.setTextPadding(0);
}
inline void drawHeroTextPadded(const char* text, int x, int y, uint16_t color, uint16_t bg,
                               uint16_t padding, uint8_t datum = TL_DATUM) {
  useFontHero(); setTextStyle(color, bg, datum); tft.setTextPadding(padding);
  tft.drawString(text, x, y); tft.setTextPadding(0);
}

inline void drawThinDivider(int x, int y, int w, uint16_t color = C_CARD_LINE) {
  tft.drawFastHLine(x, y, w, color);
}

inline void drawDegreeMark(int x, int y, uint16_t color, uint16_t bg) {
  (void)bg;
  tft.drawCircle(x, y, 3, color);
}

inline void drawDegreeValue(const char* value, int x, int y, uint16_t color, uint16_t bg,
                            bool hero = false, uint8_t datum = TL_DATUM) {
  if (hero) drawHeroText(value, x, y, color, bg, datum);
  else drawValueText(value, x, y, color, bg, datum);
  int w = tft.textWidth(value);
  if (datum == TR_DATUM || datum == MR_DATUM || datum == BR_DATUM) {
    drawDegreeMark(x + 4, y + 2, color, bg);
  } else {
    drawDegreeMark(x + w + 4, y + 2, color, bg);
  }
}
