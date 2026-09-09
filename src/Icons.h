// ============================================================================
// Icons.h — compact icons + approved concept illustrations
// ============================================================================
#pragma once

#include "HmiDisplay.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "Config.h"
#include "VisualAssets.h"

extern HmiDisplay tft;

inline void iconHome(int cx, int cy, uint16_t c) {
  tft.fillTriangle(cx - 8, cy - 1, cx, cy - 8, cx + 8, cy - 1, c);
  tft.fillRoundRect(cx - 6, cy - 1, 12, 9, 1, c);
  tft.fillRect(cx - 2, cy + 3, 4, 5, C_BG);
}

inline void iconCamera(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.fillRoundRect(cx - 12, cy - 7, 24, 16, 3, c);
  tft.fillRoundRect(cx - 6, cy - 11, 9, 5, 2, c);
  tft.fillCircle(cx, cy + 1, 7, bg);
  tft.drawCircle(cx, cy + 1, 7, c);
  tft.drawCircle(cx, cy + 1, 5, c);
  tft.fillCircle(cx, cy + 1, 2, c);
  tft.fillCircle(cx + 8, cy - 3, 1, bg);
}

inline void iconGps(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.fillCircle(cx, cy - 3, 8, c);
  tft.fillTriangle(cx - 7, cy, cx + 7, cy, cx, cy + 13, c);
  tft.fillCircle(cx, cy - 3, 3, bg);
}

inline void iconSteering(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.drawCircle(cx, cy, 10, c);
  tft.drawCircle(cx, cy, 8, c);
  tft.fillCircle(cx, cy, 3, c);
  tft.drawLine(cx - 8, cy - 3, cx - 2, cy, c);
  tft.drawLine(cx + 8, cy - 3, cx + 2, cy, c);
  tft.drawLine(cx, cy + 3, cx, cy + 9, c);
  tft.fillCircle(cx, cy, 1, bg);
}

inline void iconAutoArrow(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.fillTriangle(cx - 10, cy + 7, cx + 10, cy - 8, cx + 4, cy + 11, c);
  tft.fillTriangle(cx - 1, cy + 2, cx + 4, cy + 11, cx + 1, cy + 2, bg);
}

inline void iconStop(int cx, int cy, uint16_t c) {
  tft.fillRoundRect(cx - 6, cy - 6, 12, 12, 2, c);
}

inline void iconArrowUp(int cx, int cy, uint16_t c) {
  tft.fillTriangle(cx, cy - 9, cx - 8, cy - 1, cx + 8, cy - 1, c);
  tft.fillRoundRect(cx - 3, cy - 1, 6, 10, 2, c);
}

inline void iconArrowDown(int cx, int cy, uint16_t c) {
  tft.fillRoundRect(cx - 3, cy - 9, 6, 10, 2, c);
  tft.fillTriangle(cx - 8, cy + 1, cx + 8, cy + 1, cx, cy + 9, c);
}

inline void iconArrowLeft(int cx, int cy, uint16_t c) {
  tft.fillTriangle(cx - 9, cy, cx, cy - 8, cx, cy + 8, c);
  tft.fillRoundRect(cx - 1, cy - 3, 10, 6, 2, c);
}

inline void iconArrowRight(int cx, int cy, uint16_t c) {
  tft.fillRoundRect(cx - 9, cy - 3, 10, 6, 2, c);
  tft.fillTriangle(cx, cy - 8, cx + 9, cy, cx, cy + 8, c);
}


inline void iconBack(int cx, int cy, uint16_t c) {
  tft.fillTriangle(cx - 9, cy, cx + 1, cy - 9, cx + 1, cy + 9, c);
  tft.fillRoundRect(cx - 1, cy - 3, 11, 6, 2, c);
}

inline void iconGrid(int cx, int cy, uint16_t c) {
  for (int row = 0; row < 2; ++row) {
    for (int col = 0; col < 2; ++col) {
      tft.fillRoundRect(cx - 11 + col * 13, cy - 11 + row * 13, 9, 9, 2, c);
    }
  }
}

inline void iconBolt(int cx, int cy, uint16_t c) {
  tft.fillTriangle(cx + 2, cy - 13, cx - 7, cy + 1, cx + 1, cy + 1, c);
  tft.fillTriangle(cx - 1, cy + 13, cx + 8, cy - 2, cx, cy - 2, c);
}

inline void iconLink(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.drawRoundRect(cx - 14, cy - 6, 17, 12, 5, c);
  tft.drawRoundRect(cx - 2, cy - 6, 17, 12, 5, c);
  tft.fillRect(cx - 5, cy - 2, 10, 5, bg);
  tft.drawFastHLine(cx - 5, cy, 10, c);
}

inline void iconGauge(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.drawCircle(cx, cy + 2, 12, c);
  tft.fillRect(cx - 13, cy + 3, 27, 12, bg);
  tft.drawLine(cx, cy + 2, cx + 7, cy - 5, c);
  tft.fillCircle(cx, cy + 2, 2, c);
}

inline void iconGear(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.drawCircle(cx, cy, 9, c);
  tft.drawCircle(cx, cy, 3, c);
  for (int i = -1; i <= 1; i += 2) {
    tft.fillRect(cx + i * 9 - 2, cy - 3, 5, 6, c);
    tft.fillRect(cx - 3, cy + i * 9 - 2, 6, 5, c);
  }
  tft.fillCircle(cx, cy, 2, bg);
}

inline void iconEye(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.drawLine(cx - 14, cy, cx - 7, cy - 7, c);
  tft.drawLine(cx - 7, cy - 7, cx + 7, cy - 7, c);
  tft.drawLine(cx + 7, cy - 7, cx + 14, cy, c);
  tft.drawLine(cx - 14, cy, cx - 7, cy + 7, c);
  tft.drawLine(cx - 7, cy + 7, cx + 7, cy + 7, c);
  tft.drawLine(cx + 7, cy + 7, cx + 14, cy, c);
  tft.fillCircle(cx, cy, 5, c);
  tft.fillCircle(cx, cy, 2, bg);
}

inline void iconCompass(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.drawCircle(cx, cy, 12, c);
  tft.fillTriangle(cx, cy - 10, cx - 4, cy + 3, cx + 4, cy + 3, c);
  tft.fillTriangle(cx, cy + 10, cx - 4, cy - 3, cx + 4, cy - 3, C_DISABLED);
  tft.fillCircle(cx, cy, 2, bg);
}

inline void iconShield(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.fillTriangle(cx - 10, cy - 9, cx + 10, cy - 9, cx, cy + 13, c);
  tft.fillTriangle(cx - 6, cy - 6, cx + 6, cy - 6, cx, cy + 8, bg);
}

inline void iconWrench(int cx, int cy, uint16_t c, uint16_t bg) {
  tft.fillCircle(cx - 6, cy - 6, 7, c);
  tft.fillCircle(cx - 6, cy - 6, 3, bg);
  tft.fillRect(cx - 3, cy - 3, 15, 6, c);
  tft.fillCircle(cx + 11, cy, 4, c);
  tft.fillCircle(cx + 11, cy, 1, bg);
}

inline void iconCheck(int cx, int cy, uint16_t c) {
  tft.drawLine(cx - 9, cy, cx - 3, cy + 7, c);
  tft.drawLine(cx - 3, cy + 7, cx + 10, cy - 8, c);
  tft.drawLine(cx - 8, cy, cx - 2, cy + 6, c);
}

inline void iconPlus(int cx, int cy, uint16_t c) {
  tft.fillRoundRect(cx - 10, cy - 2, 21, 5, 2, c);
  tft.fillRoundRect(cx - 2, cy - 10, 5, 21, 2, c);
}

inline void iconMinus(int cx, int cy, uint16_t c) {
  tft.fillRoundRect(cx - 10, cy - 2, 21, 5, 2, c);
}

inline void drawVehicleIllustration(int x, int y) {
  tft.pushImage(x, y, VEHICLE_ASSET_W, VEHICLE_ASSET_H, VEHICLE_ASSET);
}

inline void drawCameraIllustration(int x, int y) {
  tft.pushImage(x, y, CAMERA_ASSET_W, CAMERA_ASSET_H, CAMERA_ASSET);
}

inline void drawGpsIllustration(int x, int y) {
  tft.pushImage(x, y, GPS_ASSET_W, GPS_ASSET_H, GPS_ASSET);
}
