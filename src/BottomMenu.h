// ============================================================================
// BottomMenu.h — three direct navigation tiles (Camera / GPS / Actuator)
// ============================================================================
#pragma once

#include "HmiDisplay.h"
#include "Config.h"
#include "Theme.h"
#include "Icons.h"

extern HmiDisplay tft;

inline int navTileX(uint8_t i) {
  return NAV_X0 + i * (NAV_W + NAV_GAP);
}

inline void drawNavTile(uint8_t i, PageId page, PageId activePage) {
  const int x = navTileX(i);
  const bool active = (page == activePage);
  const uint16_t iconColor = active ? C_ACCENT : C_TEXT;
  const uint16_t labelColor = active ? C_ACCENT : C_TEXT;

  tft.fillRoundRect(x + 1, NAV_Y + 2, NAV_W, NAV_H, 6, C_SHADOW);
  tft.fillRoundRect(x, NAV_Y, NAV_W, NAV_H, 6, C_PANEL);
  tft.drawRoundRect(x, NAV_Y, NAV_W, NAV_H, 6, C_BORDER);

  if (active) {
    tft.fillRoundRect(x + 7, NAV_Y, NAV_W - 14, 3, 1, C_ACCENT);
  }

  const int cx = x + NAV_W / 2;
  const int iconY = NAV_Y + 16;
  if (page == PAGE_CAMERA) iconCamera(cx, iconY, iconColor, C_PANEL);
  else if (page == PAGE_GPS) iconGps(cx, iconY - 1, iconColor, C_PANEL);
  else if (page == PAGE_ACTUATOR) iconSteering(cx, iconY, iconColor, C_PANEL);

  const char* label = page == PAGE_CAMERA ? "CAMERA" : (page == PAGE_GPS ? "GPS" : "ACTUATOR");
  drawCompactText(label, cx, NAV_Y + 32, labelColor, C_PANEL, TC_DATUM);
}

inline void drawBottomMenu(PageId activePage) {
  tft.fillRect(0, NAV_Y - 3, W, H - (NAV_Y - 3), C_BG);
  drawNavTile(0, PAGE_CAMERA, activePage);
  drawNavTile(1, PAGE_GPS, activePage);
  drawNavTile(2, PAGE_ACTUATOR, activePage);
}
