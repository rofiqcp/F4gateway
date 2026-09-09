// ============================================================================
// SplashScreen.h — compact ADV startup screen
// ============================================================================
#pragma once

#include "HmiDisplay.h"
#include "Config.h"
#include "Theme.h"
#include "Icons.h"

extern HmiDisplay tft;

inline void drawSplashScreen() {
  tft.fillScreen(C_BG);

  drawVehicleIllustration(W / 2 - 39, 24);
  drawValueText("ADV", W / 2, 111, C_TEXT, C_BG, MC_DATUM);
  drawSmallText("AUTONOMOUS DELIVERY VEHICLE", W / 2, 139, C_TEXT_DIM, C_BG, MC_DATUM);

  tft.fillRoundRect(PB_X - 1, PB_Y - 1, PB_W + 2, PB_H + 2, PB_R + 1, C_PANEL_ALT);
  tft.drawRoundRect(PB_X, PB_Y, PB_W, PB_H, PB_R, C_BORDER);

  drawSmallText("Initializing system...", W / 2, 189, C_TEXT_DIM, C_BG, MC_DATUM);
  drawMicroText("STM32F411  |  ILI9341  |  320x240", W / 2, 222, C_DISABLED, C_BG, MC_DATUM);
}
