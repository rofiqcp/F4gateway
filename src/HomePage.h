// ============================================================================
// HomePage.h — ADV overview matching the approved 3D-printer-style visual
// ============================================================================
#pragma once

#include "HmiDisplay.h"
#include "Config.h"
#include "Telemetry.h"
#include "Theme.h"
#include "TopBar.h"
#include "BottomMenu.h"
#include "Icons.h"

extern HmiDisplay tft;

inline const char* homeNavigationLine(const VehicleTelemetry& d) {
  static char line[38];
  const char* target = navigationHasTarget(d) ? d.activeTarget : "";
  switch (d.navigationStatus) {
    case NAV_SELECTED: snprintf(line, sizeof(line), "Target: %.20s", target); break;
    case NAV_QUEUED: snprintf(line, sizeof(line), "Menunggu: %.19s", target); break;
    case NAV_NAVIGATING: snprintf(line, sizeof(line), "Menuju: %.20s", target); break;
    case NAV_ARRIVED: snprintf(line, sizeof(line), "Tiba di: %.20s", target); break;
    case NAV_STOPPED: snprintf(line, sizeof(line), "Navigasi dihentikan"); break;
    case NAV_FAILED: snprintf(line, sizeof(line), "Navigasi gagal"); break;
    case NAV_IDLE:
    default: snprintf(line, sizeof(line), "Belum ada target"); break;
  }
  return line;
}

inline void drawHomeContent(const VehicleTelemetry& d) {
  char buf[32];

  // Large speed/vehicle card.
  drawCard(HOME_MAIN_X, HOME_MAIN_Y, HOME_MAIN_W, HOME_MAIN_H, C_CARD, C_CARD_LINE);

  snprintf(buf, sizeof(buf), "%.1f", d.speedKmh);
  drawHeroText(buf, HOME_MAIN_X + 14, HOME_MAIN_Y + 10, C_INK, C_CARD);
  drawSmallText("km/h", HOME_MAIN_X + 18, HOME_MAIN_Y + 64, C_DISABLED, C_CARD);

  const uint16_t statusColor = systemStatusColor(d.systemStatus);
  drawUiText(systemStatusText(d.systemStatus), HOME_MAIN_X + 16, HOME_MAIN_Y + 98,
             statusColor, C_CARD);

  snprintf(buf, sizeof(buf), "Heading %.0f", d.headingDeg);
  drawSmallTextPadded(buf, HOME_MAIN_X + 16, HOME_MAIN_Y + 116, C_INK, C_CARD, 116);
  drawDegreeMark(HOME_MAIN_X + 136, HOME_MAIN_Y + 117, C_INK, C_CARD);

  drawMicroText(homeNavigationLine(d), HOME_MAIN_X + 16, HOME_MAIN_Y + 136,
                d.navigationStatus == NAV_ARRIVED ? C_READY :
                (d.navigationStatus == NAV_FAILED ? C_FAULT : C_INK), C_CARD);

  // Richer delivery-vehicle illustration taken from the approved visual.
  drawVehicleIllustration(HOME_MAIN_X + 150, HOME_MAIN_Y + 16);

  // MODE card.
  drawCard(HOME_SIDE_X, HOME_MODE_Y, HOME_SIDE_W, HOME_SIDE_H, C_CARD, C_CARD_LINE);
  drawMicroText("MODE", HOME_SIDE_X + HOME_SIDE_W / 2, HOME_MODE_Y + 9,
                C_DISABLED, C_CARD, TC_DATUM);
  drawUiText(modeText(d.mode), HOME_SIDE_X + HOME_SIDE_W / 2, HOME_MODE_Y + 23,
             C_INK, C_CARD, TC_DATUM);
  iconAutoArrow(HOME_SIDE_X + HOME_SIDE_W / 2, HOME_MODE_Y + 53,
                d.mode == MODE_AUTO ? C_ACCENT : C_DISABLED, C_CARD);

  // STATE card.
  drawCard(HOME_SIDE_X, HOME_STATE_Y, HOME_SIDE_W, HOME_SIDE_H, C_CARD, C_CARD_LINE);
  drawMicroText("STATE", HOME_SIDE_X + HOME_SIDE_W / 2, HOME_STATE_Y + 9,
                C_DISABLED, C_CARD, TC_DATUM);
  const char* state = vehicleStateText(d.state);
  drawCompactText(state, HOME_SIDE_X + HOME_SIDE_W / 2, HOME_STATE_Y + 27,
                  C_INK, C_CARD, TC_DATUM);

  if (d.state == STATE_RUNNING) {
    iconAutoArrow(HOME_SIDE_X + HOME_SIDE_W / 2, HOME_STATE_Y + 54, C_READY, C_CARD);
  } else {
    iconStop(HOME_SIDE_X + HOME_SIDE_W / 2, HOME_STATE_Y + 54,
             d.state == STATE_FAULT ? C_FAULT : C_FAULT);
  }
}

inline void drawHomePage(const VehicleTelemetry& d) {
  tft.fillScreen(C_BG);
  drawTopBar("ADV", d, true, false);
  drawHomeContent(d);
  drawBottomMenu(PAGE_HOME); // no bottom tile highlighted while on HOME
}

inline void updateHomePage(const VehicleTelemetry& d) {
  // True partial redraw: never repaint the large HOME cards during telemetry.
  updateTopHealthOnly(d, false);

  static char lastSpeed[12] = "";
  static char lastHeading[20] = "";
  static char lastNav[38] = "";
  static SystemStatus lastSystem = SYS_OFF;
  static VehicleMode lastMode = MODE_AUTO;
  static VehicleState lastState = STATE_STANDBY;
  static bool cacheInit = false;
  char buf[32];

  snprintf(buf, sizeof(buf), "%.1f", d.speedKmh);
  if (!cacheInit || strcmp(buf, lastSpeed) != 0) {
    snprintf(lastSpeed, sizeof(lastSpeed), "%s", buf);
    drawHeroTextPadded(buf, HOME_MAIN_X + 14, HOME_MAIN_Y + 10, C_INK, C_CARD, 120);
  }

  if (!cacheInit || d.systemStatus != lastSystem) {
    lastSystem = d.systemStatus;
    drawUiTextPadded(systemStatusText(d.systemStatus), HOME_MAIN_X + 16, HOME_MAIN_Y + 98,
                     systemStatusColor(d.systemStatus), C_CARD, 138);
  }

  snprintf(buf, sizeof(buf), "Heading %.0f", d.headingDeg);
  if (!cacheInit || strcmp(buf, lastHeading) != 0) {
    snprintf(lastHeading, sizeof(lastHeading), "%s", buf);
    drawSmallTextPadded(buf, HOME_MAIN_X + 16, HOME_MAIN_Y + 116, C_INK, C_CARD, 116);
    drawDegreeMark(HOME_MAIN_X + 136, HOME_MAIN_Y + 117, C_INK, C_CARD);
  }

  const char* nav = homeNavigationLine(d);
  if (!cacheInit || strcmp(nav, lastNav) != 0) {
    snprintf(lastNav, sizeof(lastNav), "%s", nav);
    drawMicroTextPadded(nav, HOME_MAIN_X + 16, HOME_MAIN_Y + 136,
                        d.navigationStatus == NAV_ARRIVED ? C_READY :
                        (d.navigationStatus == NAV_FAILED ? C_FAULT : C_INK), C_CARD, HOME_MAIN_W - 28);
  }

  if (!cacheInit || d.mode != lastMode) {
    lastMode = d.mode;
    tft.fillRect(HOME_SIDE_X + 3, HOME_MODE_Y + 18, HOME_SIDE_W - 6, 49, C_CARD);
    drawUiText(modeText(d.mode), HOME_SIDE_X + HOME_SIDE_W / 2, HOME_MODE_Y + 23,
               C_INK, C_CARD, TC_DATUM);
    iconAutoArrow(HOME_SIDE_X + HOME_SIDE_W / 2, HOME_MODE_Y + 53,
                  d.mode == MODE_AUTO ? C_ACCENT : C_DISABLED, C_CARD);
  }

  if (!cacheInit || d.state != lastState) {
    lastState = d.state;
    tft.fillRect(HOME_SIDE_X + 3, HOME_STATE_Y + 20, HOME_SIDE_W - 6, 47, C_CARD);
    drawCompactText(vehicleStateText(d.state), HOME_SIDE_X + HOME_SIDE_W / 2, HOME_STATE_Y + 27,
                    C_INK, C_CARD, TC_DATUM);
    if (d.state == STATE_RUNNING)
      iconAutoArrow(HOME_SIDE_X + HOME_SIDE_W / 2, HOME_STATE_Y + 54, C_READY, C_CARD);
    else
      iconStop(HOME_SIDE_X + HOME_SIDE_W / 2, HOME_STATE_Y + 54, C_FAULT);
  }
  cacheInit = true;
}
