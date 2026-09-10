// ============================================================================
// UiShell.h — Stage-1 HOME / MAIN MENU / page-of-three domain renderer.
// ============================================================================
#pragma once

#include "Config.h"
#include "Diagnostics.h"
#include "HmiDisplay.h"
#include "Icons.h"
#include "Telemetry.h"
#include "Theme.h"
#include "UiMenu.h"
#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cstring>

extern HmiDisplay tft;

enum UiDirtyBits : uint8_t {
  UI_DIRTY_NONE = 0U,
  UI_DIRTY_TOPBAR = 1U << 0,
  UI_DIRTY_CONTENT = 1U << 1,
  UI_DIRTY_FOOTER = 1U << 2,
  UI_DIRTY_ALL = UI_DIRTY_TOPBAR | UI_DIRTY_CONTENT | UI_DIRTY_FOOTER
};

inline bool gUiDynamicPass = false;

enum class UiFooterKind : uint8_t { NONE = 0U, CAROUSEL, EDIT };
struct UiTopBarCache {
  bool valid{false};
  UiMenuId menu{UiMenuId::SPLASH};
  bool eStop{false};
  bool escReady{false}, escFresh{false};
  bool perceptionReady{false}, perceptionFresh{false};
  bool navigationReady{false}, navigationFresh{false};
  bool systemReady{false};
};
struct UiFooterCache {
  UiFooterKind kind{UiFooterKind::NONE};
  bool valid{false};
  uint8_t selected{0U};
  uint8_t count{0U};
  bool configPending{false};
};
inline UiTopBarCache gUiTopBarCache{};
inline UiFooterCache gUiFooterCache{};

struct UiHomeCache {
  bool valid{false};
  char speed[20]{};
  char state[20]{};
  char power[20]{};
  char modeLink[48]{};
  uint16_t speedColor{0U};
  uint16_t stateColor{0U};
  uint16_t powerColor{0U};
  uint16_t railColors[3]{};
};
struct UiMenuCardCache {
  bool valid{false};
  UiMenuId id{UiMenuId::SPLASH};
  uint16_t stateColor{0U};
  char status[16]{};
};
struct UiEditorCache {
  bool valid{false};
  UiEditKey key{UiEditKey::NONE};
  char value[32]{};
  char status[64]{};
  uint16_t statusColor{0U};
};
inline UiHomeCache gUiHomeCache{};
inline UiMenuCardCache gUiMenuCardCache[3]{};
inline UiEditorCache gUiEditorCache{};

inline void invalidateUiChromeCaches() {
  gUiTopBarCache = UiTopBarCache{};
  gUiFooterCache = UiFooterCache{};
  gUiHomeCache = UiHomeCache{};
  std::fill_n(gUiMenuCardCache, 3U, UiMenuCardCache{});
  gUiEditorCache = UiEditorCache{};
}

struct UiMetricCacheEntry {
  bool valid{false};
  int16_t y{0};
  uint16_t color{0U};
  char value[48]{};
};
inline UiMetricCacheEntry gUiMetricCache[4]{};
inline void resetUiMetricCache() {
  std::fill_n(gUiMetricCache, 4U, UiMetricCacheEntry{});
}
inline UiMetricCacheEntry *metricCacheForY(int y) {
  static constexpr int kRows[4] = {48, 77, 106, 135};
  for (uint8_t i = 0U; i < 4U; ++i)
    if (kRows[i] == y)
      return &gUiMetricCache[i];
  return nullptr;
}

inline uint16_t domainHealthColor(bool ready, bool fresh = true) {
  if (!fresh)
    return C_DISABLED;
  return ready ? C_READY : C_FAULT;
}

inline void drawDomainDot(int x, const char *label, bool ready,
                          bool fresh = true) {
  drawStatusDot(x, 14, domainHealthColor(ready, fresh), 3);
  drawMicroText(label, x + 7, 10, fresh ? C_TEXT_DIM : C_DISABLED, C_BG);
}

inline bool uiIsDomainRoot(UiMenuId id) { return menuIsDomainRoot(id); }

inline void drawUiTopBar(const UiState &ui, const VehicleTelemetry &d) {
  const bool menuChanged =
      !gUiTopBarCache.valid || gUiTopBarCache.menu != ui.menu;
  const bool eStopChanged =
      !gUiTopBarCache.valid || gUiTopBarCache.eStop != d.eStop;
  if (!gUiTopBarCache.valid || menuChanged || eStopChanged)
    tft.fillRect(0, 0, 190, TOP_H, C_BG);

  if (menuChanged || !gUiTopBarCache.valid) {
    if (ui.menu == UiMenuId::HOME || ui.menu == UiMenuId::OVERVIEW) {
      iconGrid(14, 14, C_ACCENT);
    } else if (ui.menu == UiMenuId::MAIN_MENU) {
      iconHome(14, 14, C_TEXT);
    } else {
      iconBack(14, 14, C_TEXT);
    }
    tft.drawFastVLine(28, 6, 17, C_BORDER);
    char title[24];
    snprintf(title, sizeof(title), "%.14s", menuTitle(ui.menu));
    drawCompactTextPadded(title, 36, 7, C_TEXT, C_BG, 145U);
  }

  const bool navigationReady = d.nav2Ready && d.motionReady;
  const bool escChanged =
      !gUiTopBarCache.valid ||
      gUiTopBarCache.escReady != (d.escReady && d.vescConnected) ||
      gUiTopBarCache.escFresh != d.escFresh;
  const bool perceptionChanged =
      !gUiTopBarCache.valid ||
      gUiTopBarCache.perceptionReady != d.perceptionReady ||
      gUiTopBarCache.perceptionFresh != d.perceptionFresh;
  const bool navigationChanged =
      !gUiTopBarCache.valid ||
      gUiTopBarCache.navigationReady != navigationReady ||
      gUiTopBarCache.navigationFresh != d.navigationFresh;
  const bool systemChanged =
      !gUiTopBarCache.valid || gUiTopBarCache.systemReady != gDiagnostics.tftOk;
  if (escChanged)
    drawDomainDot(196, "E", d.escReady && d.vescConnected, d.escFresh);
  if (perceptionChanged)
    drawDomainDot(226, "P", d.perceptionReady, d.perceptionFresh);
  if (navigationChanged)
    drawDomainDot(256, "N", navigationReady, d.navigationFresh);
  if (systemChanged)
    drawDomainDot(286, "S", gDiagnostics.tftOk, true);

  if (d.eStop && (eStopChanged || menuChanged || !gUiTopBarCache.valid)) {
    tft.fillRoundRect(116, 5, 70, 20, 4, C_FAULT);
    drawMicroText("E-STOP", 151, 10, C_WHITE, C_FAULT, TC_DATUM);
  }

  gUiTopBarCache.valid = true;
  gUiTopBarCache.menu = ui.menu;
  gUiTopBarCache.eStop = d.eStop;
  gUiTopBarCache.escReady = d.escReady && d.vescConnected;
  gUiTopBarCache.escFresh = d.escFresh;
  gUiTopBarCache.perceptionReady = d.perceptionReady;
  gUiTopBarCache.perceptionFresh = d.perceptionFresh;
  gUiTopBarCache.navigationReady = navigationReady;
  gUiTopBarCache.navigationFresh = d.navigationFresh;
  gUiTopBarCache.systemReady = gDiagnostics.tftOk;
}

inline int softKeyX(uint8_t index) {
  return SOFTKEY_X0 + index * (SOFTKEY_W + SOFTKEY_GAP);
}

inline void drawEditSoftKey(uint8_t index, SoftKey key, const char *label,
                            bool active = true) {
  const int x = softKeyX(index);
  const uint16_t border =
      active ? (key == SoftKey::OK ? C_ACCENT : C_BORDER) : C_BORDER;
  const uint16_t fg =
      active ? (key == SoftKey::OK ? C_ACCENT : C_TEXT) : C_DISABLED;
  tft.fillRoundRect(x, SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H, 7, C_PANEL);
  tft.drawRoundRect(x, SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H, 7, border);
  const int cx = x + SOFTKEY_W / 2;
  if (key == SoftKey::LEFT)
    iconMinus(cx, SOFTKEY_Y + 17, fg);
  else if (key == SoftKey::RIGHT)
    iconPlus(cx, SOFTKEY_Y + 17, fg);
  else if (key == SoftKey::OK)
    iconCheck(cx, SOFTKEY_Y + 17, fg);
  drawCompactText(label, cx, SOFTKEY_Y + 31, fg, C_PANEL, TC_DATUM);
}

inline void drawEditFooter(const UiState &ui, const VehicleTelemetry &d) {
  if (ui.menu == UiMenuId::SYSTEM_TFT_TEST) {
    drawEditSoftKey(0, SoftKey::LEFT, "-", false);
    drawEditSoftKey(1, SoftKey::OK, "TEST", true);
    drawEditSoftKey(2, SoftKey::RIGHT, "-", false);
    return;
  }
  drawEditSoftKey(0, SoftKey::LEFT, "LEFT", !d.configPending);
  drawEditSoftKey(1, SoftKey::OK, d.configPending ? "WAIT" : "OK",
                  !d.configPending);
  drawEditSoftKey(2, SoftKey::RIGHT, "RIGHT", !d.configPending);
}

inline void drawCarouselFooter(const UiState &ui) {
  uint8_t total = 1U;
  uint8_t current = 0U;
  if (menuHasChildren(ui.menu) && ui.menu != UiMenuId::MAIN_MENU) {
    uint8_t count = 0U;
    (void)menuChildren(ui.menu, count);
    total = std::max<uint8_t>(1U, menuPageCount(count));
    current = std::min<uint8_t>(ui.pageIndex, static_cast<uint8_t>(total - 1U));
  } else {
    total = std::max<uint8_t>(1U, menuViewCount(ui.menu));
    current = std::min<uint8_t>(ui.detailViewIndex, static_cast<uint8_t>(total - 1U));
  }

  const bool redrawStatic =
      !gUiFooterCache.valid || gUiFooterCache.kind != UiFooterKind::CAROUSEL;
  if (redrawStatic) {
    tft.fillRoundRect(CAROUSEL_LEFT_X, CAROUSEL_NAV_Y, CAROUSEL_NAV_W,
                      CAROUSEL_NAV_H, 7, C_PANEL);
    tft.drawRoundRect(CAROUSEL_LEFT_X, CAROUSEL_NAV_Y, CAROUSEL_NAV_W,
                      CAROUSEL_NAV_H, 7, C_BORDER);
    iconArrowLeft(CAROUSEL_LEFT_X + CAROUSEL_NAV_W / 2, CAROUSEL_NAV_Y + 18,
                  C_TEXT);
    drawMicroText("LEFT", CAROUSEL_LEFT_X + CAROUSEL_NAV_W / 2,
                  CAROUSEL_NAV_Y + 33, C_TEXT_DIM, C_PANEL, TC_DATUM);

    tft.fillRoundRect(CAROUSEL_RIGHT_X, CAROUSEL_NAV_Y, CAROUSEL_NAV_W,
                      CAROUSEL_NAV_H, 7, C_PANEL);
    tft.drawRoundRect(CAROUSEL_RIGHT_X, CAROUSEL_NAV_Y, CAROUSEL_NAV_W,
                      CAROUSEL_NAV_H, 7, C_BORDER);
    iconArrowRight(CAROUSEL_RIGHT_X + CAROUSEL_NAV_W / 2, CAROUSEL_NAV_Y + 18,
                   C_TEXT);
    drawMicroText("RIGHT", CAROUSEL_RIGHT_X + CAROUSEL_NAV_W / 2,
                  CAROUSEL_NAV_Y + 33, C_TEXT_DIM, C_PANEL, TC_DATUM);
  }

  if (redrawStatic || gUiFooterCache.selected != current ||
      gUiFooterCache.count != total) {
    const bool missionAction = ui.menu == UiMenuId::NAV_MISSION;
    const char *contextAction = menuDetailActionLabel(ui.menu, current);
    const bool actionable = missionAction || contextAction != nullptr;
    tft.fillRoundRect(CAROUSEL_PAGE_X, CAROUSEL_NAV_Y, CAROUSEL_PAGE_W,
                      CAROUSEL_NAV_H, 7,
                      actionable ? C_PANEL : C_PANEL_ALT);
    tft.drawRoundRect(CAROUSEL_PAGE_X, CAROUSEL_NAV_Y, CAROUSEL_PAGE_W,
                      CAROUSEL_NAV_H, 7,
                      actionable ? C_ACCENT : C_BORDER);
    char pos[20];
    if (missionAction) {
      static const char *const actions[3] = {"GO", "SAVE", "STOP"};
      snprintf(pos, sizeof(pos), "%s", actions[std::min<uint8_t>(current, 2U)]);
    } else if (contextAction != nullptr) {
      snprintf(pos, sizeof(pos), "%s", contextAction);
    } else {
      snprintf(pos, sizeof(pos), "%u / %u",
               static_cast<unsigned>(current + 1U),
               static_cast<unsigned>(total));
    }
    drawUiText(pos, CAROUSEL_PAGE_X + CAROUSEL_PAGE_W / 2, CAROUSEL_NAV_Y + 14,
               missionAction && current == 2U ? C_FAULT : C_ACCENT,
               actionable ? C_PANEL : C_PANEL_ALT, TC_DATUM);
  }
  gUiFooterCache.valid = true;
  gUiFooterCache.kind = UiFooterKind::CAROUSEL;
  gUiFooterCache.selected = current;
  gUiFooterCache.count = total;
}

inline void drawContentCard() {
  if (gUiDynamicPass)
    return;
  drawCard(6, CONTENT_Y, 308, CONTENT_BOTTOM - CONTENT_Y + 1, C_CARD,
           C_CARD_LINE, false);
}

inline void drawMenuNodeIcon(UiMenuId id, int cx, int cy, uint16_t c,
                             uint16_t bg) {
  switch (id) {
  case UiMenuId::ESC_ROOT:
  case UiMenuId::ESC_STEERING:
  case UiMenuId::ESC_STEERING_LIVE:
  case UiMenuId::ESC_STEERING_TEST_ANGLE:
  case UiMenuId::ESC_STEERING_CAL:
    iconSteering(cx, cy, c, bg);
    break;
  case UiMenuId::ESC_OVERVIEW:
  case UiMenuId::ESC_DRIVE:
  case UiMenuId::ESC_DRIVE_LIVE:
  case UiMenuId::ESC_MANUAL_SPEED:
  case UiMenuId::ESC_PERFORMANCE:
    iconGauge(cx, cy, c, bg);
    break;
  case UiMenuId::ESC_MODE:
  case UiMenuId::ESC_DRIVE_SCALE:
  case UiMenuId::ESC_CALIBRATION:
  case UiMenuId::PERCEPTION_CALIBRATION:
    iconGear(cx, cy, c, bg);
    break;
  case UiMenuId::ESC_POWER:
    iconBolt(cx, cy, c);
    break;
  case UiMenuId::ESC_MOTOR_TELEMETRY:
  case UiMenuId::ESC_ENCODER:
    iconChip(cx, cy, c, bg);
    break;
  case UiMenuId::ESC_LINK:
    iconLink(cx, cy, c, bg);
    break;
  case UiMenuId::ESC_FAULT_SAFETY:
  case UiMenuId::NAV_SAFETY:
    iconShield(cx, cy, c, bg);
    break;
  case UiMenuId::ESC_MANUAL_TEST:
  case UiMenuId::PERCEPTION_TEST:
  case UiMenuId::NAV_TEST:
    iconWrench(cx, cy, c, bg);
    break;
  case UiMenuId::PERCEPTION_ROOT:
  case UiMenuId::PERCEPTION_CAMERA:
    iconCamera(cx, cy, c, bg);
    break;
  case UiMenuId::PERCEPTION_OVERVIEW:
  case UiMenuId::PERCEPTION_DETECTION:
  case UiMenuId::PERCEPTION_DETECTION_LIVE:
  case UiMenuId::PERCEPTION_OBSTACLE:
    iconEye(cx, cy, c, bg);
    break;
  case UiMenuId::PERCEPTION_INFERENCE:
  case UiMenuId::PERCEPTION_PERFORMANCE:
    iconGear(cx, cy, c, bg);
    break;
  case UiMenuId::PERCEPTION_LANE:
  case UiMenuId::PERCEPTION_DRIVABLE_AREA:
    iconAutoArrow(cx, cy, c, bg);
    break;
  case UiMenuId::SYSTEM_ROOT:
  case UiMenuId::SYSTEM_OVERVIEW:
  case UiMenuId::SYSTEM_LINKS:
  case UiMenuId::SYSTEM_ERRORS:
  case UiMenuId::SYSTEM_SPI_BUS:
  case UiMenuId::SYSTEM_UART_STATUS:
    iconChip(cx, cy, c, bg);
    break;
  case UiMenuId::SYSTEM_PINS_IO:
  case UiMenuId::SYSTEM_PINS_DISPLAY:
  case UiMenuId::SYSTEM_TOUCH_PANEL:
    iconPins(cx, cy, c, bg);
    break;
  case UiMenuId::SYSTEM_POWER_STATUS:
    iconBolt(cx, cy, c);
    break;
  case UiMenuId::SYSTEM_TFT_TEST:
    iconGrid(cx, cy, c);
    break;
  case UiMenuId::NAVIGATION_ROOT:
  case UiMenuId::NAV_LOCALIZATION:
  case UiMenuId::NAV_IMU_MAG:
  case UiMenuId::NAV_EKF:
  case UiMenuId::NAV_IMU:
  case UiMenuId::NAV_MAG:
  case UiMenuId::NAV_EKF_LOCAL:
  case UiMenuId::NAV_EKF_GLOBAL:
    iconCompass(cx, cy, c, bg);
    break;
  case UiMenuId::NAV_GNSS:
    iconGps(cx, cy, c, bg);
    break;
  case UiMenuId::NAVIGATION_OVERVIEW:
  case UiMenuId::NAV_ODOMETRY:
    iconGauge(cx, cy, c, bg);
    break;
  case UiMenuId::NAV_MISSION:
  case UiMenuId::NAV_MISSION_GO:
  case UiMenuId::NAV_MISSION_SAVE:
  case UiMenuId::NAV_MISSION_STOP:
  case UiMenuId::NAV_PATH_CONTROL:
    iconAutoArrow(cx, cy, c, bg);
    break;
  case UiMenuId::NAV_NAV2:
  case UiMenuId::NAV_PLANNER:
  case UiMenuId::NAV_MPPI:
  case UiMenuId::NAV_SMOOTHER:
  case UiMenuId::NAV_COSTMAP:
    iconGrid(cx, cy, c);
    break;
  default:
    iconGrid(cx, cy, c);
    break;
  }
}

inline void drawMenuCardTitle(const char *title, int cx, int y, uint16_t fg,
                              uint16_t bg) {
  char first[18]{};
  char second[18]{};
  const size_t len = strlen(title);
  if (len <= 13U) {
    drawCompactText(title, cx, y + 7, fg, bg, TC_DATUM);
    return;
  }
  size_t split = 0U;
  for (size_t i = 1U; i < len && i <= 13U; ++i)
    if (title[i] == ' ')
      split = i;
  if (split == 0U) {
    for (size_t i = 13U; i < len; ++i)
      if (title[i] == ' ') {
        split = i;
        break;
      }
  }
  if (split == 0U)
    split = len > 15U ? 15U : len;
  const size_t n1 = split < sizeof(first) - 1U ? split : sizeof(first) - 1U;
  memcpy(first, title, n1);
  first[n1] = '\0';
  const char *rest = title + split;
  while (*rest == ' ')
    ++rest;
  snprintf(second, sizeof(second), "%.16s", rest);
  drawCompactText(first, cx, y, fg, bg, TC_DATUM);
  drawCompactText(second, cx, y + 17, fg, bg, TC_DATUM);
}
inline void drawMetricRow(int y, const char *label, const char *value,
                          uint16_t valueColor = C_INK) {
  UiMetricCacheEntry *cache = metricCacheForY(y);
  if (gUiDynamicPass) {
    if (cache != nullptr && cache->valid && cache->color == valueColor &&
        std::strncmp(cache->value, value, sizeof(cache->value)) == 0)
      return;
    drawUiTextPadded(value, 252, y - 2, valueColor, C_CARD, 138U, TR_DATUM);
  } else {
    drawSmallText(label, 18, y, C_DISABLED, C_CARD);
    drawUiText(value, 252, y - 2, valueColor, C_CARD, TR_DATUM);
    drawThinDivider(16, y + 20, 238, C_CARD_LINE);
  }
  if (cache != nullptr) {
    cache->valid = true;
    cache->y = static_cast<int16_t>(y);
    cache->color = valueColor;
    std::snprintf(cache->value, sizeof(cache->value), "%s", value);
  }
}

inline void drawMetricFloat(int y, const char *label, float value,
                            const char *unit, int precision = 1,
                            uint16_t valueColor = C_INK) {
  char text[30];
  snprintf(text, sizeof(text), precision == 2 ? "%.2f %s" : "%.1f %s", value,
           unit);
  drawMetricRow(y, label, text, valueColor);
}

inline bool operatorDomainReady(UiMenuId id, const VehicleTelemetry &d) {
  const UiDomain domain = menuDomain(id);
  if (domain == UiDomain::ESC)
    return d.escReady && d.vescConnected;
  if (domain == UiDomain::PERCEPTION)
    return d.perceptionReady;
  if (domain == UiDomain::NAVIGATION)
    return d.motionReady && d.nav2Ready;
  if (domain == UiDomain::SYSTEM)
    return gDiagnostics.tftOk;
  return false;
}

inline bool operatorDomainFresh(UiMenuId id, const VehicleTelemetry &d) {
  const UiDomain domain = menuDomain(id);
  if (domain == UiDomain::ESC) return d.escFresh;
  if (domain == UiDomain::PERCEPTION) return d.perceptionFresh;
  if (domain == UiDomain::NAVIGATION) return d.navigationFresh;
  if (domain == UiDomain::SYSTEM) return true;
  return false;
}

inline const char *groupStatus(const TelemetryGroupStamp &g) {
  if (!g.valid) return "N/A";
  return g.fresh ? "READY" : "STALE";
}

inline const char *menuCardStatus(UiMenuId id, const VehicleTelemetry &d) {
  if (menuDomain(id) == UiDomain::SYSTEM)
    return gDiagnostics.tftOk ? "LOCAL OK" : "FAULT";
  if (!operatorDomainFresh(id, d))
    return "STALE";

  switch (id) {
  case UiMenuId::ESC_POWER: return groupStatus(d.escx.power);
  case UiMenuId::ESC_MOTOR_TELEMETRY: return groupStatus(d.escx.motors);
  case UiMenuId::ESC_ENCODER:
  case UiMenuId::ESC_CALIBRATION:
    return d.escx.encoder.valid ? groupStatus(d.escx.encoder)
                                : (d.encoderReady ? "WAIT" : "N/A");
  case UiMenuId::ESC_LINK:
    return d.escx.link.valid ? groupStatus(d.escx.link)
                             : (d.rosConnected && d.vescConnected ? "READY" : "WAIT");
  case UiMenuId::ESC_FAULT_SAFETY:
    return (d.eStop || d.systemStatus == SYS_FAULT ||
            (d.escx.power.valid && d.escx.power.fresh && d.escx.faultCode != 0U) ||
            (d.escx.motors.valid && d.escx.motors.fresh &&
             (d.escx.leftFault != 0U || d.escx.rightFault != 0U))) ? "FAULT" : "READY";
  case UiMenuId::ESC_PERFORMANCE:
    return d.escx.performance.valid ? groupStatus(d.escx.performance) : "LOCAL";
  case UiMenuId::PERCEPTION_CAMERA: return groupStatus(d.perx.camera);
  case UiMenuId::PERCEPTION_DETECTION: return groupStatus(d.perx.detection);
  case UiMenuId::PERCEPTION_LANE: return groupStatus(d.perx.lane);
  case UiMenuId::PERCEPTION_DRIVABLE_AREA: return groupStatus(d.perx.drivable);
  case UiMenuId::PERCEPTION_OBSTACLE: return groupStatus(d.perx.obstacle);
  case UiMenuId::PERCEPTION_PERFORMANCE: return groupStatus(d.perx.performance);
  case UiMenuId::PERCEPTION_CALIBRATION: return "N/A";
  case UiMenuId::NAV_LOCALIZATION:
    return d.navx.pose.valid ? groupStatus(d.navx.pose) : (d.motionReady ? "WAIT" : "N/A");
  case UiMenuId::NAV_GNSS: return d.gpsReady ? "READY" : "WAIT";
  case UiMenuId::NAV_IMU_MAG:
    return d.navx.imuMag.valid ? groupStatus(d.navx.imuMag)
                               : ((d.imuReady && d.magReady) ? "READY" : "WAIT");
  case UiMenuId::NAV_EKF: return d.motionReady ? "READY" : "WAIT";
  case UiMenuId::NAV_ODOMETRY: return groupStatus(d.navx.odom);
  case UiMenuId::NAV_NAV2: return d.navx.nav2.valid ? groupStatus(d.navx.nav2)
                                                    : (d.nav2Ready ? "WAIT" : "N/A");
  case UiMenuId::NAV_COSTMAP: return groupStatus(d.navx.costmap);
  case UiMenuId::NAV_PATH_CONTROL:
    return d.navx.control.valid ? groupStatus(d.navx.control)
                                : (d.navx.nav2.valid ? groupStatus(d.navx.nav2) : "N/A");
  case UiMenuId::NAV_SAFETY:
    return d.eStop ? "FAULT" : (d.motionReady ? "READY" : "WAIT");
  default:
    return operatorDomainReady(id, d) ? "READY" : "WAIT";
  }
}

inline uint16_t menuCardStatusColor(const char *status) {
  if (strcmp(status, "READY") == 0 || strcmp(status, "LOCAL OK") == 0)
    return C_READY;
  if (strcmp(status, "FAULT") == 0)
    return C_FAULT;
  if (strcmp(status, "N/A") == 0 || strcmp(status, "STALE") == 0)
    return C_DISABLED;
  if (strcmp(status, "LOCAL") == 0)
    return C_ACCENT;
  return C_WARNING;
}

inline void drawOperatorCard(uint8_t slot, UiMenuId id,
                             const VehicleTelemetry &d, bool mainMenu = false) {
  if (slot >= DOMAIN_PAGE_SIZE || id == UiMenuId::SPLASH)
    return;
  const int x = UI_CARD_X0 + slot * (UI_CARD_W + UI_CARD_GAP);
  const char *status = menuCardStatus(id, d);
  const uint16_t state = menuCardStatusColor(status);
  UiMenuCardCache &cache = gUiMenuCardCache[slot];
  const bool fullCard = !gUiDynamicPass || !cache.valid || cache.id != id;
  const uint16_t fill = mainMenu ? C_CARD : C_PANEL;
  const uint16_t fg = mainMenu ? C_INK : C_TEXT;
  if (fullCard) {
    drawCard(x, SUBMENU_CARD_Y, UI_CARD_W, SUBMENU_CARD_H, fill,
             mainMenu ? C_CARD_LINE : C_BORDER, false);
    tft.fillRoundRect(x, SUBMENU_CARD_Y, 4, SUBMENU_CARD_H, 2, state);
    drawMenuNodeIcon(id, x + UI_CARD_W / 2, SUBMENU_CARD_Y + 39,
                     mainMenu ? C_INK : C_ACCENT, fill);
    drawMenuCardTitle(menuTitle(id), x + UI_CARD_W / 2,
                      SUBMENU_CARD_Y + 72, fg, fill);
  }
  if (fullCard || cache.stateColor != state ||
      std::strncmp(cache.status, status, sizeof(cache.status)) != 0) {
    tft.fillRoundRect(x, SUBMENU_CARD_Y, 4, SUBMENU_CARD_H, 2, state);
    drawStatusDot(x + 14, SUBMENU_CARD_Y + 116, state, 3);
    drawMicroTextPadded(status, x + UI_CARD_W / 2 + 6, SUBMENU_CARD_Y + 112,
                        state, fill, 70U, TC_DATUM);
  }
  cache.valid = true;
  cache.id = id;
  cache.stateColor = state;
  std::snprintf(cache.status, sizeof(cache.status), "%s", status);
}

inline void drawHome(const UiState &ui, const VehicleTelemetry &d) {
  (void)ui;
  const int xs[3] = {UI_CARD_X0, UI_CARD_X0 + UI_CARD_W + UI_CARD_GAP,
                     UI_CARD_X0 + 2 * (UI_CARD_W + UI_CARD_GAP)};
  if (!gUiDynamicPass) {
    const char *labels[3] = {"SPEED", "STATE", "POWER"};
    for (uint8_t i = 0U; i < 3U; ++i) {
      drawCard(xs[i], HOME_TILE_Y, UI_CARD_W, HOME_TILE_H, C_CARD, C_CARD_LINE,
               false);
      drawMicroText(labels[i], xs[i] + UI_CARD_W / 2, HOME_TILE_Y + 10,
                    C_DISABLED, C_CARD, TC_DATUM);
    }
    tft.fillRoundRect(HOME_MENU_X, HOME_MENU_Y, HOME_MENU_W, HOME_MENU_H, 7,
                      C_PANEL);
    tft.drawRoundRect(HOME_MENU_X, HOME_MENU_Y, HOME_MENU_W, HOME_MENU_H, 7,
                      C_ACCENT);
    iconGrid(HOME_MENU_X + 26, HOME_MENU_Y + 20, C_ACCENT);
    drawUiText("MENU", W / 2, HOME_MENU_Y + 10, C_ACCENT, C_PANEL, TC_DATUM);
    drawMicroText("HOLD 0.8s = SERVICE", W / 2, HOME_MENU_Y + 31, C_TEXT_DIM,
                  C_PANEL, TC_DATUM);
    drawMicroText("ESC", 62, HOME_HEALTH_Y + 18, C_TEXT_DIM, C_BG, TC_DATUM);
    drawMicroText("PER", 160, HOME_HEALTH_Y + 18, C_TEXT_DIM, C_BG, TC_DATUM);
    drawMicroText("NAV", 258, HOME_HEALTH_Y + 18, C_TEXT_DIM, C_BG, TC_DATUM);
  }

  char speed[20], state[20], power[20], modeLink[48];
  snprintf(speed, sizeof(speed), "%.1f", d.speedKmh);
  const char *stateText = d.eStop
                              ? "E-STOP"
                              : (d.systemStatus == SYS_FAULT
                                     ? "FAULT"
                                     : (d.state == STATE_RUNNING
                                            ? "RUNNING"
                                            : (d.systemStatus == SYS_READY
                                                   ? "READY"
                                                   : vehicleStateText(d.state))));
  snprintf(state, sizeof(state), "%s", stateText);
  if (d.vbusValid)
    snprintf(power, sizeof(power), "%.1f V", d.vbusV);
  else
    snprintf(power, sizeof(power), "%s", "N/A");
  snprintf(modeLink, sizeof(modeLink), "%s | ROS %s | F103 %s", modeText(d.mode),
           d.rosConnected ? "ON" : "OFF", d.vescConnected ? "ON" : "OFF");

  const uint16_t speedColor = d.eStop ? C_FAULT : C_INK;
  const uint16_t stateColor = d.eStop || d.systemStatus == SYS_FAULT
                                  ? C_FAULT
                                  : (d.systemStatus == SYS_READY ? C_READY : C_WARNING);
  const uint16_t powerColor = d.vbusValid ? C_INK : C_DISABLED;
  if (!gUiDynamicPass || !gUiHomeCache.valid ||
      strcmp(gUiHomeCache.speed, speed) != 0 || gUiHomeCache.speedColor != speedColor)
    drawValueTextPadded(speed, xs[0] + UI_CARD_W / 2, HOME_TILE_Y + 36,
                        speedColor, C_CARD, 86U, TC_DATUM);
  if (!gUiDynamicPass || !gUiHomeCache.valid ||
      strcmp(gUiHomeCache.state, state) != 0 || gUiHomeCache.stateColor != stateColor)
    drawCompactTextPadded(state, xs[1] + UI_CARD_W / 2, HOME_TILE_Y + 42,
                          stateColor, C_CARD, 88U, TC_DATUM);
  if (!gUiDynamicPass || !gUiHomeCache.valid ||
      strcmp(gUiHomeCache.power, power) != 0 || gUiHomeCache.powerColor != powerColor)
    drawValueTextPadded(power, xs[2] + UI_CARD_W / 2, HOME_TILE_Y + 36,
                        powerColor, C_CARD, 86U, TC_DATUM);

  if (!gUiDynamicPass || !gUiHomeCache.valid || strcmp(gUiHomeCache.modeLink, modeLink) != 0)
    drawMicroTextPadded(modeLink, W / 2, 169, C_TEXT_DIM, C_BG, 300U, TC_DATUM);

  const bool ready[3] = {d.escReady && d.vescConnected, d.perceptionReady,
                         d.motionReady && d.nav2Ready};
  const bool fresh[3] = {d.escFresh, d.perceptionFresh, d.navigationFresh};
  const int dotX[3] = {62, 160, 258};
  for (uint8_t i = 0U; i < 3U; ++i) {
    const uint16_t color = domainHealthColor(ready[i], fresh[i]);
    if (!gUiDynamicPass || !gUiHomeCache.valid || gUiHomeCache.railColors[i] != color)
      drawStatusDot(dotX[i], HOME_HEALTH_Y + 6, color, 4);
    gUiHomeCache.railColors[i] = color;
  }

  std::snprintf(gUiHomeCache.speed, sizeof(gUiHomeCache.speed), "%s", speed);
  std::snprintf(gUiHomeCache.state, sizeof(gUiHomeCache.state), "%s", state);
  std::snprintf(gUiHomeCache.power, sizeof(gUiHomeCache.power), "%s", power);
  std::snprintf(gUiHomeCache.modeLink, sizeof(gUiHomeCache.modeLink), "%s", modeLink);
  gUiHomeCache.speedColor = speedColor;
  gUiHomeCache.stateColor = stateColor;
  gUiHomeCache.powerColor = powerColor;
  gUiHomeCache.valid = true;
}

inline void drawMainMenu(const UiState &ui, const VehicleTelemetry &d) {
  (void)ui;
  for (uint8_t slot = 0U; slot < DOMAIN_PAGE_SIZE; ++slot)
    drawOperatorCard(slot, menuCardAt(UiMenuId::MAIN_MENU, 0U, slot), d, true);
}

inline void drawDomainMenu(const UiState &ui, const VehicleTelemetry &d) {
  for (uint8_t slot = 0U; slot < DOMAIN_PAGE_SIZE; ++slot) {
    const UiMenuId id = menuCardAt(ui.menu, ui.pageIndex, slot);
    if (id != UiMenuId::SPLASH)
      drawOperatorCard(slot, id, d, false);
    else if (!gUiDynamicPass) {
      const int x = UI_CARD_X0 + slot * (UI_CARD_W + UI_CARD_GAP);
      tft.fillRect(x, SUBMENU_CARD_Y, UI_CARD_W, SUBMENU_CARD_H, C_BG);
      gUiMenuCardCache[slot] = UiMenuCardCache{};
    }
  }
}

inline void drawEditStatus(const VehicleTelemetry &d) {
  const char *text =
      d.configPending ? "WAITING ROS ACK / READBACK"
                      : (!d.configLastOk ? d.configMessage : "SYNCED WITH ROS");
  const uint16_t color =
      d.configPending ? C_WARNING : (!d.configLastOk ? C_FAULT : C_READY);
  if (!gUiDynamicPass || !gUiEditorCache.valid ||
      gUiEditorCache.statusColor != color ||
      std::strncmp(gUiEditorCache.status, text,
                   sizeof(gUiEditorCache.status)) != 0) {
    if (gUiDynamicPass)
      drawMicroTextPadded(text, 18, 164, color, C_CARD, 280U);
    else
      drawMicroText(text, 18, 164, color, C_CARD);
  }
  gUiEditorCache.statusColor = color;
  std::snprintf(gUiEditorCache.status, sizeof(gUiEditorCache.status), "%s",
                text);
}

inline void drawEditor(const UiState &ui, const VehicleTelemetry &d,
                       UiEditKey key) {
  drawContentCard();
  if (!gUiDynamicPass)
    drawMicroText("EDIT VALUE", 18, 46, C_DISABLED, C_CARD);
  char value[32];
  const float shown =
      ui.editing
          ? ui.editValue
          : (key == UiEditKey::OPERATOR_MODE
                 ? (d.mode == MODE_MANUAL ? 1.0F : 0.0F)
                 : (key == UiEditKey::MANUAL_SPEED_PCT
                        ? static_cast<float>(d.manualSpeedPct)
                        : (key == UiEditKey::STEERING_TEST_DEG
                               ? d.steeringTestAngleDeg
                               : (key == UiEditKey::DRIVE_SCALE
                                      ? d.driveScale
                                      : (d.perceptionInference ? 1.0F
                                                               : 0.0F)))));
  if (key == UiEditKey::MANUAL_SPEED_PCT)
    snprintf(value, sizeof(value), "%.0f %%", shown);
  else if (key == UiEditKey::STEERING_TEST_DEG)
    snprintf(value, sizeof(value), "%.0f deg", shown);
  else if (key == UiEditKey::DRIVE_SCALE)
    snprintf(value, sizeof(value), "%.3f", shown);
  else if (key == UiEditKey::OPERATOR_MODE)
    snprintf(value, sizeof(value), "%s", shown > 0.5F ? "MANUAL" : "AUTO");
  else
    snprintf(value, sizeof(value), "%s", shown > 0.5F ? "ON" : "OFF");
  if (!gUiDynamicPass || !gUiEditorCache.valid || gUiEditorCache.key != key ||
      std::strncmp(gUiEditorCache.value, value, sizeof(gUiEditorCache.value)) !=
          0) {
    drawHeroTextPadded(value, 18, 70, C_INK, C_CARD, 260U);
  }
  gUiEditorCache.key = key;
  std::snprintf(gUiEditorCache.value, sizeof(gUiEditorCache.value), "%s",
                value);
  if (!gUiDynamicPass) {
    drawSmallText("LEFT / RIGHT untuk ubah nilai", 18, 127, C_DISABLED, C_CARD);
    drawSmallText("OK untuk apply + readback ROS", 18, 147, C_DISABLED, C_CARD);
  }
  drawEditStatus(d);
  gUiEditorCache.valid = true;
}

inline void drawManualTestButton(int x, int y, int w, int h, SoftKey key,
                                 const char *label, const char *sub,
                                 bool active, bool stop = false) {
  const uint16_t fill = stop ? C_FAULT : (active ? C_PANEL_ALT : C_PANEL);
  const uint16_t border = stop ? C_FAULT : (active ? C_ACCENT : C_BORDER);
  const uint16_t fg = stop ? C_WHITE : (active ? C_TEXT : C_DISABLED);
  tft.fillRoundRect(x, y, w, h, 8, fill);
  tft.drawRoundRect(x, y, w, h, 8, border);
  const int cx = x + w / 2;
  if (key == SoftKey::TEST_FORWARD)
    iconArrowUp(cx, y + 16, fg);
  else if (key == SoftKey::TEST_REVERSE)
    iconArrowDown(cx, y + 16, fg);
  else if (key == SoftKey::TEST_LEFT)
    iconArrowLeft(cx, y + h / 2 - 8, fg);
  else if (key == SoftKey::TEST_RIGHT)
    iconArrowRight(cx, y + h / 2 - 8, fg);
  else if (key == SoftKey::TEST_STOP)
    iconStop(cx, y + h / 2 - 8, fg);
  drawCompactText(label, cx, y + h - 27, fg, fill, TC_DATUM);
  drawMicroText(sub, cx, y + h - 12, stop ? C_WHITE : C_TEXT_DIM, fill,
                TC_DATUM);
}

struct ManualRenderCache {
  bool valid{false};
  bool driveReady{false};
  bool steerReady{false};
  uint8_t speedPct{0U};
  int16_t angleDeg{0};
};
inline ManualRenderCache gManualRenderCache{};

inline void drawManualTest(const UiState &ui, const VehicleTelemetry &d) {
  (void)ui;
  const bool driveReady = d.rosConnected && d.escFresh &&
                          d.mode == MODE_MANUAL && d.escReady && !d.eStop;
  const bool steerReady =
      driveReady && d.encoderReady && fabsf(d.driveActualMps) < 0.02F;
  const int16_t angleDeg =
      static_cast<int16_t>(lroundf(d.steeringTestAngleDeg));
  char speed[16], angle[16];
  snprintf(speed, sizeof(speed), "%u%%",
           static_cast<unsigned>(d.manualSpeedPct));
  snprintf(angle, sizeof(angle), "%d deg", static_cast<int>(angleDeg));

  const bool full = !gUiDynamicPass || !gManualRenderCache.valid;
  const bool driveChanged = full ||
                            gManualRenderCache.driveReady != driveReady ||
                            gManualRenderCache.speedPct != d.manualSpeedPct;
  const bool steerChanged = full ||
                            gManualRenderCache.steerReady != steerReady ||
                            gManualRenderCache.angleDeg != angleDeg;
  if (driveChanged) {
    drawManualTestButton(108, 40, 104, 50, SoftKey::TEST_FORWARD, "FORWARD",
                         speed, driveReady);
    drawManualTestButton(108, 172, 104, 52, SoftKey::TEST_REVERSE, "REVERSE",
                         speed, driveReady);
  }
  if (steerChanged) {
    drawManualTestButton(6, 96, 94, 70, SoftKey::TEST_LEFT, "LEFT", angle,
                         steerReady);
    drawManualTestButton(220, 96, 94, 70, SoftKey::TEST_RIGHT, "RIGHT", angle,
                         steerReady);
  }
  if (full) {
    drawManualTestButton(108, 96, 104, 70, SoftKey::TEST_STOP, "STOP",
                         "ALL MOTION", true, true);
    drawMicroText("HOLD 0.6s  |  RELEASE = STOP", W / 2, 229, C_WARNING, C_BG,
                  TC_DATUM);
  }
  gManualRenderCache.valid = true;
  gManualRenderCache.driveReady = driveReady;
  gManualRenderCache.steerReady = steerReady;
  gManualRenderCache.speedPct = d.manualSpeedPct;
  gManualRenderCache.angleDeg = angleDeg;
}

inline void formatAgeMs(uint32_t ageMs, char *out, size_t outLen);
inline uint16_t ageColor(uint32_t ageMs);

inline void drawNaRow(int y, const char *label) {
  drawMetricRow(y, label, "N/A", C_DISABLED);
}

inline void drawEscLeaf(UiMenuId id, const UiState &ui,
                        const VehicleTelemetry &d) {
  if (id == UiMenuId::ESC_MANUAL_TEST) {
    drawManualTest(ui, d);
    return;
  }
  drawContentCard();
  char a[48];
  uint8_t view = ui.detailViewIndex;
  if (id == UiMenuId::ESC_STEERING_LIVE) { id = UiMenuId::ESC_STEERING; view = 0U; }
  if (id == UiMenuId::ESC_STEERING_CAL) { id = UiMenuId::ESC_STEERING; view = 1U; }
  if (id == UiMenuId::ESC_DRIVE_LIVE) { id = UiMenuId::ESC_DRIVE; view = 0U; }

  if (id == UiMenuId::ESC_OVERVIEW) {
    if (view == 0U) {
      drawMetricFloat(48, "Vehicle speed", d.driveActualMps, "m/s", 2, C_ACCENT);
      snprintf(a, sizeof(a), "%.1f / %.1f", d.steeringTargetDeg, d.steeringActualDeg);
      drawMetricRow(77, "Steer tgt / act", a, C_INK);
      snprintf(a, sizeof(a), "%.2f / %.2f", d.driveTargetMps, d.driveActualMps);
      drawMetricRow(106, "Drive tgt / act", a, C_INK);
      drawMetricRow(135, "ESC", d.escFresh && d.escReady ? "READY" : "WAIT",
                    d.escFresh && d.escReady ? C_READY : C_WARNING);
    } else if (view == 1U) {
      snprintf(a, sizeof(a), "%.0f / %.0f", d.motorErpm, d.motorRpm);
      drawMetricRow(48, "eRPM / RPM", a, C_INK);
      drawMetricRow(77, "VESC link", d.vescConnected ? "ONLINE" : "OFFLINE",
                    healthColor(d.vescConnected));
      if (d.escx.power.valid) {
        snprintf(a, sizeof(a), "%u", static_cast<unsigned>(d.escx.faultCode));
        drawMetricRow(106, "Drive fault", a, d.escx.faultCode ? C_FAULT : C_READY);
      } else drawNaRow(106, "Drive fault");
      drawMetricRow(135, "Power data", groupStatus(d.escx.power),
                    menuCardStatusColor(groupStatus(d.escx.power)));
    } else {
      drawMetricRow(48, "ESC ready", d.escReady ? "YES" : "NO", healthColor(d.escReady));
      drawMetricRow(77, "Encoder", d.encoderReady ? "READY" : "WAIT", healthColor(d.encoderReady));
      drawMetricRow(106, "Mode", modeText(d.mode), C_ACCENT);
      drawMetricRow(135, "Source", d.mode == MODE_MANUAL ? "HMI / MANUAL" : "AUTO / ROS", C_INK);
    }
    return;
  }

  if (id == UiMenuId::ESC_STEERING) {
    if (view == 0U) {
      drawMetricFloat(48, "Target", d.steeringTargetDeg, "deg", 1, C_ACCENT);
      drawMetricFloat(77, "Actual", d.steeringActualDeg, "deg", 1, C_ACCENT);
      drawMetricFloat(106, "Error", d.steeringErrorDeg, "deg", 1,
                      fabsf(d.steeringErrorDeg) < 2.0F ? C_READY : C_WARNING);
      drawMetricRow(135, "Encoder", d.encoderReady ? "READY" : "OFFLINE", healthColor(d.encoderReady));
    } else if (view == 1U) {
      if (d.escx.encoder.valid) {
        snprintf(a, sizeof(a), "%ld", static_cast<long>(d.escx.encoderRaw));
        drawMetricRow(48, "Raw encoder", a, C_INK);
        snprintf(a, sizeof(a), "C:%u H:%u S:%u", d.escx.calibrated ? 1U : 0U,
                 d.escx.homed ? 1U : 0U, d.escx.encoderSynced ? 1U : 0U);
        drawMetricRow(77, "Cal / home / sync", a,
                      d.escx.calibrated && d.escx.homed && d.escx.encoderSynced ? C_READY : C_WARNING);
        drawMetricFloat(106, "Encoder position", d.escx.encoderPositionDeg, "deg", 1, C_ACCENT);
      } else {
        drawNaRow(48, "Raw encoder"); drawNaRow(77, "Cal / home / sync"); drawNaRow(106, "Encoder position");
      }
      drawMetricRow(135, "Data", groupStatus(d.escx.encoder), menuCardStatusColor(groupStatus(d.escx.encoder)));
    } else {
      drawNaRow(48, "Physical limits");
      drawNaRow(77, "Operational limit");
      snprintf(a, sizeof(a), "%.0f deg", d.steeringTestAngleDeg);
      drawMetricRow(106, "Test preset", a, C_ACCENT);
      drawMetricRow(135, "Center", fabsf(d.steeringActualDeg) < 1.0F ? "NEAR" : "OFFSET",
                    fabsf(d.steeringActualDeg) < 1.0F ? C_READY : C_WARNING);
    }
    return;
  }

  if (id == UiMenuId::ESC_DRIVE) {
    if (view == 0U) {
      drawMetricFloat(48, "Target", d.driveTargetMps, "m/s", 2, C_ACCENT);
      drawMetricFloat(77, "Actual", d.driveActualMps, "m/s", 2, C_ACCENT);
      snprintf(a, sizeof(a), "%.0f", d.motorErpm); drawMetricRow(106, "Electrical RPM", a, C_INK);
      snprintf(a, sizeof(a), "%.0f", d.motorRpm); drawMetricRow(135, "Mechanical RPM", a, C_INK);
    } else if (view == 1U) {
      snprintf(a, sizeof(a), "%.3f", d.driveScale); drawMetricRow(48, "Drive scale", a, C_ACCENT);
      if (fabsf(d.driveActualMps) > 0.02F) {
        snprintf(a, sizeof(a), "%.0f", d.motorErpm / d.driveActualMps);
        drawMetricRow(77, "eRPM per m/s", a, C_INK);
      } else drawNaRow(77, "eRPM per m/s");
      drawNaRow(106, "Odometry scale");
      if (d.vbusValid) drawMetricFloat(135, "Vbus", d.vbusV, "V", 1, C_INK); else drawNaRow(135, "Vbus");
    } else {
      snprintf(a, sizeof(a), "%u %%", static_cast<unsigned>(d.manualSpeedPct));
      drawMetricRow(48, "Manual preset", a, C_ACCENT);
      snprintf(a, sizeof(a), "%lu ms", static_cast<unsigned long>(DRIVE_TEST_MAX_MS));
      drawMetricRow(77, "Hard timeout", a, C_INK);
      drawMetricRow(106, "E-stop", d.eStop ? "ACTIVE" : "CLEAR", d.eStop ? C_FAULT : C_READY);
      drawMetricRow(135, "Manual gate", d.mode == MODE_MANUAL && d.escFresh ? "AVAILABLE" : "LOCKED",
                    d.mode == MODE_MANUAL && d.escFresh ? C_READY : C_WARNING);
    }
    return;
  }

  if (id == UiMenuId::ESC_POWER) {
    if (!d.escx.power.valid) {
      drawNaRow(48, view == 0U ? "Vbus" : (view == 1U ? "FOC Iq" : "MOS temperature"));
      drawNaRow(77, view == 0U ? "Motor current" : (view == 1U ? "FOC Id" : "Motor temperature"));
      drawNaRow(106, view == 0U ? "Input current" : (view == 1U ? "Duty cycle" : "VESC fault"));
      drawMetricRow(135, "Power data", "N/A", C_DISABLED);
      return;
    }
    if (view == 0U) {
      drawMetricFloat(48, "Vbus", d.escx.vbusV, "V", 1, C_ACCENT);
      drawMetricFloat(77, "Motor current", d.escx.motorCurrentA, "A", 1, C_INK);
      drawMetricFloat(106, "Input current", d.escx.inputCurrentA, "A", 1, C_INK);
      snprintf(a, sizeof(a), "%.1f %%", 100.0F * d.escx.duty); drawMetricRow(135, "Duty cycle", a, C_INK);
    } else if (view == 1U) {
      drawMetricFloat(48, "FOC Iq", d.escx.iqA, "A", 1, C_ACCENT);
      drawMetricFloat(77, "FOC Id", d.escx.idA, "A", 1, C_INK);
      snprintf(a, sizeof(a), "%.1f %%", 100.0F * d.escx.duty); drawMetricRow(106, "Duty cycle", a, C_INK);
      drawMetricRow(135, "Data", groupStatus(d.escx.power), menuCardStatusColor(groupStatus(d.escx.power)));
    } else {
      drawMetricFloat(48, "MOS temperature", d.escx.mosTempC, "C", 1,
                      d.escx.mosTempC > 80.0F ? C_WARNING : C_INK);
      drawNaRow(77, "Motor temperature");
      snprintf(a, sizeof(a), "%u", static_cast<unsigned>(d.escx.faultCode));
      drawMetricRow(106, "VESC fault", a, d.escx.faultCode ? C_FAULT : C_READY);
      drawMetricRow(135, "Freshness", groupStatus(d.escx.power), menuCardStatusColor(groupStatus(d.escx.power)));
    }
    return;
  }

  if (id == UiMenuId::ESC_MOTOR_TELEMETRY) {
    if (!d.escx.motors.valid) {
      drawNaRow(48, "Motor telemetry"); drawNaRow(77, "Current"); drawNaRow(106, "RPM");
      drawMetricRow(135, "Data", "N/A", C_DISABLED); return;
    }
    if (view == 0U) {
      snprintf(a, sizeof(a), "%.1f V / %.1f A", d.escx.leftVbusV, d.escx.leftCurrentA);
      drawMetricRow(48, "LEFT Vbus/current", a, C_INK);
      snprintf(a, sizeof(a), "%.0f / %.1f%%", d.escx.leftRpm, 100.0F*d.escx.leftDuty);
      drawMetricRow(77, "LEFT RPM / duty", a, C_INK);
      snprintf(a, sizeof(a), "%u", static_cast<unsigned>(d.escx.leftFault));
      drawMetricRow(106, "LEFT fault", a, d.escx.leftFault ? C_FAULT : C_READY);
      drawMetricRow(135, "LEFT role", "STEERING", C_ACCENT);
    } else if (view == 1U) {
      snprintf(a, sizeof(a), "%.1f V / %.1f A", d.escx.rightVbusV, d.escx.rightCurrentA);
      drawMetricRow(48, "RIGHT Vbus/current", a, C_INK);
      snprintf(a, sizeof(a), "%.0f / %.1f%%", d.escx.rightRpm, 100.0F*d.escx.rightDuty);
      drawMetricRow(77, "RIGHT RPM / duty", a, C_INK);
      snprintf(a, sizeof(a), "%u", static_cast<unsigned>(d.escx.rightFault));
      drawMetricRow(106, "RIGHT fault", a, d.escx.rightFault ? C_FAULT : C_READY);
      drawMetricRow(135, "RIGHT role", "DRIVE", C_ACCENT);
    } else {
      snprintf(a, sizeof(a), "L:%u R:%u", static_cast<unsigned>(d.escx.leftFault), static_cast<unsigned>(d.escx.rightFault));
      drawMetricRow(48, "Fault L / R", a, (d.escx.leftFault || d.escx.rightFault) ? C_FAULT : C_READY);
      snprintf(a, sizeof(a), "%.1f / %.1f V", d.escx.leftVbusV, d.escx.rightVbusV);
      drawMetricRow(77, "Vbus L / R", a, C_INK);
      snprintf(a, sizeof(a), "%.1f / %.1f A", d.escx.leftCurrentA, d.escx.rightCurrentA);
      drawMetricRow(106, "Current L / R", a, C_INK);
      drawMetricRow(135, "Freshness", groupStatus(d.escx.motors), menuCardStatusColor(groupStatus(d.escx.motors)));
    }
    return;
  }

  if (id == UiMenuId::ESC_ENCODER) {
    if (view == 0U) {
      drawMetricRow(48, "Encoder", d.encoderReady ? "READY" : "OFFLINE", healthColor(d.encoderReady));
      if (d.escx.encoder.valid) {
        snprintf(a,sizeof(a),"%ld",static_cast<long>(d.escx.encoderRaw)); drawMetricRow(77,"Raw count",a,C_INK);
        drawMetricFloat(106,"Position",d.escx.encoderPositionDeg,"deg",1,C_ACCENT);
        drawMetricRow(135,"Synced",d.escx.encoderSynced?"YES":"NO",d.escx.encoderSynced?C_READY:C_WARNING);
      } else { drawNaRow(77,"Raw count"); drawNaRow(106,"Position"); drawNaRow(135,"Synced"); }
    } else if (view == 1U) {
      drawNaRow(48,"Edge A count"); drawNaRow(77,"Edge B count"); drawNaRow(106,"Invalid transition");
      if(d.escx.encoder.valid) drawMetricRow(135,"Direction/invert",d.escx.encoderInverted?"INVERTED":"NORMAL",C_INK); else drawNaRow(135,"Direction/invert");
    } else {
      if(d.escx.encoder.valid) drawMetricFloat(48,"Mechanical angle",d.escx.encoderPositionDeg,"deg",1,C_ACCENT); else drawNaRow(48,"Mechanical angle");
      drawNaRow(77,"Observer angle"); drawNaRow(106,"Observer error");
      drawMetricRow(135,"Source",d.escx.encoder.valid?"ESC FOC TELEMETRY":"N/A",d.escx.encoder.valid?C_INK:C_DISABLED);
    }
    return;
  }

  if (id == UiMenuId::ESC_CALIBRATION) {
    if (view == 0U && d.escx.encoder.valid) {
      drawMetricRow(48,"Calibrated",d.escx.calibrated?"YES":"NO",d.escx.calibrated?C_READY:C_WARNING);
      drawMetricRow(77,"Homed",d.escx.homed?"YES":"NO",d.escx.homed?C_READY:C_WARNING);
      snprintf(a,sizeof(a),"%ld",static_cast<long>(d.escx.encoderSpan)); drawMetricRow(106,"Measured span",a,C_INK);
      snprintf(a,sizeof(a),"%ld",static_cast<long>(d.escx.encoderTarget)); drawMetricRow(135,"Raw target",a,C_ACCENT);
    } else if (view == 0U) {
      drawNaRow(48,"Calibrated"); drawNaRow(77,"Homed"); drawNaRow(106,"Measured span"); drawNaRow(135,"Raw target");
    } else if (view == 1U) {
      drawNaRow(48,"Left reference"); drawNaRow(77,"Center reference"); drawNaRow(106,"Right reference");
      drawMetricRow(135,"Action","ROS WEB / SERVICE",C_DISABLED);
    } else {
      snprintf(a,sizeof(a),"%.3f",d.driveScale); drawMetricRow(48,"Drive scale",a,C_ACCENT);
      drawNaRow(77,"Wheel radius");
      if(fabsf(d.driveActualMps)>0.02F){snprintf(a,sizeof(a),"%.0f",d.motorErpm/d.driveActualMps);drawMetricRow(106,"eRPM per m/s",a,C_INK);}else drawNaRow(106,"eRPM per m/s");
      drawNaRow(135,"Odometry scale");
    }
    return;
  }

  if (id == UiMenuId::ESC_LINK) {
    if (view == 0U) {
      drawMetricRow(48,"ROS <-> F411",d.rosConnected?"ONLINE":"OFFLINE",healthColor(d.rosConnected));
      drawMetricRow(77,"F411 <-> F103",d.vescConnected?"ONLINE":"OFFLINE",healthColor(d.vescConnected));
      formatAgeMs(d.escAgeMs,a,sizeof(a)); drawMetricRow(106,"ESC age",a,ageColor(d.escAgeMs));
      drawMetricRow(135,"USB CDC","1,000,000",C_INK);
    } else if (view == 1U) {
      snprintf(a,sizeof(a),"E:%lu O:%lu D:%lu",(unsigned long)gDiagnostics.vescUartErrors,(unsigned long)gDiagnostics.vescUartOverflow,(unsigned long)gDiagnostics.vescUartTxDropped);
      drawMetricRow(48,"UART err/ov/drop",a,(gDiagnostics.vescUartErrors||gDiagnostics.vescUartOverflow||gDiagnostics.vescUartTxDropped)?C_WARNING:C_READY);
      formatAgeMs(gDiagnostics.vescLastFrameAgeMs,a,sizeof(a)); drawMetricRow(77,"Valid frame age",a,ageColor(gDiagnostics.vescLastFrameAgeMs));
      snprintf(a,sizeof(a),"%lu",(unsigned long)gDiagnostics.vescRecoveryCount); drawMetricRow(106,"Recoveries",a,C_INK);
      if(d.escx.link.valid){snprintf(a,sizeof(a),"%lu",(unsigned long)d.escx.uartBaud);drawMetricRow(135,"F103 UART baud",a,C_ACCENT);}else drawNaRow(135,"F103 UART baud");
    } else {
      drawMetricRow(48,"VESC owner",d.escx.link.valid?d.escx.owner:"N/A",d.escx.link.valid?C_ACCENT:C_DISABLED);
      drawMetricRow(77,"Maintenance",d.escx.link.valid && strcmp(d.escx.owner,"MAINTENANCE")==0?"ACTIVE":"INACTIVE",C_INK);
      drawMetricRow(106,"Safety preempt","LOCAL ACTIVE",C_READY);
      drawMetricRow(135,"Link data",groupStatus(d.escx.link),menuCardStatusColor(groupStatus(d.escx.link)));
    }
    return;
  }

  if (id == UiMenuId::ESC_FAULT_SAFETY) {
    if (view == 0U) {
      drawMetricRow(48,"E-stop",d.eStop?"ACTIVE":"CLEAR",d.eStop?C_FAULT:C_READY);
      snprintf(a,sizeof(a),"%u",static_cast<unsigned>(d.escx.power.valid?d.escx.faultCode:255U));
      drawMetricRow(77,"Drive VESC fault",d.escx.power.valid?a:"N/A",d.escx.power.valid?(d.escx.faultCode?C_FAULT:C_READY):C_DISABLED);
      if(d.escx.motors.valid){snprintf(a,sizeof(a),"%u / %u",(unsigned)d.escx.leftFault,(unsigned)d.escx.rightFault);drawMetricRow(106,"Fault L / R",a,(d.escx.leftFault||d.escx.rightFault)?C_FAULT:C_READY);}else drawNaRow(106,"Fault L / R");
      drawMetricRow(135,"System",systemStatusText(d.systemStatus),systemStatusColor(d.systemStatus));
    } else if (view == 1U) {
      drawNaRow(48,"Undervoltage limit"); drawNaRow(77,"Overvoltage limit"); drawNaRow(106,"Overcurrent limit"); drawNaRow(135,"Thermal limit");
    } else {
      drawMetricRow(48,"ESC freshness",d.escFresh?"FRESH":"STALE",d.escFresh?C_READY:C_DISABLED);
      if(d.escx.performance.valid){formatAgeMs(d.escx.commandAgeMs,a,sizeof(a));drawMetricRow(77,"Command age",a,ageColor(d.escx.commandAgeMs));}else drawNaRow(77,"Command age");
      drawMetricRow(106,"Mode",modeText(d.mode),C_ACCENT);
      drawMetricRow(135,"Motion gate",(!d.eStop&&d.escFresh&&d.escReady)?"BASE OK":"LOCKED",(!d.eStop&&d.escFresh&&d.escReady)?C_READY:C_FAULT);
    }
    return;
  }

  if (id == UiMenuId::ESC_PERFORMANCE) {
    if (view == 0U) {
      if(d.escx.performance.valid){formatAgeMs(d.escx.commandAgeMs,a,sizeof(a));drawMetricRow(48,"Command age",a,ageColor(d.escx.commandAgeMs));formatAgeMs(d.escx.feedbackAgeMs,a,sizeof(a));drawMetricRow(77,"Feedback age",a,ageColor(d.escx.feedbackAgeMs));}
      else {drawNaRow(48,"Command age");drawNaRow(77,"Feedback age");}
      snprintf(a,sizeof(a),"%lu/%lu ms",(unsigned long)gDiagnostics.uiDrawLastMs,(unsigned long)gDiagnostics.uiDrawMaxMs);drawMetricRow(106,"UI last / max",a,C_ACCENT);
      snprintf(a,sizeof(a),"%lu ms",(unsigned long)gDiagnostics.maxServiceGapMs);drawMetricRow(135,"F4 service max",a,C_INK);
    } else if (view == 1U) {
      drawNaRow(48,"F103 ISR max/budget"); drawNaRow(77,"GET_VALUES latency"); drawNaRow(106,"F103 watchdog");
      drawMetricRow(135,"Policy","N/A UNTIL SOURCE",C_DISABLED);
    } else {
      snprintf(a,sizeof(a),"%lu/%lu/%lu ms",(unsigned long)gDiagnostics.serviceGapP95Ms,(unsigned long)gDiagnostics.serviceGapP99Ms,(unsigned long)gDiagnostics.maxServiceGapMs);drawMetricRow(48,"Svc p95/p99/max",a,C_INK);
      snprintf(a,sizeof(a),"%lu B",(unsigned long)gDiagnostics.stackHeadroomBytes);drawMetricRow(77,"Stack headroom",a,C_ACCENT);
      snprintf(a,sizeof(a),"%lu",(unsigned long)gDiagnostics.spiHalErrorCount);drawMetricRow(106,"SPI HAL errors",a,gDiagnostics.spiHalErrorCount?C_WARNING:C_READY);
      snprintf(a,sizeof(a),"%lu",(unsigned long)gDiagnostics.deferredCommandDrops);drawMetricRow(135,"Deferred drops",a,gDiagnostics.deferredCommandDrops?C_WARNING:C_READY);
    }
    return;
  }

  drawMetricRow(48,"ESC",d.escReady?"PASS":"FAIL",healthColor(d.escReady));
  drawMetricRow(77,"Encoder",d.encoderReady?"PASS":"FAIL",healthColor(d.encoderReady));
  drawMetricRow(106,"E-stop",d.eStop?"FAIL":"PASS",d.eStop?C_FAULT:C_READY);
  drawMetricRow(135,"Vehicle",vehicleStateText(d.state),d.state==STATE_FAULT?C_FAULT:C_INK);
}

inline void drawPerceptionLeaf(UiMenuId id, const UiState &ui,
                               const VehicleTelemetry &d) {
  drawContentCard();
  char text[48];
  uint8_t view = ui.detailViewIndex;
  if (id == UiMenuId::PERCEPTION_DETECTION_LIVE) { id = UiMenuId::PERCEPTION_DETECTION; view = 0U; }

  if (id == UiMenuId::PERCEPTION_OVERVIEW) {
    if (view == 0U) {
      drawMetricRow(48,"Camera",d.cameraReady?"READY":"OFFLINE",healthColor(d.cameraReady));
      drawMetricRow(77,"Perception",d.perceptionReady?"READY":"WAIT",healthColor(d.perceptionReady));
      drawMetricRow(106,"Inference",d.perceptionInference?"ON":"OFF",d.perceptionInference?C_READY:C_WARNING);
      snprintf(text,sizeof(text),"%.1f FPS",d.cameraFps);drawMetricRow(135,"Pipeline",text,d.cameraFps>1.0F?C_READY:C_WARNING);
    } else if (view == 1U) {
      drawMetricRow(48,"Lane",d.laneState,C_INK);
      drawMetricRow(77,"Drivable",d.drivableAreaClear?"CLEAR":"BLOCKED",d.drivableAreaClear?C_READY:C_FAULT);
      drawMetricRow(106,"Obstacle",d.obstacleDetected?"DETECTED":"CLEAR",d.obstacleDetected?C_FAULT:C_READY);
      if(d.perx.obstacle.valid && d.perx.obstacleDistanceM>0.0F)drawMetricFloat(135,"Nearest",d.perx.obstacleDistanceM,"m",2,C_ACCENT);
      else if(d.objectDistanceM>0.0F)drawMetricFloat(135,"Nearest",d.objectDistanceM,"m",2,C_ACCENT);
      else drawNaRow(135,"Nearest");
    } else {
      drawMetricRow(48,"Camera data",groupStatus(d.perx.camera),menuCardStatusColor(groupStatus(d.perx.camera)));
      drawMetricRow(77,"Detection data",groupStatus(d.perx.detection),menuCardStatusColor(groupStatus(d.perx.detection)));
      drawMetricRow(106,"Lane data",groupStatus(d.perx.lane),menuCardStatusColor(groupStatus(d.perx.lane)));
      drawMetricRow(135,"Obstacle data",groupStatus(d.perx.obstacle),menuCardStatusColor(groupStatus(d.perx.obstacle)));
    }
    return;
  }

  if (id == UiMenuId::PERCEPTION_CAMERA) {
    if (view == 0U) {
      drawMetricRow(48,"Camera",d.cameraReady?"READY":"OFFLINE",healthColor(d.cameraReady));
      snprintf(text,sizeof(text),"%.1f FPS",d.cameraFps);drawMetricRow(77,"Current FPS",text,d.cameraFps>1.0F?C_READY:C_WARNING);
      drawMetricRow(106,"Inference",d.perceptionInference?"ON":"OFF",d.perceptionInference?C_READY:C_WARNING);
      drawMetricRow(135,"Freshness",groupStatus(d.perx.camera),menuCardStatusColor(groupStatus(d.perx.camera)));
    } else if (view == 1U) {
      drawMetricRow(48,"Backend",d.perx.camera.valid?d.perx.backend:"N/A",d.perx.camera.valid?C_ACCENT:C_DISABLED);
      if(d.perx.camera.valid)drawMetricFloat(77,"Pipeline latency",d.perx.inferenceLatencyMs,"ms",1,C_INK);else drawNaRow(77,"Pipeline latency");
      drawNaRow(106,"Resolution");
      drawNaRow(135,"Requested FPS");
    } else {
      if(d.perx.camera.valid){snprintf(text,sizeof(text),"%lu",(unsigned long)d.perx.droppedFrames);drawMetricRow(48,"Dropped frames",text,d.perx.droppedFrames?C_WARNING:C_READY);snprintf(text,sizeof(text),"%lu",(unsigned long)d.perx.recoveryCount);drawMetricRow(77,"Recoveries",text,C_INK);}
      else {drawNaRow(48,"Dropped frames");drawNaRow(77,"Recoveries");}
      formatAgeMs(d.perx.camera.sourceAgeMs,text,sizeof(text));drawMetricRow(106,"Source age",text,ageColor(d.perx.camera.sourceAgeMs));
      drawMetricRow(135,"Camera health",d.cameraReady?"READY":"OFFLINE",healthColor(d.cameraReady));
    }
    return;
  }

  if (id == UiMenuId::PERCEPTION_DETECTION) {
    const bool validObject=d.perx.detection.valid && d.perx.objectCount>0U && d.objectDistanceM>0.0F;
    if (view == 0U) {
      drawMetricRow(48,"Object",validObject?d.detectedObject:"NONE",validObject?C_ACCENT:C_DISABLED);
      if(validObject)drawMetricFloat(77,"Distance",d.objectDistanceM,"m",2,C_INK);else drawNaRow(77,"Distance");
      if(validObject){snprintf(text,sizeof(text),"%.0f %%",d.confidencePct);drawMetricRow(106,"Confidence",text,C_INK);}else drawNaRow(106,"Confidence");
      if(validObject)drawMetricFloat(135,"Lateral",d.perx.nearestLateralM,"m",2,C_INK);else drawNaRow(135,"Lateral");
    } else if (view == 1U) {
      if(d.perx.detection.valid){snprintf(text,sizeof(text),"%u",(unsigned)d.perx.objectCount);drawMetricRow(48,"Object count",text,C_ACCENT);snprintf(text,sizeof(text),"%u",(unsigned)d.perx.trackCount);drawMetricRow(77,"Active tracks",text,C_INK);snprintf(text,sizeof(text),"%u",(unsigned)d.perx.missedTracks);drawMetricRow(106,"Missed tracks",text,d.perx.missedTracks?C_WARNING:C_READY);}
      else {drawNaRow(48,"Object count");drawNaRow(77,"Active tracks");drawNaRow(106,"Missed tracks");}
      drawMetricRow(135,"Detection data",groupStatus(d.perx.detection),menuCardStatusColor(groupStatus(d.perx.detection)));
    } else {
      if(d.perx.detection.valid){snprintf(text,sizeof(text),"%ld",(long)d.perx.nearestTrackId);drawMetricRow(48,"Nearest track",text,C_INK);}else drawNaRow(48,"Nearest track");
      formatAgeMs(d.perx.detection.sourceAgeMs,text,sizeof(text));drawMetricRow(77,"Source age",text,ageColor(d.perx.detection.sourceAgeMs));
      drawMetricRow(106,"Obstacle",d.obstacleDetected?"DETECTED":"CLEAR",d.obstacleDetected?C_FAULT:C_READY);
      drawMetricRow(135,"Inference",d.perceptionInference?"ON":"OFF",d.perceptionInference?C_READY:C_WARNING);
    }
    return;
  }

  if (id == UiMenuId::PERCEPTION_LANE) {
    if (view == 0U) {
      drawMetricRow(48,"Lane state",d.laneState,C_INK);
      if(d.perx.lane.valid){drawMetricFloat(77,"Center offset",d.perx.laneCenterOffsetM,"m",2,C_ACCENT);snprintf(text,sizeof(text),"%.0f %%",d.perx.laneConfidencePct);drawMetricRow(106,"Confidence",text,d.perx.laneConfidencePct>=50.0F?C_READY:C_WARNING);drawMetricRow(135,"Metric valid",d.perx.laneValid?"YES":"NO",d.perx.laneValid?C_READY:C_WARNING);}else{drawNaRow(77,"Center offset");drawNaRow(106,"Confidence");drawNaRow(135,"Metric valid");}
    } else if (view == 1U) {
      if(d.perx.lane.valid){drawMetricFloat(48,"Road width",d.perx.roadWidthM,"m",2,C_ACCENT);drawMetricFloat(77,"Left clearance",d.perx.laneLeftClearanceM,"m",2,C_INK);drawMetricFloat(106,"Right clearance",d.perx.laneRightClearanceM,"m",2,C_INK);drawMetricFloat(135,"Heading error",d.perx.laneHeadingErrorDeg,"deg",1,C_INK);}else{drawNaRow(48,"Road width");drawNaRow(77,"Left clearance");drawNaRow(106,"Right clearance");drawNaRow(135,"Heading error");}
    } else {
      drawMetricRow(48,"Lane data",groupStatus(d.perx.lane),menuCardStatusColor(groupStatus(d.perx.lane)));
      formatAgeMs(d.perx.lane.sourceAgeMs,text,sizeof(text));drawMetricRow(77,"Source age",text,ageColor(d.perx.lane.sourceAgeMs));
      drawMetricRow(106,"Perception",d.perceptionReady?"READY":"WAIT",healthColor(d.perceptionReady));
      drawMetricRow(135,"Safety",d.obstacleDetected?"CHECK":"MONITOR",d.obstacleDetected?C_WARNING:C_INK);
    }
    return;
  }

  if (id == UiMenuId::PERCEPTION_DRIVABLE_AREA) {
    if (view == 0U) {
      drawMetricRow(48,"Drivable",d.drivableAreaClear?"CLEAR":"BLOCKED",d.drivableAreaClear?C_READY:C_FAULT);
      if(d.perx.drivable.valid){drawMetricRow(77,"Metric valid",d.perx.drivableValid?"YES":"NO",d.perx.drivableValid?C_READY:C_WARNING);snprintf(text,sizeof(text),"%.0f %%",d.perx.drivableFractionPct);drawMetricRow(106,"Valid rows",text,C_ACCENT);drawMetricFloat(135,"Far lookahead",d.perx.farLookaheadM,"m",2,C_INK);}else{drawNaRow(77,"Metric valid");drawNaRow(106,"Valid rows");drawNaRow(135,"Far lookahead");}
    } else if (view == 1U) {
      if(d.perx.drivable.valid){drawMetricFloat(48,"Left corridor",d.perx.drivableLeftM,"m",2,C_INK);drawMetricFloat(77,"Right corridor",d.perx.drivableRightM,"m",2,C_INK);drawMetricFloat(106,"Far lookahead",d.perx.farLookaheadM,"m",2,C_ACCENT);}else{drawNaRow(48,"Left corridor");drawNaRow(77,"Right corridor");drawNaRow(106,"Far lookahead");}
      drawMetricRow(135,"Data",groupStatus(d.perx.drivable),menuCardStatusColor(groupStatus(d.perx.drivable)));
    } else {
      drawMetricRow(48,"Lane constraint",d.perx.laneValid?"VALID":"WAIT",d.perx.laneValid?C_READY:C_WARNING);
      drawMetricRow(77,"Obstacle",d.obstacleDetected?"BLOCKED":"CLEAR",d.obstacleDetected?C_FAULT:C_READY);
      formatAgeMs(d.perx.drivable.sourceAgeMs,text,sizeof(text));drawMetricRow(106,"Source age",text,ageColor(d.perx.drivable.sourceAgeMs));
      drawMetricRow(135,"Source","PERCEPTION METRIC",C_INK);
    }
    return;
  }

  if (id == UiMenuId::PERCEPTION_OBSTACLE) {
    if (view == 0U) {
      drawMetricRow(48,"Nearest",d.detectedObject,C_ACCENT);
      if(d.perx.obstacle.valid && d.perx.obstacleDistanceM>0.0F)drawMetricFloat(77,"Forward",d.perx.obstacleDistanceM,"m",2,C_INK);else drawNaRow(77,"Forward");
      if(d.perx.obstacle.valid)drawMetricFloat(106,"Lateral",d.perx.obstacleLateralM,"m",2,C_INK);else drawNaRow(106,"Lateral");
      drawMetricRow(135,"Obstacle",d.obstacleDetected?"BLOCKED":"CLEAR",d.obstacleDetected?C_FAULT:C_READY);
    } else if (view == 1U) {
      if(d.perx.obstacle.valid){snprintf(text,sizeof(text),"%u",(unsigned)d.perx.obstacleCount);drawMetricRow(48,"Obstacle count",text,C_ACCENT);snprintf(text,sizeof(text),"%ld",(long)d.perx.nearestTrackId);drawMetricRow(77,"Nearest track",text,C_INK);snprintf(text,sizeof(text),"%u",(unsigned)d.perx.missedTracks);drawMetricRow(106,"Missed frames",text,d.perx.missedTracks?C_WARNING:C_READY);}else{drawNaRow(48,"Obstacle count");drawNaRow(77,"Nearest track");drawNaRow(106,"Missed frames");}
      drawMetricRow(135,"Data",groupStatus(d.perx.obstacle),menuCardStatusColor(groupStatus(d.perx.obstacle)));
    } else {
      formatAgeMs(d.perx.obstacle.sourceAgeMs,text,sizeof(text));drawMetricRow(48,"Source age",text,ageColor(d.perx.obstacle.sourceAgeMs));
      if(d.perx.performance.valid){snprintf(text,sizeof(text),"%lu",(unsigned long)d.perx.confirmedObstacleCount);drawMetricRow(77,"Confirmed count",text,C_INK);}else drawNaRow(77,"Confirmed count");
      drawMetricRow(106,"Advisory",d.obstacleDetected?"CHECK":"CLEAR",d.obstacleDetected?C_WARNING:C_READY);
      drawMetricRow(135,"E-stop",d.eStop?"ACTIVE":"CLEAR",d.eStop?C_FAULT:C_READY);
    }
    return;
  }

  if (id == UiMenuId::PERCEPTION_PERFORMANCE) {
    if (view == 0U) {
      snprintf(text,sizeof(text),"%.1f FPS",d.cameraFps);drawMetricRow(48,"Pipeline FPS",text,C_ACCENT);
      if(d.perx.performance.valid)drawMetricFloat(77,"Pipeline latency",d.perx.inferenceLatencyMs,"ms",1,C_INK);else drawNaRow(77,"Pipeline latency");
      drawMetricRow(106,"Performance data",groupStatus(d.perx.performance),menuCardStatusColor(groupStatus(d.perx.performance)));
      drawMetricRow(135,"Camera",d.cameraReady?"ONLINE":"OFFLINE",healthColor(d.cameraReady));
    } else if (view == 1U) {
      drawMetricRow(48,"Backend",d.perx.performance.valid?d.perx.backend:"N/A",d.perx.performance.valid?C_ACCENT:C_DISABLED);
      drawMetricRow(77,"Inference",d.perceptionInference?"ON":"OFF",d.perceptionInference?C_READY:C_WARNING);
      if(d.perx.performance.valid){snprintf(text,sizeof(text),"%lu",(unsigned long)d.perx.droppedFrames);drawMetricRow(106,"Dropped frames",text,d.perx.droppedFrames?C_WARNING:C_READY);snprintf(text,sizeof(text),"%lu",(unsigned long)d.perx.rawDetectionCount);drawMetricRow(135,"Raw detections",text,C_INK);}else{drawNaRow(106,"Dropped frames");drawNaRow(135,"Raw detections");}
    } else {
      formatAgeMs(d.perx.performance.sourceAgeMs,text,sizeof(text));drawMetricRow(48,"Performance age",text,ageColor(d.perx.performance.sourceAgeMs));
      formatAgeMs(d.perx.lane.sourceAgeMs,text,sizeof(text));drawMetricRow(77,"Lane age",text,ageColor(d.perx.lane.sourceAgeMs));
      formatAgeMs(d.perx.obstacle.sourceAgeMs,text,sizeof(text));drawMetricRow(106,"Obstacle age",text,ageColor(d.perx.obstacle.sourceAgeMs));
      if(d.perx.performance.valid){snprintf(text,sizeof(text),"%lu",(unsigned long)d.perx.confirmedObstacleCount);drawMetricRow(135,"Confirmed obstacles",text,C_INK);}else drawNaRow(135,"Confirmed obstacles");
    }
    return;
  }

  if (id == UiMenuId::PERCEPTION_CALIBRATION) {
    if (view == 0U) {
      drawNaRow(48,"Distance scale"); drawNaRow(77,"Distance bias"); drawNaRow(106,"Class/model"); drawMetricRow(135,"Edit","ROS WEB ONLY",C_DISABLED);
    } else if (view == 1U) {
      drawNaRow(48,"Camera height"); drawNaRow(77,"Camera pitch"); drawNaRow(106,"Corridor offsets"); drawNaRow(135,"Safety margin");
    } else {
      drawNaRow(48,"Calibration version"); drawNaRow(77,"Saved state");
      drawMetricRow(106,"Lane metric",d.perx.laneValid?"VALID":"WAIT",d.perx.laneValid?C_READY:C_WARNING);
      drawMetricRow(135,"Policy","DETAIL IN ROS WEB",C_DISABLED);
    }
    return;
  }

  // PER TEST is deliberately read-only: no vehicle motion is available here.
  drawMetricRow(48,"Camera",d.perx.camera.fresh?"PASS":"FAIL",d.perx.camera.fresh?C_READY:C_FAULT);
  drawMetricRow(77,"Detection",d.perx.detection.fresh?"PASS":"FAIL",d.perx.detection.fresh?C_READY:C_FAULT);
  drawMetricRow(106,"Lane",d.perx.lane.fresh?"PASS":"FAIL",d.perx.lane.fresh?C_READY:C_FAULT);
  drawMetricRow(135,"Obstacle",d.perx.obstacle.fresh?"PASS":"FAIL",d.perx.obstacle.fresh?C_READY:C_FAULT);
}

inline void formatAgeMs(uint32_t ageMs, char *out, size_t outLen) {
  if (ageMs == 0xFFFFFFFFUL) {
    snprintf(out, outLen, "NEVER");
  } else if (ageMs < 1000U) {
    snprintf(out, outLen, "%lu ms", static_cast<unsigned long>(ageMs));
  } else {
    snprintf(out, outLen, "%.1f s", static_cast<double>(ageMs) / 1000.0);
  }
}

inline uint16_t ageColor(uint32_t ageMs) {
  if (ageMs == 0xFFFFFFFFUL || ageMs > DOMAIN_DATA_STALE_MS)
    return C_DISABLED;
  if (ageMs > DOMAIN_DATA_STALE_MS / 2U)
    return C_WARNING;
  return C_READY;
}

inline void drawSystemLeaf(UiMenuId id, const UiState &ui,
                           const VehicleTelemetry &d) {
  (void)ui;
  drawContentCard();
  char a[48];
  if (id == UiMenuId::SYSTEM_OVERVIEW) {
    snprintf(a,sizeof(a),"0x%04lX %s",(unsigned long)(gDiagnostics.tftControllerId&0xFFFFUL),gDiagnostics.tftOk?"OK":"FAULT");
    drawMetricRow(48,"TFT / display",a,gDiagnostics.tftOk?C_READY:C_FAULT);
    snprintf(a,sizeof(a),"%lu / %lu ms",(unsigned long)gDiagnostics.uiDrawLastMs,(unsigned long)gDiagnostics.uiDrawMaxMs);drawMetricRow(77,"UI last / max",a,gDiagnostics.uiDrawLastMs<=50U?C_READY:C_WARNING);
    snprintf(a,sizeof(a),"%lu / %lu B",(unsigned long)gDiagnostics.displayBytesLastFrame,(unsigned long)gDiagnostics.displayBytesMaxFrame);drawMetricRow(106,"Frame last / max",a,C_ACCENT);
    snprintf(a,sizeof(a),"%lu/%lu/%lu ms",(unsigned long)gDiagnostics.serviceGapP95Ms,(unsigned long)gDiagnostics.serviceGapP99Ms,(unsigned long)gDiagnostics.maxServiceGapMs);drawMetricRow(135,"Svc p95/p99/max",a,gDiagnostics.maxServiceGapMs<=20U?C_READY:C_WARNING);
    return;
  }
  if (id == UiMenuId::SYSTEM_PINS_IO) {
#ifdef NEO3PRO
    snprintf(a,sizeof(a),"CS:%c INT:%c %s",gDiagnostics.pb6VescTx?'H':'L',gDiagnostics.pb7VescRx?'H':'L',gDiagnostics.canOk?"OK":"ERR");drawMetricRow(48,"PB6/PB7 MCP2515",a,gDiagnostics.canOk?C_READY:C_WARNING);
    snprintf(a,sizeof(a),"%u MHz / 1 Mbps",(unsigned)gDiagnostics.canOscillatorMhz);drawMetricRow(77,"CAN clock / bitrate",a,gDiagnostics.canOk?C_READY:C_WARNING);
    snprintf(a,sizeof(a),"RX:%lu D:%lu",(unsigned long)gDiagnostics.canRxFrames,(unsigned long)gDiagnostics.canTransfers);drawMetricRow(106,"DroneCAN raw / msg",a,gDiagnostics.canRxFrames?C_READY:C_WARNING);
    drawMetricRow(135,"F4 <-> ESC","DISCONNECTED",C_READY);
#else
    drawMetricRow(48,"PB6/PB7","FREE (NO ESC)",C_READY);
    snprintf(a,sizeof(a),"%c/%c %s",gDiagnostics.pa2GnssTx?'H':'L',gDiagnostics.pa3GnssRx?'H':'L',gDiagnostics.gnssUartOk?"OK":"ERR");drawMetricRow(77,"PA2/PA3 GNSS",a,gDiagnostics.gnssUartOk?C_READY:C_WARNING);
    snprintf(a,sizeof(a),"%c/%c %s",gDiagnostics.pb8I2cScl?'H':'L',gDiagnostics.pb9I2cSda?'H':'L',gDiagnostics.magOk?"MAG OK":"MAG ERR");drawMetricRow(106,"PB8/PB9 I2C1",a,gDiagnostics.magOk?C_READY:C_WARNING);
    snprintf(a,sizeof(a),"%s/%c/%c",gDiagnostics.pb12Safety?"CLR":"PRESS",gDiagnostics.pb13SafetyLed?'H':'L',gDiagnostics.pa8Buzzer?'H':'L');drawMetricRow(135,"PB12/13 + PA8",a,gDiagnostics.pb12Safety?C_READY:C_FAULT);
#endif
    return;
  }
  if (id == UiMenuId::SYSTEM_PINS_DISPLAY) {
    snprintf(a,sizeof(a),"S:%c I:%c O:%c",gDiagnostics.pa5SpiSck?'H':'L',gDiagnostics.pa6SpiMiso?'H':'L',gDiagnostics.pa7SpiMosi?'H':'L');drawMetricRow(48,"PA5/6/7 SPI1",a,C_INK);
    snprintf(a,sizeof(a),"CS:%c DC:%c RST:%c",gDiagnostics.pb0TftCs?'H':'L',gDiagnostics.pb1TftDc?'H':'L',gDiagnostics.pb2TftRst?'H':'L');drawMetricRow(77,"PB0/1/2 TFT",a,gDiagnostics.tftOk?C_READY:C_FAULT);
    snprintf(a,sizeof(a),"CS:%c R:%lu F:%lu",gDiagnostics.pa4TouchCs?'H':'L',(unsigned long)gDiagnostics.touchReadCount,(unsigned long)gDiagnostics.touchRejectFastCount);drawMetricRow(106,"PA4 TOUCH",a,C_ACCENT);
    snprintf(a,sizeof(a),"DM:%c DP:%c CDC",gDiagnostics.pa11UsbDm?'H':'L',gDiagnostics.pa12UsbDp?'H':'L');drawMetricRow(135,"PA11/12 USB",a,d.rosConnected?C_READY:C_WARNING);
    return;
  }
  if (id == UiMenuId::SYSTEM_LINKS) {
    formatAgeMs(gDiagnostics.rosHeartbeatAgeMs,a,sizeof(a));drawMetricRow(48,"ROS heartbeat",a,d.rosConnected?ageColor(gDiagnostics.rosHeartbeatAgeMs):C_FAULT);
    formatAgeMs(d.escAgeMs,a,sizeof(a));drawMetricRow(77,"ESC direct host",a,ageColor(d.escAgeMs));
    formatAgeMs(d.perceptionAgeMs,a,sizeof(a));drawMetricRow(106,"Perception stream",a,ageColor(d.perceptionAgeMs));
    formatAgeMs(d.navigationAgeMs,a,sizeof(a));drawMetricRow(135,"Navigation stream",a,ageColor(d.navigationAgeMs));
    return;
  }
  if (id == UiMenuId::SYSTEM_ERRORS) {
#ifdef NEO3PRO
    snprintf(a,sizeof(a),"SPI:%lu D:%lu O:%lu",(unsigned long)gDiagnostics.canSpiErrors,(unsigned long)gDiagnostics.canDecodeErrors,(unsigned long)gDiagnostics.canOverflows);drawMetricRow(48,"MCP/CAN errors",a,(gDiagnostics.canSpiErrors||gDiagnostics.canDecodeErrors||gDiagnostics.canOverflows)?C_WARNING:C_READY);
    snprintf(a,sizeof(a),"REC:%lu AGE:%lums",(unsigned long)gDiagnostics.canRecoveries,(unsigned long)gDiagnostics.canLastFrameAgeMs);drawMetricRow(77,"DroneCAN recovery",a,gDiagnostics.canOk?C_READY:C_WARNING);
#else
    snprintf(a,sizeof(a),"E:%lu O:%lu M:%lu",(unsigned long)gDiagnostics.gnssUartErrors,(unsigned long)gDiagnostics.gnssUartOverflow,(unsigned long)gDiagnostics.magErrors);drawMetricRow(48,"GNSS / MAG",a,(gDiagnostics.gnssUartErrors||gDiagnostics.gnssUartOverflow||gDiagnostics.magErrors)?C_WARNING:C_READY);
    drawMetricRow(77,"ESC F4 bridge","DISABLED",C_READY);
#endif
    snprintf(a,sizeof(a),"T:%lu H:%lu R:%lu B:%lu",(unsigned long)gDiagnostics.spiTimeoutCount,(unsigned long)gDiagnostics.spiHalErrorCount,(unsigned long)gDiagnostics.spiRecoveryCount,(unsigned long)gDiagnostics.spiBusConflictCount);drawMetricRow(106,"Shared SPI",a,(gDiagnostics.spiTimeoutCount||gDiagnostics.spiHalErrorCount||gDiagnostics.spiBusConflictCount)?C_WARNING:C_READY);
    snprintf(a,sizeof(a),"U:%lu O:%lu",(unsigned long)gDiagnostics.unknownCommands,(unsigned long)gDiagnostics.overlongCommands);drawMetricRow(135,"Host parser",a,(gDiagnostics.unknownCommands||gDiagnostics.overlongCommands)?C_WARNING:C_READY);
    return;
  }
  if (id == UiMenuId::SYSTEM_SPI_BUS) {
    snprintf(a,sizeof(a),"%lu Hz",(unsigned long)gDiagnostics.tftWriteClockHz);drawMetricRow(48,"TFT SPI clock",a,gDiagnostics.tftFastWriteValidated?C_READY:C_WARNING);
    snprintf(a,sizeof(a),"%lu / %lu",(unsigned long)gDiagnostics.spiTransactions,(unsigned long)gDiagnostics.spiBytesTx);drawMetricRow(77,"Transactions / bytes",a,C_INK);
    snprintf(a,sizeof(a),"TO:%lu HAL:%lu BUS:%lu",(unsigned long)gDiagnostics.spiTimeoutCount,(unsigned long)gDiagnostics.spiHalErrorCount,(unsigned long)gDiagnostics.spiBusConflictCount);drawMetricRow(106,"Errors",a,(gDiagnostics.spiTimeoutCount||gDiagnostics.spiHalErrorCount||gDiagnostics.spiBusConflictCount)?C_WARNING:C_READY);
#ifdef NEO3PRO
    snprintf(a,sizeof(a),"MCP:%s RX:%lu",gDiagnostics.canOk?"OK":"ERR",(unsigned long)gDiagnostics.canRxFrames);drawMetricRow(135,"Shared SPI MCP2515",a,gDiagnostics.canOk?C_READY:C_WARNING);
#else
    snprintf(a,sizeof(a),"REC:%lu F:%u U:%u",(unsigned long)gDiagnostics.spiRecoveryCount,gDiagnostics.tftFastWriteValidated?1U:0U,gDiagnostics.tftUltraFastWriteValidated?1U:0U);drawMetricRow(135,"Recovery / profiles",a,C_ACCENT);
#endif
    return;
  }
  if (id == UiMenuId::SYSTEM_UART_STATUS) {
    snprintf(a,sizeof(a),"E:%lu O:%lu D:%lu",(unsigned long)gDiagnostics.vescUartErrors,(unsigned long)gDiagnostics.vescUartOverflow,(unsigned long)gDiagnostics.vescUartTxDropped);drawMetricRow(48,"VESC UART",a,gDiagnostics.vescUartOk?C_READY:C_WARNING);
    snprintf(a,sizeof(a),"E:%lu O:%lu D:%lu",(unsigned long)gDiagnostics.gnssUartErrors,(unsigned long)gDiagnostics.gnssUartOverflow,(unsigned long)gDiagnostics.gnssUartTxDropped);drawMetricRow(77,"GNSS UART",a,gDiagnostics.gnssUartOk?C_READY:C_WARNING);
    snprintf(a,sizeof(a),"X:%lu M:%lu O:%lu",(unsigned long)gDiagnostics.extendedTelemetryAccepted,(unsigned long)gDiagnostics.extendedTelemetryMalformed,(unsigned long)gDiagnostics.extendedTelemetryOutOfOrder);drawMetricRow(106,"Ext RX ok/malformed/ooo",a,(gDiagnostics.extendedTelemetryMalformed||gDiagnostics.extendedTelemetryOutOfOrder)?C_WARNING:C_READY);
    if(d.escx.link.valid){snprintf(a,sizeof(a),"%lu / %s",(unsigned long)d.escx.uartBaud,d.escx.owner);drawMetricRow(135,"Baud / owner",a,C_ACCENT);}else drawNaRow(135,"Baud / owner");
    return;
  }
  if (id == UiMenuId::SYSTEM_POWER_STATUS) {
    if(d.escx.power.valid){drawMetricFloat(48,"Vbus",d.escx.vbusV,"V",1,C_ACCENT);drawMetricFloat(77,"Motor current",d.escx.motorCurrentA,"A",1,C_INK);drawMetricFloat(106,"Input current",d.escx.inputCurrentA,"A",1,C_INK);snprintf(a,sizeof(a),"%.1f%% / F%u",100.0F*d.escx.duty,(unsigned)d.escx.faultCode);drawMetricRow(135,"Duty / fault",a,d.escx.faultCode?C_FAULT:C_INK);}
    else{drawNaRow(48,"Vbus");drawNaRow(77,"Motor current");drawNaRow(106,"Input current");drawMetricRow(135,"Source","N/A",C_DISABLED);}return;
  }
  if (id == UiMenuId::SYSTEM_TOUCH_PANEL) {
    snprintf(a,sizeof(a),"%lu",(unsigned long)gDiagnostics.touchReadCount);drawMetricRow(48,"Touch reads",a,C_ACCENT);
    snprintf(a,sizeof(a),"%lu",(unsigned long)gDiagnostics.touchRejectFastCount);drawMetricRow(77,"Fast rejects",a,gDiagnostics.touchRejectFastCount?C_WARNING:C_READY);
    drawMetricRow(106,"PA4 touch CS",gDiagnostics.pa4TouchCs?"HIGH":"LOW",C_INK);
    drawMetricRow(135,"Input policy","RELEASE + SLIDE CANCEL",C_READY);
    return;
  }
  if (id == UiMenuId::SYSTEM_TFT_TEST) {
    snprintf(a,sizeof(a),"ID:%04lX MODE:%02X",(unsigned long)(gDiagnostics.tftControllerId&0xFFFFUL),gDiagnostics.tftPowerMode);drawMetricRow(48,"Controller",a,gDiagnostics.tftOk?C_READY:C_FAULT);
    snprintf(a,sizeof(a),"MAD:%02X PIX:%02X",gDiagnostics.tftMadctl,gDiagnostics.tftPixelFormat);drawMetricRow(77,"Display registers",a,C_INK);
    drawMetricRow(106,"Clock verify",gDiagnostics.tftUltraFastWriteValidated?"24 MHz VALID":(gDiagnostics.tftFastWriteValidated?"12 MHz VALID":"BASE"),C_ACCENT);
    drawMetricRow(135,"Action","PRESS OK FOR TFT TEST",C_WARNING);
    return;
  }
}

inline void drawNavigationLeaf(UiMenuId id, const UiState &ui,
                               const VehicleTelemetry &d) {
  drawContentCard();
  char text[48];
  uint8_t view = ui.detailViewIndex;
  if (id == UiMenuId::NAV_IMU) { id = UiMenuId::NAV_IMU_MAG; view = 0U; }
  if (id == UiMenuId::NAV_MAG) { id = UiMenuId::NAV_IMU_MAG; view = 1U; }
  if (id == UiMenuId::NAV_EKF_LOCAL) { id = UiMenuId::NAV_EKF; view = 0U; }
  if (id == UiMenuId::NAV_EKF_GLOBAL) { id = UiMenuId::NAV_EKF; view = 1U; }
  if (id == UiMenuId::NAV_PLANNER) { id = UiMenuId::NAV_PATH_CONTROL; view = 0U; }
  if (id == UiMenuId::NAV_MPPI) { id = UiMenuId::NAV_PATH_CONTROL; view = 1U; }
  if (id == UiMenuId::NAV_SMOOTHER) { id = UiMenuId::NAV_PATH_CONTROL; view = 2U; }

  if (id == UiMenuId::NAVIGATION_OVERVIEW) {
    if (view == 0U) {
      drawMetricRow(48,"Localization",d.localizationState,d.motionReady?C_READY:C_WARNING);
      drawMetricRow(77,"Nav2",d.nav2Ready?"READY":"WAIT",healthColor(d.nav2Ready));
      drawMetricRow(106,"Motion gate",d.motionReady?"OPEN":"LOCKED",d.motionReady?C_READY:C_FAULT);
      drawMetricRow(135,"Mission",navigationStatusText(d.navigationStatus),C_ACCENT);
    } else if (view == 1U) {
      drawMetricRow(48,"Active target",navigationHasTarget(d)?d.activeTarget:"NONE",navigationHasTarget(d)?C_ACCENT:C_DISABLED);
      drawMetricRow(77,"Navigation",navigationStatusText(d.navigationStatus),C_INK);
      drawMetricFloat(106,"Vehicle speed",d.driveActualMps,"m/s",2,C_ACCENT);
      if(d.navx.pose.valid){snprintf(text,sizeof(text),"%.2f, %.2f",d.navx.mapX,d.navx.mapY);drawMetricRow(135,"Map X,Y",text,C_INK);}else drawNaRow(135,"Map X,Y");
    } else {
      snprintf(text,sizeof(text),"%s / %u SAT",gpsFixText(d.gpsFix),d.satellites);drawMetricRow(48,"GNSS",text,gpsFixColor(d.gpsFix));
      drawMetricRow(77,"IMU",d.imuReady?"READY":"WAIT",healthColor(d.imuReady));
      drawMetricRow(106,"Nav telemetry",d.navigationFresh?"FRESH":"STALE",d.navigationFresh?C_READY:C_DISABLED);
      drawMetricRow(135,"Autonomy",(!d.eStop&&d.motionReady&&d.nav2Ready)?"ALLOWED":"LOCKED",(!d.eStop&&d.motionReady&&d.nav2Ready)?C_READY:C_FAULT);
    }return;
  }
  if (id == UiMenuId::NAV_LOCALIZATION) {
    if(view==0U){
      drawMetricRow(48,"Localization",d.localizationState,d.motionReady?C_READY:C_WARNING);
      if(d.navx.pose.valid){snprintf(text,sizeof(text),"%.2f / %.2f",d.navx.mapX,d.navx.mapY);drawMetricRow(77,"Map X / Y",text,C_ACCENT);drawMetricFloat(106,"Map yaw",d.navx.mapYawDeg,"deg",1,C_INK);formatAgeMs(d.navx.pose.sourceAgeMs,text,sizeof(text));drawMetricRow(135,"Pose age",text,ageColor(d.navx.pose.sourceAgeMs));}
      else{drawNaRow(77,"Map X / Y");drawNaRow(106,"Map yaw");drawNaRow(135,"Pose age");}
    }else if(view==1U){
      drawMetricRow(48,"EKF local",d.ekfLocalStatus,strstr(d.ekfLocalStatus,"READY")?C_READY:C_WARNING);
      drawMetricRow(77,"EKF global",d.ekfGlobalStatus,strstr(d.ekfGlobalStatus,"READY")?C_READY:C_WARNING);
      drawMetricRow(106,"GNSS source",d.gnssStatus,C_INK);drawMetricRow(135,"IMU source",d.imuStatus,C_INK);
    }else{
      if(d.navx.pose.valid){snprintf(text,sizeof(text),"%.4f / %.4f",d.navx.covarianceX,d.navx.covarianceY);drawMetricRow(48,"Cov X / Y",text,C_INK);snprintf(text,sizeof(text),"%.5f",d.navx.yawVariance);drawMetricRow(77,"Yaw variance",text,C_INK);}else{drawNaRow(48,"Cov X / Y");drawNaRow(77,"Yaw variance");}
      drawMetricFloat(106,"GNSS hAcc",d.haccM,"m",2,d.haccM<2.5F?C_READY:C_WARNING);drawMetricRow(135,"Pose data",groupStatus(d.navx.pose),menuCardStatusColor(groupStatus(d.navx.pose)));
    }return;
  }
  if (id == UiMenuId::NAV_GNSS) {
    if(view==0U){snprintf(text,sizeof(text),"%s / %u SAT",gpsFixText(d.gpsFix),d.satellites);drawMetricRow(48,"Fix",text,gpsFixColor(d.gpsFix));drawMetricFloat(77,"hAcc",d.haccM,"m",2,d.haccM<2.5F?C_READY:C_WARNING);drawMetricFloat(106,"HDOP",d.hdop,"",2,d.hdop<2.5F?C_READY:C_WARNING);drawMetricRow(135,"GNSS",d.gpsReady?"READY":"OFFLINE",healthColor(d.gpsReady));}
    else if(view==1U){snprintf(text,sizeof(text),"%.7f",d.latitude);drawMetricRow(48,"Latitude",text,C_INK);snprintf(text,sizeof(text),"%.7f",d.longitude);drawMetricRow(77,"Longitude",text,C_INK);drawMetricFloat(106,"Heading",d.headingDeg,"deg",1,C_ACCENT);drawMetricRow(135,"Status",d.gnssStatus,C_INK);}
    else {
      drawMetricFloat(48,"GNSS age",d.gnssAgeSec,"s",2,d.gnssAgeSec<0.5F?C_READY:C_WARNING);
      drawMetricRow(77,"Fix state",gpsFixText(d.gpsFix),gpsFixColor(d.gpsFix));
      drawMetricRow(106,"Domain",d.navigationFresh?"FRESH":"STALE",d.navigationFresh?C_READY:C_DISABLED);
#ifdef NEO3PRO
      drawMetricRow(135,"Source","NEO3 PRO / DRONECAN",C_INK);
#else
      drawMetricRow(135,"Source","NEO3 / F411",C_INK);
#endif
    } return;
  }
  if (id == UiMenuId::NAV_IMU_MAG) {
    if(view==0U){drawMetricRow(48,"IMU",d.imuStatus,healthColor(d.imuReady));drawMetricFloat(77,"Gyro Z",d.gyroZRps,"rad/s",2,C_INK);if(d.navx.imuMag.valid)drawMetricFloat(106,"IMU yaw",d.navx.imuYawDeg,"deg",1,C_ACCENT);else drawNaRow(106,"IMU yaw");drawMetricRow(135,"Data",d.navx.imuMag.valid?groupStatus(d.navx.imuMag):(d.navigationFresh?"LEGACY":"STALE"),d.navx.imuMag.valid?menuCardStatusColor(groupStatus(d.navx.imuMag)):(d.navigationFresh?C_WARNING:C_DISABLED));}
    else if(view==1U) {
#ifdef NEO3PRO
      drawMetricRow(48,"RM3100 DroneCAN",d.magReady?"READY":"OFFLINE",healthColor(d.magReady));
#else
      drawMetricRow(48,"IST8310",d.magReady?"READY":"OFFLINE",healthColor(d.magReady));
#endif
      if(d.navx.imuMag.valid) drawMetricFloat(77,"Fused heading",d.navx.fusedHeadingDeg,"deg",1,C_ACCENT);
      else drawMetricFloat(77,"Fused heading",d.headingDeg,"deg",1,C_ACCENT);
      drawNaRow(106,"Raw magnetic field");
      drawMetricRow(135,"Calibration",d.magReady?"AVAILABLE":"REQUIRED",d.magReady?C_READY:C_WARNING);
    }
    else{drawMetricRow(48,"GNSS",d.gpsReady?"VALID":"WAIT",healthColor(d.gpsReady));drawMetricRow(77,"IMU",d.imuReady?"VALID":"WAIT",healthColor(d.imuReady));drawMetricRow(106,"MAG",d.magReady?"VALID":"WAIT",healthColor(d.magReady));if(d.navx.imuMag.valid)drawMetricFloat(135,"Heading disagreement",d.navx.headingDisagreementDeg,"deg",1,d.navx.headingDisagreementDeg<15.0F?C_READY:C_WARNING);else drawNaRow(135,"Heading disagreement");}return;
  }
  if (id == UiMenuId::NAV_EKF) {
    if(view==0U){drawMetricRow(48,"EKF local",d.ekfLocalStatus,strstr(d.ekfLocalStatus,"READY")?C_READY:C_WARNING);drawMetricRow(77,"Localization",d.localizationState,C_INK);if(d.navx.odom.valid)drawMetricFloat(106,"Local speed",d.navx.odomLinearMps,"m/s",2,C_ACCENT);else drawMetricFloat(106,"Vehicle speed",d.driveActualMps,"m/s",2,C_INK);if(d.navx.odom.valid)drawMetricFloat(135,"Local yaw",d.navx.odomYawDeg,"deg",1,C_ACCENT);else drawMetricFloat(135,"Heading",d.headingDeg,"deg",1,C_ACCENT);}
    else if(view==1U){drawMetricRow(48,"EKF global",d.ekfGlobalStatus,strstr(d.ekfGlobalStatus,"READY")?C_READY:C_WARNING);drawMetricRow(77,"GNSS",d.gpsReady?"VALID":"WAIT",healthColor(d.gpsReady));if(d.navx.pose.valid)drawMetricFloat(106,"Map yaw",d.navx.mapYawDeg,"deg",1,C_ACCENT);else drawNaRow(106,"Map yaw");formatAgeMs(d.navx.pose.sourceAgeMs,text,sizeof(text));drawMetricRow(135,"Map pose age",text,ageColor(d.navx.pose.sourceAgeMs));}
    else{drawMetricRow(48,"GNSS contribution",d.gpsReady?"VALID":"INVALID",d.gpsReady?C_READY:C_WARNING);drawMetricRow(77,"IMU contribution",d.imuReady?"VALID":"INVALID",d.imuReady?C_READY:C_WARNING);drawMetricRow(106,"MAG contribution",d.magReady?"VALID":"INVALID",d.magReady?C_READY:C_WARNING);if(d.navx.pose.valid){snprintf(text,sizeof(text),"%.5f",d.navx.yawVariance);drawMetricRow(135,"Yaw variance",text,C_INK);}else drawNaRow(135,"Yaw variance");}return;
  }
  if (id == UiMenuId::NAV_ODOMETRY) {
    if(!d.navx.odom.valid){drawNaRow(48,"Odom X");drawNaRow(77,"Odom Y");drawNaRow(106,"Odom yaw");drawMetricRow(135,"Data","N/A",C_DISABLED);return;}
    if(view==0U){drawMetricFloat(48,"Linear speed",d.navx.odomLinearMps,"m/s",2,C_ACCENT);drawMetricFloat(77,"Yaw rate",d.navx.odomYawRateRps,"rad/s",2,C_INK);drawMetricFloat(106,"Steering",d.steeringActualDeg,"deg",1,C_INK);drawMetricRow(135,"Data",groupStatus(d.navx.odom),menuCardStatusColor(groupStatus(d.navx.odom)));}
    else if(view==1U){drawMetricFloat(48,"Odom X",d.navx.odomX,"m",2,C_INK);drawMetricFloat(77,"Odom Y",d.navx.odomY,"m",2,C_INK);drawMetricFloat(106,"Odom yaw",d.navx.odomYawDeg,"deg",1,C_ACCENT);formatAgeMs(d.navx.odom.sourceAgeMs,text,sizeof(text));drawMetricRow(135,"Odom age",text,ageColor(d.navx.odom.sourceAgeMs));}
    else{drawMetricFloat(48,"ESC speed",d.driveActualMps,"m/s",2,C_INK);drawMetricFloat(77,"Filtered speed",d.navx.odomLinearMps,"m/s",2,C_ACCENT);drawMetricFloat(106,"Speed error",d.driveActualMps-d.navx.odomLinearMps,"m/s",2,C_INK);snprintf(text,sizeof(text),"%.3f",d.driveScale);drawMetricRow(135,"Drive scale",text,C_ACCENT);}return;
  }
  if (id == UiMenuId::NAV_MISSION) {
    const uint8_t index=d.selectedWaypoint<HMI_WAYPOINT_COUNT?d.selectedWaypoint:0U;
    if(view==0U){drawMetricRow(48,"Waypoint",d.waypointName[index],d.waypointSaved[index]?C_READY:C_WARNING);drawMetricRow(77,"Saved",d.waypointSaved[index]?"YES":"NO",d.waypointSaved[index]?C_READY:C_WARNING);drawMetricRow(106,"Navigation",navigationStatusText(d.navigationStatus),C_ACCENT);drawMetricRow(135,"GO action",d.mode==MODE_AUTO&&d.waypointSaved[index]?"PRESS GO":"LOCKED",d.mode==MODE_AUTO&&d.waypointSaved[index]?C_READY:C_FAULT);}
    else if(view==1U){drawMetricRow(48,"Waypoint",d.waypointName[index],C_ACCENT);if(d.navx.pose.valid){snprintf(text,sizeof(text),"%.2f / %.2f",d.navx.mapX,d.navx.mapY);drawMetricRow(77,"Current map X/Y",text,C_INK);}else drawNaRow(77,"Current map X/Y");drawMetricRow(106,"Vehicle",vehicleStateText(d.state),C_INK);drawMetricRow(135,"SAVE action",d.gpsReady&&d.state==STATE_STOPPED?"PRESS SAVE":"LOCKED",d.gpsReady&&d.state==STATE_STOPPED?C_READY:C_FAULT);}
    else{drawMetricRow(48,"Active target",navigationHasTarget(d)?d.activeTarget:"NONE",navigationHasTarget(d)?C_ACCENT:C_DISABLED);drawMetricRow(77,"Navigation",navigationStatusText(d.navigationStatus),C_INK);drawMetricRow(106,"STOP action","ALWAYS AVAILABLE",C_FAULT);drawMetricRow(135,"Waypoint select","LEFT / RIGHT",C_INK);}return;
  }
  if (id == UiMenuId::NAV_NAV2) {
    if(view==0U){drawMetricRow(48,"Nav2",d.nav2Ready?"READY":"WAIT",healthColor(d.nav2Ready));drawMetricRow(77,"Localization",d.localizationState,d.motionReady?C_READY:C_WARNING);drawMetricRow(106,"Mission",navigationStatusText(d.navigationStatus),C_ACCENT);drawMetricRow(135,"Extended",groupStatus(d.navx.nav2),menuCardStatusColor(groupStatus(d.navx.nav2)));}
    else if(view==1U){drawMetricRow(48,"Planner",d.navx.nav2.valid?d.navx.plannerState:"N/A",d.navx.nav2.valid?C_INK:C_DISABLED);drawMetricRow(77,"Controller",d.navx.nav2.valid?d.navx.controllerState:"N/A",d.navx.nav2.valid?C_INK:C_DISABLED);drawMetricRow(106,"Smoother",d.navx.nav2.valid?d.navx.smootherState:"N/A",d.navx.nav2.valid?C_INK:C_DISABLED);drawMetricRow(135,"Path valid",d.navx.nav2.valid?(d.navx.pathValid?"YES":"NO"):"N/A",d.navx.nav2.valid?(d.navx.pathValid?C_READY:C_WARNING):C_DISABLED);}
    else{if(d.navx.nav2.valid){formatAgeMs(d.navx.cmdAgeMs,text,sizeof(text));drawMetricRow(48,"cmd age",text,ageColor(d.navx.cmdAgeMs));drawMetricFloat(77,"Linear cmd",d.navx.commandLinearMps,"m/s",2,C_ACCENT);drawMetricFloat(106,"Angular cmd",d.navx.commandAngularRps,"rad/s",2,C_INK);}else{drawNaRow(48,"cmd age");drawNaRow(77,"Linear cmd");drawNaRow(106,"Angular cmd");}drawMetricRow(135,"ESC data",d.escFresh?"FRESH":"STALE",d.escFresh?C_READY:C_DISABLED);}return;
  }
  if (id == UiMenuId::NAV_SAFETY) {
    if(view==0U){drawMetricRow(48,"E-stop",d.eStop?"ACTIVE":"CLEAR",d.eStop?C_FAULT:C_READY);drawMetricRow(77,"Localization",d.motionReady?"PASS":"LOCKED",d.motionReady?C_READY:C_FAULT);drawMetricRow(106,"Nav2",d.nav2Ready?"PASS":"WAIT",healthColor(d.nav2Ready));drawMetricRow(135,"Autonomy",(!d.eStop&&d.motionReady&&d.nav2Ready)?"ALLOWED":"LOCKED",(!d.eStop&&d.motionReady&&d.nav2Ready)?C_READY:C_FAULT);}
    else if(view==1U){drawMetricRow(48,"Lane",d.laneState,C_INK);drawMetricRow(77,"Drivable",d.drivableAreaClear?"CLEAR":"BLOCKED",d.drivableAreaClear?C_READY:C_FAULT);drawMetricRow(106,"Obstacle",d.obstacleDetected?"BLOCKED":"CLEAR",d.obstacleDetected?C_FAULT:C_READY);drawMetricRow(135,"Perception",d.perceptionFresh?"FRESH":"STALE",d.perceptionFresh?C_READY:C_DISABLED);}
    else{drawMetricRow(48,"ESC",d.escFresh?"FRESH":"STALE",d.escFresh?C_READY:C_DISABLED);if(d.navx.control.valid){formatAgeMs(d.navx.cmdAgeMs,text,sizeof(text));drawMetricRow(77,"Command age",text,ageColor(d.navx.cmdAgeMs));}else drawNaRow(77,"Command age");drawMetricRow(106,"Path",d.navx.nav2.valid?(d.navx.pathValid?"VALID":"INVALID"):"N/A",d.navx.nav2.valid?(d.navx.pathValid?C_READY:C_WARNING):C_DISABLED);drawMetricRow(135,"Navigation",d.navigationFresh?"FRESH":"STALE",d.navigationFresh?C_READY:C_DISABLED);}return;
  }
  if (id == UiMenuId::NAV_COSTMAP) {
    if(!d.navx.costmap.valid){drawNaRow(48,"Costmap readiness");drawNaRow(77,"Obstacle points");drawNaRow(106,"Path relevant");drawMetricRow(135,"Policy","N/A UNTIL SOURCE",C_DISABLED);return;}
    if(view==0U){drawMetricRow(48,"Costmap",d.navx.costmapReady?"READY":"WAIT",d.navx.costmapReady?C_READY:C_WARNING);snprintf(text,sizeof(text),"%u",(unsigned)d.navx.obstaclePointCount);drawMetricRow(77,"Obstacle points",text,C_INK);snprintf(text,sizeof(text),"%u",(unsigned)d.navx.pathRelevantCount);drawMetricRow(106,"Path relevant",text,C_INK);drawMetricRow(135,"Blocked",d.navx.costmapBlocked?"YES":"NO",d.navx.costmapBlocked?C_FAULT:C_READY);}
    else if(view==1U){drawMetricRow(48,"Source","TRAJECTORY SAFETY",C_INK);drawMetricRow(77,"Obstacle input",d.perceptionFresh?"FRESH":"STALE",d.perceptionFresh?C_READY:C_DISABLED);drawMetricRow(106,"Path valid",d.navx.pathValid?"YES":"NO",d.navx.pathValid?C_READY:C_WARNING);drawMetricRow(135,"Nav2",d.nav2Ready?"READY":"WAIT",healthColor(d.nav2Ready));}
    else{formatAgeMs(d.navx.costmap.sourceAgeMs,text,sizeof(text));drawMetricRow(48,"Source age",text,ageColor(d.navx.costmap.sourceAgeMs));drawMetricRow(77,"Data",groupStatus(d.navx.costmap),menuCardStatusColor(groupStatus(d.navx.costmap)));drawNaRow(106,"Inflation state");drawNaRow(135,"Footprint state");}return;
  }
  if (id == UiMenuId::NAV_PATH_CONTROL) {
    if(view==0U){drawMetricRow(48,"Planner",d.navx.nav2.valid?d.navx.plannerState:"N/A",d.navx.nav2.valid?C_INK:C_DISABLED);drawMetricRow(77,"Path valid",d.navx.nav2.valid?(d.navx.pathValid?"YES":"NO"):"N/A",d.navx.nav2.valid?(d.navx.pathValid?C_READY:C_WARNING):C_DISABLED);if(d.navx.costmap.valid){snprintf(text,sizeof(text),"%u",(unsigned)d.navx.pathRelevantCount);drawMetricRow(106,"Relevant obstacles",text,C_INK);}else drawNaRow(106,"Relevant obstacles");drawMetricRow(135,"Nav2",d.nav2Ready?"READY":"WAIT",healthColor(d.nav2Ready));}
    else if(view==1U){drawMetricRow(48,"Controller",d.navx.nav2.valid?d.navx.controllerState:"N/A",d.navx.nav2.valid?C_ACCENT:C_DISABLED);if(d.navx.control.valid){drawMetricFloat(77,"Linear command",d.navx.commandLinearMps,"m/s",2,C_INK);drawMetricFloat(106,"Angular command",d.navx.commandAngularRps,"rad/s",2,C_INK);formatAgeMs(d.navx.cmdAgeMs,text,sizeof(text));drawMetricRow(135,"Controller age",text,ageColor(d.navx.cmdAgeMs));}else{drawNaRow(77,"Linear command");drawNaRow(106,"Angular command");drawNaRow(135,"Controller age");}}
    else{drawMetricRow(48,"Smoother",d.navx.nav2.valid?d.navx.smootherState:"N/A",d.navx.nav2.valid?C_INK:C_DISABLED);drawMetricFloat(77,"Target speed",d.driveTargetMps,"m/s",2,C_ACCENT);drawMetricFloat(106,"Actual speed",d.driveActualMps,"m/s",2,C_INK);drawMetricFloat(135,"Steer error",d.steeringErrorDeg,"deg",1,C_INK);}return;
  }
  if (id == UiMenuId::NAV_MISSION_GO || id == UiMenuId::NAV_MISSION_SAVE || id == UiMenuId::NAV_MISSION_STOP) {
    const uint8_t index=d.selectedWaypoint<HMI_WAYPOINT_COUNT?d.selectedWaypoint:0U;drawMetricRow(48,"Waypoint",d.waypointName[index],d.waypointSaved[index]?C_READY:C_WARNING);drawMetricRow(77,"Saved",d.waypointSaved[index]?"YES":"NO",d.waypointSaved[index]?C_READY:C_WARNING);drawMetricRow(106,"Navigation",navigationStatusText(d.navigationStatus),C_ACCENT);drawMetricRow(135,"Action",id==UiMenuId::NAV_MISSION_STOP?"STOP ON OK":(id==UiMenuId::NAV_MISSION_SAVE?"SAVE ON OK":"GO ON OK"),id==UiMenuId::NAV_MISSION_STOP?C_FAULT:C_READY);return;
  }
  drawMetricRow(48,"GNSS",d.gpsReady?"PASS":"FAIL",healthColor(d.gpsReady));
  drawMetricRow(77,"IMU",d.imuReady?"PASS":"FAIL",healthColor(d.imuReady));
  drawMetricRow(106,"Nav2",d.nav2Ready?"PASS":"FAIL",healthColor(d.nav2Ready));
  drawMetricRow(135,"Safety",d.eStop?"FAIL":"PASS",d.eStop?C_FAULT:C_READY);
}

inline bool isEscMenu(UiMenuId id) {
  return menuDomain(id) == UiDomain::ESC && !menuIsDomainRoot(id);
}
inline bool isPerceptionMenu(UiMenuId id) {
  return menuDomain(id) == UiDomain::PERCEPTION && !menuIsDomainRoot(id);
}
inline bool isSystemMenu(UiMenuId id) {
  return menuDomain(id) == UiDomain::SYSTEM;
}
inline bool isNavigationMenu(UiMenuId id) {
  return menuDomain(id) == UiDomain::NAVIGATION && !menuIsDomainRoot(id);
}

inline bool uiNeedsEditFooter(const UiState &ui) {
  return menuEditKey(ui.menu) != UiEditKey::NONE ||
         ui.menu == UiMenuId::SYSTEM_TFT_TEST ||
         ui.menu == UiMenuId::NAV_MISSION_GO ||
         ui.menu == UiMenuId::NAV_MISSION_SAVE ||
         ui.menu == UiMenuId::NAV_MISSION_STOP;
}

inline bool uiNeedsPagerFooter(const UiState &ui) {
  if (ui.menu == UiMenuId::HOME || ui.menu == UiMenuId::OVERVIEW ||
      ui.menu == UiMenuId::MAIN_MENU || ui.menu == UiMenuId::ESC_MANUAL_TEST ||
      uiNeedsEditFooter(ui))
    return false;
  if (menuHasChildren(ui.menu)) {
    uint8_t count = 0U;
    (void)menuChildren(ui.menu, count);
    return menuPageCount(count) > 1U;
  }
  return menuViewCount(ui.menu) > 1U;
}

inline void clearCarouselBackgroundGaps() {
  tft.fillRect(0, CAROUSEL_NAV_Y, 4, CAROUSEL_NAV_H, C_BG);
  tft.fillRect(CAROUSEL_LEFT_X + CAROUSEL_NAV_W, CAROUSEL_NAV_Y, 6,
               CAROUSEL_NAV_H, C_BG);
  tft.fillRect(CAROUSEL_PAGE_X + CAROUSEL_PAGE_W, CAROUSEL_NAV_Y, 6,
               CAROUSEL_NAV_H, C_BG);
  tft.fillRect(CAROUSEL_RIGHT_X + CAROUSEL_NAV_W, CAROUSEL_NAV_Y,
               W - (CAROUSEL_RIGHT_X + CAROUSEL_NAV_W), CAROUSEL_NAV_H, C_BG);
}

inline void clearEditFooterBackgroundGaps() {
  const int x1 = softKeyX(0) + SOFTKEY_W;
  const int x2 = softKeyX(1) + SOFTKEY_W;
  const int x3 = softKeyX(2) + SOFTKEY_W;
  tft.fillRect(0, SOFTKEY_Y, SOFTKEY_X0, SOFTKEY_H, C_BG);
  tft.fillRect(x1, SOFTKEY_Y, softKeyX(1) - x1, SOFTKEY_H, C_BG);
  tft.fillRect(x2, SOFTKEY_Y, softKeyX(2) - x2, SOFTKEY_H, C_BG);
  tft.fillRect(x3, SOFTKEY_Y, W - x3, SOFTKEY_H, C_BG);
}

inline void clearFullFrameBackground(const UiState &ui) {
  // Full transitions may clear the content plane, but never the entire TFT.
  // Dynamic telemetry frames remain bounded by per-value caches below.
  (void)ui;
  tft.fillRect(0, TOP_H, W, H - TOP_H, C_BG);
}

inline void drawUiContent(const UiState &ui, const VehicleTelemetry &d) {
  if (ui.menu == UiMenuId::HOME || ui.menu == UiMenuId::OVERVIEW) {
    drawHome(ui, d);
  } else if (ui.menu == UiMenuId::MAIN_MENU) {
    drawMainMenu(ui, d);
  } else if (menuEditKey(ui.menu) != UiEditKey::NONE) {
    drawEditor(ui, d, menuEditKey(ui.menu));
  } else if (menuIsDomainRoot(ui.menu)) {
    drawDomainMenu(ui, d);
  } else if (isEscMenu(ui.menu)) {
    drawEscLeaf(ui.menu, ui, d);
  } else if (isPerceptionMenu(ui.menu)) {
    drawPerceptionLeaf(ui.menu, ui, d);
  } else if (menuDomain(ui.menu) == UiDomain::SYSTEM) {
    drawSystemLeaf(ui.menu, ui, d);
  } else if (isNavigationMenu(ui.menu)) {
    drawNavigationLeaf(ui.menu, ui, d);
  }
}

inline void drawUiFrame(const UiState &ui, const VehicleTelemetry &d, bool full,
                        uint8_t dirtyMask = UI_DIRTY_ALL) {
  gUiDynamicPass = !full;
  if (full) {
    resetUiMetricCache();
    gManualRenderCache = ManualRenderCache{};
    clearFullFrameBackground(ui);
  }
  if (full || (dirtyMask & UI_DIRTY_TOPBAR) != 0U)
    drawUiTopBar(ui, d);
  if (full || (dirtyMask & UI_DIRTY_CONTENT) != 0U)
    drawUiContent(ui, d);

  if (full) {
    if (uiNeedsEditFooter(ui)) {
      drawEditFooter(ui, d);
      gUiFooterCache.valid = true;
      gUiFooterCache.kind = UiFooterKind::EDIT;
      gUiFooterCache.configPending = d.configPending;
    } else if (uiNeedsPagerFooter(ui)) {
      drawCarouselFooter(ui);
    } else {
      gUiFooterCache.valid = true;
      gUiFooterCache.kind = UiFooterKind::NONE;
    }
  } else if ((dirtyMask & UI_DIRTY_FOOTER) != 0U && uiNeedsEditFooter(ui)) {
    if (!gUiFooterCache.valid || gUiFooterCache.kind != UiFooterKind::EDIT ||
        gUiFooterCache.configPending != d.configPending) {
      drawEditFooter(ui, d);
      gUiFooterCache.valid = true;
      gUiFooterCache.kind = UiFooterKind::EDIT;
      gUiFooterCache.configPending = d.configPending;
    }
  }
  gUiDynamicPass = false;
}
