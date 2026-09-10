// ============================================================================
// UiShell.h — renderer tunggal untuk OVERVIEW / ESC / PERCEPTION / NAVIGATION.
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

struct UiOverviewCardCache {
  bool valid{false};
  UiMenuId id{UiMenuId::SPLASH};
  bool selected{false};
  uint16_t stateColor{0U};
  char status[20]{};
};
struct UiOverviewCache {
  bool valid{false};
  char summary[76]{};
  char navigation[76]{};
  uint16_t summaryColor{0U};
  uint16_t systemDotColor{0U};
  uint16_t railColors[4]{};
  UiOverviewCardCache cards[3]{};
};
struct UiEditorCache {
  bool valid{false};
  UiEditKey key{UiEditKey::NONE};
  char value[32]{};
  char status[64]{};
  uint16_t statusColor{0U};
};
inline UiOverviewCache gUiOverviewCache{};
inline UiEditorCache gUiEditorCache{};

inline void invalidateUiChromeCaches() {
  gUiTopBarCache = UiTopBarCache{};
  gUiFooterCache = UiFooterCache{};
  gUiOverviewCache = UiOverviewCache{};
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

inline bool uiIsDomainRoot(UiMenuId id) {
  return id == UiMenuId::ESC_ROOT || id == UiMenuId::PERCEPTION_ROOT ||
         id == UiMenuId::NAVIGATION_ROOT || id == UiMenuId::SYSTEM_ROOT;
}

inline void drawUiTopBar(const UiState &ui, const VehicleTelemetry &d) {
  const bool menuChanged =
      !gUiTopBarCache.valid || gUiTopBarCache.menu != ui.menu;
  const bool eStopChanged =
      !gUiTopBarCache.valid || gUiTopBarCache.eStop != d.eStop;
  if (!gUiTopBarCache.valid) {
    tft.fillRect(0, 0, W, TOP_H, C_BG);
  } else if (menuChanged || eStopChanged) {
    tft.fillRect(0, 0, 190, TOP_H, C_BG);
  }

  if (menuChanged || !gUiTopBarCache.valid) {
    if (ui.menu == UiMenuId::OVERVIEW)
      iconGrid(14, 14, C_ACCENT);
    else if (uiIsDomainRoot(ui.menu))
      iconHome(14, 14, C_TEXT);
    else
      iconBack(14, 14, C_TEXT);
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
  (void)ui;
  drawEditSoftKey(0, SoftKey::LEFT, "LEFT", !d.configPending);
  drawEditSoftKey(1, SoftKey::OK, d.configPending ? "WAIT" : "OK",
                  !d.configPending);
  drawEditSoftKey(2, SoftKey::RIGHT, "RIGHT", !d.configPending);
}

inline void drawCarouselFooter(const UiState &ui) {
  uint8_t count = 0U;
  (void)menuChildren(ui.menu, count);
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

  if (redrawStatic || gUiFooterCache.selected != ui.selectedChild ||
      gUiFooterCache.count != count) {
    tft.fillRoundRect(CAROUSEL_PAGE_X, CAROUSEL_NAV_Y, CAROUSEL_PAGE_W,
                      CAROUSEL_NAV_H, 7, C_PANEL_ALT);
    tft.drawRoundRect(CAROUSEL_PAGE_X, CAROUSEL_NAV_Y, CAROUSEL_PAGE_W,
                      CAROUSEL_NAV_H, 7, C_BORDER);
    char pos[20];
    snprintf(pos, sizeof(pos), "%u / %u",
             static_cast<unsigned>(ui.selectedChild + 1U),
             static_cast<unsigned>(count));
    drawUiText(pos, CAROUSEL_PAGE_X + CAROUSEL_PAGE_W / 2, CAROUSEL_NAV_Y + 14,
               C_ACCENT, C_PANEL_ALT, TC_DATUM);
  }
  gUiFooterCache.valid = true;
  gUiFooterCache.kind = UiFooterKind::CAROUSEL;
  gUiFooterCache.selected = ui.selectedChild;
  gUiFooterCache.count = count;
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
  case UiMenuId::ESC_DRIVE:
  case UiMenuId::ESC_DRIVE_LIVE:
  case UiMenuId::ESC_MANUAL_SPEED:
    iconGauge(cx, cy, c, bg);
    break;
  case UiMenuId::ESC_DRIVE_SCALE:
  case UiMenuId::ESC_MODE:
    iconGear(cx, cy, c, bg);
    break;
  case UiMenuId::ESC_POWER:
    iconBolt(cx, cy, c);
    break;
  case UiMenuId::ESC_LINK:
    iconLink(cx, cy, c, bg);
    break;
  case UiMenuId::ESC_MANUAL_TEST:
    iconWrench(cx, cy, c, bg);
    break;
  case UiMenuId::PERCEPTION_ROOT:
  case UiMenuId::PERCEPTION_CAMERA:
    iconCamera(cx, cy, c, bg);
    break;
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
    iconAutoArrow(cx, cy, c, bg);
    break;
  case UiMenuId::SYSTEM_ROOT:
  case UiMenuId::SYSTEM_OVERVIEW:
  case UiMenuId::SYSTEM_LINKS:
  case UiMenuId::SYSTEM_ERRORS:
    iconChip(cx, cy, c, bg);
    break;
  case UiMenuId::SYSTEM_PINS_IO:
  case UiMenuId::SYSTEM_PINS_DISPLAY:
    iconPins(cx, cy, c, bg);
    break;
  case UiMenuId::NAVIGATION_ROOT:
  case UiMenuId::NAV_LOCALIZATION:
  case UiMenuId::NAV_EKF_LOCAL:
  case UiMenuId::NAV_EKF_GLOBAL:
    iconCompass(cx, cy, c, bg);
    break;
  case UiMenuId::NAV_GNSS:
    iconGps(cx, cy, c, bg);
    break;
  case UiMenuId::NAV_MISSION:
  case UiMenuId::NAV_MISSION_GO:
  case UiMenuId::NAV_MISSION_SAVE:
  case UiMenuId::NAV_MISSION_STOP:
    iconAutoArrow(cx, cy, c, bg);
    break;
  case UiMenuId::NAV_SAFETY:
    iconShield(cx, cy, c, bg);
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

inline bool overviewDomainReady(UiMenuId id, const VehicleTelemetry &d) {
  if (id == UiMenuId::ESC_ROOT)
    return d.escReady && d.vescConnected;
  if (id == UiMenuId::PERCEPTION_ROOT)
    return d.perceptionReady;
  if (id == UiMenuId::NAVIGATION_ROOT)
    return d.motionReady && d.nav2Ready;
  if (id == UiMenuId::SYSTEM_ROOT)
    return gDiagnostics.tftOk;
  return false;
}

inline bool overviewDomainFresh(UiMenuId id, const VehicleTelemetry &d) {
  if (id == UiMenuId::ESC_ROOT)
    return d.escFresh;
  if (id == UiMenuId::PERCEPTION_ROOT)
    return d.perceptionFresh;
  if (id == UiMenuId::NAVIGATION_ROOT)
    return d.navigationFresh;
  return true;
}

inline const char *overviewDomainStatus(UiMenuId id,
                                        const VehicleTelemetry &d) {
  if (id == UiMenuId::SYSTEM_ROOT)
    return gDiagnostics.tftOk ? "LOCAL OK" : "DISPLAY ERR";
  if (!overviewDomainFresh(id, d))
    return "STALE";
  return overviewDomainReady(id, d) ? "READY" : "WAIT";
}

inline void drawOverviewDomainCard(int slot, UiMenuId id,
                                   const VehicleTelemetry &d, bool selected) {
  const int x = UI_CARD_X0 + slot * (UI_CARD_W + UI_CARD_GAP);
  const bool ready = overviewDomainReady(id, d);
  const bool fresh = overviewDomainFresh(id, d);
  const uint16_t state = domainHealthColor(ready, fresh);
  const uint16_t border = selected ? C_ACCENT : C_CARD_LINE;
  UiOverviewCardCache &cache = gUiOverviewCache.cards[slot];
  const bool fullCard = !gUiDynamicPass || !cache.valid || cache.id != id ||
                        cache.selected != selected;
  if (fullCard) {
    drawCard(x, OVERVIEW_CARD_Y, UI_CARD_W, OVERVIEW_CARD_H, C_CARD, border,
             false);
    if (selected)
      tft.fillRoundRect(x + 10, OVERVIEW_CARD_Y, UI_CARD_W - 20, 3, 1,
                        C_ACCENT);
    drawMenuNodeIcon(id, x + UI_CARD_W / 2, OVERVIEW_CARD_Y + 27,
                     selected ? C_ACCENT : state, C_CARD);
    drawMenuCardTitle(menuTitle(id), x + UI_CARD_W / 2, OVERVIEW_CARD_Y + 45,
                      C_INK, C_CARD);
  }
  const char *status = overviewDomainStatus(id, d);
  if (fullCard || cache.stateColor != state) {
    tft.fillRoundRect(x, OVERVIEW_CARD_Y, 4, OVERVIEW_CARD_H, 2, state);
    drawStatusDot(x + 14, OVERVIEW_CARD_Y + 80, state, 3);
  }
  if (fullCard ||
      std::strncmp(cache.status, status, sizeof(cache.status)) != 0 ||
      cache.stateColor != state) {
    drawMicroTextPadded(status, x + UI_CARD_W / 2 + 5, OVERVIEW_CARD_Y + 76,
                        state, C_CARD, 72U, TC_DATUM);
  }
  cache.valid = true;
  cache.id = id;
  cache.selected = selected;
  cache.stateColor = state;
  std::snprintf(cache.status, sizeof(cache.status), "%s", status);
}

inline void drawOverviewHealthRail(const VehicleTelemetry &d) {
  const UiMenuId ids[4] = {UiMenuId::ESC_ROOT, UiMenuId::PERCEPTION_ROOT,
                           UiMenuId::NAVIGATION_ROOT, UiMenuId::SYSTEM_ROOT};
  const int gap = 5;
  const int segW = 69;
  for (int i = 0; i < 4; ++i) {
    const uint16_t color = domainHealthColor(overviewDomainReady(ids[i], d),
                                             overviewDomainFresh(ids[i], d));
    if (!gUiDynamicPass || !gUiOverviewCache.valid ||
        gUiOverviewCache.railColors[i] != color) {
      const int x = 16 + i * (segW + gap);
      tft.fillRoundRect(x, OVERVIEW_SUMMARY_Y + OVERVIEW_SUMMARY_H - 6, segW, 3,
                        1, color);
    }
    gUiOverviewCache.railColors[i] = color;
  }
}

inline void drawOverview(const UiState &ui, const VehicleTelemetry &d) {
  if (!gUiDynamicPass)
    drawCard(6, OVERVIEW_SUMMARY_Y, 308, OVERVIEW_SUMMARY_H, C_PANEL, C_BORDER,
             false);
  char line[76];
  snprintf(line, sizeof(line), "%s | %s    %.1f km/h", modeText(d.mode),
           systemStatusText(d.systemStatus), d.speedKmh);
  const uint16_t summaryColor = d.eStop ? C_FAULT : C_TEXT;
  if (!gUiDynamicPass || !gUiOverviewCache.valid ||
      gUiOverviewCache.summaryColor != summaryColor ||
      std::strncmp(gUiOverviewCache.summary, line,
                   sizeof(gUiOverviewCache.summary)) != 0) {
    drawCompactTextPadded(line, 16, OVERVIEW_SUMMARY_Y + 7, summaryColor,
                          C_PANEL, 260U);
  }
  std::snprintf(gUiOverviewCache.summary, sizeof(gUiOverviewCache.summary),
                "%s", line);
  gUiOverviewCache.summaryColor = summaryColor;

  const char *target = navigationHasTarget(d) ? d.activeTarget : "NO TARGET";
  snprintf(line, sizeof(line), "%s/%u SAT | %s > %.18s", gpsFixText(d.gpsFix),
           d.satellites, navigationStatusText(d.navigationStatus), target);
  if (!gUiDynamicPass || !gUiOverviewCache.valid ||
      std::strncmp(gUiOverviewCache.navigation, line,
                   sizeof(gUiOverviewCache.navigation)) != 0) {
    drawMicroTextPadded(line, 16, OVERVIEW_SUMMARY_Y + 27, C_TEXT_DIM, C_PANEL,
                        276U);
  }
  std::snprintf(gUiOverviewCache.navigation,
                sizeof(gUiOverviewCache.navigation), "%s", line);

  const uint16_t dotColor =
      d.eStop ? C_FAULT : systemStatusColor(d.systemStatus);
  if (!gUiDynamicPass || !gUiOverviewCache.valid ||
      gUiOverviewCache.systemDotColor != dotColor) {
    drawStatusDot(297, OVERVIEW_SUMMARY_Y + 14, dotColor, 5);
  }
  gUiOverviewCache.systemDotColor = dotColor;
  drawOverviewHealthRail(d);

  uint8_t count = 0;
  const UiMenuId *children = menuChildren(UiMenuId::OVERVIEW, count);
  const uint8_t first = menuWindowFirst(ui.selectedChild, count);
  for (uint8_t slot = 0; slot < SUBMENU_VISIBLE_CARDS && first + slot < count;
       ++slot) {
    const uint8_t index = static_cast<uint8_t>(first + slot);
    drawOverviewDomainCard(slot, children[index], d, index == ui.selectedChild);
  }
  gUiOverviewCache.valid = true;
}

inline void drawMenuList(const UiState &ui) {
  if (gUiDynamicPass)
    return;
  uint8_t count = 0;
  const UiMenuId *children = menuChildren(ui.menu, count);
  if (children == nullptr || count == 0U)
    return;
  const uint8_t first = menuWindowFirst(ui.selectedChild, count);
  for (uint8_t slot = 0; slot < SUBMENU_VISIBLE_CARDS && first + slot < count;
       ++slot) {
    const uint8_t index = static_cast<uint8_t>(first + slot);
    const int x = UI_CARD_X0 + slot * (UI_CARD_W + UI_CARD_GAP);
    const bool selected = index == ui.selectedChild;
    const uint16_t border = selected ? C_ACCENT : C_BORDER;
    const uint16_t fill = selected ? C_PANEL_ALT : C_PANEL;
    const uint16_t fg = selected ? C_ACCENT : C_TEXT;
    drawCard(x, SUBMENU_CARD_Y, UI_CARD_W, SUBMENU_CARD_H, fill, border, false);
    if (selected)
      tft.fillRoundRect(x, SUBMENU_CARD_Y, 4, SUBMENU_CARD_H, 2, C_ACCENT);
    drawMenuNodeIcon(children[index], x + UI_CARD_W / 2, SUBMENU_CARD_Y + 39,
                     fg, fill);
    drawMenuCardTitle(menuTitle(children[index]), x + UI_CARD_W / 2,
                      SUBMENU_CARD_Y + 72, fg, fill);
    char slotText[12];
    snprintf(slotText, sizeof(slotText), "%u",
             static_cast<unsigned>(index + 1U));
    drawMicroText(slotText, x + UI_CARD_W / 2, SUBMENU_CARD_Y + 115,
                  selected ? C_ACCENT : C_TEXT_DIM, fill, TC_DATUM);
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

inline void drawEscLeaf(UiMenuId id, const UiState &ui,
                        const VehicleTelemetry &d) {
  if (id == UiMenuId::ESC_MANUAL_TEST) {
    drawManualTest(ui, d);
    return;
  }
  drawContentCard();
  char a[30];
  if (id == UiMenuId::ESC_OVERVIEW) {
    snprintf(a, sizeof(a), "%s", d.escReady ? "READY" : "NOT READY");
    drawMetricRow(48, "ESC state", a, healthColor(d.escReady));
    drawMetricFloat(77, "Steering", d.steeringActualDeg, "deg", 1, C_ACCENT);
    drawMetricFloat(106, "Drive", d.driveActualMps, "m/s", 2, C_ACCENT);
    drawMetricRow(135, "F103 link", d.vescConnected ? "ONLINE" : "OFFLINE",
                  healthColor(d.vescConnected));
    return;
  }
  if (id == UiMenuId::ESC_STEERING_LIVE || id == UiMenuId::ESC_STEERING_CAL) {
    drawMetricFloat(48, "Target", d.steeringTargetDeg, "deg", 1, C_ACCENT);
    drawMetricFloat(77, "Actual", d.steeringActualDeg, "deg", 1, C_ACCENT);
    drawMetricFloat(106, "Error", d.steeringErrorDeg, "deg", 1,
                    fabsf(d.steeringErrorDeg) < 2.0F ? C_READY : C_WARNING);
    drawMetricRow(135, "Encoder", d.encoderReady ? "READY" : "OFFLINE",
                  healthColor(d.encoderReady));
    return;
  }
  if (id == UiMenuId::ESC_DRIVE_LIVE) {
    drawMetricFloat(48, "Target", d.driveTargetMps, "m/s", 2, C_ACCENT);
    drawMetricFloat(77, "Actual", d.driveActualMps, "m/s", 2, C_ACCENT);
    snprintf(a, sizeof(a), "%.0f", d.motorErpm);
    drawMetricRow(106, "Electrical RPM", a, C_INK);
    char mechanicalRpm[30];
    snprintf(mechanicalRpm, sizeof(mechanicalRpm), "%.0f", d.motorRpm);
    drawMetricRow(135, "Mechanical RPM", mechanicalRpm, C_INK);
    return;
  }
  if (id == UiMenuId::ESC_POWER) {
    drawMetricRow(48, "Power telemetry",
                  d.vescConnected ? "STREAMING" : "NO LINK",
                  healthColor(d.vescConnected));
    drawMetricRow(77, "Control", "FOC 16 kHz", C_INK);
    drawMetricRow(106, "Watchdog", "300 ms", C_READY);
    drawMetricRow(135, "Fault gate",
                  d.systemStatus == SYS_FAULT ? "FAULT" : "CLEAR",
                  d.systemStatus == SYS_FAULT ? C_FAULT : C_READY);
    return;
  }
  if (id == UiMenuId::ESC_LINK) {
    drawMetricRow(48, "ROS <-> F411", d.rosConnected ? "ONLINE" : "OFFLINE",
                  healthColor(d.rosConnected));
    drawMetricRow(77, "F411 <-> F103", d.vescConnected ? "ONLINE" : "OFFLINE",
                  healthColor(d.vescConnected));
    drawMetricRow(106, "USB CDC", "1,000,000", C_INK);
    drawMetricRow(135, "F103 UART", "115,200", C_INK);
    return;
  }
  drawMetricRow(48, "ESC", d.escReady ? "PASS" : "FAIL",
                healthColor(d.escReady));
  drawMetricRow(77, "Encoder", d.encoderReady ? "PASS" : "FAIL",
                healthColor(d.encoderReady));
  drawMetricRow(106, "E-stop", d.eStop ? "FAIL" : "PASS",
                d.eStop ? C_FAULT : C_READY);
  drawMetricRow(135, "Vehicle", d.state == STATE_STOPPED ? "STOPPED" : "MOVING",
                d.state == STATE_STOPPED ? C_READY : C_WARNING);
}

inline void drawPerceptionLeaf(UiMenuId id, const UiState &ui,
                               const VehicleTelemetry &d) {
  drawContentCard();
  char text[32];
  if (id == UiMenuId::PERCEPTION_OVERVIEW ||
      id == UiMenuId::PERCEPTION_CAMERA) {
    drawMetricRow(48, "Camera", d.cameraReady ? "READY" : "OFFLINE",
                  healthColor(d.cameraReady));
    drawMetricRow(77, "Perception", d.perceptionReady ? "HEALTHY" : "WAIT",
                  healthColor(d.perceptionReady));
    snprintf(text, sizeof(text), "%.1f FPS", d.cameraFps);
    drawMetricRow(106, "Pipeline", text,
                  d.cameraFps > 1.0F ? C_READY : C_WARNING);
    drawMetricRow(135, "Inference", d.perceptionInference ? "ON" : "OFF",
                  d.perceptionInference ? C_READY : C_WARNING);
    return;
  }
  if (id == UiMenuId::PERCEPTION_DETECTION_LIVE) {
    drawMetricRow(48, "Object", d.detectedObject, C_ACCENT);
    drawMetricFloat(77, "Distance", d.objectDistanceM, "m", 2, C_INK);
    snprintf(text, sizeof(text), "%.0f %%", d.confidencePct);
    drawMetricRow(106, "Confidence", text, C_INK);
    drawMetricRow(135, "Obstacle", d.obstacleDetected ? "DETECTED" : "CLEAR",
                  d.obstacleDetected ? C_FAULT : C_READY);
    return;
  }
  if (id == UiMenuId::PERCEPTION_LANE) {
    drawMetricRow(48, "Lane state", d.laneState,
                  strcmp(d.laneState, "CLEAR") == 0 ? C_READY : C_WARNING);
    drawMetricRow(77, "Drivable", d.drivableAreaClear ? "CLEAR" : "BLOCKED",
                  d.drivableAreaClear ? C_READY : C_FAULT);
    drawMetricRow(106, "Camera metric", d.perceptionReady ? "ACTIVE" : "WAIT",
                  d.perceptionReady ? C_READY : C_WARNING);
    drawMetricRow(135, "Safety action", d.eStop ? "STOP" : "MONITOR",
                  d.eStop ? C_FAULT : C_INK);
    return;
  }
  if (id == UiMenuId::PERCEPTION_OBSTACLE) {
    drawMetricRow(48, "Nearest", d.detectedObject, C_ACCENT);
    drawMetricFloat(77, "Forward", d.objectDistanceM, "m", 2, C_INK);
    drawMetricRow(106, "Path", d.drivableAreaClear ? "CLEAR" : "BLOCKED",
                  d.drivableAreaClear ? C_READY : C_FAULT);
    drawMetricRow(135, "Emergency", d.obstacleDetected ? "CHECK" : "CLEAR",
                  d.obstacleDetected ? C_WARNING : C_READY);
    return;
  }
  if (id == UiMenuId::PERCEPTION_PERFORMANCE) {
    snprintf(text, sizeof(text), "%.1f FPS", d.cameraFps);
    drawMetricRow(48, "Inference rate", text,
                  d.cameraFps > 1.0F ? C_READY : C_WARNING);
    drawMetricRow(77, "Camera link", d.cameraReady ? "ONLINE" : "OFFLINE",
                  healthColor(d.cameraReady));
    drawMetricRow(106, "Health", d.perceptionReady ? "GOOD" : "BAD",
                  healthColor(d.perceptionReady));
    drawMetricRow(135, "ROS stream", d.rosConnected ? "LIVE" : "OFFLINE",
                  healthColor(d.rosConnected));
    return;
  }
  drawMetricRow(48, "Camera", d.cameraReady ? "PASS" : "FAIL",
                healthColor(d.cameraReady));
  drawMetricRow(77, "Inference", d.perceptionInference ? "PASS" : "OFF",
                d.perceptionInference ? C_READY : C_WARNING);
  drawMetricRow(106, "Drivable", d.drivableAreaClear ? "PASS" : "CHECK",
                d.drivableAreaClear ? C_READY : C_WARNING);
  drawMetricRow(135, "Obstacle gate", d.eStop ? "STOP" : "PASS",
                d.eStop ? C_FAULT : C_READY);
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
  char a[44];
  if (id == UiMenuId::SYSTEM_OVERVIEW) {
    snprintf(
        a, sizeof(a), "0x%04lX %s",
        static_cast<unsigned long>(gDiagnostics.tftControllerId & 0xFFFFUL),
        gDiagnostics.tftOk ? "OK" : "FAULT");
    drawMetricRow(48, "TFT / display", a,
                  gDiagnostics.tftOk ? C_READY : C_FAULT);
    snprintf(a, sizeof(a), "%lu / %lu ms",
             static_cast<unsigned long>(gDiagnostics.uiDrawLastMs),
             static_cast<unsigned long>(gDiagnostics.uiDrawMaxMs));
    drawMetricRow(77, "UI last / max", a,
                  gDiagnostics.uiDrawLastMs <= 50U ? C_READY : C_WARNING);
    snprintf(a, sizeof(a), "%lu / %lu B",
             static_cast<unsigned long>(gDiagnostics.displayBytesLastFrame),
             static_cast<unsigned long>(gDiagnostics.displayBytesMaxFrame));
    drawMetricRow(106, "Frame last / max", a, C_ACCENT);
    snprintf(a, sizeof(a), "%lu/%lu/%lu ms",
             static_cast<unsigned long>(gDiagnostics.serviceGapP95Ms),
             static_cast<unsigned long>(gDiagnostics.serviceGapP99Ms),
             static_cast<unsigned long>(gDiagnostics.maxServiceGapMs));
    drawMetricRow(135, "Svc p95/p99/max", a,
                  gDiagnostics.maxServiceGapMs <= 20U ? C_READY : C_WARNING);
    return;
  }
  if (id == UiMenuId::SYSTEM_PINS_IO) {
    snprintf(a, sizeof(a), "%c/%c %s", gDiagnostics.pb6VescTx ? 'H' : 'L',
             gDiagnostics.pb7VescRx ? 'H' : 'L',
             gDiagnostics.vescUartOk ? "OK" : "ERR");
    drawMetricRow(48, "PB6/PB7 VESC", a,
                  gDiagnostics.vescUartOk ? C_READY : C_WARNING);
    snprintf(a, sizeof(a), "%c/%c %s", gDiagnostics.pa2GnssTx ? 'H' : 'L',
             gDiagnostics.pa3GnssRx ? 'H' : 'L',
             gDiagnostics.gnssUartOk ? "OK" : "ERR");
    drawMetricRow(77, "PA2/PA3 GNSS", a,
                  gDiagnostics.gnssUartOk ? C_READY : C_WARNING);
    snprintf(a, sizeof(a), "%c/%c %s", gDiagnostics.pb8I2cScl ? 'H' : 'L',
             gDiagnostics.pb9I2cSda ? 'H' : 'L',
             gDiagnostics.magOk ? "MAG OK" : "MAG ERR");
    drawMetricRow(106, "PB8/PB9 I2C1", a,
                  gDiagnostics.magOk ? C_READY : C_WARNING);
    snprintf(a, sizeof(a), "%s/%c/%c",
             gDiagnostics.pb12Safety ? "CLR" : "PRESS",
             gDiagnostics.pb13SafetyLed ? 'H' : 'L',
             gDiagnostics.pa8Buzzer ? 'H' : 'L');
    drawMetricRow(135, "PB12/13 + PA8", a,
                  gDiagnostics.pb12Safety ? C_READY : C_FAULT);
    return;
  }
  if (id == UiMenuId::SYSTEM_PINS_DISPLAY) {
    snprintf(a, sizeof(a), "S:%c I:%c O:%c", gDiagnostics.pa5SpiSck ? 'H' : 'L',
             gDiagnostics.pa6SpiMiso ? 'H' : 'L',
             gDiagnostics.pa7SpiMosi ? 'H' : 'L');
    drawMetricRow(48, "PA5/6/7 SPI1", a, C_INK);
    snprintf(
        a, sizeof(a), "CS:%c DC:%c RST:%c", gDiagnostics.pb0TftCs ? 'H' : 'L',
        gDiagnostics.pb1TftDc ? 'H' : 'L', gDiagnostics.pb2TftRst ? 'H' : 'L');
    drawMetricRow(77, "PB0/1/2 TFT", a, gDiagnostics.tftOk ? C_READY : C_FAULT);
    snprintf(a, sizeof(a), "CS:%c R:%lu F:%lu",
             gDiagnostics.pa4TouchCs ? 'H' : 'L',
             static_cast<unsigned long>(gDiagnostics.touchReadCount),
             static_cast<unsigned long>(gDiagnostics.touchRejectFastCount));
    drawMetricRow(106, "PA4 TOUCH", a, C_ACCENT);
    snprintf(a, sizeof(a), "DM:%c DP:%c CDC",
             gDiagnostics.pa11UsbDm ? 'H' : 'L',
             gDiagnostics.pa12UsbDp ? 'H' : 'L');
    drawMetricRow(135, "PA11/12 USB", a, d.rosConnected ? C_READY : C_WARNING);
    return;
  }
  if (id == UiMenuId::SYSTEM_LINKS) {
    formatAgeMs(gDiagnostics.rosHeartbeatAgeMs, a, sizeof(a));
    drawMetricRow(48, "ROS heartbeat", a,
                  d.rosConnected ? ageColor(gDiagnostics.rosHeartbeatAgeMs)
                                 : C_FAULT);
    char hostAge[18], f103Age[18];
    formatAgeMs(d.escAgeMs, hostAge, sizeof(hostAge));
    formatAgeMs(gDiagnostics.vescLastFrameAgeMs, f103Age, sizeof(f103Age));
    snprintf(a, sizeof(a), "%s / %s", hostAge, f103Age);
    drawMetricRow(77, "ESC host / F103", a,
                  (ageColor(d.escAgeMs) == C_READY &&
                   ageColor(gDiagnostics.vescLastFrameAgeMs) == C_READY)
                      ? C_READY
                      : C_WARNING);
    formatAgeMs(d.perceptionAgeMs, a, sizeof(a));
    drawMetricRow(106, "Perception stream", a, ageColor(d.perceptionAgeMs));
    formatAgeMs(d.navigationAgeMs, a, sizeof(a));
    drawMetricRow(135, "Navigation stream", a, ageColor(d.navigationAgeMs));
    return;
  }
  if (id == UiMenuId::SYSTEM_ERRORS) {
    snprintf(a, sizeof(a), "E:%lu O:%lu D:%lu",
             static_cast<unsigned long>(gDiagnostics.vescUartErrors),
             static_cast<unsigned long>(gDiagnostics.vescUartOverflow),
             static_cast<unsigned long>(gDiagnostics.vescUartTxDropped));
    drawMetricRow(48, "VESC UART", a,
                  (gDiagnostics.vescUartErrors ||
                   gDiagnostics.vescUartOverflow ||
                   gDiagnostics.vescUartTxDropped)
                      ? C_WARNING
                      : C_READY);
    snprintf(a, sizeof(a), "E:%lu O:%lu M:%lu",
             static_cast<unsigned long>(gDiagnostics.gnssUartErrors),
             static_cast<unsigned long>(gDiagnostics.gnssUartOverflow),
             static_cast<unsigned long>(gDiagnostics.magErrors));
    drawMetricRow(77, "GNSS / MAG", a,
                  (gDiagnostics.gnssUartErrors ||
                   gDiagnostics.gnssUartOverflow || gDiagnostics.magErrors)
                      ? C_WARNING
                      : C_READY);
    snprintf(a, sizeof(a), "T:%lu H:%lu R:%lu B:%lu",
             static_cast<unsigned long>(gDiagnostics.spiTimeoutCount),
             static_cast<unsigned long>(gDiagnostics.spiHalErrorCount),
             static_cast<unsigned long>(gDiagnostics.spiRecoveryCount),
             static_cast<unsigned long>(gDiagnostics.spiBusConflictCount));
    drawMetricRow(106, "SPI fault / recovery", a,
                  (gDiagnostics.spiTimeoutCount ||
                   gDiagnostics.spiHalErrorCount ||
                   gDiagnostics.spiBusConflictCount)
                      ? C_WARNING
                      : C_READY);
    char hostErrors[44];
    snprintf(hostErrors, sizeof(hostErrors), "VF:%lu/%lu H:%lu/%lu",
             static_cast<unsigned long>(gDiagnostics.vescFrameErrors),
             static_cast<unsigned long>(gDiagnostics.vescRecoveryCount),
             static_cast<unsigned long>(gDiagnostics.unknownCommands),
             static_cast<unsigned long>(gDiagnostics.overlongCommands));
    drawMetricRow(135, "VESC frame / host", hostErrors,
                  (gDiagnostics.vescFrameErrors ||
                   gDiagnostics.unknownCommands ||
                   gDiagnostics.overlongCommands)
                      ? C_WARNING
                      : C_READY);
    return;
  }
}

inline void drawNavigationLeaf(UiMenuId id, const UiState &ui,
                               const VehicleTelemetry &d) {
  drawContentCard();
  char text[40];
  if (id == UiMenuId::NAVIGATION_OVERVIEW) {
    drawMetricRow(48, "Localization", d.localizationState,
                  d.motionReady ? C_READY : C_WARNING);
    drawMetricRow(77, "Nav2", d.nav2Ready ? "READY" : "WAIT",
                  healthColor(d.nav2Ready));
    drawMetricRow(106, "Motion gate", d.motionReady ? "OPEN" : "LOCKED",
                  d.motionReady ? C_READY : C_FAULT);
    drawMetricRow(135, "Mission", navigationStatusText(d.navigationStatus),
                  C_ACCENT);
    return;
  }
  if (id == UiMenuId::NAV_GNSS) {
    snprintf(text, sizeof(text), "%s / %u SAT", gpsFixText(d.gpsFix),
             d.satellites);
    drawMetricRow(48, "Fix", text, gpsFixColor(d.gpsFix));
    drawMetricFloat(77, "hAcc", d.haccM, "m", 2,
                    d.haccM < 2.5F ? C_READY : C_WARNING);
    drawMetricFloat(106, "Age", d.gnssAgeSec, "s", 2,
                    d.gnssAgeSec < 0.5F ? C_READY : C_WARNING);
    snprintf(text, sizeof(text), "%.0f deg", d.headingDeg);
    drawMetricRow(135, "Heading", text, C_INK);
    return;
  }
  if (id == UiMenuId::NAV_IMU) {
    drawMetricRow(48, "IMU", d.imuStatus, healthColor(d.imuReady));
    drawMetricFloat(77, "Gyro Z", d.gyroZRps, "rad/s", 2, C_INK);
    snprintf(text, sizeof(text), "%.0f deg", d.headingDeg);
    drawMetricRow(106, "Heading", text, C_INK);
    drawMetricRow(135, "Fusion", d.motionReady ? "ACTIVE" : "WAIT",
                  d.motionReady ? C_READY : C_WARNING);
    return;
  }
  if (id == UiMenuId::NAV_MAG) {
    drawMetricRow(48, "IST8310", d.magReady ? "READY" : "OFFLINE",
                  healthColor(d.magReady));
    drawMetricRow(77, "Heading source", d.gnssStatus, C_INK);
    snprintf(text, sizeof(text), "%.0f deg", d.headingDeg);
    drawMetricRow(106, "Vehicle yaw", text, C_ACCENT);
    drawMetricRow(135, "Calibration", d.magReady ? "AVAILABLE" : "REQUIRED",
                  d.magReady ? C_READY : C_WARNING);
    return;
  }
  if (id == UiMenuId::NAV_EKF_LOCAL || id == UiMenuId::NAV_EKF_GLOBAL) {
    const char *status =
        id == UiMenuId::NAV_EKF_LOCAL ? d.ekfLocalStatus : d.ekfGlobalStatus;
    drawMetricRow(
        48, id == UiMenuId::NAV_EKF_LOCAL ? "EKF local" : "EKF global", status,
        strstr(status, "READY") != nullptr ? C_READY : C_WARNING);
    drawMetricRow(77, "Localization", d.localizationState,
                  d.motionReady ? C_READY : C_WARNING);
    drawMetricFloat(106, "Vehicle speed", d.driveActualMps, "m/s", 2, C_INK);
    snprintf(text, sizeof(text), "%.0f deg", d.headingDeg);
    drawMetricRow(135, "Yaw", text, C_ACCENT);
    return;
  }
  if (id == UiMenuId::NAV_MISSION_GO || id == UiMenuId::NAV_MISSION_SAVE ||
      id == UiMenuId::NAV_MISSION_STOP) {
    const uint8_t index =
        d.selectedWaypoint < HMI_WAYPOINT_COUNT ? d.selectedWaypoint : 0;
    drawMetricRow(48, "Waypoint", d.waypointName[index],
                  d.waypointSaved[index] ? C_READY : C_WARNING);
    drawMetricRow(77, "Saved", d.waypointSaved[index] ? "YES" : "NO",
                  d.waypointSaved[index] ? C_READY : C_WARNING);
    drawMetricRow(106, "Navigation", navigationStatusText(d.navigationStatus),
                  C_ACCENT);
    if (id == UiMenuId::NAV_MISSION_GO) {
      drawMetricRow(135, "Action",
                    d.mode == MODE_AUTO ? "GO ON OK" : "AUTO REQUIRED",
                    d.mode == MODE_AUTO ? C_READY : C_FAULT);
      if (!gUiDynamicPass)
        drawMicroText("LEFT/RIGHT pilih waypoint", 18, 166, C_DISABLED, C_CARD);
    } else if (id == UiMenuId::NAV_MISSION_SAVE) {
      drawMetricRow(135, "Action",
                    d.gpsReady && d.state == STATE_STOPPED
                        ? "SAVE ON OK"
                        : "GPS/STOP REQUIRED",
                    d.gpsReady && d.state == STATE_STOPPED ? C_READY : C_FAULT);
      if (!gUiDynamicPass)
        drawMicroText("Simpan pose map + GNSS saat ini", 18, 166, C_DISABLED,
                      C_CARD);
    } else {
      drawMetricRow(135, "Action", "STOP ON OK", C_FAULT);
      if (!gUiDynamicPass)
        drawMicroText("STOP selalu boleh dipanggil", 18, 166, C_DISABLED,
                      C_CARD);
    }
    return;
  }
  if (id == UiMenuId::NAV_PLANNER || id == UiMenuId::NAV_MPPI ||
      id == UiMenuId::NAV_SMOOTHER || id == UiMenuId::NAV_COSTMAP) {
    // Jangan tampilkan parameter Nav2 hardcoded: seluruh angka harus berasal
    // dari runtime.
    drawMetricRow(48, "Nav2 state", d.nav2Ready ? "READY" : "WAIT",
                  healthColor(d.nav2Ready));
    drawMetricRow(77, "Localization", d.localizationState,
                  d.motionReady ? C_READY : C_WARNING);
    if (id == UiMenuId::NAV_COSTMAP) {
      drawMetricRow(106, "Obstacle", d.obstacleDetected ? "ACTIVE" : "CLEAR",
                    d.obstacleDetected ? C_WARNING : C_READY);
      drawMetricRow(135, "Lane", d.laneState,
                    strcmp(d.laneState, "CLEAR") == 0 ? C_READY : C_WARNING);
    } else {
      drawMetricFloat(106, "Cmd speed", d.driveTargetMps, "m/s", 2, C_ACCENT);
      drawMetricFloat(135, "Actual speed", d.driveActualMps, "m/s", 2, C_INK);
    }
    return;
  }
  if (id == UiMenuId::NAV_SAFETY) {
    drawMetricRow(48, "E-stop", d.eStop ? "ACTIVE" : "CLEAR",
                  d.eStop ? C_FAULT : C_READY);
    drawMetricRow(77, "Localization", d.motionReady ? "PASS" : "LOCKED",
                  d.motionReady ? C_READY : C_FAULT);
    drawMetricRow(106, "Nav2", d.nav2Ready ? "PASS" : "WAIT",
                  healthColor(d.nav2Ready));
    drawMetricRow(
        135, "Autonomy",
        (!d.eStop && d.motionReady && d.nav2Ready) ? "ALLOWED" : "LOCKED",
        (!d.eStop && d.motionReady && d.nav2Ready) ? C_READY : C_FAULT);
    return;
  }
  drawMetricRow(48, "GNSS", d.gpsReady ? "PASS" : "FAIL",
                healthColor(d.gpsReady));
  drawMetricRow(77, "IMU", d.imuReady ? "PASS" : "FAIL",
                healthColor(d.imuReady));
  drawMetricRow(106, "Nav2", d.nav2Ready ? "PASS" : "FAIL",
                healthColor(d.nav2Ready));
  drawMetricRow(135, "Safety", d.eStop ? "FAIL" : "PASS",
                d.eStop ? C_FAULT : C_READY);
}

inline bool isEscMenu(UiMenuId id) {
  const uint8_t value = static_cast<uint8_t>(id);
  return value >= static_cast<uint8_t>(UiMenuId::ESC_ROOT) &&
         value <= static_cast<uint8_t>(UiMenuId::ESC_MANUAL_TEST);
}
inline bool isPerceptionMenu(UiMenuId id) {
  const uint8_t value = static_cast<uint8_t>(id);
  return value >= static_cast<uint8_t>(UiMenuId::PERCEPTION_ROOT) &&
         value <= static_cast<uint8_t>(UiMenuId::PERCEPTION_TEST);
}
inline bool isSystemMenu(UiMenuId id) {
  const uint8_t value = static_cast<uint8_t>(id);
  return value >= static_cast<uint8_t>(UiMenuId::SYSTEM_ROOT) &&
         value <= static_cast<uint8_t>(UiMenuId::SYSTEM_ERRORS);
}
inline bool isNavigationMenu(UiMenuId id) {
  const uint8_t value = static_cast<uint8_t>(id);
  return value >= static_cast<uint8_t>(UiMenuId::NAVIGATION_ROOT) &&
         value <= static_cast<uint8_t>(UiMenuId::NAV_TEST);
}

inline bool uiNeedsEditFooter(const UiState &ui) {
  return menuEditKey(ui.menu) != UiEditKey::NONE ||
         ui.menu == UiMenuId::NAV_MISSION_GO ||
         ui.menu == UiMenuId::NAV_MISSION_SAVE ||
         ui.menu == UiMenuId::NAV_MISSION_STOP;
}

inline void clearCarouselBackgroundGaps() {
  tft.fillRect(0, CAROUSEL_NAV_Y, 4, CAROUSEL_NAV_H, C_BG);
  tft.fillRect(CAROUSEL_LEFT_X + CAROUSEL_NAV_W, CAROUSEL_NAV_Y, 6,
               CAROUSEL_NAV_H, C_BG);
  tft.fillRect(CAROUSEL_PAGE_X + CAROUSEL_PAGE_W, CAROUSEL_NAV_Y, 6,
               CAROUSEL_NAV_H, C_BG);
  tft.fillRect(CAROUSEL_RIGHT_X + CAROUSEL_NAV_W, CAROUSEL_NAV_Y,
               W - (CAROUSEL_RIGHT_X + CAROUSEL_NAV_W), CAROUSEL_NAV_H, C_BG);
  tft.fillRect(0, CAROUSEL_NAV_Y + CAROUSEL_NAV_H, W,
               H - (CAROUSEL_NAV_Y + CAROUSEL_NAV_H), C_BG);
}

inline void clearEditFooterBackgroundGaps() {
  const int x1 = softKeyX(0) + SOFTKEY_W;
  const int x2 = softKeyX(1) + SOFTKEY_W;
  const int x3 = softKeyX(2) + SOFTKEY_W;
  tft.fillRect(0, SOFTKEY_Y, SOFTKEY_X0, SOFTKEY_H, C_BG);
  tft.fillRect(x1, SOFTKEY_Y, softKeyX(1) - x1, SOFTKEY_H, C_BG);
  tft.fillRect(x2, SOFTKEY_Y, softKeyX(2) - x2, SOFTKEY_H, C_BG);
  tft.fillRect(x3, SOFTKEY_Y, W - x3, SOFTKEY_H, C_BG);
  tft.fillRect(0, SOFTKEY_Y + SOFTKEY_H, W, H - (SOFTKEY_Y + SOFTKEY_H), C_BG);
}

inline void clearFullFrameBackground(const UiState &ui) {
  /* Do not clear pixels that the new page will immediately overwrite. This
   * keeps page transitions deterministic while avoiding the old 153.6-kB
   * full-screen pre-clear. */
  if (ui.menu == UiMenuId::ESC_MANUAL_TEST) {
    tft.fillRect(0, TOP_H, W, H - TOP_H, C_BG);
    return;
  }

  if (ui.menu == UiMenuId::OVERVIEW) {
    tft.fillRect(0, TOP_H, W, OVERVIEW_SUMMARY_Y - TOP_H, C_BG);
    tft.fillRect(0, OVERVIEW_SUMMARY_Y, 6, OVERVIEW_SUMMARY_H, C_BG);
    tft.fillRect(314, OVERVIEW_SUMMARY_Y, 6, OVERVIEW_SUMMARY_H, C_BG);
    tft.fillRect(0, OVERVIEW_SUMMARY_Y + OVERVIEW_SUMMARY_H, W,
                 OVERVIEW_CARD_Y - (OVERVIEW_SUMMARY_Y + OVERVIEW_SUMMARY_H),
                 C_BG);
    tft.fillRect(0, OVERVIEW_CARD_Y, 6, OVERVIEW_CARD_H, C_BG);
    tft.fillRect(106, OVERVIEW_CARD_Y, 4, OVERVIEW_CARD_H, C_BG);
    tft.fillRect(210, OVERVIEW_CARD_Y, 4, OVERVIEW_CARD_H, C_BG);
    tft.fillRect(314, OVERVIEW_CARD_Y, 6, OVERVIEW_CARD_H, C_BG);
    tft.fillRect(0, OVERVIEW_CARD_Y + OVERVIEW_CARD_H, W,
                 CAROUSEL_NAV_Y - (OVERVIEW_CARD_Y + OVERVIEW_CARD_H), C_BG);
    clearCarouselBackgroundGaps();
    return;
  }

  if (menuHasChildren(ui.menu)) {
    tft.fillRect(0, TOP_H, W, SUBMENU_CARD_Y - TOP_H, C_BG);
    tft.fillRect(0, SUBMENU_CARD_Y, 6, SUBMENU_CARD_H, C_BG);
    tft.fillRect(106, SUBMENU_CARD_Y, 4, SUBMENU_CARD_H, C_BG);
    tft.fillRect(210, SUBMENU_CARD_Y, 4, SUBMENU_CARD_H, C_BG);
    tft.fillRect(314, SUBMENU_CARD_Y, 6, SUBMENU_CARD_H, C_BG);
    uint8_t count = 0U;
    (void)menuChildren(ui.menu, count);
    for (uint8_t slot = count; slot < SUBMENU_VISIBLE_CARDS; ++slot) {
      const int x = UI_CARD_X0 + slot * (UI_CARD_W + UI_CARD_GAP);
      tft.fillRect(x, SUBMENU_CARD_Y, UI_CARD_W, SUBMENU_CARD_H, C_BG);
    }
    tft.fillRect(0, SUBMENU_CARD_Y + SUBMENU_CARD_H, W,
                 CAROUSEL_NAV_Y - (SUBMENU_CARD_Y + SUBMENU_CARD_H), C_BG);
    clearCarouselBackgroundGaps();
    return;
  }

  tft.fillRect(0, TOP_H, W, CONTENT_Y - TOP_H, C_BG);
  tft.fillRect(0, CONTENT_Y, 6, CONTENT_BOTTOM - CONTENT_Y + 1, C_BG);
  tft.fillRect(314, CONTENT_Y, 6, CONTENT_BOTTOM - CONTENT_Y + 1, C_BG);
  tft.fillRect(0, CONTENT_BOTTOM + 1, W, SOFTKEY_Y - (CONTENT_BOTTOM + 1),
               C_BG);
  if (uiNeedsEditFooter(ui))
    clearEditFooterBackgroundGaps();
  else
    tft.fillRect(0, SOFTKEY_Y, W, H - SOFTKEY_Y, C_BG);
}

inline void drawUiContent(const UiState &ui, const VehicleTelemetry &d) {
  // Full frames already start with fillScreen(); dynamic frames repaint only
  // bounded value regions. Never clear the whole content area here.
  if (ui.menu == UiMenuId::OVERVIEW) {
    drawOverview(ui, d);
  } else if (menuEditKey(ui.menu) != UiEditKey::NONE) {
    drawEditor(ui, d, menuEditKey(ui.menu));
  } else if (menuHasChildren(ui.menu)) {
    drawMenuList(ui);
  } else if (isEscMenu(ui.menu)) {
    drawEscLeaf(ui.menu, ui, d);
  } else if (isPerceptionMenu(ui.menu)) {
    drawPerceptionLeaf(ui.menu, ui, d);
  } else if (isSystemMenu(ui.menu)) {
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
    if (menuHasChildren(ui.menu)) {
      drawCarouselFooter(ui);
    } else if (uiNeedsEditFooter(ui)) {
      drawEditFooter(ui, d);
      gUiFooterCache.valid = true;
      gUiFooterCache.kind = UiFooterKind::EDIT;
      gUiFooterCache.configPending = d.configPending;
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
