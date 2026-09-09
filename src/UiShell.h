// ============================================================================
// UiShell.h — renderer tunggal untuk OVERVIEW / ESC / PERCEPTION / NAVIGATION.
// ============================================================================
#pragma once

#include "HmiDisplay.h"
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "Config.h"
#include "Telemetry.h"
#include "Theme.h"
#include "Icons.h"
#include "UiMenu.h"

extern HmiDisplay tft;

inline void drawDomainDot(int x, const char* label, bool ok) {
  drawStatusDot(x, 14, healthColor(ok), 3);
  drawMicroText(label, x + 7, 10, C_TEXT_DIM, C_BG);
}

inline bool uiIsDomainRoot(UiMenuId id) {
  return id == UiMenuId::ESC_ROOT || id == UiMenuId::PERCEPTION_ROOT || id == UiMenuId::NAVIGATION_ROOT;
}

inline void drawUiTopBar(const UiState& ui, const VehicleTelemetry& d) {
  tft.fillRect(0, 0, W, TOP_H, C_BG);
  if (ui.menu == UiMenuId::OVERVIEW) {
    iconGrid(14, 14, C_ACCENT);
  } else if (uiIsDomainRoot(ui.menu)) {
    iconHome(14, 14, C_TEXT);
  } else {
    iconBack(14, 14, C_TEXT);
  }
  tft.drawFastVLine(28, 6, 17, C_BORDER);
  char title[28];
  snprintf(title, sizeof(title), "%.18s", menuTitle(ui.menu));
  drawCompactText(title, 36, 7, C_TEXT, C_BG);
  drawDomainDot(205, "E", d.escReady && d.vescConnected);
  drawDomainDot(244, "P", d.perceptionReady);
  drawDomainDot(283, "N", d.nav2Ready && d.motionReady);
  if (d.eStop) {
    tft.fillRoundRect(126, 5, 67, 20, 4, C_FAULT);
    drawMicroText("E-STOP", 159, 10, C_WHITE, C_FAULT, TC_DATUM);
  }
}

inline int softKeyX(uint8_t index) { return SOFTKEY_X0 + index * (SOFTKEY_W + SOFTKEY_GAP); }

inline void drawEditSoftKey(uint8_t index, SoftKey key, const char* label, bool active = true) {
  const int x = softKeyX(index);
  const uint16_t border = active ? (key == SoftKey::OK ? C_ACCENT : C_BORDER) : C_BORDER;
  const uint16_t fg = active ? (key == SoftKey::OK ? C_ACCENT : C_TEXT) : C_DISABLED;
  tft.fillRoundRect(x, SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H, 7, C_PANEL);
  tft.drawRoundRect(x, SOFTKEY_Y, SOFTKEY_W, SOFTKEY_H, 7, border);
  const int cx = x + SOFTKEY_W / 2;
  if (key == SoftKey::LEFT) iconMinus(cx, SOFTKEY_Y + 17, fg);
  else if (key == SoftKey::RIGHT) iconPlus(cx, SOFTKEY_Y + 17, fg);
  else if (key == SoftKey::OK) iconCheck(cx, SOFTKEY_Y + 17, fg);
  drawCompactText(label, cx, SOFTKEY_Y + 31, fg, C_PANEL, TC_DATUM);
}

inline void drawEditFooter(const UiState& ui, const VehicleTelemetry& d) {
  (void)ui;
  drawEditSoftKey(0, SoftKey::LEFT, "LEFT", !d.configPending);
  drawEditSoftKey(1, SoftKey::OK, d.configPending ? "WAIT" : "OK", !d.configPending);
  drawEditSoftKey(2, SoftKey::RIGHT, "RIGHT", !d.configPending);
}

inline void drawCarouselFooter(const UiState& ui) {
  uint8_t count = 0;
  (void)menuChildren(ui.menu, count);
  tft.fillRoundRect(CAROUSEL_LEFT_X, CAROUSEL_NAV_Y, CAROUSEL_NAV_W, CAROUSEL_NAV_H, 7, C_PANEL);
  tft.drawRoundRect(CAROUSEL_LEFT_X, CAROUSEL_NAV_Y, CAROUSEL_NAV_W, CAROUSEL_NAV_H, 7, C_BORDER);
  iconArrowLeft(CAROUSEL_LEFT_X + CAROUSEL_NAV_W / 2, CAROUSEL_NAV_Y + 18, C_TEXT);
  drawMicroText("LEFT", CAROUSEL_LEFT_X + CAROUSEL_NAV_W / 2, CAROUSEL_NAV_Y + 33, C_TEXT_DIM, C_PANEL, TC_DATUM);

  tft.fillRoundRect(CAROUSEL_PAGE_X, CAROUSEL_NAV_Y, CAROUSEL_PAGE_W, CAROUSEL_NAV_H, 7, C_PANEL_ALT);
  tft.drawRoundRect(CAROUSEL_PAGE_X, CAROUSEL_NAV_Y, CAROUSEL_PAGE_W, CAROUSEL_NAV_H, 7, C_BORDER);
  char pos[20];
  snprintf(pos, sizeof(pos), "%u / %u", static_cast<unsigned>(ui.selectedChild + 1U), static_cast<unsigned>(count));
  drawUiText(pos, CAROUSEL_PAGE_X + CAROUSEL_PAGE_W / 2, CAROUSEL_NAV_Y + 14, C_ACCENT, C_PANEL_ALT, TC_DATUM);

  tft.fillRoundRect(CAROUSEL_RIGHT_X, CAROUSEL_NAV_Y, CAROUSEL_NAV_W, CAROUSEL_NAV_H, 7, C_PANEL);
  tft.drawRoundRect(CAROUSEL_RIGHT_X, CAROUSEL_NAV_Y, CAROUSEL_NAV_W, CAROUSEL_NAV_H, 7, C_BORDER);
  iconArrowRight(CAROUSEL_RIGHT_X + CAROUSEL_NAV_W / 2, CAROUSEL_NAV_Y + 18, C_TEXT);
  drawMicroText("RIGHT", CAROUSEL_RIGHT_X + CAROUSEL_NAV_W / 2, CAROUSEL_NAV_Y + 33, C_TEXT_DIM, C_PANEL, TC_DATUM);
}

inline void drawContentCard() {
  drawCard(6, CONTENT_Y, 308, CONTENT_BOTTOM - CONTENT_Y + 1, C_CARD, C_CARD_LINE, false);
}

inline void drawMenuNodeIcon(UiMenuId id, int cx, int cy, uint16_t c, uint16_t bg) {
  switch (id) {
    case UiMenuId::ESC_ROOT:
    case UiMenuId::ESC_STEERING:
    case UiMenuId::ESC_STEERING_LIVE:
    case UiMenuId::ESC_STEERING_TEST_ANGLE:
    case UiMenuId::ESC_STEERING_CAL: iconSteering(cx, cy, c, bg); break;
    case UiMenuId::ESC_DRIVE:
    case UiMenuId::ESC_DRIVE_LIVE:
    case UiMenuId::ESC_MANUAL_SPEED: iconGauge(cx, cy, c, bg); break;
    case UiMenuId::ESC_DRIVE_SCALE:
    case UiMenuId::ESC_MODE: iconGear(cx, cy, c, bg); break;
    case UiMenuId::ESC_POWER: iconBolt(cx, cy, c); break;
    case UiMenuId::ESC_LINK: iconLink(cx, cy, c, bg); break;
    case UiMenuId::ESC_MANUAL_TEST: iconWrench(cx, cy, c, bg); break;
    case UiMenuId::PERCEPTION_ROOT:
    case UiMenuId::PERCEPTION_CAMERA: iconCamera(cx, cy, c, bg); break;
    case UiMenuId::PERCEPTION_DETECTION:
    case UiMenuId::PERCEPTION_DETECTION_LIVE:
    case UiMenuId::PERCEPTION_OBSTACLE: iconEye(cx, cy, c, bg); break;
    case UiMenuId::PERCEPTION_INFERENCE:
    case UiMenuId::PERCEPTION_PERFORMANCE: iconGear(cx, cy, c, bg); break;
    case UiMenuId::PERCEPTION_LANE: iconAutoArrow(cx, cy, c, bg); break;
    case UiMenuId::NAVIGATION_ROOT:
    case UiMenuId::NAV_LOCALIZATION:
    case UiMenuId::NAV_EKF_LOCAL:
    case UiMenuId::NAV_EKF_GLOBAL: iconCompass(cx, cy, c, bg); break;
    case UiMenuId::NAV_GNSS: iconGps(cx, cy, c, bg); break;
    case UiMenuId::NAV_MISSION:
    case UiMenuId::NAV_MISSION_GO:
    case UiMenuId::NAV_MISSION_SAVE:
    case UiMenuId::NAV_MISSION_STOP: iconAutoArrow(cx, cy, c, bg); break;
    case UiMenuId::NAV_SAFETY: iconShield(cx, cy, c, bg); break;
    case UiMenuId::NAV_NAV2:
    case UiMenuId::NAV_PLANNER:
    case UiMenuId::NAV_MPPI:
    case UiMenuId::NAV_SMOOTHER:
    case UiMenuId::NAV_COSTMAP: iconGrid(cx, cy, c); break;
    default: iconGrid(cx, cy, c); break;
  }
}

inline void drawMenuCardTitle(const char* title, int cx, int y, uint16_t fg, uint16_t bg) {
  char first[18]{};
  char second[18]{};
  const size_t len = strlen(title);
  if (len <= 13U) {
    drawCompactText(title, cx, y + 7, fg, bg, TC_DATUM);
    return;
  }
  size_t split = 0U;
  for (size_t i = 1U; i < len && i <= 13U; ++i) if (title[i] == ' ') split = i;
  if (split == 0U) {
    for (size_t i = 13U; i < len; ++i) if (title[i] == ' ') { split = i; break; }
  }
  if (split == 0U) split = len > 15U ? 15U : len;
  const size_t n1 = split < sizeof(first) - 1U ? split : sizeof(first) - 1U;
  memcpy(first, title, n1); first[n1] = '\0';
  const char* rest = title + split;
  while (*rest == ' ') ++rest;
  snprintf(second, sizeof(second), "%.16s", rest);
  drawCompactText(first, cx, y, fg, bg, TC_DATUM);
  drawCompactText(second, cx, y + 17, fg, bg, TC_DATUM);
}
inline void drawMetricRow(int y, const char* label, const char* value, uint16_t valueColor = C_INK) {
  drawSmallText(label, 18, y, C_DISABLED, C_CARD);
  drawUiText(value, 252, y - 2, valueColor, C_CARD, TR_DATUM);
  drawThinDivider(16, y + 20, 238, C_CARD_LINE);
}

inline void drawMetricFloat(int y, const char* label, float value, const char* unit, int precision = 1,
                            uint16_t valueColor = C_INK) {
  char text[30];
  snprintf(text, sizeof(text), precision == 2 ? "%.2f %s" : "%.1f %s", value, unit);
  drawMetricRow(y, label, text, valueColor);
}


inline void drawOverviewDomainCard(int index, UiMenuId id, const char* status, bool ready) {
  const int x = UI_CARD_X0 + index * (UI_CARD_W + UI_CARD_GAP);
  drawCard(x, OVERVIEW_CARD_Y, UI_CARD_W, OVERVIEW_CARD_H, C_CARD, ready ? C_ACCENT : C_CARD_LINE, false);
  drawMenuNodeIcon(id, x + UI_CARD_W / 2, OVERVIEW_CARD_Y + 31, ready ? C_ACCENT : C_DISABLED, C_CARD);
  drawMenuCardTitle(menuTitle(id), x + UI_CARD_W / 2, OVERVIEW_CARD_Y + 58, C_INK, C_CARD);
  drawStatusDot(x + 13, OVERVIEW_CARD_Y + 14, healthColor(ready), 3);
  drawMicroText(status, x + UI_CARD_W / 2, OVERVIEW_CARD_Y + 91,
                ready ? C_READY : C_DISABLED, C_CARD, TC_DATUM);
}

inline void drawOverview(const UiState& ui, const VehicleTelemetry& d) {
  (void)ui;
  drawCard(6, 36, 308, 82, C_PANEL, C_BORDER, false);
  char line[72];
  snprintf(line, sizeof(line), "%s  |  %s", modeText(d.mode), systemStatusText(d.systemStatus));
  drawUiText(line, 16, 46, d.eStop ? C_FAULT : C_TEXT, C_PANEL);
  snprintf(line, sizeof(line), "%.1f km/h   %s / %u SAT", d.speedKmh, gpsFixText(d.gpsFix), d.satellites);
  drawSmallText(line, 16, 72, C_TEXT_DIM, C_PANEL);
  const char* target = navigationHasTarget(d) ? d.activeTarget : "NO TARGET";
  snprintf(line, sizeof(line), "%s | %.22s", navigationStatusText(d.navigationStatus), target);
  drawMicroText(line, 16, 98, C_TEXT_DIM, C_PANEL);
  drawStatusDot(292, 54, d.eStop ? C_FAULT : systemStatusColor(d.systemStatus), 5);

  drawOverviewDomainCard(0, UiMenuId::ESC_ROOT, d.escReady && d.vescConnected ? "READY" : "WAIT",
                         d.escReady && d.vescConnected);
  drawOverviewDomainCard(1, UiMenuId::PERCEPTION_ROOT, d.perceptionReady ? "READY" : "WAIT",
                         d.perceptionReady);
  drawOverviewDomainCard(2, UiMenuId::NAVIGATION_ROOT, d.motionReady && d.nav2Ready ? "READY" : "WAIT",
                         d.motionReady && d.nav2Ready);
}

inline void drawMenuList(const UiState& ui) {
  uint8_t count = 0;
  const UiMenuId* children = menuChildren(ui.menu, count);
  if (children == nullptr || count == 0U) return;
  const uint8_t first = menuWindowFirst(ui.selectedChild, count);
  for (uint8_t slot = 0; slot < SUBMENU_VISIBLE_CARDS && first + slot < count; ++slot) {
    const uint8_t index = static_cast<uint8_t>(first + slot);
    const int x = UI_CARD_X0 + slot * (UI_CARD_W + UI_CARD_GAP);
    const bool selected = index == ui.selectedChild;
    const uint16_t border = selected ? C_ACCENT : C_BORDER;
    const uint16_t fill = selected ? C_PANEL_ALT : C_PANEL;
    const uint16_t fg = selected ? C_ACCENT : C_TEXT;
    drawCard(x, SUBMENU_CARD_Y, UI_CARD_W, SUBMENU_CARD_H, fill, border, false);
    drawMenuNodeIcon(children[index], x + UI_CARD_W / 2, SUBMENU_CARD_Y + 39, fg, fill);
    drawMenuCardTitle(menuTitle(children[index]), x + UI_CARD_W / 2, SUBMENU_CARD_Y + 72, fg, fill);
    char slotText[12];
    snprintf(slotText, sizeof(slotText), "%u", static_cast<unsigned>(index + 1U));
    drawMicroText(slotText, x + UI_CARD_W / 2, SUBMENU_CARD_Y + 115,
                  selected ? C_ACCENT : C_TEXT_DIM, fill, TC_DATUM);
  }
}

inline void drawEditStatus(const VehicleTelemetry& d) {
  if (d.configPending) {
    drawMicroText("WAITING ROS ACK / READBACK", 18, 164, C_WARNING, C_CARD);
  } else if (!d.configLastOk) {
    drawMicroText(d.configMessage, 18, 164, C_FAULT, C_CARD);
  } else {
    drawMicroText("SYNCED WITH ROS", 18, 164, C_READY, C_CARD);
  }
}

inline void drawEditor(const UiState& ui, const VehicleTelemetry& d, UiEditKey key) {
  drawContentCard();
  drawMicroText("EDIT VALUE", 18, 46, C_DISABLED, C_CARD);
  char value[32];
  const float shown = ui.editing ? ui.editValue :
    (key == UiEditKey::OPERATOR_MODE ? (d.mode == MODE_MANUAL ? 1.0F : 0.0F) :
     (key == UiEditKey::MANUAL_SPEED_PCT ? static_cast<float>(d.manualSpeedPct) :
      (key == UiEditKey::STEERING_TEST_DEG ? d.steeringTestAngleDeg :
       (key == UiEditKey::DRIVE_SCALE ? d.driveScale : (d.perceptionInference ? 1.0F : 0.0F)))));
  if (key == UiEditKey::MANUAL_SPEED_PCT) snprintf(value, sizeof(value), "%.0f %%", shown);
  else if (key == UiEditKey::STEERING_TEST_DEG) snprintf(value, sizeof(value), "%.0f deg", shown);
  else if (key == UiEditKey::DRIVE_SCALE) snprintf(value, sizeof(value), "%.3f", shown);
  else if (key == UiEditKey::OPERATOR_MODE) snprintf(value, sizeof(value), "%s", shown > 0.5F ? "MANUAL" : "AUTO");
  else snprintf(value, sizeof(value), "%s", shown > 0.5F ? "ON" : "OFF");
  drawHeroText(value, 18, 70, C_INK, C_CARD);
  drawSmallText("LEFT / RIGHT untuk ubah nilai", 18, 127, C_DISABLED, C_CARD);
  drawSmallText("OK untuk apply + readback ROS", 18, 147, C_DISABLED, C_CARD);
  drawEditStatus(d);
}

inline void drawManualTestButton(int x, int y, int w, int h, SoftKey key, const char* label,
                                 const char* sub, bool active, bool stop = false) {
  const uint16_t fill = stop ? C_FAULT : (active ? C_PANEL_ALT : C_PANEL);
  const uint16_t border = stop ? C_FAULT : (active ? C_ACCENT : C_BORDER);
  const uint16_t fg = stop ? C_WHITE : (active ? C_TEXT : C_DISABLED);
  tft.fillRoundRect(x, y, w, h, 8, fill);
  tft.drawRoundRect(x, y, w, h, 8, border);
  const int cx = x + w / 2;
  if (key == SoftKey::TEST_FORWARD) iconArrowUp(cx, y + 16, fg);
  else if (key == SoftKey::TEST_REVERSE) iconArrowDown(cx, y + 16, fg);
  else if (key == SoftKey::TEST_LEFT) iconArrowLeft(cx, y + h / 2 - 8, fg);
  else if (key == SoftKey::TEST_RIGHT) iconArrowRight(cx, y + h / 2 - 8, fg);
  else if (key == SoftKey::TEST_STOP) iconStop(cx, y + h / 2 - 8, fg);
  drawCompactText(label, cx, y + h - 27, fg, fill, TC_DATUM);
  drawMicroText(sub, cx, y + h - 12, stop ? C_WHITE : C_TEXT_DIM, fill, TC_DATUM);
}

inline void drawManualTest(const UiState& ui, const VehicleTelemetry& d) {
  (void)ui;
  const bool driveReady = d.rosConnected && d.mode == MODE_MANUAL && d.escReady && !d.eStop;
  const bool steerReady = driveReady && d.encoderReady && fabsf(d.driveActualMps) < 0.02F;
  char speed[16], angle[16];
  snprintf(speed, sizeof(speed), "%u%%", static_cast<unsigned>(d.manualSpeedPct));
  snprintf(angle, sizeof(angle), "%.0f deg", d.steeringTestAngleDeg);
  drawManualTestButton(108, 40, 104, 50, SoftKey::TEST_FORWARD, "FORWARD", speed, driveReady);
  drawManualTestButton(6, 96, 94, 70, SoftKey::TEST_LEFT, "LEFT", angle, steerReady);
  drawManualTestButton(108, 96, 104, 70, SoftKey::TEST_STOP, "STOP", "ALL MOTION", true, true);
  drawManualTestButton(220, 96, 94, 70, SoftKey::TEST_RIGHT, "RIGHT", angle, steerReady);
  drawManualTestButton(108, 172, 104, 52, SoftKey::TEST_REVERSE, "REVERSE", speed, driveReady);
}

inline void drawEscLeaf(UiMenuId id, const UiState& ui, const VehicleTelemetry& d) {
  if (id == UiMenuId::ESC_MANUAL_TEST) { drawManualTest(ui, d); return; }
  drawContentCard();
  char a[30], b[30];
  if (id == UiMenuId::ESC_OVERVIEW) {
    snprintf(a, sizeof(a), "%s", d.escReady ? "READY" : "NOT READY");
    drawMetricRow(48, "ESC state", a, healthColor(d.escReady));
    drawMetricFloat(77, "Steering", d.steeringActualDeg, "deg", 1, C_ACCENT);
    drawMetricFloat(106, "Drive", d.driveActualMps, "m/s", 2, C_ACCENT);
    drawMetricRow(135, "F103 link", d.vescConnected ? "ONLINE" : "OFFLINE", healthColor(d.vescConnected));
    return;
  }
  if (id == UiMenuId::ESC_STEERING_LIVE || id == UiMenuId::ESC_STEERING_CAL) {
    drawMetricFloat(48, "Target", d.steeringTargetDeg, "deg", 1, C_ACCENT);
    drawMetricFloat(77, "Actual", d.steeringActualDeg, "deg", 1, C_ACCENT);
    drawMetricFloat(106, "Error", d.steeringErrorDeg, "deg", 1,
                    fabsf(d.steeringErrorDeg) < 2.0F ? C_READY : C_WARNING);
    drawMetricRow(135, "Encoder", d.encoderReady ? "READY" : "OFFLINE", healthColor(d.encoderReady));
    return;
  }
  if (id == UiMenuId::ESC_DRIVE_LIVE) {
    drawMetricFloat(48, "Target", d.driveTargetMps, "m/s", 2, C_ACCENT);
    drawMetricFloat(77, "Actual", d.driveActualMps, "m/s", 2, C_ACCENT);
    snprintf(a, sizeof(a), "%.0f", d.motorErpm);
    drawMetricRow(106, "Electrical RPM", a, C_INK);
    snprintf(b, sizeof(b), "%.0f", d.motorRpm);
    drawMetricRow(135, "Mechanical RPM", b, C_INK);
    return;
  }
  if (id == UiMenuId::ESC_POWER) {
    drawMetricRow(48, "Power telemetry", d.vescConnected ? "STREAMING" : "NO LINK", healthColor(d.vescConnected));
    drawMetricRow(77, "Control", "FOC 16 kHz", C_INK);
    drawMetricRow(106, "Watchdog", "300 ms", C_READY);
    drawMetricRow(135, "Fault gate", d.systemStatus == SYS_FAULT ? "FAULT" : "CLEAR",
                  d.systemStatus == SYS_FAULT ? C_FAULT : C_READY);
    return;
  }
  if (id == UiMenuId::ESC_LINK) {
    drawMetricRow(48, "ROS <-> F411", d.rosConnected ? "ONLINE" : "OFFLINE", healthColor(d.rosConnected));
    drawMetricRow(77, "F411 <-> F103", d.vescConnected ? "ONLINE" : "OFFLINE", healthColor(d.vescConnected));
    drawMetricRow(106, "USB CDC", "1,000,000", C_INK);
    drawMetricRow(135, "F103 UART", "115,200", C_INK);
    return;
  }
  drawMetricRow(48, "ESC", d.escReady ? "PASS" : "FAIL", healthColor(d.escReady));
  drawMetricRow(77, "Encoder", d.encoderReady ? "PASS" : "FAIL", healthColor(d.encoderReady));
  drawMetricRow(106, "E-stop", d.eStop ? "FAIL" : "PASS", d.eStop ? C_FAULT : C_READY);
  drawMetricRow(135, "Vehicle", d.state == STATE_STOPPED ? "STOPPED" : "MOVING",
                d.state == STATE_STOPPED ? C_READY : C_WARNING);
}

inline void drawPerceptionLeaf(UiMenuId id, const UiState& ui, const VehicleTelemetry& d) {
  drawContentCard();
  char text[32];
  if (id == UiMenuId::PERCEPTION_OVERVIEW || id == UiMenuId::PERCEPTION_CAMERA) {
    drawMetricRow(48, "Camera", d.cameraReady ? "READY" : "OFFLINE", healthColor(d.cameraReady));
    drawMetricRow(77, "Perception", d.perceptionReady ? "HEALTHY" : "WAIT", healthColor(d.perceptionReady));
    snprintf(text, sizeof(text), "%.1f FPS", d.cameraFps);
    drawMetricRow(106, "Pipeline", text, d.cameraFps > 1.0F ? C_READY : C_WARNING);
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
    drawMetricRow(135, "Safety action", d.eStop ? "STOP" : "MONITOR", d.eStop ? C_FAULT : C_INK);
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
    drawMetricRow(48, "Inference rate", text, d.cameraFps > 1.0F ? C_READY : C_WARNING);
    drawMetricRow(77, "Camera link", d.cameraReady ? "ONLINE" : "OFFLINE", healthColor(d.cameraReady));
    drawMetricRow(106, "Health", d.perceptionReady ? "GOOD" : "BAD", healthColor(d.perceptionReady));
    drawMetricRow(135, "ROS stream", d.rosConnected ? "LIVE" : "OFFLINE", healthColor(d.rosConnected));
    return;
  }
  drawMetricRow(48, "Camera", d.cameraReady ? "PASS" : "FAIL", healthColor(d.cameraReady));
  drawMetricRow(77, "Inference", d.perceptionInference ? "PASS" : "OFF", d.perceptionInference ? C_READY : C_WARNING);
  drawMetricRow(106, "Drivable", d.drivableAreaClear ? "PASS" : "CHECK",
                d.drivableAreaClear ? C_READY : C_WARNING);
  drawMetricRow(135, "Obstacle gate", d.eStop ? "STOP" : "PASS", d.eStop ? C_FAULT : C_READY);
}

inline void drawNavigationLeaf(UiMenuId id, const UiState& ui, const VehicleTelemetry& d) {
  drawContentCard();
  char text[40];
  if (id == UiMenuId::NAVIGATION_OVERVIEW) {
    drawMetricRow(48, "Localization", d.localizationState,
                  d.motionReady ? C_READY : C_WARNING);
    drawMetricRow(77, "Nav2", d.nav2Ready ? "READY" : "WAIT", healthColor(d.nav2Ready));
    drawMetricRow(106, "Motion gate", d.motionReady ? "OPEN" : "LOCKED", d.motionReady ? C_READY : C_FAULT);
    drawMetricRow(135, "Mission", navigationStatusText(d.navigationStatus), C_ACCENT);
    return;
  }
  if (id == UiMenuId::NAV_GNSS) {
    snprintf(text, sizeof(text), "%s / %u SAT", gpsFixText(d.gpsFix), d.satellites);
    drawMetricRow(48, "Fix", text, gpsFixColor(d.gpsFix));
    drawMetricFloat(77, "hAcc", d.haccM, "m", 2, d.haccM < 2.5F ? C_READY : C_WARNING);
    drawMetricFloat(106, "Age", d.gnssAgeSec, "s", 2, d.gnssAgeSec < 0.5F ? C_READY : C_WARNING);
    snprintf(text, sizeof(text), "%.0f deg", d.headingDeg);
    drawMetricRow(135, "Heading", text, C_INK);
    return;
  }
  if (id == UiMenuId::NAV_IMU) {
    drawMetricRow(48, "IMU", d.imuStatus, healthColor(d.imuReady));
    drawMetricFloat(77, "Gyro Z", d.gyroZRps, "rad/s", 2, C_INK);
    snprintf(text, sizeof(text), "%.0f deg", d.headingDeg);
    drawMetricRow(106, "Heading", text, C_INK);
    drawMetricRow(135, "Fusion", d.motionReady ? "ACTIVE" : "WAIT", d.motionReady ? C_READY : C_WARNING);
    return;
  }
  if (id == UiMenuId::NAV_MAG) {
    drawMetricRow(48, "IST8310", d.magReady ? "READY" : "OFFLINE", healthColor(d.magReady));
    drawMetricRow(77, "Heading source", d.gnssStatus, C_INK);
    snprintf(text, sizeof(text), "%.0f deg", d.headingDeg);
    drawMetricRow(106, "Vehicle yaw", text, C_ACCENT);
    drawMetricRow(135, "Calibration", d.magReady ? "AVAILABLE" : "REQUIRED",
                  d.magReady ? C_READY : C_WARNING);
    return;
  }
  if (id == UiMenuId::NAV_EKF_LOCAL || id == UiMenuId::NAV_EKF_GLOBAL) {
    const char* status = id == UiMenuId::NAV_EKF_LOCAL ? d.ekfLocalStatus : d.ekfGlobalStatus;
    drawMetricRow(48, id == UiMenuId::NAV_EKF_LOCAL ? "EKF local" : "EKF global", status,
                  strstr(status, "READY") != nullptr ? C_READY : C_WARNING);
    drawMetricRow(77, "Localization", d.localizationState, d.motionReady ? C_READY : C_WARNING);
    drawMetricFloat(106, "Vehicle speed", d.driveActualMps, "m/s", 2, C_INK);
    snprintf(text, sizeof(text), "%.0f deg", d.headingDeg);
    drawMetricRow(135, "Yaw", text, C_ACCENT);
    return;
  }
  if (id == UiMenuId::NAV_MISSION_GO || id == UiMenuId::NAV_MISSION_SAVE ||
      id == UiMenuId::NAV_MISSION_STOP) {
    const uint8_t index = d.selectedWaypoint < HMI_WAYPOINT_COUNT ? d.selectedWaypoint : 0;
    drawMetricRow(48, "Waypoint", d.waypointName[index], d.waypointSaved[index] ? C_READY : C_WARNING);
    drawMetricRow(77, "Saved", d.waypointSaved[index] ? "YES" : "NO", d.waypointSaved[index] ? C_READY : C_WARNING);
    drawMetricRow(106, "Navigation", navigationStatusText(d.navigationStatus), C_ACCENT);
    if (id == UiMenuId::NAV_MISSION_GO) {
      drawMetricRow(135, "Action", d.mode == MODE_AUTO ? "GO ON OK" : "AUTO REQUIRED",
                    d.mode == MODE_AUTO ? C_READY : C_FAULT);
      drawMicroText("LEFT/RIGHT pilih waypoint", 18, 166, C_DISABLED, C_CARD);
    } else if (id == UiMenuId::NAV_MISSION_SAVE) {
      drawMetricRow(135, "Action", d.gpsReady && d.state == STATE_STOPPED ? "SAVE ON OK" : "GPS/STOP REQUIRED",
                    d.gpsReady && d.state == STATE_STOPPED ? C_READY : C_FAULT);
      drawMicroText("Simpan pose map + GNSS saat ini", 18, 166, C_DISABLED, C_CARD);
    } else {
      drawMetricRow(135, "Action", "STOP ON OK", C_FAULT);
      drawMicroText("STOP selalu boleh dipanggil", 18, 166, C_DISABLED, C_CARD);
    }
    return;
  }
  if (id == UiMenuId::NAV_PLANNER || id == UiMenuId::NAV_MPPI ||
      id == UiMenuId::NAV_SMOOTHER || id == UiMenuId::NAV_COSTMAP) {
    // Jangan tampilkan parameter Nav2 hardcoded: seluruh angka harus berasal dari runtime.
    drawMetricRow(48, "Nav2 state", d.nav2Ready ? "READY" : "WAIT", healthColor(d.nav2Ready));
    drawMetricRow(77, "Localization", d.localizationState, d.motionReady ? C_READY : C_WARNING);
    if (id == UiMenuId::NAV_COSTMAP) {
      drawMetricRow(106, "Obstacle", d.obstacleDetected ? "ACTIVE" : "CLEAR",
                    d.obstacleDetected ? C_WARNING : C_READY);
      drawMetricRow(135, "Lane", d.laneState, strcmp(d.laneState, "CLEAR") == 0 ? C_READY : C_WARNING);
    } else {
      drawMetricFloat(106, "Cmd speed", d.driveTargetMps, "m/s", 2, C_ACCENT);
      drawMetricFloat(135, "Actual speed", d.driveActualMps, "m/s", 2, C_INK);
    }
    return;
  }
  if (id == UiMenuId::NAV_SAFETY) {
    drawMetricRow(48, "E-stop", d.eStop ? "ACTIVE" : "CLEAR", d.eStop ? C_FAULT : C_READY);
    drawMetricRow(77, "Localization", d.motionReady ? "PASS" : "LOCKED", d.motionReady ? C_READY : C_FAULT);
    drawMetricRow(106, "Nav2", d.nav2Ready ? "PASS" : "WAIT", healthColor(d.nav2Ready));
    drawMetricRow(135, "Autonomy", (!d.eStop && d.motionReady && d.nav2Ready) ? "ALLOWED" : "LOCKED",
                  (!d.eStop && d.motionReady && d.nav2Ready) ? C_READY : C_FAULT);
    return;
  }
  drawMetricRow(48, "GNSS", d.gpsReady ? "PASS" : "FAIL", healthColor(d.gpsReady));
  drawMetricRow(77, "IMU", d.imuReady ? "PASS" : "FAIL", healthColor(d.imuReady));
  drawMetricRow(106, "Nav2", d.nav2Ready ? "PASS" : "FAIL", healthColor(d.nav2Ready));
  drawMetricRow(135, "Safety", d.eStop ? "FAIL" : "PASS", d.eStop ? C_FAULT : C_READY);
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
inline bool isNavigationMenu(UiMenuId id) {
  return static_cast<uint8_t>(id) >= static_cast<uint8_t>(UiMenuId::NAVIGATION_ROOT);
}

inline bool uiNeedsEditFooter(const UiState& ui) {
  return menuEditKey(ui.menu) != UiEditKey::NONE || ui.menu == UiMenuId::NAV_MISSION_GO ||
         ui.menu == UiMenuId::NAV_MISSION_SAVE || ui.menu == UiMenuId::NAV_MISSION_STOP;
}

inline void drawUiContent(const UiState& ui, const VehicleTelemetry& d) {
  tft.fillRect(0, CONTENT_Y - 1, W, H - CONTENT_Y + 1, C_BG);
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
  } else if (isNavigationMenu(ui.menu)) {
    drawNavigationLeaf(ui.menu, ui, d);
  }
}

inline void drawUiFrame(const UiState& ui, const VehicleTelemetry& d, bool full) {
  if (full) tft.fillScreen(C_BG);
  drawUiTopBar(ui, d);
  drawUiContent(ui, d);
  if (menuHasChildren(ui.menu)) drawCarouselFooter(ui);
  else if (uiNeedsEditFooter(ui)) drawEditFooter(ui, d);
}
