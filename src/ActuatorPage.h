// ============================================================================
// ActuatorPage.h — one-tap manual drive/steering + actuator feedback
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

inline bool manualDriveEnabled(const VehicleTelemetry& d) {
  return d.rosConnected && d.systemStatus == SYS_READY && d.mode == MODE_MANUAL && d.escReady;
}

inline bool manualSteerEnabled(const VehicleTelemetry& d) {
  return manualDriveEnabled(d) && d.encoderReady;
}

inline void drawControlButton(int x, int y, ControlAction action, const char* label,
                              bool enabled, bool active) {
  const bool isStop = action == CTRL_STOP;
  const uint16_t border = isStop ? C_FAULT : (active ? C_ACCENT : C_BORDER);
  const uint16_t iconColor = enabled ? (isStop ? C_FAULT : (active ? C_ACCENT : C_TEXT)) : C_DISABLED;
  const uint16_t labelColor = enabled ? (isStop ? C_FAULT : C_TEXT) : C_DISABLED;

  tft.fillRoundRect(x + 1, y + 2, ACT_BTN_W, ACT_BTN_H, 6, C_SHADOW);
  tft.fillRoundRect(x, y, ACT_BTN_W, ACT_BTN_H, 6, C_PANEL_ALT);
  tft.drawRoundRect(x, y, ACT_BTN_W, ACT_BTN_H, 6, border);

  const int cx = x + ACT_BTN_W / 2;

  if (action == CTRL_CENTER) {
    drawValueText("0", cx - 3, y + 6, iconColor, C_PANEL_ALT, TC_DATUM);
    drawDegreeMark(cx + 10, y + 10, iconColor, C_PANEL_ALT);
    return;
  }

  const int iconY = y + 11;
  if (action == CTRL_FORWARD) iconArrowUp(cx, iconY, iconColor);
  else if (action == CTRL_REVERSE) iconArrowDown(cx, iconY, iconColor);
  else if (action == CTRL_LEFT) iconArrowLeft(cx, iconY, iconColor);
  else if (action == CTRL_RIGHT) iconArrowRight(cx, iconY, iconColor);
  else if (action == CTRL_STOP) iconStop(cx, iconY, iconColor);

  drawMicroText(label, cx, y + 25, labelColor, C_PANEL_ALT, TC_DATUM);
}

inline void drawActuatorControlPanel(const VehicleTelemetry& d,
                                     ControlAction activeDrive,
                                     ControlAction activeSteer) {
  drawCard(ACT_CTRL_X, ACT_CTRL_Y, ACT_CTRL_W, ACT_CTRL_H, C_BG, C_BORDER);

  const bool driveEnabled = manualDriveEnabled(d);
  const bool steerEnabled = manualSteerEnabled(d);

  drawControlButton(ACT_FWD_X, ACT_FWD_Y, CTRL_FORWARD, "MAJU", driveEnabled,
                    activeDrive == CTRL_FORWARD);
  drawControlButton(ACT_LEFT_X, ACT_LEFT_Y, CTRL_LEFT, "KIRI", steerEnabled,
                    activeSteer == CTRL_LEFT);
  drawControlButton(ACT_CENTER_X, ACT_CENTER_Y, CTRL_CENTER, "", steerEnabled,
                    activeSteer == CTRL_CENTER);
  drawControlButton(ACT_RIGHT_X, ACT_RIGHT_Y, CTRL_RIGHT, "KANAN", steerEnabled,
                    activeSteer == CTRL_RIGHT);
  drawControlButton(ACT_REV_X, ACT_REV_Y, CTRL_REVERSE, "MUNDUR", driveEnabled,
                    activeDrive == CTRL_REVERSE);
  drawControlButton(ACT_STOP_X, ACT_STOP_Y, CTRL_STOP, "STOP", true, false);
}

inline void drawActuatorInfoPanel(const VehicleTelemetry& d) {
  drawCard(ACT_INFO_X, ACT_INFO_Y, ACT_INFO_W, ACT_INFO_H, C_CARD, C_CARD_LINE);

  char buf[24];
  drawMicroText("MODE", ACT_INFO_X + 9, ACT_INFO_Y + 10, C_DISABLED, C_CARD);
  drawCompactText(modeText(d.mode), ACT_INFO_X + ACT_INFO_W - 8, ACT_INFO_Y + 9,
                  C_INK, C_CARD, TR_DATUM);
  drawThinDivider(ACT_INFO_X + 8, ACT_INFO_Y + 29, ACT_INFO_W - 16);

  drawMicroText("Speed", ACT_INFO_X + 9, ACT_INFO_Y + 40, C_INK, C_CARD);
  snprintf(buf, sizeof(buf), "%u%%", d.manualSpeedPct);
  drawUiText(buf, ACT_INFO_X + ACT_INFO_W - 9, ACT_INFO_Y + 33,
             C_ACCENT, C_CARD, TR_DATUM);

  drawMicroText("Steering", ACT_INFO_X + 9, ACT_INFO_Y + 63, C_INK, C_CARD);
  snprintf(buf, sizeof(buf), "%+.1f", d.steeringActualDeg);
  drawUiText(buf, ACT_INFO_X + ACT_INFO_W - 13, ACT_INFO_Y + 56,
             C_ACCENT, C_CARD, TR_DATUM);
  drawDegreeMark(ACT_INFO_X + ACT_INFO_W - 8, ACT_INFO_Y + 61, C_ACCENT, C_CARD);

  drawMicroText("Target", ACT_INFO_X + 9, ACT_INFO_Y + 84, C_INK, C_CARD);
  snprintf(buf, sizeof(buf), "%+.1f", d.steeringTargetDeg);
  drawUiText(buf, ACT_INFO_X + ACT_INFO_W - 13, ACT_INFO_Y + 77,
             C_ACCENT, C_CARD, TR_DATUM);
  drawDegreeMark(ACT_INFO_X + ACT_INFO_W - 8, ACT_INFO_Y + 82, C_ACCENT, C_CARD);

  drawThinDivider(ACT_INFO_X + 8, ACT_INFO_Y + 99, ACT_INFO_W - 16);

  drawMicroText("ESC Ready", ACT_INFO_X + 9, ACT_INFO_Y + 105, C_INK, C_CARD);
  drawStatusDot(ACT_INFO_X + ACT_INFO_W - 12, ACT_INFO_Y + 109,
                healthColor(d.escReady), 3);

  drawMicroText("Encoder", ACT_INFO_X + 9, ACT_INFO_Y + 118, C_INK, C_CARD);
  drawStatusDot(ACT_INFO_X + ACT_INFO_W - 12, ACT_INFO_Y + 122,
                healthColor(d.encoderReady), 3);

  drawMicroText("RPM", ACT_INFO_X + 9, ACT_INFO_Y + 134, C_DISABLED, C_CARD);
  snprintf(buf, sizeof(buf), "%.0f", d.motorRpm);
  drawCompactText(buf, ACT_INFO_X + ACT_INFO_W - 8, ACT_INFO_Y + 128,
                  C_ACCENT, C_CARD, TR_DATUM);
}

inline void drawActuatorPage(const VehicleTelemetry& d,
                             ControlAction activeDrive = CTRL_NONE,
                             ControlAction activeSteer = CTRL_NONE) {
  tft.fillScreen(C_BG);
  drawTopBar("ACTUATOR", d, true, true);
  drawActuatorControlPanel(d, activeDrive, activeSteer);
  drawActuatorInfoPanel(d);
  drawBottomMenu(PAGE_ACTUATOR);
}

inline void updateActuatorPage(const VehicleTelemetry& d,
                               ControlAction activeDrive = CTRL_NONE,
                               ControlAction activeSteer = CTRL_NONE) {
  // Keep controls/static card stable. Redraw buttons only when their state changes,
  // and redraw feedback values in small rectangles.
  updateTopHealthOnly(d, true);
  static VehicleMode lastMode = MODE_AUTO; static SystemStatus lastSys = SYS_OFF;
  static int lastEsc=-1,lastEnc=-1; static ControlAction lastDrive=CTRL_NONE,lastSteer=CTRL_NONE;
  static char lastSpeed[10]="",lastActual[16]="",lastTarget[16]="",lastRpm[16]="";
  static bool init=false; char buf[24];

  const bool escChanged = !init || lastEsc != static_cast<int>(d.escReady);
  const bool encChanged = !init || lastEnc != static_cast<int>(d.encoderReady);
  if(!init || lastMode!=d.mode || lastSys!=d.systemStatus || escChanged || encChanged || lastDrive!=activeDrive || lastSteer!=activeSteer){
    lastMode=d.mode; lastSys=d.systemStatus; lastEsc=d.escReady; lastEnc=d.encoderReady; lastDrive=activeDrive; lastSteer=activeSteer;
    drawActuatorControlPanel(d,activeDrive,activeSteer);
    tft.fillRect(ACT_INFO_X+5,ACT_INFO_Y+6,ACT_INFO_W-10,20,C_CARD);
    drawMicroText("MODE",ACT_INFO_X+9,ACT_INFO_Y+10,C_DISABLED,C_CARD);
    drawCompactText(modeText(d.mode),ACT_INFO_X+ACT_INFO_W-8,ACT_INFO_Y+9,C_INK,C_CARD,TR_DATUM);
  }

  snprintf(buf,sizeof(buf),"%u%%",d.manualSpeedPct);
  if(!init || strcmp(buf,lastSpeed)!=0){ snprintf(lastSpeed,sizeof(lastSpeed),"%s",buf); drawUiTextPadded(buf,ACT_INFO_X+ACT_INFO_W-9,ACT_INFO_Y+33,C_ACCENT,C_CARD,46,TR_DATUM); }
  snprintf(buf,sizeof(buf),"%+.1f",d.steeringActualDeg);
  if(!init || strcmp(buf,lastActual)!=0){ snprintf(lastActual,sizeof(lastActual),"%s",buf); drawUiTextPadded(buf,ACT_INFO_X+ACT_INFO_W-13,ACT_INFO_Y+56,C_ACCENT,C_CARD,50,TR_DATUM); drawDegreeMark(ACT_INFO_X+ACT_INFO_W-8,ACT_INFO_Y+61,C_ACCENT,C_CARD); }
  snprintf(buf,sizeof(buf),"%+.1f",d.steeringTargetDeg);
  if(!init || strcmp(buf,lastTarget)!=0){ snprintf(lastTarget,sizeof(lastTarget),"%s",buf); drawUiTextPadded(buf,ACT_INFO_X+ACT_INFO_W-13,ACT_INFO_Y+77,C_ACCENT,C_CARD,50,TR_DATUM); drawDegreeMark(ACT_INFO_X+ACT_INFO_W-8,ACT_INFO_Y+82,C_ACCENT,C_CARD); }
  if (escChanged) drawStatusDot(ACT_INFO_X+ACT_INFO_W-12,ACT_INFO_Y+109,healthColor(d.escReady),3);
  if (encChanged) drawStatusDot(ACT_INFO_X+ACT_INFO_W-12,ACT_INFO_Y+122,healthColor(d.encoderReady),3);
  snprintf(buf,sizeof(buf),"%.0f",d.motorRpm);
  if(!init || strcmp(buf,lastRpm)!=0){ snprintf(lastRpm,sizeof(lastRpm),"%s",buf); drawCompactTextPadded(buf,ACT_INFO_X+ACT_INFO_W-8,ACT_INFO_Y+128,C_ACCENT,C_CARD,48,TR_DATUM); }
  init=true;
}
