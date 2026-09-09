// ============================================================================
// TopBar.h — compact status bar used on every HMI page
// ============================================================================
#pragma once

#include "HmiDisplay.h"
#include "Config.h"
#include "Telemetry.h"
#include "Theme.h"
#include "Icons.h"

extern HmiDisplay tft;

inline void drawTopHealth(const VehicleTelemetry& d, bool actuatorPage = false) {
  if (actuatorPage) {
    drawMicroText("ROS", 209, 10, C_TEXT, C_BG);
    drawStatusDot(239, 13, healthColor(d.rosConnected), 3);
    drawMicroText("ESC", 247, 10, C_TEXT, C_BG);
    drawStatusDot(278, 13, healthColor(d.escReady), 3);
    drawMicroText("ENC", 286, 10, C_TEXT, C_BG);
    drawStatusDot(315, 13, healthColor(d.encoderReady), 3);
    return;
  }

  drawMicroText("ROS", 165, 10, C_TEXT, C_BG);
  drawStatusDot(195, 13, healthColor(d.rosConnected), 3);
  drawMicroText("GPS", 202, 10, C_TEXT, C_BG);
  drawStatusDot(231, 13, d.gpsReady ? gpsFixColor(d.gpsFix) : C_FAULT, 3);
  drawMicroText("CAM", 239, 10, C_TEXT, C_BG);
  drawStatusDot(270, 13, healthColor(d.cameraReady), 3);
  drawMicroText("ESC", 278, 10, C_TEXT, C_BG);
  drawStatusDot(309, 13, healthColor(d.escReady), 3);
}

inline void updateTopHealthOnly(const VehicleTelemetry& d, bool actuatorPage = false) {
  // Only repaint a status dot when its visible state changes.
  static int lastActRos=-1,lastActEsc=-1,lastActEnc=-1;
  static int lastRos=-1,lastGpsReady=-1,lastGpsFix=-1,lastCam=-1,lastEsc=-1;
  if (actuatorPage) {
    if(lastActRos!=(int)d.rosConnected){ lastActRos=d.rosConnected; drawStatusDot(239,13,healthColor(d.rosConnected),3); }
    if(lastActEsc!=(int)d.escReady){ lastActEsc=d.escReady; drawStatusDot(278,13,healthColor(d.escReady),3); }
    if(lastActEnc!=(int)d.encoderReady){ lastActEnc=d.encoderReady; drawStatusDot(315,13,healthColor(d.encoderReady),3); }
    return;
  }
  if(lastRos!=(int)d.rosConnected){ lastRos=d.rosConnected; drawStatusDot(195,13,healthColor(d.rosConnected),3); }
  if(lastGpsReady!=(int)d.gpsReady || lastGpsFix!=(int)d.gpsFix){ lastGpsReady=d.gpsReady; lastGpsFix=d.gpsFix; drawStatusDot(231,13,d.gpsReady?gpsFixColor(d.gpsFix):C_FAULT,3); }
  if(lastCam!=(int)d.cameraReady){ lastCam=d.cameraReady; drawStatusDot(270,13,healthColor(d.cameraReady),3); }
  if(lastEsc!=(int)d.escReady){ lastEsc=d.escReady; drawStatusDot(309,13,healthColor(d.escReady),3); }
}

inline void drawTopBar(const char* title, const VehicleTelemetry& d, bool showHome,
                       bool actuatorPage = false) {
  tft.fillRect(0, 0, W, TOP_H, C_BG);

  if (showHome) {
    iconHome(14, 14, C_TEXT);
    tft.drawFastVLine(28, 6, 16, C_BORDER);
    drawUiText(title, 36, 5, C_TEXT, C_BG);
  } else {
    drawUiText(title, 8, 5, C_TEXT, C_BG);
  }

  drawTopHealth(d, actuatorPage);
}
