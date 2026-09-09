// ============================================================================
// CameraPage.h — touch-friendly CAMERA submenus for the 320x240 ADV HMI
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

inline int cameraTabX(uint8_t index) {
  return CAM_TAB_X0 + index * (CAM_TAB_W + CAM_TAB_GAP);
}

inline const char* cameraTabName(CameraSubPage tab) {
  switch (tab) {
    case CAM_VIEW: return "VIEW";
    case CAM_DETECT: return "DETECT";
    case CAM_DRIVE: return "DRIVE";
    case CAM_STATUS: return "STATUS";
    default: return "VIEW";
  }
}

inline void drawCameraTabs(CameraSubPage active) {
  const CameraSubPage tabs[4] = {CAM_VIEW, CAM_DETECT, CAM_DRIVE, CAM_STATUS};
  for (uint8_t i = 0; i < 4; ++i) {
    const int x = cameraTabX(i);
    const bool selected = tabs[i] == active;
    const uint16_t fill = selected ? C_PANEL_ALT : C_PANEL;
    const uint16_t border = selected ? C_ACCENT : C_BORDER;
    const uint16_t text = selected ? C_ACCENT : C_TEXT;
    tft.fillRoundRect(x, CAM_TAB_Y, CAM_TAB_W, CAM_TAB_H, 5, fill);
    tft.drawRoundRect(x, CAM_TAB_Y, CAM_TAB_W, CAM_TAB_H, 5, border);
    drawCompactText(cameraTabName(tabs[i]), x + CAM_TAB_W / 2, CAM_TAB_Y + 7,
                    text, fill, TC_DATUM);
  }
}

inline void drawCameraView(const VehicleTelemetry& d) {
  drawCameraIllustration(FULL_CARD_X + 12, CAM_CONTENT_Y + 16);
  const int x = FULL_CARD_X + 112;
  drawUiText(d.cameraReady ? "Camera Ready" : "Camera Offline", x, CAM_CONTENT_Y + 10,
             d.cameraReady ? C_INK : C_FAULT, C_CARD);
  drawSmallText(d.perceptionReady ? "Perception Active" : "Perception Waiting", x,
                CAM_CONTENT_Y + 33, d.perceptionReady ? C_READY : C_WARNING, C_CARD);
  drawThinDivider(x, CAM_CONTENT_Y + 57, 184);
  char fps[20];
  snprintf(fps, sizeof(fps), d.cameraFps > 0.01f ? "%.1f FPS" : "-- FPS", d.cameraFps);
  drawMicroText("LIVE TELEMETRY", x, CAM_CONTENT_Y + 65, C_DISABLED, C_CARD);
  drawUiText(fps, x, CAM_CONTENT_Y + 80, d.cameraReady ? C_INK : C_DISABLED, C_CARD);
}

inline void drawCameraDetect(const VehicleTelemetry& d) {
  char buf[24];
  drawMicroText("OBJECT", FULL_CARD_X + 16, CAM_CONTENT_Y + 12, C_DISABLED, C_CARD);
  char objectBuf[18];
  snprintf(objectBuf, sizeof(objectBuf), "%.17s", d.detectedObject);
  drawValueText(objectBuf, FULL_CARD_X + 16, CAM_CONTENT_Y + 28, C_INK, C_CARD);

  drawMicroText("DISTANCE", FULL_CARD_X + 170, CAM_CONTENT_Y + 12, C_DISABLED, C_CARD);
  if (d.objectDistanceM > 0.01f) snprintf(buf, sizeof(buf), "%.2f m", d.objectDistanceM);
  else snprintf(buf, sizeof(buf), "-- m");
  drawValueText(buf, FULL_CARD_X + 170, CAM_CONTENT_Y + 28, C_INK, C_CARD);

  drawThinDivider(FULL_CARD_X + 14, CAM_CONTENT_Y + 58, FULL_CARD_W - 28);
  drawSmallText("Confidence", FULL_CARD_X + 16, CAM_CONTENT_Y + 67, C_DISABLED, C_CARD);
  snprintf(buf, sizeof(buf), d.confidencePct > 0.01f ? "%.0f%%" : "--", d.confidencePct);
  drawUiText(buf, FULL_CARD_X + 111, CAM_CONTENT_Y + 64, C_INK, C_CARD);

  drawSmallText("Obstacle", FULL_CARD_X + 170, CAM_CONTENT_Y + 67, C_DISABLED, C_CARD);
  drawCompactText(d.obstacleDetected ? "DETECTED" : "NONE", FULL_CARD_X + 245,
                  CAM_CONTENT_Y + 68, d.obstacleDetected ? C_FAULT : C_READY, C_CARD);
}

inline void drawCameraDrive(const VehicleTelemetry& d) {
  drawMicroText("DRIVABLE AREA", FULL_CARD_X + 16, CAM_CONTENT_Y + 12, C_DISABLED, C_CARD);
  drawValueText(d.drivableAreaClear ? "CLEAR" : "BLOCKED", FULL_CARD_X + 16,
                CAM_CONTENT_Y + 28, d.drivableAreaClear ? C_READY : C_FAULT, C_CARD);
  drawStatusDot(FULL_CARD_X + 143, CAM_CONTENT_Y + 40,
                d.drivableAreaClear ? C_READY : C_FAULT, 4);

  drawMicroText("OBSTACLE", FULL_CARD_X + 175, CAM_CONTENT_Y + 12, C_DISABLED, C_CARD);
  drawValueText(d.obstacleDetected ? "YES" : "NONE", FULL_CARD_X + 175,
                CAM_CONTENT_Y + 28, d.obstacleDetected ? C_FAULT : C_READY, C_CARD);

  drawThinDivider(FULL_CARD_X + 14, CAM_CONTENT_Y + 60, FULL_CARD_W - 28);
  drawSmallText("Perception", FULL_CARD_X + 16, CAM_CONTENT_Y + 70, C_DISABLED, C_CARD);
  drawUiText(d.perceptionReady ? "ACTIVE" : "WAIT", FULL_CARD_X + 112,
             CAM_CONTENT_Y + 67, d.perceptionReady ? C_READY : C_WARNING, C_CARD);
  drawSmallText(d.obstacleDetected ? "Path requires safety action" : "Path monitor normal",
                FULL_CARD_X + 16, CAM_CONTENT_Y + 92,
                d.obstacleDetected ? C_FAULT : C_INK, C_CARD);
}

inline void drawCameraStatus(const VehicleTelemetry& d) {
  char fps[16];
  snprintf(fps, sizeof(fps), d.cameraFps > 0.01f ? "%.1f FPS" : "-- FPS", d.cameraFps);
  const int lx = FULL_CARD_X + 18;
  const int rx = FULL_CARD_X + 182;
  drawSmallText("Camera", lx, CAM_CONTENT_Y + 14, C_DISABLED, C_CARD);
  drawUiText(d.cameraReady ? "READY" : "OFFLINE", lx, CAM_CONTENT_Y + 33,
             d.cameraReady ? C_READY : C_FAULT, C_CARD);
  drawSmallText("Perception", rx, CAM_CONTENT_Y + 14, C_DISABLED, C_CARD);
  drawUiText(d.perceptionReady ? "ACTIVE" : "WAIT", rx, CAM_CONTENT_Y + 33,
             d.perceptionReady ? C_READY : C_WARNING, C_CARD);
  drawThinDivider(FULL_CARD_X + 14, CAM_CONTENT_Y + 61, FULL_CARD_W - 28);
  drawSmallText("Frame rate", lx, CAM_CONTENT_Y + 70, C_DISABLED, C_CARD);
  drawUiText(fps, lx, CAM_CONTENT_Y + 87, C_INK, C_CARD);
  drawSmallText("GPS link", rx, CAM_CONTENT_Y + 70, C_DISABLED, C_CARD);
  drawUiText(d.gpsReady ? "READY" : "OFFLINE", rx, CAM_CONTENT_Y + 87,
             d.gpsReady ? C_READY : C_FAULT, C_CARD);
}

inline void drawCameraContent(const VehicleTelemetry& d, CameraSubPage active) {
  drawCard(FULL_CARD_X, CAM_CONTENT_Y, FULL_CARD_W, CAM_CONTENT_H, C_CARD, C_CARD_LINE);
  switch (active) {
    case CAM_DETECT: drawCameraDetect(d); break;
    case CAM_DRIVE: drawCameraDrive(d); break;
    case CAM_STATUS: drawCameraStatus(d); break;
    case CAM_VIEW:
    default: drawCameraView(d); break;
  }
}

inline void drawCameraPage(const VehicleTelemetry& d, CameraSubPage active = CAM_VIEW) {
  tft.fillScreen(C_BG);
  drawTopBar("CAMERA", d, true, false);
  drawCameraTabs(active);
  drawCameraContent(d, active);
  drawBottomMenu(PAGE_CAMERA);
}

inline void updateCameraPage(const VehicleTelemetry& d, CameraSubPage active = CAM_VIEW) {
  // Telemetry only touches the value pixels. Tabs/card/background stay intact.
  updateTopHealthOnly(d, false);
  char buf[32];

  if (active == CAM_VIEW) {
    static bool lastCam = false, lastPer = false, init = false;
    static char lastFps[16] = "";
    if (!init || d.cameraReady != lastCam || d.perceptionReady != lastPer) {
      lastCam = d.cameraReady; lastPer = d.perceptionReady;
      drawUiTextPadded(d.cameraReady ? "Camera Ready" : "Camera Offline", FULL_CARD_X + 112, CAM_CONTENT_Y + 10,
                       d.cameraReady ? C_INK : C_FAULT, C_CARD, 190);
      drawSmallTextPadded(d.perceptionReady ? "Perception Active" : "Perception Waiting", FULL_CARD_X + 112,
                         CAM_CONTENT_Y + 33, d.perceptionReady ? C_READY : C_WARNING, C_CARD, 190);
    }
    snprintf(buf, sizeof(buf), d.cameraFps > 0.01f ? "%.1f FPS" : "-- FPS", d.cameraFps);
    if (!init || strcmp(buf, lastFps) != 0) {
      snprintf(lastFps, sizeof(lastFps), "%s", buf);
      drawUiTextPadded(buf, FULL_CARD_X + 112, CAM_CONTENT_Y + 80,
                       d.cameraReady ? C_INK : C_DISABLED, C_CARD, 120);
    }
    init = true;
    return;
  }

  if (active == CAM_DETECT) {
    static char lastObj[18] = "", lastDist[16] = "", lastConf[12] = "", lastObs[12] = "";
    char obj[18], dist[16], conf[12];
    snprintf(obj, sizeof(obj), "%.17s", d.detectedObject);
    if (d.objectDistanceM > 0.01f) snprintf(dist, sizeof(dist), "%.2f m", d.objectDistanceM); else snprintf(dist, sizeof(dist), "-- m");
    snprintf(conf, sizeof(conf), d.confidencePct > 0.01f ? "%.0f%%" : "--", d.confidencePct);
    const char* obs = d.obstacleDetected ? "DETECTED" : "NONE";
    if (strcmp(obj,lastObj)!=0) { snprintf(lastObj,sizeof(lastObj),"%s",obj); drawValueTextPadded(obj,FULL_CARD_X+16,CAM_CONTENT_Y+28,C_INK,C_CARD,145); }
    if (strcmp(dist,lastDist)!=0) { snprintf(lastDist,sizeof(lastDist),"%s",dist); drawValueTextPadded(dist,FULL_CARD_X+170,CAM_CONTENT_Y+28,C_INK,C_CARD,132); }
    if (strcmp(conf,lastConf)!=0) { snprintf(lastConf,sizeof(lastConf),"%s",conf); drawUiTextPadded(conf,FULL_CARD_X+111,CAM_CONTENT_Y+64,C_INK,C_CARD,56); }
    if (strcmp(obs,lastObs)!=0) { snprintf(lastObs,sizeof(lastObs),"%s",obs); drawCompactTextPadded(obs,FULL_CARD_X+245,CAM_CONTENT_Y+68,d.obstacleDetected?C_FAULT:C_READY,C_CARD,63); }
    return;
  }

  if (active == CAM_DRIVE) {
    static int lastDrv=-1,lastObs=-1,lastPer=-1;
    if (lastDrv!=(int)d.drivableAreaClear) { lastDrv=d.drivableAreaClear; drawValueTextPadded(d.drivableAreaClear?"CLEAR":"BLOCKED",FULL_CARD_X+16,CAM_CONTENT_Y+28,d.drivableAreaClear?C_READY:C_FAULT,C_CARD,145); drawStatusDot(FULL_CARD_X+143,CAM_CONTENT_Y+40,d.drivableAreaClear?C_READY:C_FAULT,4); }
    if (lastObs!=(int)d.obstacleDetected) { lastObs=d.obstacleDetected; drawValueTextPadded(d.obstacleDetected?"YES":"NONE",FULL_CARD_X+175,CAM_CONTENT_Y+28,d.obstacleDetected?C_FAULT:C_READY,C_CARD,126); drawSmallTextPadded(d.obstacleDetected?"Path requires safety action":"Path monitor normal",FULL_CARD_X+16,CAM_CONTENT_Y+92,d.obstacleDetected?C_FAULT:C_INK,C_CARD,286); }
    if (lastPer!=(int)d.perceptionReady) { lastPer=d.perceptionReady; drawUiTextPadded(d.perceptionReady?"ACTIVE":"WAIT",FULL_CARD_X+112,CAM_CONTENT_Y+67,d.perceptionReady?C_READY:C_WARNING,C_CARD,78); }
    return;
  }

  // STATUS
  static int lastCam=-1,lastPer=-1,lastGps=-1; static char lastFps[16]="";
  const int lx=FULL_CARD_X+18, rx=FULL_CARD_X+182;
  if (lastCam!=(int)d.cameraReady) { lastCam=d.cameraReady; drawUiTextPadded(d.cameraReady?"READY":"OFFLINE",lx,CAM_CONTENT_Y+33,d.cameraReady?C_READY:C_FAULT,C_CARD,142); }
  if (lastPer!=(int)d.perceptionReady) { lastPer=d.perceptionReady; drawUiTextPadded(d.perceptionReady?"ACTIVE":"WAIT",rx,CAM_CONTENT_Y+33,d.perceptionReady?C_READY:C_WARNING,C_CARD,116); }
  snprintf(buf,sizeof(buf),d.cameraFps>0.01f?"%.1f FPS":"-- FPS",d.cameraFps);
  if (strcmp(buf,lastFps)!=0) { snprintf(lastFps,sizeof(lastFps),"%s",buf); drawUiTextPadded(buf,lx,CAM_CONTENT_Y+87,C_INK,C_CARD,140); }
  if (lastGps!=(int)d.gpsReady) { lastGps=d.gpsReady; drawUiTextPadded(d.gpsReady?"READY":"OFFLINE",rx,CAM_CONTENT_Y+87,d.gpsReady?C_READY:C_FAULT,C_CARD,116); }
}
