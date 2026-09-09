// ============================================================================
// GpsPage.h — GNSS + waypoint/navigation workflow, optimized for 320x240 touch
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

inline bool waypointSaveEnabled(const VehicleTelemetry& d) {
  return d.rosConnected && d.gpsReady && (d.gpsFix == GPS_3D_FIX || d.gpsFix == GPS_2D_FIX);
}

inline bool waypointGoEnabled(const VehicleTelemetry& d) {
  return d.rosConnected && d.selectedWaypoint < HMI_WAYPOINT_COUNT && d.waypointSaved[d.selectedWaypoint] &&
         d.mode == MODE_AUTO && d.systemStatus == SYS_READY;
}

inline const char* activeNavigationLine(const VehicleTelemetry& d) {
  static char line[42];
  const char* target = navigationHasTarget(d) ? d.activeTarget : d.waypointName[d.selectedWaypoint];
  switch (d.navigationStatus) {
    case NAV_SELECTED: snprintf(line, sizeof(line), "Target: %.20s", target); break;
    case NAV_QUEUED: snprintf(line, sizeof(line), "Menunggu Nav2: %.18s", target); break;
    case NAV_NAVIGATING: snprintf(line, sizeof(line), "Menuju: %.20s", target); break;
    case NAV_ARRIVED: snprintf(line, sizeof(line), "Tiba di: %.20s", target); break;
    case NAV_STOPPED: snprintf(line, sizeof(line), "Navigasi dihentikan"); break;
    case NAV_FAILED: snprintf(line, sizeof(line), "Navigasi gagal: %.18s", target); break;
    case NAV_IDLE:
    default: snprintf(line, sizeof(line), "Belum ada target"); break;
  }
  return line;
}

inline void drawWaypointButton(int x, int w, const char* label, bool enabled,
                               bool accent = false, bool danger = false) {
  const uint16_t fill = C_PANEL_ALT;
  const uint16_t border = danger ? C_FAULT : (accent ? C_ACCENT : C_BORDER);
  const uint16_t fg = !enabled ? C_DISABLED : (danger ? C_FAULT : (accent ? C_ACCENT : C_TEXT));
  tft.fillRoundRect(x, GPS_WP_Y, w, GPS_WP_H, 5, fill);
  tft.drawRoundRect(x, GPS_WP_Y, w, GPS_WP_H, 5, enabled ? border : C_BORDER);
  drawCompactText(label, x + w / 2, GPS_WP_Y + 9, fg, fill, TC_DATUM);
}

inline void drawGpsContent(const VehicleTelemetry& d) {
  drawCard(FULL_CARD_X, FULL_CARD_Y, FULL_CARD_W, FULL_CARD_H, C_CARD, C_CARD_LINE);

  char buf[32];
  iconGps(FULL_CARD_X + 23, FULL_CARD_Y + 23, C_ACCENT, C_CARD);
  snprintf(buf, sizeof(buf), "%.0f", d.headingDeg);
  drawValueTextPadded(buf, FULL_CARD_X + 43, FULL_CARD_Y + 9, C_INK, C_CARD, 58);
  drawDegreeMark(FULL_CARD_X + 103, FULL_CARD_Y + 15, C_INK, C_CARD);
  drawMicroText("HEADING", FULL_CARD_X + 43, FULL_CARD_Y + 38, C_DISABLED, C_CARD);

  drawMicroText("LAT", FULL_CARD_X + 112, FULL_CARD_Y + 8, C_DISABLED, C_CARD);
  snprintf(buf, sizeof(buf), "%.6f", d.latitude);
  drawCompactText(buf, FULL_CARD_X + 142, FULL_CARD_Y + 6, C_INK, C_CARD);
  drawMicroText("LON", FULL_CARD_X + 112, FULL_CARD_Y + 28, C_DISABLED, C_CARD);
  snprintf(buf, sizeof(buf), "%.6f", d.longitude);
  drawCompactText(buf, FULL_CARD_X + 142, FULL_CARD_Y + 26, C_INK, C_CARD);

  drawThinDivider(FULL_CARD_X + 12, FULL_CARD_Y + 51, FULL_CARD_W - 24);
  drawMicroText("SAT", FULL_CARD_X + 14, FULL_CARD_Y + 58, C_DISABLED, C_CARD);
  snprintf(buf, sizeof(buf), "%u", d.satellites);
  drawUiText(buf, FULL_CARD_X + 14, FULL_CARD_Y + 72, C_INK, C_CARD);

  drawMicroText("HDOP", FULL_CARD_X + 62, FULL_CARD_Y + 58, C_DISABLED, C_CARD);
  snprintf(buf, sizeof(buf), "%.2f", d.hdop);
  drawUiText(buf, FULL_CARD_X + 62, FULL_CARD_Y + 72, C_INK, C_CARD);

  drawMicroText("GNSS", FULL_CARD_X + 119, FULL_CARD_Y + 58, C_DISABLED, C_CARD);
  drawCompactText(gpsFixText(d.gpsFix), FULL_CARD_X + 119, FULL_CARD_Y + 76,
                  gpsFixColor(d.gpsFix), C_CARD);

  drawMicroText("IMU", FULL_CARD_X + 185, FULL_CARD_Y + 58, C_DISABLED, C_CARD);
  drawCompactText(d.imuReady ? "READY" : "OFF", FULL_CARD_X + 185, FULL_CARD_Y + 76,
                  d.imuReady ? C_READY : C_FAULT, C_CARD);

  drawMicroText("SPEED", FULL_CARD_X + 244, FULL_CARD_Y + 58, C_DISABLED, C_CARD);
  snprintf(buf, sizeof(buf), "%.1f", d.speedKmh);
  drawCompactText(buf, FULL_CARD_X + 244, FULL_CARD_Y + 76, C_INK, C_CARD);
  drawMicroText("km/h", FULL_CARD_X + 276, FULL_CARD_Y + 79, C_INK, C_CARD);

  // Target/navigation status is deliberately high-contrast and visible at a glance.
  tft.fillRoundRect(FULL_CARD_X + 8, FULL_CARD_Y + 82, FULL_CARD_W - 16, 20, 5, C_PANEL);
  drawCompactText(activeNavigationLine(d), FULL_CARD_X + 17, FULL_CARD_Y + 87,
                  d.navigationStatus == NAV_FAILED ? C_FAULT :
                  (d.navigationStatus == NAV_ARRIVED ? C_READY : C_TEXT), C_PANEL);

  const uint8_t selected = d.selectedWaypoint < HMI_WAYPOINT_COUNT ? d.selectedWaypoint : 0;
  drawWaypointButton(GPS_WP_PREV_X, GPS_WP_PREV_W, "<", true);
  drawWaypointButton(GPS_WP_NAME_X, GPS_WP_NAME_W, d.waypointName[selected], true,
                     d.waypointSaved[selected]);
  drawWaypointButton(GPS_WP_NEXT_X, GPS_WP_NEXT_W, ">", true);
  drawWaypointButton(GPS_WP_SAVE_X, GPS_WP_SAVE_W, "SAVE", waypointSaveEnabled(d));
  drawWaypointButton(GPS_WP_GO_X, GPS_WP_GO_W, "GO", waypointGoEnabled(d), true);
  drawWaypointButton(GPS_WP_STOP_X, GPS_WP_STOP_W, "STOP", true, false, true);
}

inline void drawGpsPage(const VehicleTelemetry& d) {
  tft.fillScreen(C_BG);
  drawTopBar("GPS / NAV", d, true, false);
  drawGpsContent(d);
  drawBottomMenu(PAGE_GPS);
}

inline void updateGpsPage(const VehicleTelemetry& d) {
  // Partial redraw only: static GPS card and buttons are never repainted at telemetry rate.
  updateTopHealthOnly(d, false);
  static char lastHead[12]="", lastLat[20]="", lastLon[20]="", lastSat[8]="", lastHdop[12]="", lastSpeed[12]="", lastNav[42]="";
  static GpsFixState lastFix=GPS_LOST; static int lastImu=-1;
  static uint8_t lastSel=255; static int lastSaved=-1; static VehicleMode lastMode=MODE_AUTO;
  static SystemStatus lastSys=SYS_OFF; static int lastGpsReady=-1; static char lastName[HMI_WAYPOINT_NAME_LEN]="";
  char buf[32];

  snprintf(buf,sizeof(buf),"%.0f",d.headingDeg);
  if(strcmp(buf,lastHead)!=0){ snprintf(lastHead,sizeof(lastHead),"%s",buf); drawValueTextPadded(buf,FULL_CARD_X+43,FULL_CARD_Y+9,C_INK,C_CARD,58); drawDegreeMark(FULL_CARD_X+103,FULL_CARD_Y+15,C_INK,C_CARD); }
  snprintf(buf,sizeof(buf),"%.6f",d.latitude);
  if(strcmp(buf,lastLat)!=0){ snprintf(lastLat,sizeof(lastLat),"%s",buf); drawCompactTextPadded(buf,FULL_CARD_X+142,FULL_CARD_Y+6,C_INK,C_CARD,156); }
  snprintf(buf,sizeof(buf),"%.6f",d.longitude);
  if(strcmp(buf,lastLon)!=0){ snprintf(lastLon,sizeof(lastLon),"%s",buf); drawCompactTextPadded(buf,FULL_CARD_X+142,FULL_CARD_Y+26,C_INK,C_CARD,156); }
  snprintf(buf,sizeof(buf),"%u",d.satellites);
  if(strcmp(buf,lastSat)!=0){ snprintf(lastSat,sizeof(lastSat),"%s",buf); drawUiTextPadded(buf,FULL_CARD_X+14,FULL_CARD_Y+72,C_INK,C_CARD,42); }
  snprintf(buf,sizeof(buf),"%.2f",d.hdop);
  if(strcmp(buf,lastHdop)!=0){ snprintf(lastHdop,sizeof(lastHdop),"%s",buf); drawUiTextPadded(buf,FULL_CARD_X+62,FULL_CARD_Y+72,C_INK,C_CARD,54); }
  if(lastFix!=d.gpsFix){ lastFix=d.gpsFix; drawCompactTextPadded(gpsFixText(d.gpsFix),FULL_CARD_X+119,FULL_CARD_Y+76,gpsFixColor(d.gpsFix),C_CARD,63); }
  if(lastImu!=(int)d.imuReady){ lastImu=d.imuReady; drawCompactTextPadded(d.imuReady?"READY":"OFF",FULL_CARD_X+185,FULL_CARD_Y+76,d.imuReady?C_READY:C_FAULT,C_CARD,56); }
  snprintf(buf,sizeof(buf),"%.1f",d.speedKmh);
  if(strcmp(buf,lastSpeed)!=0){ snprintf(lastSpeed,sizeof(lastSpeed),"%s",buf); drawCompactTextPadded(buf,FULL_CARD_X+244,FULL_CARD_Y+76,C_INK,C_CARD,38); }

  const char* nav=activeNavigationLine(d);
  if(strcmp(nav,lastNav)!=0){ snprintf(lastNav,sizeof(lastNav),"%s",nav); drawCompactTextPadded(nav,FULL_CARD_X+17,FULL_CARD_Y+87,d.navigationStatus==NAV_FAILED?C_FAULT:(d.navigationStatus==NAV_ARRIVED?C_READY:C_TEXT),C_PANEL,FULL_CARD_W-34); }

  const uint8_t sel=d.selectedWaypoint<HMI_WAYPOINT_COUNT?d.selectedWaypoint:0;
  const int saved=d.waypointSaved[sel]?1:0;
  if(lastSel!=sel || lastSaved!=saved || lastMode!=d.mode || lastSys!=d.systemStatus || lastGpsReady!=(int)d.gpsReady || strcmp(lastName,d.waypointName[sel])!=0){
    lastSel=sel; lastSaved=saved; lastMode=d.mode; lastSys=d.systemStatus; lastGpsReady=d.gpsReady; snprintf(lastName,sizeof(lastName),"%s",d.waypointName[sel]);
    drawWaypointButton(GPS_WP_PREV_X,GPS_WP_PREV_W,"<",true);
    char shortName[13]; snprintf(shortName,sizeof(shortName),"%.12s",d.waypointName[sel]);
    drawWaypointButton(GPS_WP_NAME_X,GPS_WP_NAME_W,shortName,true,d.waypointSaved[sel]);
    drawWaypointButton(GPS_WP_NEXT_X,GPS_WP_NEXT_W,">",true);
    drawWaypointButton(GPS_WP_SAVE_X,GPS_WP_SAVE_W,"SAVE",waypointSaveEnabled(d));
    drawWaypointButton(GPS_WP_GO_X,GPS_WP_GO_W,"GO",waypointGoEnabled(d),true);
    drawWaypointButton(GPS_WP_STOP_X,GPS_WP_STOP_W,"STOP",true,false,true);
  }
}
