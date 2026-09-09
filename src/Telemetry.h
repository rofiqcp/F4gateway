// ============================================================================
// Telemetry.h — model state tunggal HMI; semua halaman membaca snapshot ini.
// ============================================================================
#pragma once

#include <cstdint>
#include <cstring>
#include "Config.h"

struct VehicleTelemetry {
  SystemStatus systemStatus{SYS_INITIALIZING};
  VehicleMode mode{MODE_AUTO};
  VehicleState state{STATE_STOPPED};
  bool rosConnected{false};
  bool eStop{false};
  bool vescConnected{false};

  float speedKmh{0.0F};
  float driveTargetMps{0.0F};
  float driveActualMps{0.0F};
  float motorErpm{0.0F};
  float motorRpm{0.0F};
  float steeringTargetDeg{0.0F};
  float steeringActualDeg{0.0F};
  float steeringErrorDeg{0.0F};
  char steeringTestState[12]{"IDLE"};
  float steeringTestAngleDeg{STEER_TEST_ANGLE_DEFAULT_DEG};
  bool escReady{false};
  bool encoderReady{false};
  uint8_t manualSpeedPct{MANUAL_SPEED_DEFAULT};
  float driveScale{1.0F};

  bool gpsReady{false};
  GpsFixState gpsFix{GPS_NO_FIX};
  double latitude{0.0};
  double longitude{0.0};
  uint8_t satellites{0};
  float hdop{0.0F};
  float haccM{0.0F};
  float gnssAgeSec{99.0F};
  float headingDeg{0.0F};

  bool imuReady{false};
  float gyroZRps{0.0F};
  bool magReady{false};

  bool cameraReady{false};
  bool perceptionReady{false};
  bool perceptionInference{false};
  float cameraFps{0.0F};
  char detectedObject[24]{"NONE"};
  float objectDistanceM{0.0F};
  float confidencePct{0.0F};
  bool drivableAreaClear{false};
  bool obstacleDetected{false};
  char laneState[20]{"UNKNOWN"};

  bool motionReady{false};
  bool nav2Ready{false};
  char localizationState[24]{"WAIT"};
  char gnssStatus[20]{"WAIT"};
  char imuStatus[20]{"WAIT"};
  char ekfLocalStatus[20]{"WAIT"};
  char ekfGlobalStatus[20]{"WAIT"};

  bool waypointSaved[HMI_WAYPOINT_COUNT]{};
  char waypointName[HMI_WAYPOINT_COUNT][HMI_WAYPOINT_NAME_LEN]{};
  uint8_t selectedWaypoint{0};
  char activeTarget[HMI_WAYPOINT_NAME_LEN]{"NONE"};
  NavigationStatus navigationStatus{NAV_IDLE};

  // Status transaksi konfigurasi HMI -> ROS. Nilai aktual hanya diubah setelah ACK/readback.
  bool configPending{false};
  bool configLastOk{true};
  uint16_t configTxn{0};
  char configKey[20]{"NONE"};
  char configMessage[32]{"READY"};
};

inline VehicleTelemetry defaultTelemetry() {
  VehicleTelemetry t{};
  const char* defaults[HMI_WAYPOINT_COUNT] = {"Titik A", "Titik B", "Titik C", "Titik D"};
  for (uint8_t i = 0; i < HMI_WAYPOINT_COUNT; ++i) {
    snprintf(t.waypointName[i], HMI_WAYPOINT_NAME_LEN, "%s", defaults[i]);
  }
  return t;
}

inline const char* systemStatusText(SystemStatus s) {
  switch (s) {
    case SYS_OFF: return "OFF";
    case SYS_STARTING: return "STARTING";
    case SYS_INITIALIZING: return "INITIALIZING";
    case SYS_READY: return "READY";
    case SYS_NOT_READY: return "NOT READY";
    case SYS_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

inline const char* modeText(VehicleMode mode) { return mode == MODE_MANUAL ? "MANUAL" : "AUTO"; }

inline const char* vehicleStateText(VehicleState s) {
  switch (s) {
    case STATE_STANDBY: return "STANDBY";
    case STATE_RUNNING: return "RUNNING";
    case STATE_STOPPED: return "STOPPED";
    case STATE_FAULT: return "FAULT";
    default: return "UNKNOWN";
  }
}

inline const char* gpsFixText(GpsFixState s) {
  switch (s) {
    case GPS_LOST: return "LOST";
    case GPS_NO_FIX: return "NO FIX";
    case GPS_2D_FIX: return "2D";
    case GPS_3D_FIX: return "3D FIX";
    case GPS_DEGRADED: return "DEGRADED";
    default: return "NO FIX";
  }
}

inline uint16_t healthColor(bool ok) { return ok ? C_READY : C_FAULT; }
inline uint16_t systemStatusColor(SystemStatus s) {
  if (s == SYS_READY) return C_READY;
  if (s == SYS_FAULT || s == SYS_NOT_READY || s == SYS_OFF) return C_FAULT;
  return C_WARNING;
}
inline uint16_t gpsFixColor(GpsFixState s) {
  if (s == GPS_3D_FIX) return C_READY;
  if (s == GPS_2D_FIX || s == GPS_DEGRADED) return C_WARNING;
  return C_FAULT;
}

inline const char* navigationStatusText(NavigationStatus s) {
  switch (s) {
    case NAV_SELECTED: return "SELECTED";
    case NAV_QUEUED: return "QUEUED";
    case NAV_NAVIGATING: return "NAVIGATING";
    case NAV_ARRIVED: return "ARRIVED";
    case NAV_STOPPED: return "STOPPED";
    case NAV_FAILED: return "FAILED";
    case NAV_IDLE:
    default: return "IDLE";
  }
}

inline bool navigationHasTarget(const VehicleTelemetry& d) {
  return strcmp(d.activeTarget, "NONE") != 0 && d.activeTarget[0] != '\0';
}
