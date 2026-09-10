// ============================================================================
// Telemetry.h — model state tunggal HMI; semua halaman membaca snapshot ini.
// ============================================================================
#pragma once

#include <cstdint>
#include <cstdio>
#include <cstring>
#include "Config.h"

struct TelemetryGroupStamp {
  uint32_t lastRxMs{0U};
  uint32_t sourceAgeMs{0xFFFFFFFFUL};
  uint32_t seq{0U};
  bool valid{false};
  bool fresh{false};
};

struct EscExtendedTelemetry {
  TelemetryGroupStamp power{};
  TelemetryGroupStamp motors{};
  TelemetryGroupStamp encoder{};
  TelemetryGroupStamp link{};
  TelemetryGroupStamp performance{};
  float vbusV{0.0F}, motorCurrentA{0.0F}, inputCurrentA{0.0F};
  float iqA{0.0F}, idA{0.0F}, duty{0.0F}, mosTempC{0.0F}, motorTempC{0.0F};
  uint8_t faultCode{0U};
  float leftVbusV{0.0F}, leftCurrentA{0.0F}, leftDuty{0.0F}, leftRpm{0.0F};
  float rightVbusV{0.0F}, rightCurrentA{0.0F}, rightDuty{0.0F}, rightRpm{0.0F};
  uint8_t leftFault{0U}, rightFault{0U};
  int32_t encoderRaw{0}, encoderSpan{0}, encoderTarget{0};
  bool calibrated{false}, homed{false}, encoderSynced{false}, encoderInverted{false};
  float encoderPositionDeg{0.0F};
  uint32_t uartBaud{0U};
  char owner[12]{"UNKNOWN"};
  uint32_t commandAgeMs{0xFFFFFFFFUL}, feedbackAgeMs{0xFFFFFFFFUL};
};

struct PerceptionExtendedTelemetry {
  TelemetryGroupStamp camera{};
  TelemetryGroupStamp detection{};
  TelemetryGroupStamp lane{};
  TelemetryGroupStamp drivable{};
  TelemetryGroupStamp obstacle{};
  TelemetryGroupStamp performance{};
  float inferenceLatencyMs{0.0F};
  uint32_t droppedFrames{0U}, recoveryCount{0U};
  char backend[12]{"UNKNOWN"};
  uint16_t objectCount{0U}, trackCount{0U}, missedTracks{0U};
  int32_t nearestTrackId{-1};
  float nearestLateralM{0.0F};
  bool laneValid{false};
  float laneCenterOffsetM{0.0F}, laneConfidencePct{0.0F}, roadWidthM{0.0F};
  float laneLeftClearanceM{0.0F}, laneRightClearanceM{0.0F}, laneHeadingErrorDeg{0.0F};
  bool drivableValid{false};
  float drivableFractionPct{0.0F}, drivableLeftM{0.0F}, drivableRightM{0.0F}, farLookaheadM{0.0F};
  uint16_t obstacleCount{0U};
  bool obstacleValid{false}, obstacleBlocked{false};
  float obstacleDistanceM{0.0F}, obstacleLateralM{0.0F};
  uint32_t rawDetectionCount{0U}, confirmedObstacleCount{0U};
};

struct NavigationExtendedTelemetry {
  TelemetryGroupStamp pose{};
  TelemetryGroupStamp odom{};
  TelemetryGroupStamp imuMag{};
  TelemetryGroupStamp nav2{};
  TelemetryGroupStamp costmap{};
  TelemetryGroupStamp control{};
  float mapX{0.0F}, mapY{0.0F}, mapYawDeg{0.0F};
  float covarianceX{0.0F}, covarianceY{0.0F}, yawVariance{0.0F};
  float odomX{0.0F}, odomY{0.0F}, odomYawDeg{0.0F}, odomLinearMps{0.0F}, odomYawRateRps{0.0F};
  float imuYawDeg{0.0F}, fusedHeadingDeg{0.0F}, headingDisagreementDeg{0.0F};
  bool pathValid{false}, nav2StackReady{false}, costmapReady{false}, costmapBlocked{false};
  uint32_t cmdAgeMs{0xFFFFFFFFUL};
  float commandLinearMps{0.0F}, commandAngularRps{0.0F};
  uint16_t obstaclePointCount{0U}, pathRelevantCount{0U};
  char plannerState[12]{"UNKNOWN"}, controllerState[12]{"UNKNOWN"}, smootherState[12]{"UNKNOWN"};
};

struct VehicleTelemetry {
  SystemStatus systemStatus{SYS_INITIALIZING};
  VehicleMode mode{MODE_AUTO};
  VehicleState state{STATE_STOPPED};
  bool rosConnected{false};
  bool eStop{false};
  bool vescConnected{false};
  bool escFresh{false};
  bool perceptionFresh{false};
  bool navigationFresh{false};
  uint32_t escAgeMs{0xFFFFFFFFUL};
  uint32_t perceptionAgeMs{0xFFFFFFFFUL};
  uint32_t navigationAgeMs{0xFFFFFFFFUL};

  EscExtendedTelemetry escx{};
  PerceptionExtendedTelemetry perx{};
  NavigationExtendedTelemetry navx{};

  float speedKmh{0.0F};
  float driveTargetMps{0.0F};
  float driveActualMps{0.0F};
  float motorErpm{0.0F};
  float motorRpm{0.0F};
  float vbusV{0.0F};
  bool vbusValid{false};
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

inline void updateTelemetryGroupFreshness(TelemetryGroupStamp &g, uint32_t now,
                                           uint32_t timeoutMs) {
  if (!g.valid || g.lastRxMs == 0U || g.sourceAgeMs == 0xFFFFFFFFUL) {
    g.fresh = false;
    return;
  }
  const uint32_t localAge = static_cast<uint32_t>(now - g.lastRxMs);
  g.fresh = localAge <= timeoutMs && g.sourceAgeMs <= timeoutMs;
}

inline void updateExtendedFreshness(VehicleTelemetry &t, uint32_t now) {
  updateTelemetryGroupFreshness(t.escx.power, now, 1500U);
  updateTelemetryGroupFreshness(t.escx.motors, now, 1500U);
  updateTelemetryGroupFreshness(t.escx.encoder, now, 2000U);
  updateTelemetryGroupFreshness(t.escx.link, now, 2500U);
  updateTelemetryGroupFreshness(t.escx.performance, now, 2500U);
  updateTelemetryGroupFreshness(t.perx.camera, now, 2000U);
  updateTelemetryGroupFreshness(t.perx.detection, now, 1500U);
  updateTelemetryGroupFreshness(t.perx.lane, now, 1500U);
  updateTelemetryGroupFreshness(t.perx.drivable, now, 1500U);
  updateTelemetryGroupFreshness(t.perx.obstacle, now, 1500U);
  updateTelemetryGroupFreshness(t.perx.performance, now, 2500U);
  updateTelemetryGroupFreshness(t.navx.pose, now, 2500U);
  updateTelemetryGroupFreshness(t.navx.odom, now, 1500U);
  updateTelemetryGroupFreshness(t.navx.imuMag, now, 1500U);
  updateTelemetryGroupFreshness(t.navx.nav2, now, 2500U);
  updateTelemetryGroupFreshness(t.navx.costmap, now, 2500U);
  updateTelemetryGroupFreshness(t.navx.control, now, 1500U);
  t.vbusValid = t.escx.power.valid && t.escx.power.fresh;
  if (t.vbusValid) t.vbusV = t.escx.vbusV;
}

inline uint32_t extendedFreshMask(const VehicleTelemetry &t) {
  uint32_t mask = 0U;
  const bool flags[] = {
      t.escx.power.fresh, t.escx.motors.fresh, t.escx.encoder.fresh,
      t.escx.link.fresh, t.escx.performance.fresh,
      t.perx.camera.fresh, t.perx.detection.fresh, t.perx.lane.fresh,
      t.perx.drivable.fresh, t.perx.obstacle.fresh, t.perx.performance.fresh,
      t.navx.pose.fresh, t.navx.odom.fresh, t.navx.imuMag.fresh,
      t.navx.nav2.fresh, t.navx.costmap.fresh, t.navx.control.fresh};
  for (uint8_t i = 0U; i < sizeof(flags) / sizeof(flags[0]); ++i)
    if (flags[i]) mask |= (1UL << i);
  return mask;
}

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
