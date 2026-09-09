// ============================================================================
// UiMenu.h — tree menu data-driven; setiap node harus reachable dari OVERVIEW.
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "Config.h"

struct UiState {
  UiMenuId menu{UiMenuId::SPLASH};
  uint8_t selectedChild{0};
  bool editing{false};
  float editValue{0.0F};
  uint16_t nextTxn{1};
  uint32_t pendingSinceMs{0};
};

inline const char* menuTitle(UiMenuId id) {
  switch (id) {
    case UiMenuId::SPLASH: return "SPLASH";
    case UiMenuId::OVERVIEW: return "OVERVIEW";
    case UiMenuId::ESC_ROOT: return "ESC";
    case UiMenuId::ESC_OVERVIEW: return "ESC OVERVIEW";
    case UiMenuId::ESC_MODE: return "OPERATOR MODE";
    case UiMenuId::ESC_STEERING: return "STEERING";
    case UiMenuId::ESC_STEERING_LIVE: return "STEERING LIVE";
    case UiMenuId::ESC_STEERING_TEST_ANGLE: return "TEST ANGLE";
    case UiMenuId::ESC_STEERING_CAL: return "STEERING CAL";
    case UiMenuId::ESC_DRIVE: return "DRIVE";
    case UiMenuId::ESC_DRIVE_LIVE: return "DRIVE LIVE";
    case UiMenuId::ESC_MANUAL_SPEED: return "MANUAL SPEED";
    case UiMenuId::ESC_DRIVE_SCALE: return "DRIVE SCALE";
    case UiMenuId::ESC_POWER: return "FOC / POWER";
    case UiMenuId::ESC_LINK: return "COMMUNICATION";
    case UiMenuId::ESC_MANUAL_TEST: return "MANUAL TEST";
    case UiMenuId::PERCEPTION_ROOT: return "PERCEPTION";
    case UiMenuId::PERCEPTION_OVERVIEW: return "PER OVERVIEW";
    case UiMenuId::PERCEPTION_CAMERA: return "CAMERA";
    case UiMenuId::PERCEPTION_DETECTION: return "DETECTION";
    case UiMenuId::PERCEPTION_DETECTION_LIVE: return "DETECTION LIVE";
    case UiMenuId::PERCEPTION_INFERENCE: return "INFERENCE";
    case UiMenuId::PERCEPTION_LANE: return "LANE";
    case UiMenuId::PERCEPTION_OBSTACLE: return "OBSTACLE";
    case UiMenuId::PERCEPTION_PERFORMANCE: return "PERFORMANCE";
    case UiMenuId::PERCEPTION_TEST: return "PER TEST WIZARD";
    case UiMenuId::NAVIGATION_ROOT: return "NAVIGATION";
    case UiMenuId::NAVIGATION_OVERVIEW: return "NAV OVERVIEW";
    case UiMenuId::NAV_LOCALIZATION: return "LOCALIZATION";
    case UiMenuId::NAV_GNSS: return "GNSS";
    case UiMenuId::NAV_IMU: return "IMU";
    case UiMenuId::NAV_MAG: return "MAGNETOMETER";
    case UiMenuId::NAV_EKF_LOCAL: return "EKF LOCAL";
    case UiMenuId::NAV_EKF_GLOBAL: return "EKF GLOBAL";
    case UiMenuId::NAV_MISSION: return "MISSION";
    case UiMenuId::NAV_MISSION_GO: return "GO WAYPOINT";
    case UiMenuId::NAV_MISSION_SAVE: return "SAVE CURRENT";
    case UiMenuId::NAV_MISSION_STOP: return "STOP NAVIGATION";
    case UiMenuId::NAV_NAV2: return "NAV2";
    case UiMenuId::NAV_PLANNER: return "SMAC PLANNER";
    case UiMenuId::NAV_MPPI: return "MPPI";
    case UiMenuId::NAV_SMOOTHER: return "SMOOTHER";
    case UiMenuId::NAV_COSTMAP: return "COSTMAP";
    case UiMenuId::NAV_SAFETY: return "SAFETY";
    case UiMenuId::NAV_TEST: return "NAV TEST WIZARD";
    default: return "UNKNOWN";
  }
}

inline UiMenuId menuParent(UiMenuId id) {
  switch (id) {
    case UiMenuId::ESC_ROOT:
    case UiMenuId::PERCEPTION_ROOT:
    case UiMenuId::NAVIGATION_ROOT: return UiMenuId::OVERVIEW;
    case UiMenuId::ESC_OVERVIEW:
    case UiMenuId::ESC_MODE:
    case UiMenuId::ESC_STEERING:
    case UiMenuId::ESC_DRIVE:
    case UiMenuId::ESC_POWER:
    case UiMenuId::ESC_LINK:
    case UiMenuId::ESC_MANUAL_TEST: return UiMenuId::ESC_ROOT;
    case UiMenuId::ESC_STEERING_LIVE:
    case UiMenuId::ESC_STEERING_TEST_ANGLE:
    case UiMenuId::ESC_STEERING_CAL: return UiMenuId::ESC_STEERING;
    case UiMenuId::ESC_DRIVE_LIVE:
    case UiMenuId::ESC_MANUAL_SPEED:
    case UiMenuId::ESC_DRIVE_SCALE: return UiMenuId::ESC_DRIVE;
    case UiMenuId::PERCEPTION_OVERVIEW:
    case UiMenuId::PERCEPTION_CAMERA:
    case UiMenuId::PERCEPTION_DETECTION:
    case UiMenuId::PERCEPTION_LANE:
    case UiMenuId::PERCEPTION_OBSTACLE:
    case UiMenuId::PERCEPTION_PERFORMANCE:
    case UiMenuId::PERCEPTION_TEST: return UiMenuId::PERCEPTION_ROOT;
    case UiMenuId::PERCEPTION_DETECTION_LIVE:
    case UiMenuId::PERCEPTION_INFERENCE: return UiMenuId::PERCEPTION_DETECTION;
    case UiMenuId::NAVIGATION_OVERVIEW:
    case UiMenuId::NAV_LOCALIZATION:
    case UiMenuId::NAV_MISSION:
    case UiMenuId::NAV_NAV2:
    case UiMenuId::NAV_SAFETY:
    case UiMenuId::NAV_TEST: return UiMenuId::NAVIGATION_ROOT;
    case UiMenuId::NAV_GNSS:
    case UiMenuId::NAV_IMU:
    case UiMenuId::NAV_MAG:
    case UiMenuId::NAV_EKF_LOCAL:
    case UiMenuId::NAV_EKF_GLOBAL: return UiMenuId::NAV_LOCALIZATION;
    case UiMenuId::NAV_MISSION_GO:
    case UiMenuId::NAV_MISSION_SAVE:
    case UiMenuId::NAV_MISSION_STOP: return UiMenuId::NAV_MISSION;
    case UiMenuId::NAV_PLANNER:
    case UiMenuId::NAV_MPPI:
    case UiMenuId::NAV_SMOOTHER:
    case UiMenuId::NAV_COSTMAP: return UiMenuId::NAV_NAV2;
    case UiMenuId::SPLASH:
    case UiMenuId::OVERVIEW:
    default: return UiMenuId::OVERVIEW;
  }
}

inline const UiMenuId* menuChildren(UiMenuId id, uint8_t& count) {
  static const UiMenuId overview[] = {UiMenuId::ESC_ROOT, UiMenuId::PERCEPTION_ROOT, UiMenuId::NAVIGATION_ROOT};
  static const UiMenuId esc[] = {UiMenuId::ESC_OVERVIEW, UiMenuId::ESC_MODE, UiMenuId::ESC_STEERING, UiMenuId::ESC_DRIVE, UiMenuId::ESC_POWER, UiMenuId::ESC_LINK, UiMenuId::ESC_MANUAL_TEST};
  static const UiMenuId steering[] = {UiMenuId::ESC_STEERING_LIVE, UiMenuId::ESC_STEERING_TEST_ANGLE, UiMenuId::ESC_STEERING_CAL};
  static const UiMenuId drive[] = {UiMenuId::ESC_DRIVE_LIVE, UiMenuId::ESC_MANUAL_SPEED, UiMenuId::ESC_DRIVE_SCALE};
  static const UiMenuId perception[] = {UiMenuId::PERCEPTION_OVERVIEW, UiMenuId::PERCEPTION_CAMERA, UiMenuId::PERCEPTION_DETECTION, UiMenuId::PERCEPTION_LANE, UiMenuId::PERCEPTION_OBSTACLE, UiMenuId::PERCEPTION_PERFORMANCE, UiMenuId::PERCEPTION_TEST};
  static const UiMenuId detection[] = {UiMenuId::PERCEPTION_DETECTION_LIVE, UiMenuId::PERCEPTION_INFERENCE};
  static const UiMenuId navigation[] = {UiMenuId::NAVIGATION_OVERVIEW, UiMenuId::NAV_LOCALIZATION, UiMenuId::NAV_MISSION, UiMenuId::NAV_NAV2, UiMenuId::NAV_SAFETY, UiMenuId::NAV_TEST};
  static const UiMenuId mission[] = {UiMenuId::NAV_MISSION_GO, UiMenuId::NAV_MISSION_SAVE, UiMenuId::NAV_MISSION_STOP};
  static const UiMenuId localization[] = {UiMenuId::NAV_GNSS, UiMenuId::NAV_IMU, UiMenuId::NAV_MAG, UiMenuId::NAV_EKF_LOCAL, UiMenuId::NAV_EKF_GLOBAL};
  static const UiMenuId nav2[] = {UiMenuId::NAV_PLANNER, UiMenuId::NAV_MPPI, UiMenuId::NAV_SMOOTHER, UiMenuId::NAV_COSTMAP};
  switch (id) {
    case UiMenuId::OVERVIEW: count = 3; return overview;
    case UiMenuId::ESC_ROOT: count = 7; return esc;
    case UiMenuId::ESC_STEERING: count = 3; return steering;
    case UiMenuId::ESC_DRIVE: count = 3; return drive;
    case UiMenuId::PERCEPTION_ROOT: count = 7; return perception;
    case UiMenuId::PERCEPTION_DETECTION: count = 2; return detection;
    case UiMenuId::NAVIGATION_ROOT: count = 6; return navigation;
    case UiMenuId::NAV_MISSION: count = 3; return mission;
    case UiMenuId::NAV_LOCALIZATION: count = 5; return localization;
    case UiMenuId::NAV_NAV2: count = 4; return nav2;
    default: count = 0; return nullptr;
  }
}

inline uint8_t menuWindowFirst(uint8_t selected, uint8_t count) {
  if (count <= SUBMENU_VISIBLE_CARDS) return 0U;
  uint8_t first = selected > 0U ? static_cast<uint8_t>(selected - 1U) : 0U;
  const uint8_t maxFirst = static_cast<uint8_t>(count - SUBMENU_VISIBLE_CARDS);
  if (first > maxFirst) first = maxFirst;
  return first;
}

inline UiEditKey menuEditKey(UiMenuId id) {
  if (id == UiMenuId::ESC_MODE) return UiEditKey::OPERATOR_MODE;
  if (id == UiMenuId::ESC_MANUAL_SPEED) return UiEditKey::MANUAL_SPEED_PCT;
  if (id == UiMenuId::ESC_STEERING_TEST_ANGLE) return UiEditKey::STEERING_TEST_DEG;
  if (id == UiMenuId::ESC_DRIVE_SCALE) return UiEditKey::DRIVE_SCALE;
  if (id == UiMenuId::PERCEPTION_INFERENCE) return UiEditKey::PERCEPTION_INFERENCE;
  return UiEditKey::NONE;
}

inline bool menuHasChildren(UiMenuId id) {
  uint8_t count = 0;
  (void)menuChildren(id, count);
  return count > 0;
}

inline const char* menuWireName(UiMenuId id) {
  switch (id) {
    case UiMenuId::SPLASH: return "SPLASH";
    case UiMenuId::OVERVIEW: return "OVERVIEW";
    case UiMenuId::ESC_ROOT: return "ESC";
    case UiMenuId::PERCEPTION_ROOT: return "PERCEPTION";
    case UiMenuId::NAVIGATION_ROOT: return "NAVIGATION";
    default: break;
  }
  // Detail page dikirim dengan nama enum-stabil yang cukup untuk sinkronisasi ROS.
  // Hindari spasi agar parser serial dan consumer topic /hmi/page tetap sederhana.
  static char wire[28];
  const char* title = menuTitle(id);
  size_t out = 0;
  for (size_t i = 0; title[i] != '\0' && out + 1 < sizeof(wire); ++i) {
    char c = title[i];
    if (c == ' ' || c == '/') c = '_';
    if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_') wire[out++] = c;
  }
  wire[out] = '\0';
  return wire;
}
