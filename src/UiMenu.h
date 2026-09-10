// ============================================================================
// UiMenu.h — Stage-1 page-of-three operator catalog + compatibility routing.
// ============================================================================
#pragma once

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include "Config.h"

struct UiState {
  UiMenuId menu{UiMenuId::SPLASH};
  uint8_t pageIndex{0U};
  uint8_t detailViewIndex{0U};
  bool editing{false};
  float editValue{0.0F};
  uint16_t nextTxn{1U};
  uint32_t pendingSinceMs{0U};
  bool serviceSession{false};
};

enum class UiDomain : uint8_t { NONE = 0U, ESC, PERCEPTION, NAVIGATION, SYSTEM };

inline bool menuIsOperatorDomainRoot(UiMenuId id) {
  return id == UiMenuId::ESC_ROOT || id == UiMenuId::PERCEPTION_ROOT ||
         id == UiMenuId::NAVIGATION_ROOT;
}

inline bool menuIsDomainRoot(UiMenuId id) {
  return menuIsOperatorDomainRoot(id) || id == UiMenuId::SYSTEM_ROOT;
}

inline UiDomain menuDomain(UiMenuId id) {
  switch (id) {
  case UiMenuId::ESC_ROOT:
  case UiMenuId::ESC_OVERVIEW:
  case UiMenuId::ESC_MODE:
  case UiMenuId::ESC_STEERING:
  case UiMenuId::ESC_DRIVE:
  case UiMenuId::ESC_POWER:
  case UiMenuId::ESC_MOTOR_TELEMETRY:
  case UiMenuId::ESC_ENCODER:
  case UiMenuId::ESC_CALIBRATION:
  case UiMenuId::ESC_LINK:
  case UiMenuId::ESC_FAULT_SAFETY:
  case UiMenuId::ESC_PERFORMANCE:
  case UiMenuId::ESC_MANUAL_TEST:
  case UiMenuId::ESC_STEERING_LIVE:
  case UiMenuId::ESC_STEERING_TEST_ANGLE:
  case UiMenuId::ESC_STEERING_CAL:
  case UiMenuId::ESC_DRIVE_LIVE:
  case UiMenuId::ESC_MANUAL_SPEED:
  case UiMenuId::ESC_DRIVE_SCALE:
    return UiDomain::ESC;
  case UiMenuId::PERCEPTION_ROOT:
  case UiMenuId::PERCEPTION_OVERVIEW:
  case UiMenuId::PERCEPTION_CAMERA:
  case UiMenuId::PERCEPTION_DETECTION:
  case UiMenuId::PERCEPTION_LANE:
  case UiMenuId::PERCEPTION_DRIVABLE_AREA:
  case UiMenuId::PERCEPTION_OBSTACLE:
  case UiMenuId::PERCEPTION_PERFORMANCE:
  case UiMenuId::PERCEPTION_CALIBRATION:
  case UiMenuId::PERCEPTION_TEST:
  case UiMenuId::PERCEPTION_DETECTION_LIVE:
  case UiMenuId::PERCEPTION_INFERENCE:
    return UiDomain::PERCEPTION;
  case UiMenuId::NAVIGATION_ROOT:
  case UiMenuId::NAVIGATION_OVERVIEW:
  case UiMenuId::NAV_LOCALIZATION:
  case UiMenuId::NAV_GNSS:
  case UiMenuId::NAV_IMU_MAG:
  case UiMenuId::NAV_EKF:
  case UiMenuId::NAV_ODOMETRY:
  case UiMenuId::NAV_MISSION:
  case UiMenuId::NAV_NAV2:
  case UiMenuId::NAV_SAFETY:
  case UiMenuId::NAV_COSTMAP:
  case UiMenuId::NAV_PATH_CONTROL:
  case UiMenuId::NAV_TEST:
  case UiMenuId::NAV_IMU:
  case UiMenuId::NAV_MAG:
  case UiMenuId::NAV_EKF_LOCAL:
  case UiMenuId::NAV_EKF_GLOBAL:
  case UiMenuId::NAV_MISSION_GO:
  case UiMenuId::NAV_MISSION_SAVE:
  case UiMenuId::NAV_MISSION_STOP:
  case UiMenuId::NAV_PLANNER:
  case UiMenuId::NAV_MPPI:
  case UiMenuId::NAV_SMOOTHER:
    return UiDomain::NAVIGATION;
  case UiMenuId::SYSTEM_ROOT:
  case UiMenuId::SYSTEM_OVERVIEW:
  case UiMenuId::SYSTEM_PINS_IO:
  case UiMenuId::SYSTEM_PINS_DISPLAY:
  case UiMenuId::SYSTEM_LINKS:
  case UiMenuId::SYSTEM_ERRORS:
  case UiMenuId::SYSTEM_SPI_BUS:
  case UiMenuId::SYSTEM_UART_STATUS:
  case UiMenuId::SYSTEM_POWER_STATUS:
  case UiMenuId::SYSTEM_TOUCH_PANEL:
  case UiMenuId::SYSTEM_TFT_TEST:
    return UiDomain::SYSTEM;
  default:
    return UiDomain::NONE;
  }
}

inline const char *menuTitle(UiMenuId id) {
  switch (id) {
  case UiMenuId::SPLASH: return "SPLASH";
  case UiMenuId::OVERVIEW: return "HOME";
  case UiMenuId::HOME: return "AGV";
  case UiMenuId::MAIN_MENU: return "MAIN MENU";
  case UiMenuId::ESC_ROOT: return "ESC";
  case UiMenuId::ESC_OVERVIEW: return "ESC OVERVIEW";
  case UiMenuId::ESC_MODE: return "OPERATOR MODE";
  case UiMenuId::ESC_STEERING: return "STEERING";
  case UiMenuId::ESC_DRIVE: return "DRIVE";
  case UiMenuId::ESC_POWER: return "FOC / POWER";
  case UiMenuId::ESC_MOTOR_TELEMETRY: return "MOTOR TELEMETRY";
  case UiMenuId::ESC_ENCODER: return "ENCODER";
  case UiMenuId::ESC_CALIBRATION: return "CALIBRATION";
  case UiMenuId::ESC_LINK: return "COMMUNICATION";
  case UiMenuId::ESC_FAULT_SAFETY: return "FAULT & SAFETY";
  case UiMenuId::ESC_PERFORMANCE: return "PERFORMANCE";
  case UiMenuId::ESC_MANUAL_TEST: return "MANUAL TEST";
  case UiMenuId::PERCEPTION_ROOT: return "PERCEPTION";
  case UiMenuId::PERCEPTION_OVERVIEW: return "PER OVERVIEW";
  case UiMenuId::PERCEPTION_CAMERA: return "CAMERA";
  case UiMenuId::PERCEPTION_DETECTION: return "DETECTION";
  case UiMenuId::PERCEPTION_LANE: return "LANE";
  case UiMenuId::PERCEPTION_DRIVABLE_AREA: return "DRIVABLE AREA";
  case UiMenuId::PERCEPTION_OBSTACLE: return "OBSTACLE";
  case UiMenuId::PERCEPTION_PERFORMANCE: return "PERFORMANCE";
  case UiMenuId::PERCEPTION_CALIBRATION: return "CALIBRATION";
  case UiMenuId::PERCEPTION_TEST: return "PER TEST";
  case UiMenuId::NAVIGATION_ROOT: return "NAVIGATION";
  case UiMenuId::NAVIGATION_OVERVIEW: return "NAV OVERVIEW";
  case UiMenuId::NAV_LOCALIZATION: return "LOCALIZATION";
  case UiMenuId::NAV_GNSS: return "GNSS";
  case UiMenuId::NAV_IMU_MAG: return "IMU / MAG";
  case UiMenuId::NAV_EKF: return "EKF";
  case UiMenuId::NAV_ODOMETRY: return "ODOMETRY";
  case UiMenuId::NAV_MISSION: return "MISSION";
  case UiMenuId::NAV_NAV2: return "NAV2";
  case UiMenuId::NAV_SAFETY: return "SAFETY";
  case UiMenuId::NAV_COSTMAP: return "COSTMAP";
  case UiMenuId::NAV_PATH_CONTROL: return "PATH / CONTROL";
  case UiMenuId::NAV_TEST: return "NAV TEST";
  case UiMenuId::SYSTEM_ROOT: return "SERVICE";
  case UiMenuId::SYSTEM_OVERVIEW: return "SYSTEM HEALTH";
  case UiMenuId::SYSTEM_PINS_IO: return "IO PIN MONITOR";
  case UiMenuId::SYSTEM_PINS_DISPLAY: return "DISPLAY PINS";
  case UiMenuId::SYSTEM_LINKS: return "LINK DIAGNOSTICS";
  case UiMenuId::SYSTEM_ERRORS: return "ERROR COUNTERS";
  case UiMenuId::SYSTEM_SPI_BUS: return "SPI BUS";
  case UiMenuId::SYSTEM_UART_STATUS: return "UART STATUS";
  case UiMenuId::SYSTEM_POWER_STATUS: return "POWER STATUS";
  case UiMenuId::SYSTEM_TOUCH_PANEL: return "TOUCH PANEL";
  case UiMenuId::SYSTEM_TFT_TEST: return "TFT TEST";
  // Compatibility titles.
  case UiMenuId::ESC_STEERING_LIVE: return "STEERING LIVE";
  case UiMenuId::ESC_STEERING_TEST_ANGLE: return "TEST ANGLE";
  case UiMenuId::ESC_STEERING_CAL: return "STEERING CAL";
  case UiMenuId::ESC_DRIVE_LIVE: return "DRIVE LIVE";
  case UiMenuId::ESC_MANUAL_SPEED: return "MANUAL SPEED";
  case UiMenuId::ESC_DRIVE_SCALE: return "DRIVE SCALE";
  case UiMenuId::PERCEPTION_DETECTION_LIVE: return "DETECTION LIVE";
  case UiMenuId::PERCEPTION_INFERENCE: return "INFERENCE";
  case UiMenuId::NAV_IMU: return "IMU";
  case UiMenuId::NAV_MAG: return "MAGNETOMETER";
  case UiMenuId::NAV_EKF_LOCAL: return "EKF LOCAL";
  case UiMenuId::NAV_EKF_GLOBAL: return "EKF GLOBAL";
  case UiMenuId::NAV_MISSION_GO: return "GO WAYPOINT";
  case UiMenuId::NAV_MISSION_SAVE: return "SAVE CURRENT";
  case UiMenuId::NAV_MISSION_STOP: return "STOP NAVIGATION";
  case UiMenuId::NAV_PLANNER: return "SMAC PLANNER";
  case UiMenuId::NAV_MPPI: return "MPPI";
  case UiMenuId::NAV_SMOOTHER: return "SMOOTHER";
  default: return "UNKNOWN";
  }
}

inline UiMenuId menuParent(UiMenuId id) {
  switch (id) {
  case UiMenuId::MAIN_MENU: return UiMenuId::HOME;
  case UiMenuId::ESC_ROOT:
  case UiMenuId::PERCEPTION_ROOT:
  case UiMenuId::NAVIGATION_ROOT: return UiMenuId::MAIN_MENU;
  case UiMenuId::SYSTEM_ROOT: return UiMenuId::HOME;
  case UiMenuId::ESC_OVERVIEW:
  case UiMenuId::ESC_MODE:
  case UiMenuId::ESC_STEERING:
  case UiMenuId::ESC_DRIVE:
  case UiMenuId::ESC_POWER:
  case UiMenuId::ESC_MOTOR_TELEMETRY:
  case UiMenuId::ESC_ENCODER:
  case UiMenuId::ESC_CALIBRATION:
  case UiMenuId::ESC_LINK:
  case UiMenuId::ESC_FAULT_SAFETY:
  case UiMenuId::ESC_PERFORMANCE:
  case UiMenuId::ESC_MANUAL_TEST: return UiMenuId::ESC_ROOT;
  case UiMenuId::PERCEPTION_OVERVIEW:
  case UiMenuId::PERCEPTION_CAMERA:
  case UiMenuId::PERCEPTION_DETECTION:
  case UiMenuId::PERCEPTION_LANE:
  case UiMenuId::PERCEPTION_DRIVABLE_AREA:
  case UiMenuId::PERCEPTION_OBSTACLE:
  case UiMenuId::PERCEPTION_PERFORMANCE:
  case UiMenuId::PERCEPTION_CALIBRATION:
  case UiMenuId::PERCEPTION_TEST: return UiMenuId::PERCEPTION_ROOT;
  case UiMenuId::NAVIGATION_OVERVIEW:
  case UiMenuId::NAV_LOCALIZATION:
  case UiMenuId::NAV_GNSS:
  case UiMenuId::NAV_IMU_MAG:
  case UiMenuId::NAV_EKF:
  case UiMenuId::NAV_ODOMETRY:
  case UiMenuId::NAV_MISSION:
  case UiMenuId::NAV_NAV2:
  case UiMenuId::NAV_SAFETY:
  case UiMenuId::NAV_COSTMAP:
  case UiMenuId::NAV_PATH_CONTROL:
  case UiMenuId::NAV_TEST: return UiMenuId::NAVIGATION_ROOT;
  case UiMenuId::SYSTEM_OVERVIEW:
  case UiMenuId::SYSTEM_PINS_IO:
  case UiMenuId::SYSTEM_PINS_DISPLAY:
  case UiMenuId::SYSTEM_LINKS:
  case UiMenuId::SYSTEM_ERRORS:
  case UiMenuId::SYSTEM_SPI_BUS:
  case UiMenuId::SYSTEM_UART_STATUS:
  case UiMenuId::SYSTEM_POWER_STATUS:
  case UiMenuId::SYSTEM_TOUCH_PANEL:
  case UiMenuId::SYSTEM_TFT_TEST: return UiMenuId::SYSTEM_ROOT;
  case UiMenuId::ESC_STEERING_LIVE:
  case UiMenuId::ESC_STEERING_TEST_ANGLE:
  case UiMenuId::ESC_STEERING_CAL: return UiMenuId::ESC_STEERING;
  case UiMenuId::ESC_DRIVE_LIVE:
  case UiMenuId::ESC_MANUAL_SPEED:
  case UiMenuId::ESC_DRIVE_SCALE: return UiMenuId::ESC_DRIVE;
  case UiMenuId::PERCEPTION_DETECTION_LIVE: return UiMenuId::PERCEPTION_DETECTION;
  case UiMenuId::PERCEPTION_INFERENCE: return UiMenuId::PERCEPTION_CAMERA;
  case UiMenuId::NAV_IMU:
  case UiMenuId::NAV_MAG: return UiMenuId::NAV_IMU_MAG;
  case UiMenuId::NAV_EKF_LOCAL:
  case UiMenuId::NAV_EKF_GLOBAL: return UiMenuId::NAV_EKF;
  case UiMenuId::NAV_MISSION_GO:
  case UiMenuId::NAV_MISSION_SAVE:
  case UiMenuId::NAV_MISSION_STOP: return UiMenuId::NAV_MISSION;
  case UiMenuId::NAV_PLANNER:
  case UiMenuId::NAV_MPPI:
  case UiMenuId::NAV_SMOOTHER: return UiMenuId::NAV_PATH_CONTROL;
  case UiMenuId::SPLASH:
  case UiMenuId::OVERVIEW:
  case UiMenuId::HOME:
  default: return UiMenuId::HOME;
  }
}

inline const UiMenuId *menuChildren(UiMenuId id, uint8_t &count) {
  static const UiMenuId mainMenu[] = {UiMenuId::ESC_ROOT, UiMenuId::PERCEPTION_ROOT,
                                      UiMenuId::NAVIGATION_ROOT};
  static const UiMenuId esc[] = {
      UiMenuId::ESC_OVERVIEW, UiMenuId::ESC_MODE, UiMenuId::ESC_STEERING,
      UiMenuId::ESC_DRIVE, UiMenuId::ESC_POWER, UiMenuId::ESC_MOTOR_TELEMETRY,
      UiMenuId::ESC_ENCODER, UiMenuId::ESC_CALIBRATION, UiMenuId::ESC_LINK,
      UiMenuId::ESC_FAULT_SAFETY, UiMenuId::ESC_PERFORMANCE, UiMenuId::ESC_MANUAL_TEST};
  static const UiMenuId perception[] = {
      UiMenuId::PERCEPTION_OVERVIEW, UiMenuId::PERCEPTION_CAMERA,
      UiMenuId::PERCEPTION_DETECTION, UiMenuId::PERCEPTION_LANE,
      UiMenuId::PERCEPTION_DRIVABLE_AREA, UiMenuId::PERCEPTION_OBSTACLE,
      UiMenuId::PERCEPTION_PERFORMANCE, UiMenuId::PERCEPTION_CALIBRATION,
      UiMenuId::PERCEPTION_TEST};
  static const UiMenuId navigation[] = {
      UiMenuId::NAVIGATION_OVERVIEW, UiMenuId::NAV_LOCALIZATION, UiMenuId::NAV_GNSS,
      UiMenuId::NAV_IMU_MAG, UiMenuId::NAV_EKF, UiMenuId::NAV_ODOMETRY,
      UiMenuId::NAV_MISSION, UiMenuId::NAV_NAV2, UiMenuId::NAV_SAFETY,
      UiMenuId::NAV_COSTMAP, UiMenuId::NAV_PATH_CONTROL, UiMenuId::NAV_TEST};
  static const UiMenuId system[] = {
      UiMenuId::SYSTEM_OVERVIEW, UiMenuId::SYSTEM_PINS_IO, UiMenuId::SYSTEM_PINS_DISPLAY,
      UiMenuId::SYSTEM_LINKS, UiMenuId::SYSTEM_ERRORS, UiMenuId::SYSTEM_SPI_BUS,
      UiMenuId::SYSTEM_UART_STATUS, UiMenuId::SYSTEM_POWER_STATUS, UiMenuId::SYSTEM_TOUCH_PANEL,
      UiMenuId::SYSTEM_TFT_TEST};
  switch (id) {
  case UiMenuId::MAIN_MENU: count = 3U; return mainMenu;
  case UiMenuId::ESC_ROOT: count = 12U; return esc;
  case UiMenuId::PERCEPTION_ROOT: count = 9U; return perception;
  case UiMenuId::NAVIGATION_ROOT: count = 12U; return navigation;
  case UiMenuId::SYSTEM_ROOT: count = 10U; return system;
  default: count = 0U; return nullptr;
  }
}

inline uint8_t menuPageCount(uint8_t count) {
  return count == 0U ? 0U : static_cast<uint8_t>((count + DOMAIN_PAGE_SIZE - 1U) / DOMAIN_PAGE_SIZE);
}

inline uint8_t menuPageFirst(uint8_t pageIndex) {
  return static_cast<uint8_t>(pageIndex * DOMAIN_PAGE_SIZE);
}

inline UiMenuId menuCardAt(UiMenuId root, uint8_t pageIndex, uint8_t slot) {
  if (slot >= DOMAIN_PAGE_SIZE) return UiMenuId::SPLASH;
  uint8_t count = 0U;
  const UiMenuId *children = menuChildren(root, count);
  if (children == nullptr) return UiMenuId::SPLASH;
  const uint16_t index = static_cast<uint16_t>(menuPageFirst(pageIndex)) + slot;
  return index < count ? children[index] : UiMenuId::SPLASH;
}

inline UiMenuId menuDetailActionTarget(UiMenuId id, uint8_t view) {
  if (id == UiMenuId::ESC_STEERING && view == 2U)
    return UiMenuId::ESC_STEERING_TEST_ANGLE;
  if (id == UiMenuId::ESC_DRIVE && view == 1U)
    return UiMenuId::ESC_DRIVE_SCALE;
  if (id == UiMenuId::ESC_DRIVE && view == 2U)
    return UiMenuId::ESC_MANUAL_SPEED;
  if (id == UiMenuId::PERCEPTION_CAMERA && view == 0U)
    return UiMenuId::PERCEPTION_INFERENCE;
  return UiMenuId::SPLASH;
}

inline const char *menuDetailActionLabel(UiMenuId id, uint8_t view) {
  const UiMenuId target = menuDetailActionTarget(id, view);
  if (target == UiMenuId::ESC_STEERING_TEST_ANGLE) return "ANGLE";
  if (target == UiMenuId::ESC_DRIVE_SCALE) return "SCALE";
  if (target == UiMenuId::ESC_MANUAL_SPEED) return "SPEED";
  if (target == UiMenuId::PERCEPTION_INFERENCE) return "INFER";
  return nullptr;
}

inline uint8_t menuViewCount(UiMenuId id) {
  switch (id) {
  case UiMenuId::ESC_MODE:
  case UiMenuId::ESC_MANUAL_TEST:
  case UiMenuId::PERCEPTION_TEST:
  case UiMenuId::NAV_TEST:
  case UiMenuId::SYSTEM_OVERVIEW:
  case UiMenuId::SYSTEM_PINS_IO:
  case UiMenuId::SYSTEM_PINS_DISPLAY:
  case UiMenuId::SYSTEM_LINKS:
  case UiMenuId::SYSTEM_ERRORS:
  case UiMenuId::SYSTEM_SPI_BUS:
  case UiMenuId::SYSTEM_UART_STATUS:
  case UiMenuId::SYSTEM_POWER_STATUS:
  case UiMenuId::SYSTEM_TOUCH_PANEL:
  case UiMenuId::SYSTEM_TFT_TEST:
  case UiMenuId::ESC_STEERING_TEST_ANGLE:
  case UiMenuId::ESC_MANUAL_SPEED:
  case UiMenuId::ESC_DRIVE_SCALE:
  case UiMenuId::PERCEPTION_INFERENCE:
  case UiMenuId::NAV_MISSION_GO:
  case UiMenuId::NAV_MISSION_SAVE:
  case UiMenuId::NAV_MISSION_STOP:
    return 1U;
  default:
    return menuDomain(id) == UiDomain::NONE || menuIsDomainRoot(id) ? 1U : 3U;
  }
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
  uint8_t count = 0U;
  (void)menuChildren(id, count);
  return count > 0U;
}

inline const char *menuWireName(UiMenuId id) {
  switch (id) {
  case UiMenuId::SPLASH: return "SPLASH";
  case UiMenuId::OVERVIEW:
  case UiMenuId::HOME: return "HOME";
  case UiMenuId::MAIN_MENU: return "MAIN_MENU";
  case UiMenuId::ESC_ROOT: return "ESC";
  case UiMenuId::PERCEPTION_ROOT: return "PERCEPTION";
  case UiMenuId::SYSTEM_ROOT: return "SYSTEM";
  case UiMenuId::NAVIGATION_ROOT: return "NAVIGATION";
  default: break;
  }
  static char wire[32];
  const char *title = menuTitle(id);
  size_t out = 0U;
  for (size_t i = 0U; title[i] != '\0' && out + 1U < sizeof(wire); ++i) {
    char c = title[i];
    if (c == ' ' || c == '/' || c == '&') c = '_';
    if ((c >= 'A' && c <= 'Z') || (c >= '0' && c <= '9') || c == '_') wire[out++] = c;
  }
  wire[out] = '\0';
  return wire;
}
