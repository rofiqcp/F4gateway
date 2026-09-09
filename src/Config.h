// ============================================================================
// Config.h — konstanta tunggal HMI commissioning ADV 320x240
// ============================================================================
#pragma once

#include <cstdint>

static constexpr int W = 320;
static constexpr int H = 240;

enum class UiMenuId : uint8_t {
  SPLASH = 0,
  OVERVIEW,
  ESC_ROOT,
  ESC_OVERVIEW,
  ESC_MODE,
  ESC_STEERING,
  ESC_STEERING_LIVE,
  ESC_STEERING_TEST_ANGLE,
  ESC_STEERING_CAL,
  ESC_DRIVE,
  ESC_DRIVE_LIVE,
  ESC_MANUAL_SPEED,
  ESC_DRIVE_SCALE,
  ESC_POWER,
  ESC_LINK,
  ESC_MANUAL_TEST,
  PERCEPTION_ROOT,
  PERCEPTION_OVERVIEW,
  PERCEPTION_CAMERA,
  PERCEPTION_DETECTION,
  PERCEPTION_DETECTION_LIVE,
  PERCEPTION_INFERENCE,
  PERCEPTION_LANE,
  PERCEPTION_OBSTACLE,
  PERCEPTION_PERFORMANCE,
  PERCEPTION_TEST,
  NAVIGATION_ROOT,
  NAVIGATION_OVERVIEW,
  NAV_LOCALIZATION,
  NAV_GNSS,
  NAV_IMU,
  NAV_MAG,
  NAV_EKF_LOCAL,
  NAV_EKF_GLOBAL,
  NAV_MISSION,
  NAV_MISSION_GO,
  NAV_MISSION_SAVE,
  NAV_MISSION_STOP,
  NAV_NAV2,
  NAV_PLANNER,
  NAV_MPPI,
  NAV_SMOOTHER,
  NAV_COSTMAP,
  NAV_SAFETY,
  NAV_TEST
};

enum SystemStatus : uint8_t {
  SYS_OFF = 0,
  SYS_STARTING,
  SYS_INITIALIZING,
  SYS_READY,
  SYS_NOT_READY,
  SYS_FAULT
};

enum VehicleMode : uint8_t { MODE_AUTO = 0, MODE_MANUAL };
enum VehicleState : uint8_t { STATE_STANDBY = 0, STATE_RUNNING, STATE_STOPPED, STATE_FAULT };
enum GpsFixState : uint8_t { GPS_LOST = 0, GPS_NO_FIX, GPS_2D_FIX, GPS_3D_FIX, GPS_DEGRADED };
enum NavigationStatus : uint8_t {
  NAV_IDLE = 0,
  NAV_SELECTED,
  NAV_QUEUED,
  NAV_NAVIGATING,
  NAV_ARRIVED,
  NAV_STOPPED,
  NAV_FAILED
};

enum class SoftKey : uint8_t {
  NONE = 0, TOP_LEFT, LEFT, RIGHT, OK, CARD_0, CARD_1, CARD_2,
  TEST_LEFT, TEST_FORWARD, TEST_STOP, TEST_RIGHT, TEST_REVERSE
};
enum class UiEditKey : uint8_t {
  NONE = 0, OPERATOR_MODE, MANUAL_SPEED_PCT, STEERING_TEST_DEG, DRIVE_SCALE, PERCEPTION_INFERENCE
};

static constexpr uint8_t HMI_WAYPOINT_COUNT = 4;
static constexpr uint8_t HMI_WAYPOINT_NAME_LEN = 20;

constexpr uint16_t RGB565(uint8_t r, uint8_t g, uint8_t b) {
  return static_cast<uint16_t>(((r & 0xF8U) << 8U) | ((g & 0xFCU) << 3U) | (b >> 3U));
}

// Palet dipertahankan dari HMI lama agar identitas visual tidak berubah.
static constexpr uint16_t C_BG        = RGB565(9, 18, 30);
static constexpr uint16_t C_PANEL     = RGB565(16, 30, 47);
static constexpr uint16_t C_PANEL_ALT = RGB565(27, 42, 60);
static constexpr uint16_t C_CARD      = RGB565(245, 237, 229);
static constexpr uint16_t C_TEXT      = RGB565(246, 246, 244);
static constexpr uint16_t C_TEXT_DIM  = RGB565(178, 187, 198);
static constexpr uint16_t C_INK       = RGB565(10, 25, 42);
static constexpr uint16_t C_ACCENT    = RGB565(10, 199, 229);
static constexpr uint16_t C_READY     = RGB565(47, 198, 63);
static constexpr uint16_t C_WARNING   = RGB565(242, 181, 56);
static constexpr uint16_t C_FAULT     = RGB565(255, 59, 48);
static constexpr uint16_t C_DISABLED  = RGB565(105, 119, 136);
static constexpr uint16_t C_BORDER    = RGB565(62, 82, 105);
static constexpr uint16_t C_CARD_LINE = RGB565(205, 198, 190);
static constexpr uint16_t C_SHADOW    = RGB565(5, 11, 19);
static constexpr uint16_t C_WHITE     = 0xFFFFU;
static constexpr uint16_t C_BLACK     = 0x0000U;

// Layout global 320x240. Kontrol hanya muncul saat memang diperlukan.
static constexpr int TOP_H = 30;
static constexpr int CONTENT_Y = 33;
static constexpr int CONTENT_BOTTOM = 184;
static constexpr int SOFTKEY_Y = 188;
static constexpr int SOFTKEY_H = 48;
static constexpr int SOFTKEY_X0 = 4;
static constexpr int SOFTKEY_W = 102;
static constexpr int SOFTKEY_GAP = 4;
static constexpr int CARD_RADIUS = 7;

// Tiga kartu besar untuk overview dan carousel submenu.
static constexpr int UI_CARD_X0 = 6;
static constexpr int UI_CARD_W = 100;
static constexpr int UI_CARD_GAP = 4;
static constexpr int OVERVIEW_CARD_Y = 126;
static constexpr int OVERVIEW_CARD_H = 108;
static constexpr int SUBMENU_CARD_Y = 42;
static constexpr int SUBMENU_CARD_H = 132;
static constexpr uint8_t SUBMENU_VISIBLE_CARDS = 3;

// Footer carousel: tombol kiri/kanan besar dan indikator posisi di tengah.
static constexpr int CAROUSEL_NAV_Y = 188;
static constexpr int CAROUSEL_NAV_H = 48;
static constexpr int CAROUSEL_NAV_W = 76;
static constexpr int CAROUSEL_LEFT_X = 4;
static constexpr int CAROUSEL_RIGHT_X = W - 4 - CAROUSEL_NAV_W;
static constexpr int CAROUSEL_PAGE_X = CAROUSEL_LEFT_X + CAROUSEL_NAV_W + 6;
static constexpr int CAROUSEL_PAGE_W = CAROUSEL_RIGHT_X - 6 - CAROUSEL_PAGE_X;

// HOME visual tetap kecil, tetapi area sentuh sengaja lebih besar ke kanan/bawah.
static constexpr int HOME_TOUCH_X = 0;
static constexpr int HOME_TOUCH_Y = 0;
static constexpr int HOME_TOUCH_W = 52;
static constexpr int HOME_TOUCH_H = 44;

// Parameter operator yang benar-benar aman diedit dari HMI.
static constexpr int MANUAL_SPEED_DEFAULT = 20;
static constexpr int MANUAL_SPEED_MIN = 10;
static constexpr int MANUAL_SPEED_MAX = 50;
static constexpr int MANUAL_SPEED_STEP = 5;
static constexpr float STEER_TEST_ANGLE_DEFAULT_DEG = 20.0F;
static constexpr float STEER_TEST_ANGLE_MIN_DEG = 5.0F;
static constexpr float STEER_TEST_ANGLE_MAX_DEG = 30.0F;
static constexpr float STEER_TEST_ANGLE_STEP_DEG = 5.0F;
static constexpr float DRIVE_SCALE_MIN = 0.20F;
static constexpr float DRIVE_SCALE_MAX = 5.00F;
static constexpr float DRIVE_SCALE_STEP = 0.01F;

// Touch dan refresh: parser telemetry tetap jauh lebih cepat dari paint TFT.
static constexpr uint16_t TOUCH_THRESHOLD = 300;
static constexpr uint32_t TOUCH_POLL_MS = 20;
static constexpr uint32_t DISPLAY_REFRESH_MS = 100;
static constexpr uint32_t ROS_LINK_TIMEOUT_MS = 2500;
static constexpr uint32_t ROS_LINK_STARTUP_TIMEOUT_MS = 12000;
static constexpr uint32_t ROS_HEARTBEAT_STABLE_GAP_MS = 1500;
static constexpr uint8_t ROS_HEARTBEAT_STABLE_COUNT = 3;
static constexpr uint32_t CONFIG_ACK_TIMEOUT_MS = 1500;
// Motion test harus berakhir sendiri meskipun operator tidak menekan STOP.
static constexpr uint32_t DRIVE_TEST_MAX_MS = 3000;

// Splash timing.
static constexpr int PB_W = 180;
static constexpr int PB_H = 10;
static constexpr int PB_R = 5;
static constexpr int PB_X = (W - PB_W) / 2;
static constexpr int PB_Y = 168;
static constexpr uint32_t PROGRESS_MS = 1900;
static constexpr uint32_t READY_HOLD = 350;
static constexpr uint16_t FRAME_MS = 33;

#ifndef HMI_DEMO_MODE
#define HMI_DEMO_MODE 0
#endif
