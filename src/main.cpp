// ============================================================================
// ADV HMI Firmware — STM32F411CEU6 + ILI9341 320x240 + XPT2046
// Native STM32Cube production: OVERVIEW -> ESC / PERCEPTION / NAVIGATION.
// ============================================================================

#include "BoardSupport.h"
#include "UsbCdcPort.h"
#include "HmiDisplay.h"
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cstdint>

#include "Config.h"
#include "Telemetry.h"
#include "Neo3Sensors.h"
#include "VescGateway.h"
#include "Theme.h"
#include "Icons.h"
#include "SplashScreen.h"
#include "UiMenu.h"
#include "UiShell.h"
#include "TouchButtons.h"

#ifndef HMI_LEGACY_UART
#define HMI_LEGACY_UART 0
#endif

HmiDisplay tft;
VehicleTelemetry gTelemetry = defaultTelemetry();
UiState gUi;
Neo3Sensors gNeo3;
VescGateway gVesc;

static volatile uint32_t gMainLoopHeartbeatMs = 0U;
static volatile bool gAppWatchdogArmed = false;
static constexpr uint32_t APP_WATCHDOG_TIMEOUT_MS = 3500U;

static bool splashComplete = false;
static uint8_t splashProgress = 0;
static bool splashReadyText = false;
static uint32_t splashStartMs = 0;
static uint32_t splashReadyMs = 0;
static uint32_t lastFrameMs = 0;
static bool uiDirty = false;
static uint32_t lastUiRefreshMs = 0;
static uint32_t lastRosHeartbeatMs = 0;
static uint32_t lastTouchPollMs = 0;
static uint8_t rosHeartbeatStableCount = 0;
static bool rosHeartbeatStable = false;
static bool driveTestRunning = false;
static uint32_t driveTestDeadlineMs = 0U;
static volatile uint32_t gTftControllerId = 0U;
static volatile uint8_t gTftPowerMode = 0U;
static volatile uint8_t gTftMadctl = 0U;
static volatile uint8_t gTftPixelFormat = 0U;

static char serialRx[640];
static size_t serialRxLen = 0;
static bool serialRxDiscarding = false;
#if HMI_LEGACY_UART
static char serial1Rx[128];
static size_t serial1RxLen = 0;
static bool serial1RxDiscarding = false;
#endif

static uint32_t gDfuArmDeadlineMs = 0U;
static constexpr uint32_t kBootRequestMagic = 0x42465544UL;  // DFUB
static constexpr uint32_t kAppCrashMagic = 0x48535243UL;      // CRSH
static constexpr uint32_t kCrashCounterClearMs = 10000U;
static bool gCrashCounterCleared = false;
static uint32_t gWatchdogHealthySinceMs = 0U;

// Stoppable application watchdog. TIM11 is deliberately used instead of IWDG so
// ROM-DFU erase/program transactions cannot be interrupted by a firmware watchdog.
static void appWatchdogIsr() {
  if (!gAppWatchdogArmed) return;
  if (static_cast<uint32_t>(HAL_GetTick() - gMainLoopHeartbeatMs) > APP_WATCHDOG_TIMEOUT_MS) {
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    RTC->BKP1R = kAppCrashMagic;
    __DSB();
    NVIC_SystemReset();
  }
}

static void startAppWatchdog() {
  gMainLoopHeartbeatMs = HAL_GetTick();
  Board_SetWatchdogCallback(appWatchdogIsr);
  Board_WatchdogStart();
  gAppWatchdogArmed = true;
  gWatchdogHealthySinceMs = HAL_GetTick();
  gCrashCounterCleared = false;
}

static void stopAppWatchdog() {
  gAppWatchdogArmed = false;
  Board_WatchdogStop();
}

static bool tryUsbLine(const char* line) {
  if (line == nullptr) return false;
  const size_t len = strnlen(line, 190U);
  if (len >= 190U) return false;
  char out[192];
  std::memcpy(out, line, len);
  out[len] = '\n';
  const size_t total = len + 1U;
  if (gUsb.availableForWrite() < static_cast<int>(total)) return false;
  return gUsb.write(reinterpret_cast<const uint8_t*>(out), total) == total;
}

static void printBoth(const char* line) {
  (void)tryUsbLine(line);
}

static void publishPage() {
  char line[48];
  snprintf(line, sizeof(line), "PAGE:%s", menuWireName(gUi.menu));
  printBoth(line);
}

static void publishLinkState() {
  printBoth(gTelemetry.rosConnected ? "LINK:ROS:ONLINE" : "LINK:ROS:OFFLINE");
}

static void enterSystemDfu() {
  stopAppWatchdog();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  for (volatile uint32_t i = 0; i < 1000U; ++i) __NOP();
  RTC->BKP0R = kBootRequestMagic;
  __DSB();
  __ISB();
  gUsb.flush(150U);
  HAL_Delay(20U);
  gUsb.end();
  NVIC_SystemReset();
  while (true) { }
}

static void stopDriveTest() {
  // STOP dikirim walau state lokal sudah idle agar tombol STOP selalu idempotent.
  printBoth("CMD:DRIVE:STOP");
  driveTestRunning = false;
  driveTestDeadlineMs = 0U;
  gTelemetry.state = STATE_STOPPED;
  uiDirty = true;
}

static void stopSteeringTest() {
  printBoth("CMD:STEER:STOP");
  snprintf(gTelemetry.steeringTestState, sizeof(gTelemetry.steeringTestState), "%s", "IDLE");
  uiDirty = true;
}

static void stopAllManualTest() {
  stopDriveTest();
  stopSteeringTest();
}

static float actualEditValue(UiEditKey key);

static void drawUiNow(bool full = true) {
  if (!splashComplete || gUi.menu == UiMenuId::SPLASH) return;
  drawUiFrame(gUi, gTelemetry, full);
  uiDirty = false;
  lastUiRefreshMs = HAL_GetTick();
}

static void setMenu(UiMenuId next) {
  if (next == UiMenuId::SPLASH) return;
  if (gUi.menu == UiMenuId::ESC_MANUAL_TEST && next != UiMenuId::ESC_MANUAL_TEST) stopAllManualTest();
  gUi.menu = next;
  gUi.selectedChild = 0;
  const UiEditKey editKey = menuEditKey(next);
  gUi.editing = editKey != UiEditKey::NONE;
  if (gUi.editing) gUi.editValue = actualEditValue(editKey);
  drawUiNow(true);
  publishPage();
}

static void goOverview() {
  if (gUi.menu == UiMenuId::ESC_MANUAL_TEST) stopAllManualTest();
  setMenu(UiMenuId::OVERVIEW);
}

static float actualEditValue(UiEditKey key) {
  switch (key) {
    case UiEditKey::OPERATOR_MODE: return gTelemetry.mode == MODE_MANUAL ? 1.0F : 0.0F;
    case UiEditKey::MANUAL_SPEED_PCT: return static_cast<float>(gTelemetry.manualSpeedPct);
    case UiEditKey::STEERING_TEST_DEG: return gTelemetry.steeringTestAngleDeg;
    case UiEditKey::DRIVE_SCALE: return gTelemetry.driveScale;
    case UiEditKey::PERCEPTION_INFERENCE: return gTelemetry.perceptionInference ? 1.0F : 0.0F;
    case UiEditKey::NONE:
    default: return 0.0F;
  }
}

static void changeDraft(UiEditKey key, int direction) {
  if (!gUi.editing || direction == 0) return;
  if (key == UiEditKey::OPERATOR_MODE || key == UiEditKey::PERCEPTION_INFERENCE) {
    gUi.editValue = gUi.editValue > 0.5F ? 0.0F : 1.0F;
  } else if (key == UiEditKey::MANUAL_SPEED_PCT) {
    gUi.editValue = std::clamp(gUi.editValue + direction * MANUAL_SPEED_STEP,
                              static_cast<float>(MANUAL_SPEED_MIN), static_cast<float>(MANUAL_SPEED_MAX));
  } else if (key == UiEditKey::STEERING_TEST_DEG) {
    gUi.editValue = std::clamp(gUi.editValue + direction * STEER_TEST_ANGLE_STEP_DEG,
                              STEER_TEST_ANGLE_MIN_DEG, STEER_TEST_ANGLE_MAX_DEG);
  } else if (key == UiEditKey::DRIVE_SCALE) {
    gUi.editValue = std::clamp(gUi.editValue + direction * DRIVE_SCALE_STEP,
                              DRIVE_SCALE_MIN, DRIVE_SCALE_MAX);
  }
  drawUiNow(true);
}

static const char* editWireKey(UiEditKey key) {
  switch (key) {
    case UiEditKey::OPERATOR_MODE: return "MODE";
    case UiEditKey::MANUAL_SPEED_PCT: return "MANSPD";
    case UiEditKey::STEERING_TEST_DEG: return "STEERTEST";
    case UiEditKey::DRIVE_SCALE: return "DRVSCALE";
    case UiEditKey::PERCEPTION_INFERENCE: return "PERINF";
    case UiEditKey::NONE:
    default: return "NONE";
  }
}

static void requestConfig(UiEditKey key, float value) {
  if (key == UiEditKey::NONE || gTelemetry.configPending || !gTelemetry.rosConnected) {
    if (!gTelemetry.rosConnected) {
      gTelemetry.configLastOk = false;
      snprintf(gTelemetry.configMessage, sizeof(gTelemetry.configMessage), "%s", "ROS OFFLINE");
      drawUiNow(true);
    }
    return;
  }
  uint16_t txn = gUi.nextTxn++;
  if (txn == 0U) txn = gUi.nextTxn++;
  char line[80];
  const char* keyName = editWireKey(key);
  if (key == UiEditKey::DRIVE_SCALE) {
    snprintf(line, sizeof(line), "CMD:CFG:%u:%s:%.4f", txn, keyName, value);
  } else if (key == UiEditKey::STEERING_TEST_DEG) {
    snprintf(line, sizeof(line), "CMD:CFG:%u:%s:%.1f", txn, keyName, value);
  } else if (key == UiEditKey::MANUAL_SPEED_PCT) {
    snprintf(line, sizeof(line), "CMD:CFG:%u:%s:%d", txn, keyName, static_cast<int>(lroundf(value)));
  } else {
    snprintf(line, sizeof(line), "CMD:CFG:%u:%s:%d", txn, keyName, value > 0.5F ? 1 : 0);
  }
  printBoth(line);
  gTelemetry.configPending = true;
  gTelemetry.configLastOk = false;
  gTelemetry.configTxn = txn;
  snprintf(gTelemetry.configKey, sizeof(gTelemetry.configKey), "%s", keyName);
  snprintf(gTelemetry.configMessage, sizeof(gTelemetry.configMessage), "%s", "WAITING ACK");
  gUi.pendingSinceMs = HAL_GetTick();
  gUi.editing = false;
  drawUiNow(true);
}

static void applyEditor() {
  const UiEditKey key = menuEditKey(gUi.menu);
  if (key == UiEditKey::NONE) return;
  if (!gUi.editing) {
    gUi.editValue = actualEditValue(key);
    gUi.editing = true;
    drawUiNow(true);
    return;
  }
  requestConfig(key, gUi.editValue);
}

static void selectRelative(int direction) {
  if (direction == 0 || !menuHasChildren(gUi.menu)) return;
  uint8_t count = 0;
  const UiMenuId* children = menuChildren(gUi.menu, count);
  if (children == nullptr || count == 0U) return;
  int next = static_cast<int>(gUi.selectedChild) + direction;
  if (next < 0) next = count - 1;
  if (next >= count) next = 0;
  gUi.selectedChild = static_cast<uint8_t>(next);
  drawUiNow(false);
}


static void chooseVisibleCard(uint8_t slot) {
  if (slot >= SUBMENU_VISIBLE_CARDS) return;
  if (gUi.menu == UiMenuId::OVERVIEW) {
    uint8_t count = 0;
    const UiMenuId* children = menuChildren(UiMenuId::OVERVIEW, count);
    if (children != nullptr && slot < count) setMenu(children[slot]);
    return;
  }
  uint8_t count = 0;
  const UiMenuId* children = menuChildren(gUi.menu, count);
  if (children == nullptr || count == 0U) return;
  const uint8_t index = static_cast<uint8_t>(menuWindowFirst(gUi.selectedChild, count) + slot);
  if (index >= count) return;
  gUi.selectedChild = index;
  setMenu(children[index]);
}

static bool steeringTestAllowed() {
  return gTelemetry.rosConnected && gTelemetry.mode == MODE_MANUAL && gTelemetry.escReady &&
         gTelemetry.encoderReady && !gTelemetry.eStop && gTelemetry.state == STATE_STOPPED;
}

static void runSteeringTest(float targetDeg) {
  if (!steeringTestAllowed()) {
    gTelemetry.configLastOk = false;
    snprintf(gTelemetry.configMessage, sizeof(gTelemetry.configMessage), "%s", "STEER TEST LOCKED");
    drawUiNow(false);
    return;
  }
  char line[48];
  const float limited = std::clamp(targetDeg, -STEER_TEST_ANGLE_MAX_DEG, STEER_TEST_ANGLE_MAX_DEG);
  snprintf(line, sizeof(line), "CMD:STEER:%.1f", limited);
  printBoth(line);
}

static bool driveTestAllowed() {
  return gTelemetry.rosConnected && gTelemetry.mode == MODE_MANUAL && gTelemetry.escReady &&
         !gTelemetry.eStop && gTelemetry.state == STATE_STOPPED;
}

static void runDriveTest(bool forward) {
  if (driveTestRunning) {
    // Pergantian arah wajib melewati STOP; satu sentuhan saat bergerak hanya menghentikan.
    stopDriveTest();
    return;
  }
  if (!driveTestAllowed()) {
    gTelemetry.configLastOk = false;
    snprintf(gTelemetry.configMessage, sizeof(gTelemetry.configMessage), "%s", "DRIVE TEST LOCKED");
    drawUiNow(false);
    return;
  }
  char line[48];
  snprintf(line, sizeof(line), "CMD:DRIVE:%s:%u", forward ? "FWD" : "REV", gTelemetry.manualSpeedPct);
  printBoth(line);
  driveTestRunning = true;
  driveTestDeadlineMs = HAL_GetTick() + DRIVE_TEST_MAX_MS;
  gTelemetry.state = STATE_RUNNING;
  uiDirty = true;
}

static void selectWaypoint(int direction) {
  int index = static_cast<int>(gTelemetry.selectedWaypoint) + direction;
  if (index < 0) index = HMI_WAYPOINT_COUNT - 1;
  if (index >= HMI_WAYPOINT_COUNT) index = 0;
  char line[32];
  snprintf(line, sizeof(line), "CMD:WP:SELECT:%d", index);
  printBoth(line);
  // Jangan ubah state authoritative sebelum mirror ROS kembali.
}

static void goSelectedWaypoint() {
  const uint8_t index = gTelemetry.selectedWaypoint < HMI_WAYPOINT_COUNT ? gTelemetry.selectedWaypoint : 0U;
  if (!gTelemetry.rosConnected || gTelemetry.mode != MODE_AUTO ||
      gTelemetry.systemStatus != SYS_READY || !gTelemetry.waypointSaved[index]) {
    gTelemetry.configLastOk = false;
    snprintf(gTelemetry.configMessage, sizeof(gTelemetry.configMessage), "%s", "MISSION LOCKED");
    drawUiNow(false);
    return;
  }
  char line[32];
  snprintf(line, sizeof(line), "CMD:WP:GO:%u", index);
  printBoth(line);
}

static void saveSelectedWaypoint() {
  const uint8_t index = gTelemetry.selectedWaypoint < HMI_WAYPOINT_COUNT ? gTelemetry.selectedWaypoint : 0U;
  if (!gTelemetry.rosConnected || !gTelemetry.gpsReady || gTelemetry.state != STATE_STOPPED) {
    gTelemetry.configLastOk = false;
    snprintf(gTelemetry.configMessage, sizeof(gTelemetry.configMessage), "%s", "SAVE REQUIRES GPS+STOP");
    drawUiNow(false);
    return;
  }
  char line[32];
  snprintf(line, sizeof(line), "CMD:WP:SAVE:%u", index);
  printBoth(line);
}

static void stopNavigation() {
  printBoth("CMD:NAV:STOP");
}

static void handleOk() {
  if (gTelemetry.configPending) return;
  if (menuEditKey(gUi.menu) != UiEditKey::NONE) { applyEditor(); return; }
  if (gUi.menu == UiMenuId::NAV_MISSION_GO) { goSelectedWaypoint(); return; }
  if (gUi.menu == UiMenuId::NAV_MISSION_SAVE) { saveSelectedWaypoint(); return; }
  if (gUi.menu == UiMenuId::NAV_MISSION_STOP) { stopNavigation(); return; }
}

static void handleSoftKey(SoftKey key) {
  if (key == SoftKey::NONE) return;

  if (key == SoftKey::TOP_LEFT) {
    if (gUi.menu == UiMenuId::ESC_MANUAL_TEST) stopAllManualTest();
    if (uiIsDomainRoot(gUi.menu)) goOverview();
    else if (gUi.menu != UiMenuId::OVERVIEW) setMenu(menuParent(gUi.menu));
    return;
  }

  if (key == SoftKey::CARD_0 || key == SoftKey::CARD_1 || key == SoftKey::CARD_2) {
    const uint8_t slot = key == SoftKey::CARD_0 ? 0U : (key == SoftKey::CARD_1 ? 1U : 2U);
    chooseVisibleCard(slot);
    return;
  }

  if (gUi.menu == UiMenuId::ESC_MANUAL_TEST) {
    if (key == SoftKey::TEST_STOP) { stopAllManualTest(); drawUiNow(false); return; }
    if (key == SoftKey::TEST_FORWARD) { runDriveTest(true); return; }
    if (key == SoftKey::TEST_REVERSE) { runDriveTest(false); return; }
    if (key == SoftKey::TEST_LEFT) { runSteeringTest(-gTelemetry.steeringTestAngleDeg); return; }
    if (key == SoftKey::TEST_RIGHT) { runSteeringTest(gTelemetry.steeringTestAngleDeg); return; }
    return;
  }

  if (menuHasChildren(gUi.menu)) {
    if (key == SoftKey::LEFT) selectRelative(-1);
    else if (key == SoftKey::RIGHT) selectRelative(+1);
    return;
  }

  const UiEditKey editKey = menuEditKey(gUi.menu);
  if (editKey != UiEditKey::NONE) {
    if (gTelemetry.configPending) return;
    if (key == SoftKey::LEFT || key == SoftKey::RIGHT) {
      if (!gUi.editing) {
        gUi.editValue = actualEditValue(editKey);
        gUi.editing = true;
      }
      changeDraft(editKey, key == SoftKey::LEFT ? -1 : +1);
      return;
    }
    if (key == SoftKey::OK) { handleOk(); return; }
    return;
  }

  if (gUi.menu == UiMenuId::NAV_MISSION_GO || gUi.menu == UiMenuId::NAV_MISSION_SAVE) {
    if (key == SoftKey::LEFT) selectWaypoint(-1);
    else if (key == SoftKey::RIGHT) selectWaypoint(+1);
    else if (key == SoftKey::OK) handleOk();
    return;
  }
  if (gUi.menu == UiMenuId::NAV_MISSION_STOP && key == SoftKey::OK) handleOk();
}

static void handleTouch() {
  const TouchEvent ev = pollTouch(gUi);
  if (ev.type == TouchEvent::PRESS || ev.type == TouchEvent::REPEAT) handleSoftKey(ev.key);
}

static bool eqIgnoreCase(const char* a, const char* b) {
  while (*a && *b) {
    char ca = *a++;
    char cb = *b++;
    if (ca >= 'a' && ca <= 'z') ca -= 32;
    if (cb >= 'a' && cb <= 'z') cb -= 32;
    if (ca != cb) return false;
  }
  return *a == '\0' && *b == '\0';
}

static bool parseBool(const char* s) {
  return !strcmp(s, "1") || eqIgnoreCase(s, "ON") || eqIgnoreCase(s, "READY") || eqIgnoreCase(s, "TRUE");
}

static void parseSystemStatus(const char* s) {
  if (eqIgnoreCase(s, "OFF")) gTelemetry.systemStatus = SYS_OFF;
  else if (eqIgnoreCase(s, "STARTING")) gTelemetry.systemStatus = SYS_STARTING;
  else if (eqIgnoreCase(s, "INITIALIZING")) gTelemetry.systemStatus = SYS_INITIALIZING;
  else if (eqIgnoreCase(s, "READY") || eqIgnoreCase(s, "VEHICLE READY")) gTelemetry.systemStatus = SYS_READY;
  else if (eqIgnoreCase(s, "NOT READY")) gTelemetry.systemStatus = SYS_NOT_READY;
  else if (eqIgnoreCase(s, "FAULT")) gTelemetry.systemStatus = SYS_FAULT;
}

static void parseVehicleState(const char* s) {
  if (eqIgnoreCase(s, "STANDBY")) gTelemetry.state = STATE_STANDBY;
  else if (eqIgnoreCase(s, "RUNNING")) gTelemetry.state = STATE_RUNNING;
  else if (eqIgnoreCase(s, "STOPPED") || eqIgnoreCase(s, "STOP")) gTelemetry.state = STATE_STOPPED;
  else if (eqIgnoreCase(s, "FAULT")) gTelemetry.state = STATE_FAULT;
  if (gTelemetry.state != STATE_RUNNING) driveTestRunning = false;
}

static void sanitizeTelemetry() {
  if (!std::isfinite(gTelemetry.speedKmh)) gTelemetry.speedKmh = 0.0F;
  gTelemetry.speedKmh = std::clamp(gTelemetry.speedKmh, 0.0F, 100.0F);
  if (!std::isfinite(gTelemetry.driveTargetMps)) gTelemetry.driveTargetMps = 0.0F;
  if (!std::isfinite(gTelemetry.driveActualMps)) gTelemetry.driveActualMps = 0.0F;
  if (!std::isfinite(gTelemetry.headingDeg)) gTelemetry.headingDeg = 0.0F;
  gTelemetry.headingDeg = fmodf(gTelemetry.headingDeg, 360.0F);
  if (gTelemetry.headingDeg < 0.0F) gTelemetry.headingDeg += 360.0F;
  if (!std::isfinite(gTelemetry.hdop)) gTelemetry.hdop = 99.9F;
  if (!std::isfinite(gTelemetry.haccM)) gTelemetry.haccM = 999.0F;
  if (!std::isfinite(gTelemetry.gnssAgeSec)) gTelemetry.gnssAgeSec = 99.0F;
  if (!std::isfinite(gTelemetry.cameraFps)) gTelemetry.cameraFps = 0.0F;
  if (!std::isfinite(gTelemetry.objectDistanceM) || gTelemetry.objectDistanceM < 0.0F) gTelemetry.objectDistanceM = 0.0F;
  if (!std::isfinite(gTelemetry.confidencePct)) gTelemetry.confidencePct = 0.0F;
  gTelemetry.confidencePct = std::clamp(gTelemetry.confidencePct, 0.0F, 100.0F);
  if (!std::isfinite(gTelemetry.driveScale)) gTelemetry.driveScale = 1.0F;
  if (!std::isfinite(gTelemetry.steeringTestAngleDeg)) gTelemetry.steeringTestAngleDeg = STEER_TEST_ANGLE_DEFAULT_DEG;
  gTelemetry.steeringTestAngleDeg = std::clamp(gTelemetry.steeringTestAngleDeg, STEER_TEST_ANGLE_MIN_DEG, STEER_TEST_ANGLE_MAX_DEG);
}

static void configAck(bool ok, uint16_t txn, const char* key, const char* valueOrReason) {
  if (!gTelemetry.configPending || txn != gTelemetry.configTxn) return;
  if (strcmp(key, gTelemetry.configKey) != 0) return;
  gTelemetry.configPending = false;
  gTelemetry.configLastOk = ok;
  snprintf(gTelemetry.configMessage, sizeof(gTelemetry.configMessage), "%.31s", valueOrReason);
  drawUiNow(true);
}

static void parseConfigResult(char* command, bool ok) {
  // ACK:CFG:<txn>:<key>:<value> / ERR:CFG:<txn>:<key>:<reason>
  char* p = strchr(command, ':');
  if (p == nullptr) return;
  p = strchr(p + 1, ':');
  if (p == nullptr) return;
  const uint16_t txn = static_cast<uint16_t>(strtoul(p + 1, &p, 10));
  if (p == nullptr || *p != ':') return;
  char* key = p + 1;
  char* sep = strchr(key, ':');
  if (sep == nullptr) return;
  *sep = '\0';
  configAck(ok, txn, key, sep + 1);
}

static void markRosHeartbeat() {
  const uint32_t now = HAL_GetTick();
  if (!gTelemetry.rosConnected) {
    rosHeartbeatStableCount = 1U;
    rosHeartbeatStable = false;
    gTelemetry.rosConnected = true;
    publishLinkState();
    uiDirty = true;
  } else {
    const uint32_t gap = static_cast<uint32_t>(now - lastRosHeartbeatMs);
    if (gap <= ROS_HEARTBEAT_STABLE_GAP_MS) {
      if (rosHeartbeatStableCount < 255U) ++rosHeartbeatStableCount;
      if (rosHeartbeatStableCount >= ROS_HEARTBEAT_STABLE_COUNT) rosHeartbeatStable = true;
    } else {
      rosHeartbeatStableCount = 1U;
      rosHeartbeatStable = false;
    }
  }
  lastRosHeartbeatMs = now;
}

static void forceRosOffline() {
  stopAllManualTest();
  const bool wasConnected = gTelemetry.rosConnected;
  gTelemetry.rosConnected = false;
  rosHeartbeatStableCount = 0U;
  rosHeartbeatStable = false;
  gTelemetry.systemStatus = SYS_NOT_READY;
  gTelemetry.state = STATE_STOPPED;
  gTelemetry.escReady = false;
  gTelemetry.encoderReady = false;
  gTelemetry.vescConnected = false;
  gTelemetry.gpsReady = false;
  gTelemetry.imuReady = false;
  gTelemetry.magReady = false;
  gTelemetry.cameraReady = false;
  gTelemetry.perceptionReady = false;
  gTelemetry.motionReady = false;
  gTelemetry.nav2Ready = false;
  if (gTelemetry.configPending) {
    gTelemetry.configPending = false;
    gTelemetry.configLastOk = false;
    snprintf(gTelemetry.configMessage, sizeof(gTelemetry.configMessage), "%s", "ROS LINK LOST");
  }
  uiDirty = true;
  if (wasConnected) publishLinkState();
}

static void checkRosLinkTimeout() {
  if (!gTelemetry.rosConnected) return;
  const uint32_t timeoutMs = rosHeartbeatStable ? ROS_LINK_TIMEOUT_MS : ROS_LINK_STARTUP_TIMEOUT_MS;
  if (static_cast<uint32_t>(HAL_GetTick() - lastRosHeartbeatMs) > timeoutMs) forceRosOffline();
}

static void checkConfigTimeout() {
  if (!gTelemetry.configPending) return;
  if (static_cast<uint32_t>(HAL_GetTick() - gUi.pendingSinceMs) <= CONFIG_ACK_TIMEOUT_MS) return;
  gTelemetry.configPending = false;
  gTelemetry.configLastOk = false;
  snprintf(gTelemetry.configMessage, sizeof(gTelemetry.configMessage), "%s", "ACK TIMEOUT");
  drawUiNow(true);
}

static void setExternalMenu(const char* name) {
  if (eqIgnoreCase(name, "OVERVIEW")) setMenu(UiMenuId::OVERVIEW);
  else if (eqIgnoreCase(name, "ESC")) setMenu(UiMenuId::ESC_ROOT);
  else if (eqIgnoreCase(name, "PERCEPTION")) setMenu(UiMenuId::PERCEPTION_ROOT);
  else if (eqIgnoreCase(name, "NAVIGATION")) setMenu(UiMenuId::NAVIGATION_ROOT);
}

static void handleSerialCommand(char* command) {
  while (*command == ' ' || *command == '\t') ++command;
  if (*command == '\0') return;

  // Gateway hardware frame selalu diprioritaskan dan tidak menyentuh UI parser.
  if (!strncmp(command, "VESC:", 5)) {
    (void)gVesc.handleHostCommand(command);
    return;
  }
  if (!strncmp(command, "NEO:", 4)) {
    (void)gNeo3.handleHostCommand(command);
    return;
  }
  if (!strncmp(command, "ACK:CFG:", 8)) { parseConfigResult(command, true); return; }
  if (!strncmp(command, "ERR:CFG:", 8)) { parseConfigResult(command, false); return; }

  if (!strcmp(command, "GET:STATE")) {
    publishPage();
    publishLinkState();
    return;
  }
  if (!strcmp(command, "PING")) {
    (void)gUsb.writeLineCritical("ACK:PONG", 120U);
    publishPage();
    publishLinkState();
    return;
  }
  if (!strcmp(command, "TFT:STATUS")) {
    char line[96];
    std::snprintf(line, sizeof(line), "TFT:ID:%08lX:MODE=%02X:MADCTL=%02X:PIXFMT=%02X:%s",
                  static_cast<unsigned long>(gTftControllerId), gTftPowerMode, gTftMadctl, gTftPixelFormat,
                  (gTftControllerId & 0xFFFFU) == 0x9341U ? "OK" : "FAULT");
    (void)gUsb.writeLineCritical(line, 120U);
    return;
  }
  if (!strcmp(command, "TFT:TEST")) {
    tft.fillScreen(C_FAULT); HAL_Delay(250U);
    tft.fillScreen(C_READY); HAL_Delay(250U);
    tft.fillScreen(0x001FU); HAL_Delay(250U);
    if (splashComplete) drawUiNow(true); else drawSplashScreen();
    (void)gUsb.writeLineCritical("ACK:TFT:TEST", 120U);
    return;
  }
  if (!strcmp(command, "BOOT:DFU:ARM")) {
    gDfuArmDeadlineMs = HAL_GetTick() + 5000U;
    printBoth("ACK:DFU:ARMED");
    return;
  }
  if (!strcmp(command, "BOOT:DFU:CONFIRM")) {
    const uint32_t now = HAL_GetTick();
    if (gDfuArmDeadlineMs == 0U || static_cast<int32_t>(gDfuArmDeadlineMs - now) <= 0) {
      gDfuArmDeadlineMs = 0U;
      printBoth("ERR:DFU:NOT_ARMED");
      return;
    }
    stopAllManualTest();
    const bool nav_active = gTelemetry.navigationStatus == NAV_QUEUED || gTelemetry.navigationStatus == NAV_NAVIGATING;
    const bool motion_safe = (gTelemetry.state == STATE_STOPPED || gTelemetry.state == STATE_STANDBY) &&
                             std::fabs(gTelemetry.speedKmh) <= 0.2F && !nav_active;
    if (!motion_safe) {
      (void)gUsb.writeLineCritical("ERR:DFU:WAIT_SAFE");
      return;
    }
    if (!gUsb.writeLineCritical("ACK:DFU")) return;
    gDfuArmDeadlineMs = 0U;
    gUsb.flush(150U);
    HAL_Delay(20U);
    enterSystemDfu();
    return;
  }
  if (!strcmp(command, "BOOT:DFU")) { printBoth("ERR:DFU:TWO_STEP_REQUIRED"); return; }
  if (!strncmp(command, "GOTO:", 5)) { setExternalMenu(command + 5); return; }

  bool recognized = true;
  if (!strncmp(command, "ROS:", 4)) {
    if (parseBool(command + 4)) markRosHeartbeat(); else forceRosOffline();
  } else if (!strncmp(command, "SYS:", 4)) {
    parseSystemStatus(command + 4);
  } else if (!strncmp(command, "MODE:", 5)) {
    gTelemetry.mode = eqIgnoreCase(command + 5, "MANUAL") ? MODE_MANUAL : MODE_AUTO;
  } else if (!strncmp(command, "STATE:", 6)) {
    parseVehicleState(command + 6);
  } else if (!strncmp(command, "SPD:", 4)) {
    gTelemetry.speedKmh = static_cast<float>(atof(command + 4));
  } else if (!strncmp(command, "DRIVE_TGT:", 10)) {
    gTelemetry.driveTargetMps = static_cast<float>(atof(command + 10));
  } else if (!strncmp(command, "DRIVE_ACT:", 10)) {
    gTelemetry.driveActualMps = static_cast<float>(atof(command + 10));
  } else if (!strncmp(command, "RPM:", 4)) {
    gTelemetry.motorRpm = static_cast<float>(atof(command + 4));
  } else if (!strncmp(command, "ERPM:", 5)) {
    gTelemetry.motorErpm = static_cast<float>(atof(command + 5));
  } else if (!strncmp(command, "STEER_TARGET:", 13)) {
    gTelemetry.steeringTargetDeg = static_cast<float>(atof(command + 13));
    gTelemetry.steeringErrorDeg = gTelemetry.steeringTargetDeg - gTelemetry.steeringActualDeg;
  } else if (!strncmp(command, "STEER_ACTUAL:", 13)) {
    gTelemetry.steeringActualDeg = static_cast<float>(atof(command + 13));
    gTelemetry.steeringErrorDeg = gTelemetry.steeringTargetDeg - gTelemetry.steeringActualDeg;
  } else if (!strncmp(command, "STEER_ERR:", 10)) {
    gTelemetry.steeringErrorDeg = static_cast<float>(atof(command + 10));
  } else if (!strncmp(command, "STEERTEST:", 10)) {
    snprintf(gTelemetry.steeringTestState, sizeof(gTelemetry.steeringTestState), "%.11s", command + 10);
  } else if (!strncmp(command, "ESC:", 4)) {
    gTelemetry.escReady = parseBool(command + 4);
  } else if (!strncmp(command, "ENC:", 4)) {
    gTelemetry.encoderReady = parseBool(command + 4);
  } else if (!strncmp(command, "VESC_LINK:", 10)) {
    gTelemetry.vescConnected = parseBool(command + 10);
  } else if (!strncmp(command, "ESTOP:", 6)) {
    gTelemetry.eStop = parseBool(command + 6);
    if (gTelemetry.eStop) stopDriveTest();
  } else if (!strncmp(command, "MANUAL_SPEED:", 13)) {
    gTelemetry.manualSpeedPct = static_cast<uint8_t>(std::clamp(atoi(command + 13), MANUAL_SPEED_MIN, MANUAL_SPEED_MAX));
  } else if (!strncmp(command, "CFGSTEERTEST:", 13)) {
    gTelemetry.steeringTestAngleDeg = static_cast<float>(atof(command + 13));
  } else if (!strncmp(command, "CFGDRVSCALE:", 13)) {
    gTelemetry.driveScale = static_cast<float>(atof(command + 13));
  } else if (!strncmp(command, "CFGPERINF:", 10)) {
    gTelemetry.perceptionInference = parseBool(command + 10);
  } else if (!strncmp(command, "GPS:", 4)) {
    gTelemetry.gpsReady = parseBool(command + 4);
  } else if (!strncmp(command, "FIX:", 4)) {
    const int fix = atoi(command + 4);
    if (fix <= 1) gTelemetry.gpsFix = GPS_NO_FIX;
    else if (fix == 2) gTelemetry.gpsFix = GPS_2D_FIX;
    else if (fix == 3) gTelemetry.gpsFix = GPS_3D_FIX;
    else gTelemetry.gpsFix = GPS_DEGRADED;
  } else if (!strncmp(command, "LAT:", 4)) {
    gTelemetry.latitude = atof(command + 4);
  } else if (!strncmp(command, "LON:", 4)) {
    gTelemetry.longitude = atof(command + 4);
  } else if (!strncmp(command, "SAT:", 4)) {
    gTelemetry.satellites = static_cast<uint8_t>(std::clamp(atoi(command + 4), 0, 99));
  } else if (!strncmp(command, "HDOP:", 5)) {
    gTelemetry.hdop = static_cast<float>(atof(command + 5));
  } else if (!strncmp(command, "HACC:", 5)) {
    gTelemetry.haccM = static_cast<float>(atof(command + 5));
  } else if (!strncmp(command, "GAGE:", 5)) {
    gTelemetry.gnssAgeSec = static_cast<float>(atof(command + 5));
  } else if (!strncmp(command, "HEAD:", 5)) {
    gTelemetry.headingDeg = static_cast<float>(atof(command + 5));
  } else if (!strncmp(command, "IMU:", 4)) {
    gTelemetry.imuReady = parseBool(command + 4);
  } else if (!strncmp(command, "GYROZ:", 6)) {
    gTelemetry.gyroZRps = static_cast<float>(atof(command + 6));
  } else if (!strncmp(command, "MAG:", 4)) {
    gTelemetry.magReady = parseBool(command + 4);
  } else if (!strncmp(command, "CAM:", 4)) {
    gTelemetry.cameraReady = parseBool(command + 4);
  } else if (!strncmp(command, "PER:", 4)) {
    gTelemetry.perceptionReady = parseBool(command + 4);
  } else if (!strncmp(command, "FPS:", 4)) {
    gTelemetry.cameraFps = static_cast<float>(atof(command + 4));
  } else if (!strncmp(command, "OBJ:", 4)) {
    snprintf(gTelemetry.detectedObject, sizeof(gTelemetry.detectedObject), "%.23s", command + 4);
  } else if (!strncmp(command, "DIST:", 5)) {
    gTelemetry.objectDistanceM = static_cast<float>(atof(command + 5));
  } else if (!strncmp(command, "CONF:", 5)) {
    gTelemetry.confidencePct = static_cast<float>(atof(command + 5));
  } else if (!strncmp(command, "DRV:", 4)) {
    gTelemetry.drivableAreaClear = parseBool(command + 4);
  } else if (!strncmp(command, "OBS:", 4)) {
    gTelemetry.obstacleDetected = parseBool(command + 4);
  } else if (!strncmp(command, "LANE:", 5)) {
    snprintf(gTelemetry.laneState, sizeof(gTelemetry.laneState), "%.19s", command + 5);
  } else if (!strncmp(command, "MOTION:", 7)) {
    gTelemetry.motionReady = parseBool(command + 7);
  } else if (!strncmp(command, "NAV2:", 5)) {
    gTelemetry.nav2Ready = parseBool(command + 5);
  } else if (!strncmp(command, "LOCSTATE:", 9)) {
    snprintf(gTelemetry.localizationState, sizeof(gTelemetry.localizationState), "%.23s", command + 9);
  } else if (!strncmp(command, "GNSSSTATUS:", 11)) {
    snprintf(gTelemetry.gnssStatus, sizeof(gTelemetry.gnssStatus), "%.19s", command + 11);
  } else if (!strncmp(command, "IMUSTATUS:", 10)) {
    snprintf(gTelemetry.imuStatus, sizeof(gTelemetry.imuStatus), "%.19s", command + 10);
  } else if (!strncmp(command, "EKFLOCAL:", 9)) {
    snprintf(gTelemetry.ekfLocalStatus, sizeof(gTelemetry.ekfLocalStatus), "%.19s", command + 9);
  } else if (!strncmp(command, "EKFGLOBAL:", 10)) {
    snprintf(gTelemetry.ekfGlobalStatus, sizeof(gTelemetry.ekfGlobalStatus), "%.19s", command + 10);
  } else if (!strncmp(command, "WPSEL:", 6)) {
    gTelemetry.selectedWaypoint = static_cast<uint8_t>(std::clamp(atoi(command + 6), 0, HMI_WAYPOINT_COUNT - 1));
  } else if (!strncmp(command, "TARGET:", 7)) {
    snprintf(gTelemetry.activeTarget, sizeof(gTelemetry.activeTarget), "%.19s", command + 7);
  } else if (!strncmp(command, "NAV:", 4)) {
    const char* state = command + 4;
    if (eqIgnoreCase(state, "SELECTED")) gTelemetry.navigationStatus = NAV_SELECTED;
    else if (eqIgnoreCase(state, "QUEUED") || eqIgnoreCase(state, "SENDING")) gTelemetry.navigationStatus = NAV_QUEUED;
    else if (eqIgnoreCase(state, "NAVIGATING") || eqIgnoreCase(state, "ACTIVE")) gTelemetry.navigationStatus = NAV_NAVIGATING;
    else if (eqIgnoreCase(state, "ARRIVED") || eqIgnoreCase(state, "SUCCEEDED")) gTelemetry.navigationStatus = NAV_ARRIVED;
    else if (eqIgnoreCase(state, "STOPPED") || eqIgnoreCase(state, "CANCELED")) gTelemetry.navigationStatus = NAV_STOPPED;
    else if (eqIgnoreCase(state, "FAILED") || eqIgnoreCase(state, "ABORTED") || eqIgnoreCase(state, "REJECTED")) gTelemetry.navigationStatus = NAV_FAILED;
    else gTelemetry.navigationStatus = NAV_IDLE;
  } else if (!strncmp(command, "WP", 2) && command[2] >= '0' && command[2] <= '3' && command[3] == ':') {
    const uint8_t index = static_cast<uint8_t>(command[2] - '0');
    char* payload = command + 4;
    char* colon = strchr(payload, ':');
    if (colon != nullptr) {
      *colon = '\0';
      gTelemetry.waypointSaved[index] = parseBool(payload);
      snprintf(gTelemetry.waypointName[index], HMI_WAYPOINT_NAME_LEN, "%.19s", colon + 1);
    }
  } else {
    recognized = false;
  }

  if (!recognized) {
    char line[224];
    std::snprintf(line, sizeof(line), "ERR:UNKNOWN_COMMAND:%s", command);
    (void)gUsb.writeLine(line);
    return;
  }
  sanitizeTelemetry();
  uiDirty = true;
}

static void pollSerialGui() {
  gUsb.poll();
  while (gUsb.available() > 0) {
    const int value = gUsb.read();
    if (value < 0) break;
    const char c = static_cast<char>(value);
    if (c == '\r') continue;
    if (serialRxDiscarding) {
      if (c == '\n') serialRxDiscarding = false;
      continue;
    }
    if (c == '\n') {
      serialRx[serialRxLen] = '\0';
      if (serialRxLen > 0U) handleSerialCommand(serialRx);
      serialRxLen = 0U;
    } else if (serialRxLen + 1U < sizeof(serialRx)) {
      serialRx[serialRxLen++] = c;
    } else {
      serialRxLen = 0U;
      serialRxDiscarding = true;
      (void)gUsb.writeLine("ERR:COMMAND_TOO_LONG");
    }
  }
}

static void initDisplay() {
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
  gTftControllerId = 0U;
  for (uint8_t attempt = 0U; attempt < 2U; ++attempt) {
    tft.init();
    gTftControllerId = tft.readId();
    if ((gTftControllerId & 0xFFFFU) == 0x9341U) break;
    HAL_Delay(40U);
  }
  tft.setRotation(1);
  gTftPowerMode = tft.readRegister8(0x0AU, 0U);
  gTftMadctl = tft.readRegister8(0x0BU, 0U);
  gTftPixelFormat = tft.readRegister8(0x0CU, 0U);
  tft.setSwapBytes(true);
  tft.fillScreen(C_BG);
  tft.setTextDatum(MC_DATUM);
}

static void restartSplash() {
  stopDriveTest();
  gUi = UiState{};
  gUi.menu = UiMenuId::SPLASH;
  splashComplete = false;
  splashProgress = 0U;
  splashReadyText = false;
  lastFrameMs = 0U;
  resetTouchState();
  uiDirty = false;
  lastUiRefreshMs = 0U;
  gTelemetry.systemStatus = SYS_INITIALIZING;
  drawSplashScreen();
  splashStartMs = HAL_GetTick();
  publishPage();
}

static bool updateProgressBar() {
  if (splashComplete) return false;
  const uint32_t now = HAL_GetTick();
  if (now - lastFrameMs < FRAME_MS) return true;
  lastFrameMs = now;
  const uint32_t elapsed = now - splashStartMs;
  if (elapsed < PROGRESS_MS) {
    const uint8_t target = static_cast<uint8_t>((elapsed * 100UL) / PROGRESS_MS);
    if (target != splashProgress) {
      splashProgress = target;
      const int innerW = PB_W - 4;
      const int fillW = static_cast<int>((splashProgress * innerW) / 100);
      tft.fillRoundRect(PB_X + 2, PB_Y + 2, innerW, PB_H - 4, PB_R - 2, C_BG);
      if (fillW > 0) tft.fillRoundRect(PB_X + 2, PB_Y + 2, fillW, PB_H - 4, PB_R - 2, C_ACCENT);
    }
    return true;
  }
  if (!splashReadyText) {
    splashReadyText = true;
    splashReadyMs = now;
    tft.fillRoundRect(PB_X + 2, PB_Y + 2, PB_W - 4, PB_H - 4, PB_R - 2, C_READY);
    tft.fillRect(70, 186, 180, 18, C_BG);
    drawUiText("HMI ready", W / 2, 187, C_READY, C_BG, MC_DATUM);
  }
  if (now - splashReadyMs >= READY_HOLD) {
    splashComplete = true;
    beginTouch();
    if (gTelemetry.systemStatus == SYS_INITIALIZING) gTelemetry.systemStatus = SYS_NOT_READY;
    gUi.menu = UiMenuId::OVERVIEW;
    gUi.selectedChild = 0U;
    drawUiNow(true);
    publishPage();
    return false;
  }
  return true;
}

int main() {
  Board_Init();
  bool usbInitOk = false;
  for (uint8_t attempt = 0U; attempt < 3U && !usbInitOk; ++attempt) {
    usbInitOk = gUsb.begin();
    if (!usbInitOk) {
      gUsb.end();
      HAL_Delay(50U * static_cast<uint32_t>(attempt + 1U));
    }
  }
  HAL_Delay(50U);
  printBoth("ADV HMI native realtime menu firmware - boot");
  gNeo3.begin();
  gVesc.begin();
  initDisplay();
  restartSplash();
  if (!usbInitOk) gTelemetry.systemStatus = SYS_FAULT;
  startAppWatchdog();

  while (true) {
    gMainLoopHeartbeatMs = HAL_GetTick();
    if (!gCrashCounterCleared && static_cast<uint32_t>(HAL_GetTick() - gWatchdogHealthySinceMs) >= kCrashCounterClearMs) {
      __HAL_RCC_PWR_CLK_ENABLE();
      HAL_PWR_EnableBkUpAccess();
      RTC->BKP1R = 0U;
      RTC->BKP2R = 0U;
      __DSB();
      gCrashCounterCleared = true;
    }
    gNeo3.pollSafetyIo();
    gVesc.setSafetyStop(gNeo3.safetyPressed());
    pollSerialGui();
    gVesc.poll();

    if (gVesc.maintenanceMode()) {
      for (uint8_t i = 0U; i < 4U; ++i) {
        pollSerialGui();
        gNeo3.pollSafetyIo();
        gVesc.setSafetyStop(gNeo3.safetyPressed());
        gVesc.poll();
      }
      gUsb.poll();
    }

    gNeo3.poll();
    gVesc.setSafetyStop(gNeo3.safetyPressed());
    gVesc.poll();
    pollSerialGui();
    checkRosLinkTimeout();
    checkConfigTimeout();
    if (driveTestRunning && static_cast<int32_t>(driveTestDeadlineMs - HAL_GetTick()) <= 0) stopDriveTest();

    if (!splashComplete) {
      (void)updateProgressBar();
    } else {
      const uint32_t now = HAL_GetTick();
      if (static_cast<uint32_t>(now - lastTouchPollMs) >= TOUCH_POLL_MS) {
        lastTouchPollMs = now;
        handleTouch();
      }
      if (uiDirty && !touchWasDown && static_cast<uint32_t>(now - lastUiRefreshMs) >= DISPLAY_REFRESH_MS) {
        drawUiNow(false);
      }
    }

    static uint32_t ledMs = 0U;
    if (static_cast<uint32_t>(HAL_GetTick() - ledMs) >= 500U) {
      ledMs = HAL_GetTick();
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
    gMainLoopHeartbeatMs = HAL_GetTick();
  }
}
