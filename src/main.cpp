#if !defined(F103_BUILD_BOOTLOADER)
// ============================================================================
// ADV HMI Firmware — STM32F103C8T6 + ILI9341 320x240 + XPT2046
// Native STM32Cube production: HOME -> MAIN MENU -> ESC / PERCEPTION / NAVIGATION.
// ============================================================================

#include "BoardSupport.h"
#include "Bts7960Winch.h"
#include "FirmwareConfig.h"
#include "BuildInfo.h"
#include "HmiDisplay.h"
#include "PersistentConfigStore.h"
#include "NumericParse.h"
#include "usb/UsbCdcPort.h"
#include <algorithm>
#include <cmath>
#include <cerrno>
#include <limits>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "HmiConfig.h"
#include "HmiDiagnostics.h"
#include "SplashScreen.h"
#include "Telemetry.h"
#include "TelemetryProtocol.h"
#include "Theme.h"
#include "UiMenu.h"
#include "HmiOperatorUi.h"
#include "HmiTouchService.h"
#ifndef HMI_LEGACY_UART
#define HMI_LEGACY_UART 0
#endif

HmiDisplay tft;
VehicleTelemetry gTelemetry = defaultTelemetry();
UiState gUi;
HmiOperatorUi gOperatorUi;

HmiDiagnostics gDiagnostics{};
PersistentConfigStore gPersistentConfig;
static bool gPersistentConfigReady = false;

static constexpr uint16_t kTouchCalibrationKey = 0x5443U; // "TC"
// Version 3 invalidates the compressed-span calibration introduced during the
// F103 migration so this installed panel falls back to the proven v2 mapping.
static constexpr uint16_t kTouchCalibrationVersion = 3U;
struct TouchCalibrationRecord {
  uint16_t version;
  uint16_t parameters[5];
};
static_assert(sizeof(TouchCalibrationRecord) <= PersistentConfigStore::kMaxValueBytes,
              "touch calibration record must fit persistent config");

static bool persistTouchCalibration(const uint16_t *parameters) {
  if (!gPersistentConfigReady || parameters == nullptr)
    return false;
  TouchCalibrationRecord record{kTouchCalibrationVersion,
                                {parameters[0], parameters[1], parameters[2],
                                 parameters[3], parameters[4]}};
  return gPersistentConfig.write(kTouchCalibrationKey, &record, sizeof(record));
}

static bool loadTouchCalibration() {
  if (!gPersistentConfigReady)
    return false;
  TouchCalibrationRecord record{};
  if (!gPersistentConfig.read(kTouchCalibrationKey, &record, sizeof(record)) ||
      record.version != kTouchCalibrationVersion)
    return false;
  const uint16_t *p = record.parameters;
  if (p[0] > 4095U || p[2] > 4095U || p[1] < 500U || p[1] > 4095U ||
      p[3] < 500U || p[3] > 4095U || (p[4] & ~7U) != 0U)
    return false;
  tft.setTouch(p);
  return true;
}

static void emitServiceLine(const char *line) {
  if (line != nullptr) (void)gUsb.writeLineCritical(line, 120U);
}

static volatile uint32_t gWatchdogProgressEpoch = 0U;
static volatile uint32_t gAppWatchdogLastEpoch = 0U;
static volatile uint8_t gAppWatchdogStallTicks = 0U;
static volatile bool gAppWatchdogArmed = false;
static uint32_t gResetCauseFlags = 0U;
// TIM3 fires at 10 Hz on the STM32F103C8 application clock.
static constexpr uint8_t APP_WATCHDOG_TIMEOUT_TICKS = 35U;

static bool splashComplete = false;
static bool gUiDirty = false;
static inline void markUiDirty() { gUiDirty = true; }
static uint32_t lastUiRefreshMs = 0;
static uint32_t lastRosHeartbeatMs = 0;
static uint32_t lastTouchPollMs = 0;
static uint32_t lastDiagnosticsMs = 0;
static uint32_t lastSystemUiMarkMs = 0;
static uint32_t lastDisplayRecoveryMs = 0;
static uint32_t lastUiInteractionMs = 0U;
#if BTS_WINCH_ENABLED
static bool gWinchUiWasBusy = false;
static bool gPostWinchDisplayReinitPending = false;
static uint32_t gPostWinchDisplayReinitAtMs = 0U;
#endif
enum class DisplayInitState : uint8_t { IDLE = 0U, RUNNING, RETRY_WAIT };
static DisplayInitState gDisplayInitState = DisplayInitState::IDLE;
static uint8_t gDisplayInitAttempt = 0U;
static uint32_t gDisplayInitRetryAtMs = 0U;
static bool gDisplayInitLastSuccess = false;
static uint32_t lastEscDomainMs = 0;
static uint32_t lastPerceptionDomainMs = 0;
static uint32_t lastNavigationDomainMs = 0;
static bool seenEscDomain = false;
static bool seenPerceptionDomain = false;
static bool seenNavigationDomain = false;
static uint8_t rosHeartbeatStableCount = 0;
static bool rosHeartbeatStable = false;
static bool driveTestRunning = false;
static bool steeringTestRunning = false;
static uint32_t driveTestDeadlineMs = 0U;
static volatile uint32_t gTftControllerId = 0U;
static volatile uint8_t gTftPowerMode = 0U;
static volatile uint8_t gTftMadctl = 0U;
static volatile uint8_t gTftPixelFormat = 0U;

static char serialRx[640];
static size_t serialRxLen = 0;
static bool serialRxDiscarding = false;
static constexpr uint8_t kDeferredCommandSlots = 12U;
static constexpr size_t kDeferredCommandLength = 256U;
static char gDeferredCommands[kDeferredCommandSlots][kDeferredCommandLength]{};
static uint8_t gDeferredCommandHead = 0U;
static uint8_t gDeferredCommandTail = 0U;
static volatile bool gRealtimeParserActive = false;

static uint8_t deferredCommandCount() {
  return gDeferredCommandHead >= gDeferredCommandTail
             ? static_cast<uint8_t>(gDeferredCommandHead - gDeferredCommandTail)
             : static_cast<uint8_t>(kDeferredCommandSlots -
                                    gDeferredCommandTail +
                                    gDeferredCommandHead);
}

static bool enqueueDeferredCommand(const char *command) {
  if (command == nullptr)
    return false;
  const size_t len = strnlen(command, kDeferredCommandLength);
  if (len == 0U || len >= kDeferredCommandLength) {
    ++gDiagnostics.deferredCommandDrops;
    return false;
  }
  const uint8_t next =
      static_cast<uint8_t>((gDeferredCommandHead + 1U) % kDeferredCommandSlots);
  if (next == gDeferredCommandTail) {
    ++gDiagnostics.deferredCommandDrops;
    return false;
  }
  std::memcpy(gDeferredCommands[gDeferredCommandHead], command, len + 1U);
  gDeferredCommandHead = next;
  gDiagnostics.deferredCommandPeak = std::max<uint32_t>(
      gDiagnostics.deferredCommandPeak, deferredCommandCount());
  return true;
}
#if HMI_LEGACY_UART
static char serial1Rx[128];
static size_t serial1RxLen = 0;
static bool serial1RxDiscarding = false;
#endif

#if defined(BOARD_F103_USBBOOT)
static uint32_t gDfuArmDeadlineMs = 0U;
#endif
static constexpr uint16_t kF103BootReqLo = 0x5544U;
static constexpr uint16_t kF103BootReqHi = 0x4246U;
static constexpr uint32_t kAppCrashMagic = 0x48535243UL;    // CRSH
static constexpr uint32_t kCrashCounterClearMs = 30000U;
static bool gCrashCounterCleared = false;
static uint32_t gWatchdogHealthySinceMs = 0U;

// Stoppable application watchdog uses TIM3 instead of IWDG so
// maintenance/recovery paths can explicitly control supervision.
static void appWatchdogIsr() {
  if (!gAppWatchdogArmed)
    return;

  // Use TIM3's own periodic interrupt as the timebase. This keeps the watchdog
  // effective even if the HAL/SysTick timebase itself stops advancing.
  const uint32_t epoch = gWatchdogProgressEpoch;
  if (epoch != gAppWatchdogLastEpoch) {
    gAppWatchdogLastEpoch = epoch;
    gAppWatchdogStallTicks = 0U;
    return;
  }
  if (gAppWatchdogStallTicks < 0xFFU)
    ++gAppWatchdogStallTicks;
  if (gAppWatchdogStallTicks >= APP_WATCHDOG_TIMEOUT_TICKS) {
#if BTS_WINCH_ENABLED
    // Fail closed before reset: cut the actual GPIO-based BTS backend.
    Board_BtsEmergencyCut();
    __DSB();
#endif
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    Board_BackupWrite(1U, kAppCrashMagic);
    Board_BackupWrite(3U, 0x100U); // application main-loop watchdog
    Board_BackupWrite(4U, SCB->CFSR);
    Board_BackupWrite(5U, SCB->HFSR);
    __DSB();
    NVIC_SystemReset();
  }
}

static inline void noteWatchdogProgress() {
  ++gWatchdogProgressEpoch;
}

static void startAppWatchdog() {
  noteWatchdogProgress();
  gAppWatchdogLastEpoch = gWatchdogProgressEpoch;
  gAppWatchdogStallTicks = 0U;
  Board_SetWatchdogCallback(appWatchdogIsr);
  Board_WatchdogStart();
  gAppWatchdogArmed = true;
  gWatchdogHealthySinceMs = HAL_GetTick();
  gCrashCounterCleared = false;
}

#if defined(BOARD_F103_USBBOOT)
static void stopAppWatchdog() {
  gAppWatchdogArmed = false;
  Board_WatchdogStop();
}
#endif

static bool tryUsbLine(const char *line) {
  if (line == nullptr)
    return false;
  const size_t len = strnlen(line, 190U);
  if (len >= 190U)
    return false;
  char out[192];
  std::memcpy(out, line, len);
  out[len] = '\n';
  const size_t total = len + 1U;
  if (gUsb.availableForWrite() < static_cast<int>(total))
    return false;
  return gUsb.write(reinterpret_cast<const uint8_t *>(out), total) == total;
}

static bool printBoth(const char *line) { return tryUsbLine(line); }

// Safety/control messages must never depend on the best-effort telemetry ring.
// A failed enqueue is latched and retried cooperatively from main/realtime
// service until the high-priority CDC ring accepts the STOP.
static bool gEStopAssertPending = false;
static bool gEStopClearPending = false;
static bool gDriveStopPending = false;
static bool gSteerStopPending = false;
static bool gNavStopPending = false;
static uint32_t gSafetyControlTxRetries = 0U;
static uint32_t gSafetyControlTxDrops = 0U;
static uint32_t gSafetyControlTxQueued = 0U;
static uint32_t gLastUsbSafetyGeneration = 0U;
static uint32_t gLastEStopStopRefreshMs = 0U;
static constexpr uint32_t ESTOP_STOP_REFRESH_MS = 100U;
static bool gLocalEStopLatched = false;
static bool gHostEStopActive = false;
static uint32_t gLastUsbRxResyncCount = 0U;
static int8_t gDriveTestDirection = 0;
static uint32_t gDriveLeaseLastTxMs = 0U;
static constexpr uint32_t DRIVE_LEASE_REFRESH_MS = 100U;

static bool sendSafetyControlLine(const char *line) {
  if (gUsb.writeLineHighPriority(line)) {
    ++gSafetyControlTxQueued;
    return true;
  }
  ++gSafetyControlTxRetries;
  ++gSafetyControlTxDrops;
  return false;
}

static void latchAllSafetyStops() {
  gDriveStopPending = true;
  gSteerStopPending = true;
  gNavStopPending = true;
}

static void refreshWinchSafetyInputs() {
#if BTS_WINCH_ENABLED
  const bool navActive = gTelemetry.navigationStatus == NAV_QUEUED ||
                         gTelemetry.navigationStatus == NAV_NAVIGATING;
  // Local fork service is allowed when measured motion is effectively zero.
  // Do not let an unrelated ROS/global FAULT state lock the physical HMI while
  // the vehicle is stationary; E-stop, navigation activity and limits remain
  // independent hard interlocks below.
  const bool stationary =
      gTelemetry.state != STATE_RUNNING &&
      std::fabs(gTelemetry.speedKmh) <= 0.2F &&
      std::fabs(gTelemetry.driveActualMps) <= 0.02F;
  Bts7960Winch::SafetyInputs safety{};
  safety.emergencyStop = gTelemetry.eStop;
  // Physical plausibility is independent of the latched fault so a recovered
  // contradictory-limit event can be explicitly cleared while stopped.
  safety.physicalSafetyValid = Bts7960Winch::limitInputsPlausible();
  safety.systemFault = gTelemetry.systemStatus == SYS_FAULT ||
                       gTelemetry.state == STATE_FAULT;
  safety.hostSessionValid =
      gUsb.connected() && gUsb.hostSessionEstablished();
  safety.commandSessionValid = gTelemetry.rosConnected;
  safety.vehicleSafe =
      stationary && !navActive && !driveTestRunning && !steeringTestRunning;
  Bts7960Winch::setSafetyInputs(safety);
#endif
}

static void forceRosOffline();
static void beginDisplayInitialization();

static void serviceUsbTransportState() {
  const uint32_t generation = gUsb.transportGeneration();
  if (generation != gLastUsbSafetyGeneration) {
    gLastUsbSafetyGeneration = generation;
    // ANY destructive USB queue reset can discard a STOP, including endpoint
    // split-brain repair that does not re-enumerate the USB class. Re-latch all
    // fail-safe controls whenever the transport generation changes.
    latchAllSafetyStops();
  }

  // Physical class loss/re-enumeration invalidates the host session immediately.
  // Do not wait for the ROS heartbeat timeout before revoking motion authority.
  const bool transport_authoritative = gUsb.connected() && gUsb.hostSessionEstablished();
  const bool nav_active = gTelemetry.navigationStatus == NAV_QUEUED ||
                          gTelemetry.navigationStatus == NAV_NAVIGATING;
  if (!transport_authoritative &&
      (gTelemetry.rosConnected || driveTestRunning || steeringTestRunning || nav_active))
    forceRosOffline();
}

static void serviceSafetyControlTx() {
  // E-stop state has first priority. Assert/clear are mutually exclusive and
  // are retried until the high-priority USB queue accepts them.
  if (gEStopAssertPending && sendSafetyControlLine("CMD:ESTOP:1"))
    gEStopAssertPending = false;
  if (gEStopClearPending && sendSafetyControlLine("CMD:ESTOP:0"))
    gEStopClearPending = false;
  if (gDriveStopPending && sendSafetyControlLine("CMD:DRIVE:STOP"))
    gDriveStopPending = false;
  if (gSteerStopPending && sendSafetyControlLine("CMD:STEER:STOP"))
    gSteerStopPending = false;
  if (gNavStopPending && sendSafetyControlLine("CMD:NAV:STOP"))
    gNavStopPending = false;
}

static void publishPage() {
  char line[48];
  snprintf(line, sizeof(line), "PAGE:%s", gOperatorUi.wireName());
  printBoth(line);
}

static void publishLinkState() {
  printBoth(gTelemetry.rosConnected ? "LINK:ROS:ONLINE" : "LINK:ROS:OFFLINE");
}

#if defined(BOARD_F103_USBBOOT)
static void enterSystemDfu() {
  stopAppWatchdog();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  for (volatile uint32_t i = 0; i < 1000U; ++i)
    __NOP();

  Board_BackupWrite(0U, kF103BootReqLo);
  Board_BackupWrite(9U, kF103BootReqHi);
  __DSB();
  __ISB();

  gUsb.flush(150U);
  HAL_Delay(20U);
  gUsb.end();

  // Force a real USB detach before reset. The backup request makes the
  // resident C8 bootloader stay in update mode after the reset.
  __HAL_RCC_GPIOA_CLK_ENABLE();
  GPIO_InitTypeDef usbDetach{};
  usbDetach.Pin = GPIO_PIN_12;
  usbDetach.Mode = GPIO_MODE_OUTPUT_PP;
  usbDetach.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &usbDetach);
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_12, GPIO_PIN_RESET);
  HAL_Delay(350U);
  __DSB();
  NVIC_SystemReset();
  while (true) {}
}
#endif

static void stopDriveTest() {
  // STOP is fail-safe and idempotent. Latch it before clearing local motion
  // state, then retry via the dedicated high-priority CDC ring until accepted.
  gDriveStopPending = true;
  driveTestRunning = false;
  driveTestDeadlineMs = 0U;
  gDriveTestDirection = 0;
  gDriveLeaseLastTxMs = 0U;
  serviceSafetyControlTx();
  gTelemetry.state = STATE_STOPPED;
  markUiDirty();
}

static void stopSteeringTest() {
  gSteerStopPending = true;
  serviceSafetyControlTx();
  steeringTestRunning = false;
  markUiDirty();
}

static void stopAllManualTest() {
  stopDriveTest();
  stopSteeringTest();
}

static void zeroCommandedMotionState() {
  gTelemetry.navx.commandLinearMps = 0.0F;
  gTelemetry.navx.commandAngularRps = 0.0F;
  gTelemetry.navx.cmdAgeMs = 0U;
}

static void updateEffectiveEStopState() {
  gTelemetry.eStop = gLocalEStopLatched || gHostEStopActive;
}

static void enforceEmergencyStopOutputs() {
  updateEffectiveEStopState();
  if (!gTelemetry.eStop)
    return;
  zeroCommandedMotionState();
#if BTS_WINCH_ENABLED
  Bts7960Winch::emergencyStop();
#endif
  stopAllManualTest();
  latchAllSafetyStops();
  gLastEStopStopRefreshMs = 0U;
  serviceSafetyControlTx();
  markUiDirty();
}

static void triggerLocalEmergencyStop() {
  // The HMI latch is independent from host ESTOP telemetry. A periodic ESTOP:0
  // from ROS is therefore unable to undo an operator stop from the touchscreen.
  gLocalEStopLatched = true;
  gEStopClearPending = false;
  gEStopAssertPending = true;
  enforceEmergencyStopOutputs();
}

static void setHostEmergencyStop(bool active) {
  // ESTOP:<state> from ROS is a synchronized status, not a new HMI command.
  // Ignore repeated copies of the same state so an F4-originated transient STOP
  // cannot be echoed back into ROS forever through /esc/estop_state.
  if (gHostEStopActive == active) {
    updateEffectiveEStopState();
    return;
  }

  gHostEStopActive = active;
  updateEffectiveEStopState();
  if (gTelemetry.eStop) {
    enforceEmergencyStopOutputs();
  } else {
    // Releasing the host source only removes the stop latch. It never restores
    // an old non-zero motion command.
    zeroCommandedMotionState();
    gLastEStopStopRefreshMs = 0U;
    markUiDirty();
  }
}

static bool localEStopResetSafe() {
  const bool navActive = gTelemetry.navigationStatus == NAV_QUEUED ||
                         gTelemetry.navigationStatus == NAV_NAVIGATING;
  const bool stateSafe = gTelemetry.state == STATE_STOPPED ||
                         gTelemetry.state == STATE_STANDBY;
  return stateSafe && !navActive && !driveTestRunning && !steeringTestRunning &&
         !HmiTouchService::active() &&
         std::fabs(gTelemetry.speedKmh) <= 0.2F &&
         std::fabs(gTelemetry.driveActualMps) <= 0.02F;
}

static bool resetLocalEmergencyStop() {
  if (!gLocalEStopLatched)
    return true;
  if (!localEStopResetSafe())
    return false;
  gLocalEStopLatched = false;
  updateEffectiveEStopState();
  zeroCommandedMotionState();
  gLastEStopStopRefreshMs = 0U;
  // The local HMI request has its own source, so clearing the local latch
  // always clears only that source. Any independent host /safety/estop source
  // remains active in the ESC mux and continues to force zero.
  gEStopAssertPending = false;
  gEStopClearPending = true;
  serviceSafetyControlTx();
  markUiDirty();
  return true;
}

static void serviceEmergencyStopHold() {
  updateEffectiveEStopState();
  if (!gTelemetry.eStop)
    return;
#if BTS_WINCH_ENABLED
  Bts7960Winch::emergencyStop();
#endif
  zeroCommandedMotionState();
  const uint32_t now = HAL_GetTick();
  if (gLastEStopStopRefreshMs == 0U ||
      static_cast<uint32_t>(now - gLastEStopStopRefreshMs) >=
          ESTOP_STOP_REFRESH_MS) {
    gLastEStopStopRefreshMs = now;
    if (gLocalEStopLatched) {
      // Only the physical/local HMI E-stop is allowed to refresh commands back
      // toward ROS. A host E-stop is already authoritative upstream; echoing
      // DRIVE/STEER/NAV STOP here creates a self-sustaining F4_ESTOP loop.
      gEStopClearPending = false;
      gEStopAssertPending = true;
      latchAllSafetyStops();
    }
  }
  serviceSafetyControlTx();
}

static float actualEditValue(UiEditKey key);
static bool gUiDrawActive = false;
static uint32_t gUiDrawLastMs = 0U;
static uint32_t gUiDrawMaxMs = 0U;

static void drawUiNow(bool full = true) {
  if (!splashComplete || gUi.menu == UiMenuId::SPLASH)
    return;
  if (!tft.displayReady() || tft.displayFaulted()) {
    markUiDirty();
    return;
  }
  if (gUiDrawActive) {
    markUiDirty();
    return;
  }

  // Clear the pending flag before drawing. If transport activity changes
  // telemetry during the frame, markUiDirty() will set it again for a later pass.
  gUiDirty = false;
  gUiDrawActive = true;
  const VehicleTelemetry telemetrySnapshot = gTelemetry;
  const uint32_t started_ms = HAL_GetTick();
  tft.beginFrame();
  gOperatorUi.draw(telemetrySnapshot, full);
  tft.endFrame();
  ++gDiagnostics.uiFrames;
  if (full)
    ++gDiagnostics.fullUiFrames;
  else
    ++gDiagnostics.dirtyUiFrames;
  const uint32_t elapsed_ms = static_cast<uint32_t>(HAL_GetTick() - started_ms);
  gUiDrawLastMs = elapsed_ms;
  if (elapsed_ms > gUiDrawMaxMs)
    gUiDrawMaxMs = elapsed_ms;
  gDiagnostics.uiDrawLastMs = gUiDrawLastMs;
  gDiagnostics.uiDrawMaxMs = gUiDrawMaxMs;
  gDiagnostics.displayBytesLastFrame = tft.lastFrameBytes();
  gDiagnostics.displayBytesMaxFrame = tft.maxFrameBytes();
  gUiDrawActive = false;
  lastUiRefreshMs = HAL_GetTick();
}

static bool manualMotionGateValid(bool steering) {
  if (!gUsb.connected() || !gUsb.hostSessionEstablished() ||
      !gTelemetry.rosConnected || !gTelemetry.escFresh ||
      gTelemetry.mode != MODE_MANUAL || !gTelemetry.escReady || gTelemetry.eStop)
    return false;
  if (steering)
    return gTelemetry.encoderReady && gTelemetry.state == STATE_STOPPED;
  return gTelemetry.state == STATE_STOPPED || gTelemetry.state == STATE_RUNNING;
}



static void setMenu(UiMenuId next) {
  if (next == UiMenuId::SPLASH)
    return;
  if (next == UiMenuId::OVERVIEW)
    next = UiMenuId::HOME;

  lastUiInteractionMs = HAL_GetTick();
  const UiMenuId previous = gUi.menu;
  if ((driveTestRunning || steeringTestRunning || previous == UiMenuId::ESC_MANUAL_TEST) &&
      next != previous)
    stopAllManualTest();

  const UiDomain previousDomain = menuDomain(previous);
  const UiDomain nextDomain = menuDomain(next);
  const bool keepDomainPage =
      previousDomain != UiDomain::NONE && previousDomain == nextDomain;
  if (!keepDomainPage)
    gUi.pageIndex = 0U;
  gUi.detailViewIndex = 0U;
  gOperatorUi.cancelTouch();
  gUi.menu = next;
  if (next == UiMenuId::HOME || next == UiMenuId::OVERVIEW)
    gOperatorUi.showPage(HmiOperatorUi::Page::HOME);
  else if (next == UiMenuId::MAIN_MENU)
    gOperatorUi.showPage(HmiOperatorUi::Page::MAIN_MENU);
  else if (menuDomain(next) == UiDomain::ESC)
    gOperatorUi.showPage(HmiOperatorUi::Page::ESC);
  else if (menuDomain(next) == UiDomain::PERCEPTION)
    gOperatorUi.showPage(HmiOperatorUi::Page::PERCEPTION);
  else if (menuDomain(next) == UiDomain::NAVIGATION)
    gOperatorUi.showPage(HmiOperatorUi::Page::NAVIGATION);
  if (next != UiMenuId::SYSTEM_ROOT && menuDomain(next) != UiDomain::SYSTEM)
    gUi.serviceSession = false;

  const UiEditKey editKey = menuEditKey(next);
  gUi.editing = editKey != UiEditKey::NONE;
  if (gUi.editing)
    gUi.editValue = actualEditValue(editKey);
  drawUiNow(true);
  publishPage();
}



static float actualEditValue(UiEditKey key) {
  switch (key) {
  case UiEditKey::OPERATOR_MODE:
    return gTelemetry.mode == MODE_MANUAL ? 1.0F : 0.0F;
  case UiEditKey::MANUAL_SPEED_PCT:
    return static_cast<float>(gTelemetry.manualSpeedPct);
  case UiEditKey::STEERING_TEST_DEG:
    return gTelemetry.steeringTestAngleDeg;
  case UiEditKey::ERPM_PER_MPS:
    return gTelemetry.driveErpmPerMps;
  case UiEditKey::PERCEPTION_INFERENCE:
    return gTelemetry.perceptionInference ? 1.0F : 0.0F;
  case UiEditKey::NONE:
  default:
    return 0.0F;
  }
}




static void serviceManualDriveLease() {
  if (!driveTestRunning || gDriveTestDirection == 0) return;
  const uint32_t now = HAL_GetTick();
  if (static_cast<uint32_t>(now - gDriveLeaseLastTxMs) < DRIVE_LEASE_REFRESH_MS) return;
  if (!manualMotionGateValid(false)) {
    stopDriveTest();
    return;
  }
  char line[48];
  snprintf(line, sizeof(line), "CMD:DRIVE:%s:%u",
           gDriveTestDirection > 0 ? "FWD" : "REV", gTelemetry.manualSpeedPct);
  if (gUsb.writeLineHighPriority(line)) gDriveLeaseLastTxMs = now;
}



static void handleTouch() {
  if (!tft.displayReady() || tft.displayFaulted())
    return;
  if (HmiTouchService::active()) {
    HmiTouchService::service();
    if (HmiTouchService::consumeRedrawRequest()) { markUiDirty(); drawUiNow(true); }
    return;
  }
  bool touchAction = gOperatorUi.pollTouch(gTelemetry);
  if (gOperatorUi.consumeEmergencyStopRequest()) {
    triggerLocalEmergencyStop();
    touchAction = true;
  }
  if (touchAction) {
    ++gDiagnostics.touchActions;
    markUiDirty();
    // Do not start a large TFT SPI write in the same instant that the BTS7960
    // begins switching.  This avoids coupling the actuator start transient into
    // the ILI9341 transaction.  The frame is repainted after motion stops.
#if BTS_WINCH_ENABLED
    const bool winchBusy =
        Bts7960Winch::direction()!=0 || Bts7960Winch::motionPending();
    if (!winchBusy)
      drawUiNow(false);
#else
    drawUiNow(false);
#endif
    publishPage();
  }
  return;
}

static bool parseU32Strict(const char *s, uint32_t &out) {
  return CompactParse::u32(s, out);
}


static void markRosHeartbeat() {
  const uint32_t now = HAL_GetTick();
  if (!gTelemetry.rosConnected) {
    rosHeartbeatStableCount = 1U;
    rosHeartbeatStable = false;
    gTelemetry.rosConnected = true;
    publishLinkState();
    markUiDirty();
  } else {
    const uint32_t gap = static_cast<uint32_t>(now - lastRosHeartbeatMs);
    if (gap <= ROS_HEARTBEAT_STABLE_GAP_MS) {
      if (rosHeartbeatStableCount < 255U)
        ++rosHeartbeatStableCount;
      if (rosHeartbeatStableCount >= ROS_HEARTBEAT_STABLE_COUNT)
        rosHeartbeatStable = true;
    } else {
      rosHeartbeatStableCount = 1U;
      rosHeartbeatStable = false;
    }
  }
  lastRosHeartbeatMs = now;
}

static void forceRosOffline() {
  // ROS/host loss is fail-closed for drive, steering, navigation, and fork.
#if BTS_WINCH_ENABLED
  Bts7960Winch::emergencyStop();
#endif
  stopAllManualTest();
  gNavStopPending = true;
  serviceSafetyControlTx();
  const bool wasConnected = gTelemetry.rosConnected;
  gTelemetry.rosConnected = false;
  rosHeartbeatStableCount = 0U;
  rosHeartbeatStable = false;
  gTelemetry.systemStatus = SYS_NOT_READY;
  gTelemetry.state = STATE_STOPPED;
  // Motion telemetry from ROS/ESC is no longer authoritative after link loss.
  // Leaving stale non-zero values here falsely blocks LOCAL winch commands even
  // though the offline state is explicitly forced STOPPED. Local winch still
  // retains E-stop, directional limits, navigation/test and physical-safety gates.
  gTelemetry.speedKmh = 0.0F;
  gTelemetry.driveActualMps = 0.0F;
  gTelemetry.escReady = false;
  gTelemetry.encoderReady = false;
  gTelemetry.vescConnected = false;
  gTelemetry.vbusValid = false;
  gTelemetry.imuReady = false;
  gTelemetry.cameraReady = false;
  gTelemetry.perceptionReady = false;
  gTelemetry.perceptionInference = false;
  gTelemetry.motionReady = false;
  gTelemetry.nav2Ready = false;
  gTelemetry.navigationStatus = NAV_STOPPED;
  snprintf(gTelemetry.missionState, sizeof(gTelemetry.missionState), "%s", "OFFLINE");
  snprintf(gTelemetry.activeTarget, sizeof(gTelemetry.activeTarget), "%s", "NONE");
  gTelemetry.goalRemainingDistanceM = 0.0F;
  gTelemetry.goalRemainingDistanceValid = false;
  gTelemetry.escFresh = false;
  gTelemetry.perceptionFresh = false;
  gTelemetry.navigationFresh = false;
  gTelemetry.escAgeMs = 0xFFFFFFFFUL;
  gTelemetry.perceptionAgeMs = 0xFFFFFFFFUL;
  gTelemetry.navigationAgeMs = 0xFFFFFFFFUL;
  gTelemetry.escx = EscExtendedTelemetry{};
  gTelemetry.perx = PerceptionExtendedTelemetry{};
  gTelemetry.navx = NavigationExtendedTelemetry{};
  seenEscDomain = false;
  seenPerceptionDomain = false;
  seenNavigationDomain = false;
  if (gTelemetry.configPending) {
    gTelemetry.configPending = false;
    gTelemetry.configLastOk = false;
    snprintf(gTelemetry.configMessage, sizeof(gTelemetry.configMessage), "%s",
             "ROS LINK LOST");
  }
  markUiDirty();
  if (wasConnected)
    publishLinkState();
}

static void checkRosLinkTimeout() {
  if (!gTelemetry.rosConnected)
    return;
  const uint32_t timeoutMs =
      rosHeartbeatStable ? ROS_LINK_TIMEOUT_MS : ROS_LINK_STARTUP_TIMEOUT_MS;
  if (static_cast<uint32_t>(HAL_GetTick() - lastRosHeartbeatMs) > timeoutMs)
    forceRosOffline();
}

static void checkConfigTimeout() {
  if (!gTelemetry.configPending)
    return;
  if (static_cast<uint32_t>(HAL_GetTick() - gUi.pendingSinceMs) <=
      CONFIG_ACK_TIMEOUT_MS)
    return;
  gTelemetry.configPending = false;
  gTelemetry.configLastOk = false;
  snprintf(gTelemetry.configMessage, sizeof(gTelemetry.configMessage), "%s",
           "ACK TIMEOUT");
  drawUiNow(true);
}


// Legacy key/value telemetry is display compatibility only. Domain freshness is
// authoritative exclusively from accepted, host-session-bound F4X3 frames.


static void updateDomainFreshness(uint32_t now) {
  const bool oldEsc = gTelemetry.escFresh;
  const uint32_t oldExtendedMask = extendedFreshMask(gTelemetry);
  const bool oldVbusValid = gTelemetry.vbusValid;
  const bool oldPer = gTelemetry.perceptionFresh;
  const bool oldNav = gTelemetry.navigationFresh;
  gTelemetry.escAgeMs = seenEscDomain
                            ? static_cast<uint32_t>(now - lastEscDomainMs)
                            : 0xFFFFFFFFUL;
  gTelemetry.perceptionAgeMs =
      seenPerceptionDomain ? static_cast<uint32_t>(now - lastPerceptionDomainMs)
                           : 0xFFFFFFFFUL;
  gTelemetry.navigationAgeMs =
      seenNavigationDomain ? static_cast<uint32_t>(now - lastNavigationDomainMs)
                           : 0xFFFFFFFFUL;
  gTelemetry.escFresh = gTelemetry.rosConnected && seenEscDomain &&
                        gTelemetry.escAgeMs <= DOMAIN_DATA_STALE_MS;
  gTelemetry.perceptionFresh =
      gTelemetry.rosConnected && seenPerceptionDomain &&
      gTelemetry.perceptionAgeMs <= DOMAIN_DATA_STALE_MS;
  gTelemetry.navigationFresh =
      gTelemetry.rosConnected && seenNavigationDomain &&
      gTelemetry.navigationAgeMs <= DOMAIN_DATA_STALE_MS;
  if (oldEsc && !gTelemetry.escFresh && (driveTestRunning || steeringTestRunning))
    stopAllManualTest();
  updateExtendedFreshness(gTelemetry, now);
  if (oldEsc != gTelemetry.escFresh || oldPer != gTelemetry.perceptionFresh ||
      oldNav != gTelemetry.navigationFresh || oldExtendedMask != extendedFreshMask(gTelemetry) ||
      oldVbusValid != gTelemetry.vbusValid)
    markUiDirty();
}

static bool pinHigh(GPIO_TypeDef *port, uint16_t pin) {
  return HAL_GPIO_ReadPin(port, pin) == GPIO_PIN_SET;
}

static void sampleDiagnostics(uint32_t now) {
  gDiagnostics.uptimeMs = now;
  gDiagnostics.tftControllerId = gTftControllerId;
  gDiagnostics.tftPowerMode = gTftPowerMode;
  gDiagnostics.tftMadctl = gTftMadctl;
  gDiagnostics.tftPixelFormat = gTftPixelFormat;
  const bool oldTftOk = gDiagnostics.tftOk;
  gDiagnostics.displayReady = tft.displayReady();
  gDiagnostics.displayFaulted = tft.displayFaulted();
  // Operational health is the proven write-path state. Controller ID remains
  // separately visible in diagnostics and may be zero on write-only modules.
  gDiagnostics.tftOk = gDiagnostics.displayReady &&
                       !gDiagnostics.displayFaulted;
  gDiagnostics.spiTransactions = tft.spiTransactions();
  gDiagnostics.spiBytesTx = tft.spiBytesTx();
  gDiagnostics.spiTimeoutCount = tft.spiTimeoutCount();
  gDiagnostics.spiHalErrorCount = tft.spiHalErrorCount();
  gDiagnostics.spiRecoveryCount = tft.spiRecoveryCount();
  gDiagnostics.spiBusConflictCount = tft.spiBusConflictCount() + Board_SpiContentionCount();
  gDiagnostics.tftWriteClockHz = tft.writeClockHz();
  gDiagnostics.tftFastWriteValidated = tft.fastWriteValidated();
  gDiagnostics.tftUltraFastWriteValidated = tft.ultraFastWriteValidated();
  gDiagnostics.touchReadCount = tft.touchReadCount();
  gDiagnostics.touchRejectFastCount = tft.touchRejectFastCount();
  gDiagnostics.displayBytesLastFrame = tft.lastFrameBytes();
  gDiagnostics.displayBytesMaxFrame = tft.maxFrameBytes();
  gDiagnostics.uiDrawLastMs = gUiDrawLastMs;
  gDiagnostics.uiDrawMaxMs = gUiDrawMaxMs;
  gDiagnostics.serviceGapLifetimeMaxMs = Board_MaxServiceGapMs();
  gDiagnostics.maxServiceGapMs = Board_SteadyMaxServiceGapMs();
  gDiagnostics.serviceGapP95Ms = Board_ServiceGapP95Ms();
  gDiagnostics.serviceGapP99Ms = Board_ServiceGapP99Ms();
  gDiagnostics.stackHeadroomBytes = Board_StackHeadroomBytes();
  gDiagnostics.minStackHeadroomBytes = Board_MinStackHeadroomBytes();
  if (oldTftOk != gDiagnostics.tftOk)
    markUiDirty();
  gDiagnostics.rosHeartbeatAgeMs =
      gTelemetry.rosConnected ? static_cast<uint32_t>(now - lastRosHeartbeatMs)
                              : 0xFFFFFFFFUL;

  // ESC transport is direct USB on the host; the gateway owns no ESC UART.
  gDiagnostics.vescUartOk = false;
  gDiagnostics.vescUartErrors = 0U;
  gDiagnostics.vescUartOverflow = 0U;
  gDiagnostics.vescUartTxDropped = 0U;
  gDiagnostics.vescFrameErrors = 0U;
  gDiagnostics.vescRecoveryCount = 0U;
  gDiagnostics.vescLastFrameAgeMs = 0xFFFFFFFFUL;
  gDiagnostics.gnssUartOk = false;
  gDiagnostics.magOk = false;
  gDiagnostics.gnssUartErrors = 0U;
  gDiagnostics.gnssUartOverflow = 0U;
  gDiagnostics.gnssUartTxDropped = 0U;
  gDiagnostics.magErrors = 0U;
  gDiagnostics.pa2GnssTx = false;
  gDiagnostics.pa3GnssRx = false;
  gDiagnostics.pb8I2cScl = false;
  gDiagnostics.pb9I2cSda = false;
  gDiagnostics.pb12Safety = true;
  gDiagnostics.pb13SafetyLed = false;
  gDiagnostics.pa8Buzzer = pinHigh(GPIOA, GPIO_PIN_8);
  gDiagnostics.pa5SpiSck = pinHigh(GPIOA, GPIO_PIN_5);
  gDiagnostics.pa6SpiMiso = pinHigh(GPIOA, GPIO_PIN_6);
  gDiagnostics.pa7SpiMosi = pinHigh(GPIOA, GPIO_PIN_7);
  gDiagnostics.pb0TftCs = pinHigh(GPIOB, GPIO_PIN_0);
  gDiagnostics.pb1TftDc = pinHigh(GPIOB, GPIO_PIN_1);
  gDiagnostics.tftRst = pinHigh(GPIOB, GPIO_PIN_10);
  gDiagnostics.pa4TouchCs = pinHigh(GPIOA, GPIO_PIN_4);
  gDiagnostics.pa11UsbDm = pinHigh(GPIOA, GPIO_PIN_11);
  gDiagnostics.pa12UsbDp = pinHigh(GPIOA, GPIO_PIN_12);
}


static void handleSerialCommand(char *command);

#if defined(BOARD_F103_USBBOOT)
static bool maintenanceOutputsStopped() {
  const bool navActive = gTelemetry.navigationStatus == NAV_QUEUED ||
                         gTelemetry.navigationStatus == NAV_NAVIGATING;
  const bool stateSafe =
      gTelemetry.state == STATE_STOPPED || gTelemetry.state == STATE_STANDBY;
#if BTS_WINCH_ENABLED
  const bool winchSafe = Bts7960Winch::direction() == 0 &&
                         !Bts7960Winch::motionPending() &&
                         Bts7960Winch::appliedPwm() == 0U;
#else
  const bool winchSafe = true;
#endif
  return stateSafe && std::fabs(gTelemetry.speedKmh) <= 0.2F &&
         std::fabs(gTelemetry.driveActualMps) <= 0.02F && !navActive &&
         !driveTestRunning && !steeringTestRunning && winchSafe;
}


static bool firmwareUpdateSafe() {
  return maintenanceOutputsStopped();
}
#endif



#if BTS_WINCH_ENABLED
static const char *winchRejectReason(const char *command) {
  if (Bts7960Winch::faulted()) return "FAULT";
  if (Bts7960Winch::topLimitActive() &&
      (std::strncmp(command, "UP", 2) == 0 ||
       std::strncmp(command, "WINCH UP", 8) == 0))
    return "TOP_LIMIT";
  if (Bts7960Winch::bottomLimitActive() &&
      (std::strncmp(command, "DOWN", 4) == 0 ||
       std::strncmp(command, "WINCH DOWN", 10) == 0))
    return "BOTTOM_LIMIT";
  if (gTelemetry.eStop) return "ESTOP";
  const bool navActive = gTelemetry.navigationStatus == NAV_QUEUED ||
                         gTelemetry.navigationStatus == NAV_NAVIGATING;
  if (navActive) return "NAV_ACTIVE";
  if (driveTestRunning || steeringTestRunning) return "MOTOR_TEST_ACTIVE";
  if (gTelemetry.state == STATE_RUNNING ||
      std::fabs(gTelemetry.speedKmh) > 0.2F ||
      std::fabs(gTelemetry.driveActualMps) > 0.02F)
    return "VEHICLE_MOVING";
  return "SAFETY_GATE";
}

static bool executeLocalWinchCommand(const char *command) {
  if (command == nullptr || std::strncmp(command, "LOCAL ", 6) != 0)
    return false;

  const char *payload = command + 6;
  refreshWinchSafetyInputs();

  uint32_t localSeconds = 0U;
  bool parsedTimed = false;
  if (!std::strncmp(payload, "UP ", 3) &&
      std::strcmp(payload, "UP HOME") != 0 &&
      parseU32Strict(payload + 3, localSeconds) &&
      localSeconds >= 1U && localSeconds <= 30U) {
    Bts7960Winch::timedLocal(+1, localSeconds * 1000U);
    parsedTimed = true;
  } else if (!std::strncmp(payload, "DOWN ", 5) &&
             std::strcmp(payload, "DOWN HOME") != 0 &&
             parseU32Strict(payload + 5, localSeconds) &&
             localSeconds >= 1U && localSeconds <= 30U) {
    Bts7960Winch::timedLocal(-1, localSeconds * 1000U);
    parsedTimed = true;
  }

  if (parsedTimed) {
    // already dispatched above
  } else if (!std::strcmp(payload, "UP HOME"))
    Bts7960Winch::upHomeLocal();
  else if (!std::strcmp(payload, "DOWN HOME"))
    Bts7960Winch::downHomeLocal();
  else if (!std::strcmp(payload, "STOP"))
    Bts7960Winch::stop();
  else
    return false;

  const bool stopCommand = !std::strcmp(payload, "STOP");
  const bool accepted = stopCommand || Bts7960Winch::direction() != 0 ||
                        Bts7960Winch::motionPending();
  char line[96];
  if (accepted) {
    std::snprintf(line, sizeof(line), "ACK:WINCH:LOCAL:%s",
                  Bts7960Winch::stateName());
  } else {
    std::snprintf(line, sizeof(line), "ERR:WINCH:LOCAL:%s",
                  winchRejectReason(payload));
  }
  (void)gUsb.writeLineCritical(line, 120U);
  markUiDirty();
  return true;
}

static bool executeWinchCommand(
    const char *command, uint32_t transactionId, bool transactional) {
  refreshWinchSafetyInputs();
  if (!Bts7960Winch::processCommand(command))
    return false;

  const bool stopCommand = !std::strcmp(command, "STOP") ||
                           !std::strcmp(command, "WINCH STOP");
  const bool accepted = stopCommand || Bts7960Winch::direction() != 0 ||
                        Bts7960Winch::motionPending();

  if (transactional) {
    char line[112];
    if (accepted) {
      std::snprintf(
          line, sizeof(line), "ACK:WINCH:%lu:%s",
          static_cast<unsigned long>(transactionId),
          Bts7960Winch::stateName());
    } else {
      std::snprintf(
          line, sizeof(line), "ERR:WINCH:%lu:%s",
          static_cast<unsigned long>(transactionId),
          winchRejectReason(command));
    }
    (void)gUsb.writeLineCritical(line, 120U);
  } else {
    (void)gUsb.writeLineCritical(
        accepted ? "ACK:WINCH:CMD" : "ERR:WINCH:SAFETY_GATE", 120U);
  }
  markUiDirty();
  return true;
}

static bool handleTransactionalWinchCommand(char *command) {
  static constexpr char prefix[] = "WINCH:CMD:";
  if (std::strncmp(command, prefix, sizeof(prefix) - 1U) != 0)
    return false;

  char *idText = command + sizeof(prefix) - 1U;
  char *separator = std::strchr(idText, ':');
  if (separator == nullptr) {
    (void)gUsb.writeLineCritical("ERR:WINCH:0:INVALID_TRANSACTION", 120U);
    return true;
  }

  *separator = '\0';
  uint32_t transactionId = 0U;
  const bool validId = parseU32Strict(idText, transactionId) &&
                       transactionId != 0U;
  *separator = ':';
  const char *payload = separator + 1;
  if (!validId || *payload == '\0') {
    (void)gUsb.writeLineCritical("ERR:WINCH:0:INVALID_TRANSACTION", 120U);
    return true;
  }

  if (!executeWinchCommand(payload, transactionId, true)) {
    char line[80];
    std::snprintf(
        line, sizeof(line), "ERR:WINCH:%lu:INVALID_COMMAND",
        static_cast<unsigned long>(transactionId));
    (void)gUsb.writeLineCritical(line, 120U);
  }
  return true;
}
#endif

#include "F103MinimalCommandHandler.inc"
static void handleSerialCommand(char *command) { handleSerialCommandF103(command); }

static void serviceDeferredCommands(uint8_t budget = 8U) {
  if (gRealtimeParserActive || gUiDrawActive)
    return;
  while (gDeferredCommandTail != gDeferredCommandHead && budget-- > 0U) {
    char command[kDeferredCommandLength];
    std::snprintf(command, sizeof(command), "%s",
                  gDeferredCommands[gDeferredCommandTail]);
    gDeferredCommandTail = static_cast<uint8_t>((gDeferredCommandTail + 1U) %
                                                kDeferredCommandSlots);
    handleSerialCommand(command);
  }
}

static void
pollSerialGui(std::size_t byteBudget = 256U) {
  // Host traffic is untrusted with respect to realtime scheduling. Bound both
  // bytes and complete commands per service pass so a PC flood cannot starve
  // watchdog heartbeat, touch, safety control, or UI refresh.
  gUsb.poll();
  const uint32_t usbResync = gUsb.rxResyncCount();
  if (usbResync != gLastUsbRxResyncCount) {
    const uint32_t delta = static_cast<uint32_t>(usbResync - gLastUsbRxResyncCount);
    gLastUsbRxResyncCount = usbResync;
    gDiagnostics.usbRxResyncs += delta;
    serialRxLen = 0U;
    serialRxDiscarding = false;
  }
  std::size_t processed = 0U;
  uint8_t commandBudget = 4U;
  while (gUsb.available() > 0 && processed < byteBudget && commandBudget > 0U) {
    const int value = gUsb.read();
    if (value < 0)
      break;
    ++processed;
    const char c = static_cast<char>(value);
    if (c == '\r')
      continue;
    if (serialRxDiscarding) {
      if (c == '\n')
        serialRxDiscarding = false;
      continue;
    }
    if (c == '\n') {
      serialRx[serialRxLen] = '\0';
      if (serialRxLen > 0U) {
        handleSerialCommand(serialRx);
        --commandBudget;
      }
      serialRxLen = 0U;
    } else if (serialRxLen + 1U < sizeof(serialRx)) {
      serialRx[serialRxLen++] = c;
    } else {
      serialRxLen = 0U;
      serialRxDiscarding = true;
      ++gDiagnostics.overlongCommands;
      (void)gUsb.writeLine("ERR:COMMAND_TOO_LONG");
    }
  }
}

static void updateDisplayDiagnostics(bool ready) {
  gDiagnostics.tftControllerId = gTftControllerId;
  gDiagnostics.tftPowerMode = gTftPowerMode;
  gDiagnostics.tftMadctl = gTftMadctl;
  gDiagnostics.tftPixelFormat = gTftPixelFormat;
  gDiagnostics.displayReady = ready;
  gDiagnostics.displayFaulted = tft.displayFaulted();
  gDiagnostics.tftOk = ready;
}

static bool validateDisplayController() {
  // Register readback is useful for identification, but it must not gate the
  // whole HMI. Some ILI9341/XPT2046 modules have a reliable write path while
  // DOUT/readback is weak or unavailable. Previously a zero ID forced
  // displayReady=false, which permanently disabled handleTouch() and caused a
  // recovery/reset loop every two seconds even though the visible TFT worked.
  gTftControllerId = tft.displayFaulted() ? 0U : tft.readId();
  const bool controllerIdentified =
      (gTftControllerId & 0xFFFFU) == 0x9341U && !tft.displayFaulted();

  if (!tft.displayFaulted()) {
    tft.setRotation(1U);
  }

  if (controllerIdentified && !tft.displayFaulted()) {
    gTftPowerMode = tft.readRegister8(0x0AU, 0U);
    gTftMadctl = tft.readRegister8(0x0BU, 0U);
    gTftPixelFormat = tft.readRegister8(0x0CU, 0U);
    (void)tft.validateFastWriteClock();
  } else {
    // Stay at the conservative 6 MHz write clock when readback cannot prove
    // the controller identity. The write-path remains usable and touch polling
    // is allowed; diagnostics still expose ID=0 for field troubleshooting.
    gTftPowerMode = gTftMadctl = gTftPixelFormat = 0U;
  }

  const bool ready = !tft.displayFaulted();
  tft.setDisplayReady(ready);
  tft.setSwapBytes(true);
  updateDisplayDiagnostics(ready);
  if (ready)
    tft.setTextDatum(MC_DATUM);
  return ready;
}

static void beginDisplayInitialization() {
  gDisplayInitAttempt = 1U;
  gDisplayInitRetryAtMs = 0U;
  gDisplayInitLastSuccess = false;
  gTftControllerId = 0U;
  tft.beginInit();
  gDisplayInitState = DisplayInitState::RUNNING;
}

static void serviceDisplayInitialization(uint32_t now) {
  if (gDisplayInitState == DisplayInitState::IDLE)
    return;
  if (gDisplayInitState == DisplayInitState::RETRY_WAIT) {
    if (static_cast<int32_t>(gDisplayInitRetryAtMs - now) > 0)
      return;
    tft.beginInit();
    gDisplayInitState = DisplayInitState::RUNNING;
    return;
  }
  if (!tft.serviceInit())
    return;

  if (validateDisplayController()) {
    gDisplayInitLastSuccess = true;
    gDisplayInitState = DisplayInitState::IDLE;
    return;
  }
  if (gDisplayInitAttempt < 2U) {
    ++gDisplayInitAttempt;
    gDisplayInitRetryAtMs = now + 40U;
    gDisplayInitState = DisplayInitState::RETRY_WAIT;
    return;
  }
  gDisplayInitLastSuccess = false;
  gDisplayInitState = DisplayInitState::IDLE;
}

static bool initDisplayBlockingAtBoot() {
  beginDisplayInitialization();
  while (gDisplayInitState != DisplayInitState::IDLE) {
    serviceDisplayInitialization(HAL_GetTick());
    Board_RealtimeService();
  }
  if (gDisplayInitLastSuccess)
    tft.fillScreen(C_BG);
  return gDisplayInitLastSuccess;
}

static void serviceDisplayRecovery(uint32_t now) {
  if (gDisplayInitState != DisplayInitState::IDLE) {
    serviceDisplayInitialization(now);
    if (gDisplayInitState != DisplayInitState::IDLE || !gDisplayInitLastSuccess)
      return;
    gOperatorUi.cancelTouch();
        markUiDirty();
    drawUiNow(true);
    return;
  }
  if (tft.displayReady() && !tft.displayFaulted())
    return;
  if (gUiDrawActive ||
      static_cast<uint32_t>(now - lastDisplayRecoveryMs) < 2000U)
    return;
  lastDisplayRecoveryMs = now;
  /* A display recovery invalidates the physical touch surface. Stop any manual
   * motion session before resetting the controller, then rebuild the display
   * asynchronously so safety and USB service continue each main-loop iteration.
   */
  gOperatorUi.cancelTouch();
  #if BTS_WINCH_ENABLED
  Bts7960Winch::emergencyStop();
#endif
  if (driveTestRunning || steeringTestRunning ||
      gUi.menu == UiMenuId::ESC_MANUAL_TEST)
    stopAllManualTest();
  beginDisplayInitialization();
}


int main() {
  // Capture reset flags before board/HAL initialization mutates clock/reset state.
  gResetCauseFlags = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();
  Board_Init();
  // TIM3 supervises every subsystem after the base clock/peripheral bring-up:
  // persistent config, winch/touch init, USB, SPI/display init, then main loop.
  startAppWatchdog();
  noteWatchdogProgress();
  gPersistentConfigReady = gPersistentConfig.begin();
  (void)loadTouchCalibration();
  noteWatchdogProgress();
#if BTS_WINCH_ENABLED
  Bts7960Winch::begin(gPersistentConfigReady ? &gPersistentConfig : nullptr);
  HmiTouchService::begin(emitServiceLine, persistTouchCalibration);
  noteWatchdogProgress();
#endif
  bool usbInitOk = false;
  for (uint8_t attempt = 0U; attempt < 3U && !usbInitOk; ++attempt) {
    noteWatchdogProgress();
    usbInitOk = gUsb.begin();
    noteWatchdogProgress();
    if (!usbInitOk) {
      gUsb.end();
      HAL_Delay(50U * static_cast<uint32_t>(attempt + 1U));
      noteWatchdogProgress();
    }
  }
  HAL_Delay(50U);
  noteWatchdogProgress();
  printBoth("ADV HMI native realtime menu firmware - boot");
  if (!gPersistentConfigReady) printBoth("ERR:EEPROM:INIT");
  Board_SetRealtimeServiceCallback([]() {
    // Realtime safety/transport service is valid watchdog progress even while
    // startup or a long TFT operation has not yet completed a full main loop.
    noteWatchdogProgress();
    // USB completion IRQ never starts the next packet. Service it here in
    // thread context so long TFT transfers cannot starve CDC progress.
    gUsb.service();
    serviceUsbTransportState();
    serviceEmergencyStopHold();
    serviceSafetyControlTx();
    serviceManualDriveLease();
  #if BTS_WINCH_ENABLED
    refreshWinchSafetyInputs();
    Bts7960Winch::update();
#endif
    // Keep USB/HMI command parsing live during long TFT bursts so safety and
    // transport state continue to progress while pixels are being written.
    gRealtimeParserActive = true;
    pollSerialGui(512U);
    gRealtimeParserActive = false;
  });
  (void)initDisplayBlockingAtBoot();
  splashComplete = true;
  gUi.menu = UiMenuId::HOME;
  gOperatorUi.reset();
  drawUiNow(true);
  publishPage();
  if (!usbInitOk)
    // USB/ROS unavailable means the remote system is NOT READY, but it is not
    // an actuator safety fault. Local TFT fork/winch control must remain
    // available while physical limits, E-stop and vehicle-motion interlocks
    // continue to gate Bts7960Winch.
    gTelemetry.systemStatus = SYS_NOT_READY;

  while (true) {
    /* Service deferred UART recovery/queue work every loop. Motor-link TX also
     * chains in its ISR, so this is a fallback rather than the realtime clock.
     */
    Board_Service();
  #if BTS_WINCH_ENABLED
    refreshWinchSafetyInputs();
    Bts7960Winch::update();
    const bool winchUiBusy =
        Bts7960Winch::direction()!=0 || Bts7960Winch::motionPending();
    if (gOperatorUi.serviceActuatorState())
      markUiDirty();
    if (gWinchUiWasBusy && !winchUiBusy) {
      // The ILI9341 may have internally reset during an actuator supply/EMI
      // transient without producing an SPI HAL error.  Re-initialize it once
      // after motion has safely stopped so a white panel recovers automatically.
      gPostWinchDisplayReinitPending = true;
      gPostWinchDisplayReinitAtMs = HAL_GetTick() + 60U;
    }
    gWinchUiWasBusy = winchUiBusy;
#endif
    gUsb.service();
    serviceUsbTransportState();
    serviceEmergencyStopHold();
    serviceSafetyControlTx();
    serviceManualDriveLease();
    if (!gCrashCounterCleared &&
        static_cast<uint32_t>(HAL_GetTick() - gWatchdogHealthySinceMs) >=
            kCrashCounterClearMs) {
      __HAL_RCC_PWR_CLK_ENABLE();
      HAL_PWR_EnableBkUpAccess();
      Board_BackupWrite(1U, 0U);
      Board_BackupWrite(2U, 0U);
      __DSB();
      gCrashCounterCleared = true;
    }
      pollSerialGui();
    serviceDeferredCommands();


    Board_Service();
    pollSerialGui();
    serviceDeferredCommands();
    checkRosLinkTimeout();
    checkConfigTimeout();
    const uint32_t serviceNow = HAL_GetTick();
    updateDomainFreshness(serviceNow);
    if (static_cast<uint32_t>(serviceNow - lastDiagnosticsMs) >= 100U) {
      lastDiagnosticsMs = serviceNow;
      sampleDiagnostics(serviceNow);
    }
#if BTS_WINCH_ENABLED
    if (gPostWinchDisplayReinitPending &&
        static_cast<int32_t>(serviceNow - gPostWinchDisplayReinitAtMs) >= 0 &&
        gDisplayInitState == DisplayInitState::IDLE && !gUiDrawActive) {
      gPostWinchDisplayReinitPending = false;
      gOperatorUi.cancelTouch();
      beginDisplayInitialization();
    }
#endif
    serviceDisplayRecovery(serviceNow);
    if ((gUi.menu == UiMenuId::SYSTEM_OVERVIEW ||
         gUi.menu == UiMenuId::SYSTEM_PINS_IO ||
         gUi.menu == UiMenuId::SYSTEM_PINS_DISPLAY ||
         gUi.menu == UiMenuId::SYSTEM_LINKS ||
         gUi.menu == UiMenuId::SYSTEM_ERRORS) &&
        static_cast<uint32_t>(serviceNow - lastSystemUiMarkMs) >= 500U) {
      lastSystemUiMarkMs = serviceNow;
      markUiDirty();
    }
    if (splashComplete && menuDomain(gUi.menu) == UiDomain::SYSTEM && !gOperatorUi.touchDown() &&
        static_cast<uint32_t>(serviceNow - lastUiInteractionMs) >=
            DIAGNOSTIC_AUTO_RETURN_MS) {
      setMenu(UiMenuId::HOME);
    }
    if (driveTestRunning &&
        static_cast<int32_t>(driveTestDeadlineMs - serviceNow) <= 0)
      stopDriveTest();

    const uint32_t now = HAL_GetTick();
    if (static_cast<uint32_t>(now - lastTouchPollMs) >= TOUCH_POLL_MS) {
      lastTouchPollMs = now;
      handleTouch();
    }
    if (gUiDirty && !gOperatorUi.touchDown() &&
#if BTS_WINCH_ENABLED
        Bts7960Winch::direction()==0 && !Bts7960Winch::motionPending() &&
#endif
        gDisplayInitState == DisplayInitState::IDLE &&
        static_cast<uint32_t>(now - lastUiRefreshMs) >= DISPLAY_REFRESH_MS) {
      drawUiNow(false);
    }

    static uint32_t ledMs = 0U;
    if (static_cast<uint32_t>(HAL_GetTick() - ledMs) >= 500U) {
      ledMs = HAL_GetTick();
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
      noteWatchdogProgress();
  }
}

#endif
