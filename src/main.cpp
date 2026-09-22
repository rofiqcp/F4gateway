// ============================================================================
// ADV HMI Firmware — STM32F411CEU6 + ILI9341 320x240 + XPT2046
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

static void emitServiceLine(const char *line) {
  if (line != nullptr) (void)gUsb.writeLineCritical(line, 120U);
}

static volatile uint32_t gWatchdogProgressEpoch = 0U;
static volatile uint32_t gAppWatchdogLastEpoch = 0U;
static volatile uint8_t gAppWatchdogStallTicks = 0U;
static volatile bool gAppWatchdogArmed = false;
static uint32_t gResetCauseFlags = 0U;
// TIM11 fires at 10 Hz on both supported MCU clock profiles.
static constexpr uint8_t APP_WATCHDOG_TIMEOUT_TICKS = 35U;

static bool splashComplete = false;
static uint8_t splashProgress = 0;
static bool splashReadyText = false;
static uint32_t splashStartMs = 0;
static uint32_t splashReadyMs = 0;
static uint32_t lastFrameMs = 0;
static bool gUiDirty = false;
static inline void markUiDirty() { gUiDirty = true; }
static uint32_t lastUiRefreshMs = 0;
static uint32_t lastRosHeartbeatMs = 0;
static uint32_t lastTouchPollMs = 0;
static uint32_t lastDiagnosticsMs = 0;
static uint32_t lastSystemUiMarkMs = 0;
static uint32_t lastDisplayRecoveryMs = 0;
static uint32_t lastUiInteractionMs = 0U;
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
#if defined(BOARD_F103C8)
static constexpr uint8_t kDeferredCommandSlots = 12U;
#else
static constexpr uint8_t kDeferredCommandSlots = 64U;
#endif
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

#if !defined(BOARD_F103C8) || defined(BOARD_F103_256K)
static uint32_t gDfuArmDeadlineMs = 0U;
#endif
static constexpr uint32_t kBootRequestMagic = 0x42465544UL; // DFUB
static constexpr uint16_t kF103BootReqLo = 0x5544U;
static constexpr uint16_t kF103BootReqHi = 0x4246U;
static constexpr uint32_t kAppCrashMagic = 0x48535243UL;    // CRSH
static constexpr uint32_t kCrashCounterClearMs = 30000U;
static bool gCrashCounterCleared = false;
static uint32_t gWatchdogHealthySinceMs = 0U;

// Stoppable application watchdog. F411 uses TIM11 and F103 uses TIM3 instead
// of IWDG so maintenance/recovery paths can explicitly control supervision.
static void appWatchdogIsr() {
  if (!gAppWatchdogArmed)
    return;

  // Use TIM11's own periodic interrupt as the timebase. This keeps the watchdog
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
    // Fail closed before reset: a stalled main loop must never leave the
    // BTS7960 energized while reset diagnostics are being recorded.
    TIM2->CCR3 = 0U;
    TIM4->CCR3 = 0U;
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

#if !defined(BOARD_F103C8) || defined(BOARD_F103_256K)
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
  const bool stationary =
      (gTelemetry.state == STATE_STOPPED || gTelemetry.state == STATE_STANDBY) &&
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

#if !defined(BOARD_F103C8) || defined(BOARD_F103_256K)
static void enterSystemDfu() {
  stopAppWatchdog();
  __HAL_RCC_PWR_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  for (volatile uint32_t i = 0; i < 1000U; ++i)
    __NOP();
#if defined(BOARD_F103_256K)
  Board_BackupWrite(0U, kF103BootReqLo);
  Board_BackupWrite(9U, kF103BootReqHi);
#else
  Board_BackupWrite(0U, kBootRequestMagic);
#endif
  __DSB();
  __ISB();
  gUsb.flush(150U);
  HAL_Delay(20U);
  gUsb.end();

#if defined(BOARD_F103_256K)
  /*
   * The resident bootloader now asserts PA12 LOW at the very start of main()
   * before HAL/clock setup. Use a real core reset here so USB, SPI, timers,
   * DMA and NVIC state are reset by hardware instead of being inherited by a
   * direct image jump. The BKP request above selects maintenance mode.
   */
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
#else
  NVIC_SystemReset();
#endif
  while (true) {
  }
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
  snprintf(gTelemetry.steeringTestState, sizeof(gTelemetry.steeringTestState),
           "%s", "IDLE");
  markUiDirty();
}

static void stopAllManualTest() {
  stopDriveTest();
  stopSteeringTest();
}

static void zeroCommandedMotionState() {
  gTelemetry.driveTargetMps = 0.0F;
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
  // The F4 request has its own ROS source, so clearing the local HMI latch
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
      gEStopClearPending = false;
      gEStopAssertPending = true;
    }
    latchAllSafetyStops();
  }
  serviceSafetyControlTx();
}

#if !defined(BOARD_F103C8) || defined(BOARD_F103_256K)
static bool motionSafeForHeavyMaintenance();
#endif
#if !defined(BOARD_F103C8) || defined(BOARD_F103_256K)
static bool runTftSelfTest();
#endif
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

static void enforceManualMotionGate() {
  if (driveTestRunning && !manualMotionGateValid(false))
    stopDriveTest();
  if (steeringTestRunning && !manualMotionGateValid(true))
    stopSteeringTest();
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
    drawUiNow(true);
    publishPage();
  }
  return;
}

static bool eqIgnoreCase(const char *a, const char *b) {
  while (*a && *b) {
    char ca = *a++;
    char cb = *b++;
    if (ca >= 'a' && ca <= 'z')
      ca -= 32;
    if (cb >= 'a' && cb <= 'z')
      cb -= 32;
    if (ca != cb)
      return false;
  }
  return *a == '\0' && *b == '\0';
}

static bool parseBool(const char *s) {
  return !strcmp(s, "1") || eqIgnoreCase(s, "ON") || eqIgnoreCase(s, "READY") ||
         eqIgnoreCase(s, "TRUE");
}


static bool parseBoolStrict(const char *s, bool &out) {
  if (s == nullptr || *s == '\0') return false;
  if (!strcmp(s, "1") || eqIgnoreCase(s, "ON") || eqIgnoreCase(s, "READY") ||
      eqIgnoreCase(s, "TRUE")) { out = true; return true; }
  if (!strcmp(s, "0") || eqIgnoreCase(s, "OFF") || eqIgnoreCase(s, "NOT READY") ||
      eqIgnoreCase(s, "FALSE")) { out = false; return true; }
  return false;
}

#if defined(BOARD_F103C8)
using LegacyReal = float;
static bool parseFiniteLegacyRealStrict(const char *s, LegacyReal &out) {
  return CompactParse::realf(s, out);
}
#else
using LegacyReal = double;
static bool parseFiniteLegacyRealStrict(const char *s, LegacyReal &out) {
  return CompactParse::real(s, out);
}
#endif

static bool parseLongStrict(const char *s, long &out) {
  int32_t value = 0;
  if (!CompactParse::i32(s, value)) return false;
  out = static_cast<long>(value);
  return true;
}

static bool parseU32Strict(const char *s, uint32_t &out) {
  return CompactParse::u32(s, out);
}

static LegacyReal parseDoubleOrZero(const char *s) {
  LegacyReal value = static_cast<LegacyReal>(0);
  return parseFiniteLegacyRealStrict(s, value) ? value : static_cast<LegacyReal>(0);
}

static int parseIntOrZero(const char *s) {
  int32_t value = 0;
  return CompactParse::i32(s, value) ? static_cast<int>(value) : 0;
}

static bool legacyTelemetryPayloadValid(const char *command) {
  if (command == nullptr) return false;
  auto after = [command](const char *prefix) -> const char * {
    const size_t n = std::strlen(prefix);
    return std::strncmp(command, prefix, n) == 0 ? command + n : nullptr;
  };
  const char *p = nullptr;
  LegacyReal d = static_cast<LegacyReal>(0);
  long i = 0;
  bool b = false;

  // Every scalar that can refresh a domain must be fully parseable and finite.
  static const char *const floatPrefixes[] = {
      "SPD:", "DRIVE_TGT:", "DRIVE_ACT:", "ERPM:", "VBUS:",
      "STEER_TARGET:", "STEER_ACTUAL:", "STEER_ERR:", "CFGSTEERTEST:",
      "CFGERPMMPS:", "LAT:", "LON:", "HDOP:", "HACC:", "GAGE:", "HEAD:",
      "GYROZ:", "FPS:", "DIST:", "CONF:"};
  for (const char *prefix : floatPrefixes) {
    if ((p = after(prefix)) != nullptr) {
      if (!parseFiniteLegacyRealStrict(p, d)) return false;
      if (!std::strcmp(prefix, "LAT:") &&
          (d < static_cast<LegacyReal>(-90.0F) || d > static_cast<LegacyReal>(90.0F))) return false;
      if (!std::strcmp(prefix, "LON:") &&
          (d < static_cast<LegacyReal>(-180.0F) || d > static_cast<LegacyReal>(180.0F))) return false;
      if (!std::strcmp(prefix, "CFGSTEERTEST:") &&
          (d < STEER_TEST_ANGLE_MIN_DEG || d > STEER_TEST_ANGLE_MAX_DEG)) return false;
      if (!std::strcmp(prefix, "CFGERPMMPS:") && (d < ERPM_PER_MPS_MIN || d > ERPM_PER_MPS_MAX)) return false;
      if (!std::strcmp(prefix, "CONF:") &&
          (d < static_cast<LegacyReal>(0.0F) || d > static_cast<LegacyReal>(100.0F))) return false;
      if ((!std::strcmp(prefix, "HDOP:") || !std::strcmp(prefix, "HACC:") ||
           !std::strcmp(prefix, "GAGE:") || !std::strcmp(prefix, "FPS:") ||
           !std::strcmp(prefix, "DIST:")) && d < static_cast<LegacyReal>(0.0F)) return false;
      return true;
    }
  }

  static const char *const intPrefixes[] = {"MANUAL_SPEED:", "FIX:", "SAT:", "WPSEL:"};
  for (const char *prefix : intPrefixes) {
    if ((p = after(prefix)) != nullptr) {
      if (!parseLongStrict(p, i)) return false;
      if (!std::strcmp(prefix, "MANUAL_SPEED:") && (i < MANUAL_SPEED_MIN || i > MANUAL_SPEED_MAX)) return false;
      if (!std::strcmp(prefix, "SAT:") && (i < 0 || i > 99)) return false;
      if (!std::strcmp(prefix, "WPSEL:") && (i < 0 || i >= HMI_WAYPOINT_COUNT)) return false;
      if (!std::strcmp(prefix, "FIX:") && (i < 0 || i > 6)) return false;
      return true;
    }
  }

  static const char *const boolPrefixes[] = {
      "ROS:", "ESC:", "ENC:", "VESC_LINK:", "VESC_DRIVE:", "VESC_STEER:", "ESTOP:", "CFGPERINF:",
      "GPS:", "IMU:", "MAG:", "CAM:", "PER:", "DRV:", "OBS:", "MOTION:", "NAV2:"};
  for (const char *prefix : boolPrefixes)
    if ((p = after(prefix)) != nullptr) return parseBoolStrict(p, b);

  if ((p = after("MODE:")) != nullptr)
    return eqIgnoreCase(p, "AUTO") || eqIgnoreCase(p, "MANUAL");
  if ((p = after("STATE:")) != nullptr)
    return eqIgnoreCase(p, "STANDBY") || eqIgnoreCase(p, "RUNNING") ||
           eqIgnoreCase(p, "STOPPED") || eqIgnoreCase(p, "STOP") || eqIgnoreCase(p, "FAULT");
  if ((p = after("NAV:")) != nullptr)
    return eqIgnoreCase(p, "IDLE") || eqIgnoreCase(p, "SELECTED") ||
           eqIgnoreCase(p, "QUEUED") || eqIgnoreCase(p, "SENDING") ||
           eqIgnoreCase(p, "NAVIGATING") || eqIgnoreCase(p, "ACTIVE") ||
           eqIgnoreCase(p, "ARRIVED") || eqIgnoreCase(p, "SUCCEEDED") ||
           eqIgnoreCase(p, "STOPPED") || eqIgnoreCase(p, "CANCELED") ||
           eqIgnoreCase(p, "FAILED") || eqIgnoreCase(p, "ABORTED") || eqIgnoreCase(p, "REJECTED");
  if (!std::strncmp(command, "WP", 2) && command[2] >= '0' && command[2] <= '3' && command[3] == ':') {
    const char *payload = command + 4;
    const char *colon = std::strchr(payload, ':');
    if (colon == nullptr || colon == payload) return false;
    char saved[16]{};
    const size_t n = static_cast<size_t>(colon - payload);
    if (n >= sizeof(saved)) return false;
    std::memcpy(saved, payload, n);
    saved[n] = '\0';
    return parseBoolStrict(saved, b);
  }
  return true; // unknown commands are rejected by the normal command dispatcher.
}

static void parseSystemStatus(const char *s) {
  if (eqIgnoreCase(s, "OFF"))
    gTelemetry.systemStatus = SYS_OFF;
  else if (eqIgnoreCase(s, "STARTING"))
    gTelemetry.systemStatus = SYS_STARTING;
  else if (eqIgnoreCase(s, "INITIALIZING"))
    gTelemetry.systemStatus = SYS_INITIALIZING;
  else if (eqIgnoreCase(s, "READY") || eqIgnoreCase(s, "VEHICLE READY"))
    gTelemetry.systemStatus = SYS_READY;
  else if (eqIgnoreCase(s, "NOT READY"))
    gTelemetry.systemStatus = SYS_NOT_READY;
  else if (eqIgnoreCase(s, "FAULT"))
    gTelemetry.systemStatus = SYS_FAULT;
}

static void parseVehicleState(const char *s) {
  if (eqIgnoreCase(s, "STANDBY"))
    gTelemetry.state = STATE_STANDBY;
  else if (eqIgnoreCase(s, "RUNNING"))
    gTelemetry.state = STATE_RUNNING;
  else if (eqIgnoreCase(s, "STOPPED") || eqIgnoreCase(s, "STOP"))
    gTelemetry.state = STATE_STOPPED;
  else if (eqIgnoreCase(s, "FAULT"))
    gTelemetry.state = STATE_FAULT;
}

static void sanitizeTelemetry() {
  if (!std::isfinite(gTelemetry.speedKmh))
    gTelemetry.speedKmh = 0.0F;
  gTelemetry.speedKmh = std::clamp(gTelemetry.speedKmh, 0.0F, 100.0F);
  if (!std::isfinite(gTelemetry.driveTargetMps))
    gTelemetry.driveTargetMps = 0.0F;
  if (!std::isfinite(gTelemetry.driveActualMps))
    gTelemetry.driveActualMps = 0.0F;
  if (!std::isfinite(gTelemetry.vbusV) || gTelemetry.vbusV <= 0.0F) {
    gTelemetry.vbusV = 0.0F;
    gTelemetry.vbusValid = false;
  }
  if (!std::isfinite(gTelemetry.headingDeg))
    gTelemetry.headingDeg = 0.0F;
  gTelemetry.headingDeg = fmodf(gTelemetry.headingDeg, 360.0F);
  if (gTelemetry.headingDeg < 0.0F)
    gTelemetry.headingDeg += 360.0F;
  if (!std::isfinite(gTelemetry.hdop))
    gTelemetry.hdop = 99.9F;
  if (!std::isfinite(gTelemetry.haccM))
    gTelemetry.haccM = 999.0F;
  if (!std::isfinite(gTelemetry.gnssAgeSec))
    gTelemetry.gnssAgeSec = 99.0F;
  if (!std::isfinite(gTelemetry.cameraFps))
    gTelemetry.cameraFps = 0.0F;
  if (!std::isfinite(gTelemetry.objectDistanceM) ||
      gTelemetry.objectDistanceM < 0.0F)
    gTelemetry.objectDistanceM = 0.0F;
  if (!std::isfinite(gTelemetry.goalRemainingDistanceM) ||
      gTelemetry.goalRemainingDistanceM < 0.0F) {
    gTelemetry.goalRemainingDistanceM = 0.0F;
    gTelemetry.goalRemainingDistanceValid = false;
  }
  if (!std::isfinite(gTelemetry.confidencePct))
    gTelemetry.confidencePct = 0.0F;
  gTelemetry.confidencePct = std::clamp(gTelemetry.confidencePct, 0.0F, 100.0F);
  if (!std::isfinite(gTelemetry.driveErpmPerMps))
    gTelemetry.driveErpmPerMps = 8000.0F;
  if (!std::isfinite(gTelemetry.steeringTestAngleDeg))
    gTelemetry.steeringTestAngleDeg = STEER_TEST_ANGLE_DEFAULT_DEG;
  gTelemetry.steeringTestAngleDeg =
      std::clamp(gTelemetry.steeringTestAngleDeg, STEER_TEST_ANGLE_MIN_DEG,
                 STEER_TEST_ANGLE_MAX_DEG);
}

static void configAck(bool ok, uint16_t txn, const char *key,
                      const char *valueOrReason) {
  if (!gTelemetry.configPending || txn != gTelemetry.configTxn)
    return;
  if (strcmp(key, gTelemetry.configKey) != 0)
    return;
  gTelemetry.configPending = false;
  gTelemetry.configLastOk = ok;
  snprintf(gTelemetry.configMessage, sizeof(gTelemetry.configMessage), "%.31s",
           valueOrReason);
  drawUiNow(true);
}

static void parseConfigResult(char *command, bool ok) {
  // ACK:CFG:<txn>:<key>:<value> / ERR:CFG:<txn>:<key>:<reason>
  char *p1 = std::strchr(command, ':');
  char *p2 = p1 != nullptr ? std::strchr(p1 + 1, ':') : nullptr;
  char *txn_end = p2 != nullptr ? std::strchr(p2 + 1, ':') : nullptr;
  if (p1 == nullptr || p2 == nullptr || txn_end == nullptr || txn_end == p2 + 1) {
    ++gDiagnostics.configAckMalformed;
    return;
  }
  const char saved = *txn_end;
  *txn_end = '\0';
  uint32_t txn32 = 0U;
  const bool txn_ok = parseU32Strict(p2 + 1, txn32) && txn32 <= 0xFFFFU;
  *txn_end = saved;
  if (!txn_ok) {
    ++gDiagnostics.configAckMalformed;
    return;
  }
  char *key = txn_end + 1;
  char *sep = std::strchr(key, ':');
  if (sep == nullptr || sep == key || sep[1] == '\0') {
    ++gDiagnostics.configAckMalformed;
    return;
  }
  *sep = '\0';
  configAck(ok, static_cast<uint16_t>(txn32), key, sep + 1);
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
  gTelemetry.escReady = false;
  gTelemetry.encoderReady = false;
  gTelemetry.vescConnected = false;
  gTelemetry.vbusValid = false;
  gTelemetry.gpsReady = false;
  gTelemetry.imuReady = false;
  gTelemetry.magReady = false;
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

static bool startsWithAny(const char *command, const char *const *prefixes,
                          size_t count) {
  for (size_t i = 0; i < count; ++i) {
    const size_t n = strlen(prefixes[i]);
    if (strncmp(command, prefixes[i], n) == 0)
      return true;
  }
  return false;
}

// Legacy key/value telemetry is display compatibility only. Domain freshness is
// authoritative exclusively from accepted, host-session-bound F4X3 frames.

static void markUiForCommand(const char *command) {
  (void)command;
  markUiDirty();
}

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

  // ESC transport is direct USB on the host; F411 owns no ESC UART.
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
#if defined(BOARD_F103C8)
  gDiagnostics.tftRst = pinHigh(GPIOB, GPIO_PIN_10);
#else
  gDiagnostics.tftRst = pinHigh(GPIOB, GPIO_PIN_2);
#endif
  gDiagnostics.pa4TouchCs = pinHigh(GPIOA, GPIO_PIN_4);
  gDiagnostics.pa11UsbDm = pinHigh(GPIOA, GPIO_PIN_11);
  gDiagnostics.pa12UsbDp = pinHigh(GPIOA, GPIO_PIN_12);
}

static void setExternalMenu(const char *name) {
  if (eqIgnoreCase(name, "OVERVIEW") || eqIgnoreCase(name, "HOME")) {
    setMenu(UiMenuId::HOME);
  } else if (eqIgnoreCase(name, "MAIN") || eqIgnoreCase(name, "MAIN_MENU") ||
             eqIgnoreCase(name, "MENU")) {
    setMenu(UiMenuId::MAIN_MENU);
  } else if (eqIgnoreCase(name, "ESC")) {
    setMenu(UiMenuId::ESC_ROOT);
  } else if (eqIgnoreCase(name, "PERCEPTION")) {
    setMenu(UiMenuId::PERCEPTION_ROOT);
  } else if (eqIgnoreCase(name, "NAVIGATION")) {
    setMenu(UiMenuId::NAVIGATION_ROOT);
  } else if (eqIgnoreCase(name, "SYSTEM") || eqIgnoreCase(name, "SERVICE")) {
    gUi.serviceSession = true;
    setMenu(UiMenuId::SYSTEM_ROOT);
    gUi.serviceSession = true;
  }
}

static void handleSerialCommand(char *command);

#if !defined(BOARD_F103C8) || defined(BOARD_F103_256K)
static bool motionSafeForHeavyMaintenance() {
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
         !driveTestRunning && !steeringTestRunning && !gTelemetry.eStop &&
         winchSafe;
}
#endif

#if !defined(BOARD_F103C8) || defined(BOARD_F103_256K)
static bool runTftSelfTest() {
  if (gUiDrawActive || !splashComplete || !tft.displayReady() ||
      tft.displayFaulted() || !motionSafeForHeavyMaintenance()) {
    gTelemetry.configLastOk = false;
    snprintf(gTelemetry.configMessage, sizeof(gTelemetry.configMessage), "%s",
             "TFT TEST LOCKED");
    markUiDirty();
    return false;
  }
  gOperatorUi.cancelTouch();
  const uint16_t colors[3] = {C_FAULT, C_READY, 0x001FU};
  for (uint8_t i = 0U; i < 3U; ++i) {
    if (!motionSafeForHeavyMaintenance()) {
      markUiDirty();
      drawUiNow(true);
      return false;
    }
    tft.fillScreen(colors[i]);
    Board_RealtimeDelayMs(200U);
  }
  markUiDirty();
  drawUiNow(true);
  return true;
}
#endif

static bool hostCommandRealtimeCritical(const char *command) {
  if (command == nullptr) return false;
  static const char *const prefixes[] = {
      "HOST:HELLO:", "F4X3:", "ROS:", "SYS:", "MODE:", "STATE:",
      "ESTOP:", "ESC:", "ENC:", "VESC_LINK:", "VESC_DRIVE:", "VESC_STEER:", "MOTION:", "NAV2:", "NAV:"};
  return startsWithAny(command, prefixes, sizeof(prefixes) / sizeof(prefixes[0]));
}

static bool commandAllowedBeforeHostSession(const char *command) {
  if (command == nullptr) return false;
  if (!std::strncmp(command, "HOST:HELLO:", 11)) return true;
  static const char *const exact[] = {
      "PING", "GET:STATE", "USB:STATUS", "USB:RECOVER", "FAULT:STATUS",
      "FW:INFO", "TFT:STATUS", "TOUCH:STATUS", "TFT:DIAG",
      "STATUS", "WINCH STATUS", "LIMITS", "WINCH LIMITS", "LS", "LS STATUS",
      "CONFIG", "GET CONFIG", "WINCH CONFIG",
      "RAWTOUCH", "TOUCHTEST", "WINCH TOUCHTEST", "CALIBRATE", "TOUCHCAL", "CALSTOP",
      "HMI RESET", "TFT RESET", "HMI REDRAW",
      "BOOT:DFU:ARM", "BOOT:DFU:CONFIRM", "BOOT:DFU"};
  for (const char *item : exact)
    if (std::strcmp(command, item) == 0) return true;
  return false;
}

static void handleSerialCommand(char *command) {
  while (*command == ' ' || *command == '\t')
    ++command;
  if (*command == '\0')
    return;
  const bool transportRealtime =
      !std::strncmp(command, "VESC:", 5) || hostCommandRealtimeCritical(command);
  if (gRealtimeParserActive && !transportRealtime) {
    (void)enqueueDeferredCommand(command);
    return;
  }
  if (gRealtimeParserActive && transportRealtime)
    ++gDiagnostics.realtimeCriticalCommands;
  ++gDiagnostics.hostCommands;
  gDiagnostics.lastHostCommandMs = HAL_GetTick();

  // HOST:HELLO is the only authority-establishing command. Same-token retries are
  // idempotent: ACK again without queue purge, ROS invalidation, or safety reset.
  if (!std::strncmp(command, "HOST:HELLO:", 11)) {
    uint32_t token = 0U;
    if (!parseU32Strict(command + 11, token) || token == 0U) {
      ++gDiagnostics.hostSessionMalformed;
      (void)gUsb.writeLineHighPriority("ERR:HOST:SESSION:ARGS");
      return;
    }
    char ack[96];
    std::snprintf(ack, sizeof(ack), "ACK:HOST:SESSION:%lu:%lu",
                  static_cast<unsigned long>(token),
                  static_cast<unsigned long>(gUsb.transportGeneration()));
    if (gUsb.hostSessionEstablished() && gUsb.hostSessionToken() == token) {
      ++gDiagnostics.hostSessionDuplicateHello;
      (void)gUsb.writeLineHighPriority(ack);
      return;
    }

    gUsb.beginHostSession(token);
    if (!gUsb.writeLineHighPriority(ack)) {
      // Do not silently establish an epoch the host cannot acknowledge. The host
      // repeats HELLO; local authority remains fail-closed in the meantime.
      gUsb.invalidateHostSession();
      forceRosOffline();
      return;
    }
    gUsb.confirmHostSession();
    // ACK precedes STOP lines, then every old ROS/domain authority is revoked.
    forceRosOffline();
    publishPage();
    publishLinkState();
    return;
  }

  // Before HELLO/ACK synchronization only read-only diagnostics and explicit
  // recovery paths are accepted. Telemetry, configuration ACKs, menu/control,
  // and peripheral mutation cannot acquire authority from stale CDC bytes.
  if (!gUsb.hostSessionEstablished() && !commandAllowedBeforeHostSession(command)) {
    ++gDiagnostics.preSessionRejected;
    (void)gUsb.writeLineHighPriority("ERR:HOST:SESSION:REQUIRED");
    return;
  }

#if BTS_WINCH_ENABLED
  // Local BTS7960 fork/winch contract, aligned with /forclift/f4.
  if (!std::strcmp(command, "WINCH STATUS") || !std::strcmp(command, "STATUS")) {
    char line[144];
    std::snprintf(line, sizeof(line),
                  "WINCH:STATE=%s:PWM=%u:APPLIED=%u:TOP=%u:BOTTOM=%u:LS_READY=%u:LIMIT_FAULT=%u:TIMEOUT=%u:GATE=%u",
                  Bts7960Winch::stateName(), static_cast<unsigned>(Bts7960Winch::configuredPwm()),
                  static_cast<unsigned>(Bts7960Winch::appliedPwm()),
                  Bts7960Winch::topLimitActive()?1U:0U, Bts7960Winch::bottomLimitActive()?1U:0U,
                  Bts7960Winch::limitsReady()?1U:0U,
                  Bts7960Winch::limitFault()?1U:0U, Bts7960Winch::movementTimedOut()?1U:0U,
                  Bts7960Winch::motionAllowed()?1U:0U);
    (void)gUsb.writeLineCritical(line, 120U);
    return;
  }
  if (!std::strcmp(command, "LIMITS") || !std::strcmp(command, "WINCH LIMITS") ||
      !std::strcmp(command, "LS") || !std::strcmp(command, "LS STATUS")) {
    char line[128];
    std::snprintf(line, sizeof(line), "LIMITS:READY=%u:TOP=%u:BOTTOM=%u:TOP_RAW_ACTIVE=%u:BOTTOM_RAW_ACTIVE=%u",
                  Bts7960Winch::limitsReady()?1U:0U,
                  Bts7960Winch::topLimitActive()?1U:0U, Bts7960Winch::bottomLimitActive()?1U:0U,
                  Bts7960Winch::topLimitRaw()?1U:0U, Bts7960Winch::bottomLimitRaw()?1U:0U);
    (void)gUsb.writeLineCritical(line,120U);
    return;
  }
  if (!std::strcmp(command, "CONFIG") || !std::strcmp(command, "GET CONFIG") ||
      !std::strcmp(command, "WINCH CONFIG")) {
    char line[80];
    std::snprintf(line, sizeof(line), "CONFIG:WINCH_PWM=%u",
                  static_cast<unsigned>(Bts7960Winch::configuredPwm()));
    (void)gUsb.writeLineCritical(line,120U);
    return;
  }
  if (!std::strcmp(command, "CONFIG RESET") || !std::strcmp(command, "WINCH CONFIG RESET")) {
    if (Bts7960Winch::resetPwm(true)) (void)gUsb.writeLineCritical("ACK:WINCH:CONFIG_RESET",120U);
    else (void)gUsb.writeLineCritical("ERR:WINCH:CONFIG_RESET",120U);
    markUiDirty();
    return;
  }
  if (!std::strcmp(command, "WINCH FAULT CLEAR")) {
    refreshWinchSafetyInputs();
    if (Bts7960Winch::clearFault()) (void)gUsb.writeLineCritical("ACK:WINCH:FAULT_CLEAR",120U);
    else (void)gUsb.writeLineCritical("ERR:WINCH:FAULT_CLEAR",120U);
    markUiDirty();
    return;
  }
  if (!std::strncmp(command, "WINCH PWM ", 10) || !std::strncmp(command, "PWM ", 4)) {
    uint32_t pwm=0U; const char *value = command + (!std::strncmp(command,"WINCH PWM ",10)?10:4);
    if (!parseU32Strict(value,pwm) || pwm>Bts7960Winch::PWM_MAX || !Bts7960Winch::setPwm(static_cast<uint16_t>(pwm),true))
      (void)gUsb.writeLineCritical("ERR:WINCH:PWM",120U);
    else (void)gUsb.writeLineCritical("ACK:WINCH:PWM",120U);
    markUiDirty();
    return;
  }
  refreshWinchSafetyInputs();
  if (Bts7960Winch::processCommand(command)) {
    const bool stopCommand = !std::strcmp(command, "STOP") ||
                             !std::strcmp(command, "WINCH STOP");
    const bool accepted = stopCommand || Bts7960Winch::direction() != 0 ||
                          Bts7960Winch::motionPending();
    (void)gUsb.writeLineCritical(
        accepted ? "ACK:WINCH:CMD" : "ERR:WINCH:SAFETY_GATE", 120U);
    markUiDirty();
    return;
  }
#endif

  // F411 never owns ESC transport. Reject stale/legacy gateway commands explicitly.
  if (!strncmp(command, "VESC:", 5)) {
    (void)gUsb.writeLineCritical("ERR:VESC:DIRECT_ESC_ONLY", 120U);
    return;
  }
  if (!strncmp(command, "ACK:CFG:", 8)) {
    parseConfigResult(command, true);
    return;
  }
  if (!strncmp(command, "ERR:CFG:", 8)) {
    parseConfigResult(command, false);
    return;
  }

  if (!strcmp(command, "EEPROM:STATUS")) {
#if defined(BOARD_F103C8)
    const char *state = !gPersistentConfig.storageOk()
        ? "ERR:EEPROM:FAULT"
        : (gPersistentConfig.storageFull() ? "EEPROM:STAT:FULL"
                                           : "EEPROM:STAT:OK");
    (void)gUsb.writeLineCritical(state, 120U);
#else
    char line[144];
    std::snprintf(line, sizeof(line),
                  "EEPROM:STAT:ok=%u:full=%u:used=%lu:total=%lu:invalid=%lu:writes=%lu",
                  gPersistentConfig.storageOk() ? 1U : 0U,
                  gPersistentConfig.storageFull() ? 1U : 0U,
                  static_cast<unsigned long>(gPersistentConfig.usedSlots()),
                  static_cast<unsigned long>(gPersistentConfig.totalSlots()),
                  static_cast<unsigned long>(gPersistentConfig.invalidRecords()),
                  static_cast<unsigned long>(gPersistentConfig.writes()));
    (void)gUsb.writeLineCritical(line, 120U);
#endif
    return;
  }
#if !defined(BOARD_F103C8) || defined(BOARD_F103_256K)
  if (!std::strncmp(command, "EEPROM:GET:", 11)) {
    uint32_t key = 0U, value = 0U;
    if (!parseU32Strict(command + 11, key) || key == 0U || key > 0xFFFFU ||
        !gPersistentConfig.readU32(static_cast<uint16_t>(key), value)) {
      (void)gUsb.writeLineCritical("ERR:EEPROM:GET", 120U);
      return;
    }
    char line[96];
    std::snprintf(line, sizeof(line), "ACK:EEPROM:GET:%lu:%lu",
                  static_cast<unsigned long>(key), static_cast<unsigned long>(value));
    (void)gUsb.writeLineCritical(line, 120U);
    return;
  }
  if (!std::strncmp(command, "EEPROM:SET:", 11)) {
    char *separator = std::strchr(command + 11, ':');
    if (separator == nullptr) {
      (void)gUsb.writeLineCritical("ERR:EEPROM:SET:ARGS", 120U);
      return;
    }
    *separator = '\0';
    uint32_t key = 0U, value = 0U;
    if (!parseU32Strict(command + 11, key) || !parseU32Strict(separator + 1, value) ||
        key == 0U || key > 0xFFFFU) {
      (void)gUsb.writeLineCritical("ERR:EEPROM:SET:ARGS", 120U);
      return;
    }
    if (!motionSafeForHeavyMaintenance()) {
      (void)gUsb.writeLineCritical("ERR:EEPROM:WAIT_SAFE", 120U);
      return;
    }
    if (!gPersistentConfig.writeU32(static_cast<uint16_t>(key), value)) {
      (void)gUsb.writeLineCritical("ERR:EEPROM:WRITE", 120U);
      return;
    }
    char line[96];
    std::snprintf(line, sizeof(line), "ACK:EEPROM:SET:%lu:%lu",
                  static_cast<unsigned long>(key), static_cast<unsigned long>(value));
    (void)gUsb.writeLineCritical(line, 120U);
    return;
  }
#endif
  if (!strcmp(command, "GET:STATE")) {
    publishPage();
    publishLinkState();
    return;
  }
  if (!strcmp(command, "USB:RECOVER")) {
    // Recovery intentionally destroys the current USB session. Revoke every
    // motion authority first so the physical D+ detach can never leave an
    // actuator running while host communication is unavailable.
    forceRosOffline();
    latchAllSafetyStops();
    serviceSafetyControlTx();
    gUsb.requestRecovery();
    return;
  }
#if defined(BOARD_F103C8)
  if (!strcmp(command, "USB:STATUS")) {
#if defined(BOARD_F103_256K)
    char line[224];
    std::snprintf(
        line, sizeof(line),
        "USB:STAT:host=%u,gen=%lu,init=%lu,deinit=%lu,restart=%lu,auto=%lu,reason=%lu,tx_busy=%u,rx=%lu,rx_age=%lu,drop=%lu,reset=%08lX",
        gUsb.hostSessionEstablished() ? 1U : 0U,
        static_cast<unsigned long>(gUsb.transportGeneration()),
        static_cast<unsigned long>(gUsb.classInitCount()),
        static_cast<unsigned long>(gUsb.classDeInitCount()),
        static_cast<unsigned long>(gUsb.softRestartCount()),
        static_cast<unsigned long>(gUsb.autoRestartCount()),
        static_cast<unsigned long>(gUsb.lastRecoveryReason()),
        gUsb.txBusy() ? 1U : 0U,
        static_cast<unsigned long>(gUsb.rxPacketCount()),
        static_cast<unsigned long>(gUsb.lastRxAgeMs()),
        static_cast<unsigned long>(gUsb.rxDropped()),
        static_cast<unsigned long>(gResetCauseFlags));
#else
    char line[160];
    std::snprintf(
        line, sizeof(line),
        "USB:STAT:host=%u,gen=%lu,restart=%lu,auto=%lu,reason=%lu,rx=%lu,drop=%lu,reset=%08lX",
        gUsb.hostSessionEstablished() ? 1U : 0U,
        static_cast<unsigned long>(gUsb.transportGeneration()),
        static_cast<unsigned long>(gUsb.softRestartCount()),
        static_cast<unsigned long>(gUsb.autoRestartCount()),
        static_cast<unsigned long>(gUsb.lastRecoveryReason()),
        static_cast<unsigned long>(gUsb.rxPacketCount()),
        static_cast<unsigned long>(gUsb.rxDropped()),
        static_cast<unsigned long>(gResetCauseFlags));
#endif
    (void)gUsb.writeLineCritical(line, 120U);
    return;
  }
#else
  if (!strcmp(command, "USB:STATUS")) {
    char line[620];
    std::snprintf(line, sizeof(line),
                  "USB:STAT:session=%lu,init=%lu,deinit=%lu,abort=%lu,"
                  "tx_complete=%lu,stall_recover=%lu,soft_restart=%lu,"
                  "tx_busy=%u,busy_age_ms=%lu,last_tx_age_ms=%lu,last_rx_age_ms=%lu,"
                  "rx_pkts=%lu,rx_resync=%lu,rx_resync_done=%lu,rx_q=%u,tx_q=%u,tx_hi_q=%u,st_tx=%lu,"
                  "rx_hwm=%u,tx_hwm=%u,tx_hi_hwm=%u,drop_lo=%lu,drop_hi=%lu,kick=%u,"
                  "progress_stall=%lu,recovery_reason=%lu,repair_age_ms=%lu,"
                  "repair_pending=%u,repair_ep_len=%lu,repair_flags=%u,"
                  "host_token=%lu,host_sessions=%lu,host_est=%u,low_purges=%lu,high_purges=%lu,purge_pending=%u,"
                  "safety_retry=%lu,safety_drop=%lu,safety_queued=%lu,reset_csr=%08lX",
                  static_cast<unsigned long>(gUsb.sessionGeneration()),
                  static_cast<unsigned long>(gUsb.classInitCount()),
                  static_cast<unsigned long>(gUsb.classDeInitCount()),
                  static_cast<unsigned long>(gUsb.txAbortCount()),
                  static_cast<unsigned long>(gUsb.txCompleteCount()),
                  static_cast<unsigned long>(gUsb.txStallRecoveryCount()),
                  static_cast<unsigned long>(gUsb.softRestartCount()),
                  gUsb.txBusy() ? 1U : 0U,
                  static_cast<unsigned long>(gUsb.txBusyAgeMs()),
                  static_cast<unsigned long>(gUsb.lastTxCompleteAgeMs()),
                  static_cast<unsigned long>(gUsb.lastRxAgeMs()),
                  static_cast<unsigned long>(gUsb.rxPacketCount()),
                  static_cast<unsigned long>(gUsb.rxResyncCount()),
                  static_cast<unsigned long>(gUsb.rxResyncCompleteCount()),
                  static_cast<unsigned>(gUsb.rxQueueDepth()),
                  static_cast<unsigned>(gUsb.txLowQueueDepth()),
                  static_cast<unsigned>(gUsb.txHighQueueDepth()),
                  static_cast<unsigned long>(gUsb.cdcTxState()),
                  static_cast<unsigned>(gUsb.rxHighWater()),
                  static_cast<unsigned>(gUsb.txLowHighWater()),
                  static_cast<unsigned>(gUsb.txHighHighWater()),
                  static_cast<unsigned long>(gUsb.txLowDropped()),
                  static_cast<unsigned long>(gUsb.txHighDropped()),
                  gUsb.txServicePending() ? 1U : 0U,
                  static_cast<unsigned long>(gUsb.txProgressStallCount()),
                  static_cast<unsigned long>(gUsb.lastRecoveryReason()),
                  static_cast<unsigned long>(gUsb.lastRepairAgeMs()),
                  static_cast<unsigned>(gUsb.lastRepairPending()),
                  static_cast<unsigned long>(gUsb.lastRepairEpLength()),
                  static_cast<unsigned>(gUsb.lastRepairFlags()),
                  static_cast<unsigned long>(gUsb.hostSessionToken()),
                  static_cast<unsigned long>(gUsb.hostSessionCount()),
                  gUsb.hostSessionEstablished() ? 1U : 0U,
                  static_cast<unsigned long>(gUsb.lowSessionPurgeCount()),
                  static_cast<unsigned long>(gUsb.highSessionPurgeCount()),
                  gUsb.lowSessionPurgePending() ? 1U : 0U,
                  static_cast<unsigned long>(gSafetyControlTxRetries),
                  static_cast<unsigned long>(gSafetyControlTxDrops),
                  static_cast<unsigned long>(gSafetyControlTxQueued),
                  static_cast<unsigned long>(gResetCauseFlags));
    (void)gUsb.writeLineCritical(line, 120U);
    return;
  }
#endif
  if (!strcmp(command, "FAULT:STATUS")) {
#if defined(BOARD_F103C8)
    char line[112];
    std::snprintf(line, sizeof(line),
                  "FAULT:STAT:reason=%04lX:reset=%08lX",
                  static_cast<unsigned long>(Board_BackupRead(3U)),
                  static_cast<unsigned long>(gResetCauseFlags));
    (void)gUsb.writeLineCritical(line, 120U);
    return;
#else
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    char line[260];
    std::snprintf(line, sizeof(line),
                  "FAULT:STAT:reason=%08lX:cfsr=%08lX:hfsr=%08lX:pc=%08lX:lr=%08lX:mmfar=%08lX:bfar=%08lX:reset=%08lX",
                  static_cast<unsigned long>(Board_BackupRead(3U)),
                  static_cast<unsigned long>(Board_BackupRead(4U)),
                  static_cast<unsigned long>(Board_BackupRead(5U)),
                  static_cast<unsigned long>(Board_BackupRead(6U)),
                  static_cast<unsigned long>(Board_BackupRead(7U)),
                  static_cast<unsigned long>(Board_BackupRead(8U)),
                  static_cast<unsigned long>(Board_BackupRead(9U)),
                  static_cast<unsigned long>(gResetCauseFlags));
    (void)gUsb.writeLineCritical(line, 120U);
    return;
#endif
  }
  if (!strcmp(command, "PING")) {
    (void)gUsb.writeLineCritical("ACK:PONG", 120U);
    publishPage();
    publishLinkState();
    return;
  }
  if (!strcmp(command, "FW:INFO")) {
#if defined(BOARD_F103_256K)
    (void)gUsb.writeLineCritical("FW:INFO:V3:F103RC:256K", 120U);
#elif defined(BOARD_F103C8)
    (void)gUsb.writeLineCritical("FW:INFO:V3:F103C8:64K", 120U);
#else
    char line[190];
    std::snprintf(
        line, sizeof(line),
        "FW:INFO:SHA=%s:DIRTY=%u:SCHEMA=%u:EPOCH=%lu:BASE=%08lX:LIMIT=%08lX",
        BuildInfo::kGitSha, BuildInfo::kGitDirty ? 1U : 0U,
        static_cast<unsigned>(BuildInfo::kSchemaVersion),
        static_cast<unsigned long>(BuildInfo::kBuildEpoch),
        static_cast<unsigned long>(BuildInfo::kAppBase),
        static_cast<unsigned long>(BuildInfo::kAppLimit));
    (void)gUsb.writeLineCritical(line, 120U);
#endif
    return;
  }
  if (HmiTouchService::handleCommand(command)) {
#if BTS_WINCH_ENABLED
    Bts7960Winch::emergencyStop();
#endif
    // RAW/CALIBRATION owns the complete touch surface, so make it a maintenance
    // interlock: if that service screen is active, latch the same global E-stop
    // before allowing calibration/raw-touch work to continue.
    if (HmiTouchService::active())
      triggerLocalEmergencyStop();
    gOperatorUi.cancelTouch();
    return;
  }
  if (!std::strcmp(command, "HMI RESET") || !std::strcmp(command, "TFT RESET") ||
      !std::strcmp(command, "HMI REDRAW")) {
#if BTS_WINCH_ENABLED
    Bts7960Winch::emergencyStop();
#endif
    stopAllManualTest();
    HmiTouchService::cancel();
    gOperatorUi.cancelTouch();
    (void)gUsb.writeLineCritical("ACK:HMI:RECOVERY", 120U);
    beginDisplayInitialization();
    return;
  }

#if !defined(BOARD_F103C8) || defined(BOARD_F103_256K)
  if (!strcmp(command, "TFT:STATUS")) {
    char line[96];
    const bool idOk = (gTftControllerId & 0xFFFFU) == 0x9341U;
    const char *state = !gDiagnostics.tftOk ? "FAULT" : (idOk ? "OK" : "WRITE_ONLY");
    std::snprintf(
        line, sizeof(line), "TFT:ID:%08lX:MODE=%02X:MADCTL=%02X:PIXFMT=%02X:%s",
        static_cast<unsigned long>(gTftControllerId), gTftPowerMode, gTftMadctl,
        gTftPixelFormat, state);
    (void)gUsb.writeLineCritical(line, 120U);
    return;
  }
  if (!strcmp(command, "TOUCH:STATUS")) {
    char line[190];
    sampleDiagnostics(HAL_GetTick());
    std::snprintf(line, sizeof(line),
                  "TOUCH:STATUS:READ=%lu:REJECT=%lu:Z=%u:RAW=%u,%u:XY=%u,%u:CS=%u:BUS=%lu:PAGE=%s",
                  static_cast<unsigned long>(gDiagnostics.touchReadCount),
                  static_cast<unsigned long>(gDiagnostics.touchRejectFastCount),
                  static_cast<unsigned>(tft.touchCurrentZ()),
                  static_cast<unsigned>(tft.touchLastRawX()),
                  static_cast<unsigned>(tft.touchLastRawY()),
                  static_cast<unsigned>(tft.touchLastX()),
                  static_cast<unsigned>(tft.touchLastY()),
                  gDiagnostics.pa4TouchCs ? 1U : 0U,
                  static_cast<unsigned long>(gDiagnostics.spiBusConflictCount),
                  gOperatorUi.wireName());
    (void)gUsb.writeLineCritical(line, 120U);
    return;
  }
  if (!strcmp(command, "TFT:DIAG")) {
    char line[190];
    sampleDiagnostics(HAL_GetTick());
    std::snprintf(line, sizeof(line),
                  "TFT:DIAG:TXN=%lu:BYTES=%lu:TO=%lu:HAL=%lu:REC=%lu:BUS=%lu:"
                  "CLK=%lu:FAST=%u:ULTRA=%u",
                  static_cast<unsigned long>(gDiagnostics.spiTransactions),
                  static_cast<unsigned long>(gDiagnostics.spiBytesTx),
                  static_cast<unsigned long>(gDiagnostics.spiTimeoutCount),
                  static_cast<unsigned long>(gDiagnostics.spiHalErrorCount),
                  static_cast<unsigned long>(gDiagnostics.spiRecoveryCount),
                  static_cast<unsigned long>(gDiagnostics.spiBusConflictCount),
                  static_cast<unsigned long>(gDiagnostics.tftWriteClockHz),
                  gDiagnostics.tftFastWriteValidated ? 1U : 0U,
                  gDiagnostics.tftUltraFastWriteValidated ? 1U : 0U);
    (void)gUsb.writeLineCritical(line, 120U);
    std::snprintf(
        line, sizeof(line),
        "TFT:FRAME:LAST=%lu:MAX=%lu:UI=%lu/%lu:SVC=%lu/%lu/"
        "%lu:LIFE=%lu:TOUCH=%lu/%lu",
        static_cast<unsigned long>(gDiagnostics.displayBytesLastFrame),
        static_cast<unsigned long>(gDiagnostics.displayBytesMaxFrame),
        static_cast<unsigned long>(gDiagnostics.uiDrawLastMs),
        static_cast<unsigned long>(gDiagnostics.uiDrawMaxMs),
        static_cast<unsigned long>(gDiagnostics.serviceGapP95Ms),
        static_cast<unsigned long>(gDiagnostics.serviceGapP99Ms),
        static_cast<unsigned long>(gDiagnostics.maxServiceGapMs),
        static_cast<unsigned long>(gDiagnostics.serviceGapLifetimeMaxMs),
        static_cast<unsigned long>(gDiagnostics.touchReadCount),
        static_cast<unsigned long>(gDiagnostics.touchRejectFastCount));
    (void)gUsb.writeLineCritical(line, 120U);
    std::snprintf(
        line, sizeof(line),
        "SYS:PERF:STACK=%lu:STACKMIN=%lu:QPEAK=%lu:QDROP=%lu",
        static_cast<unsigned long>(gDiagnostics.stackHeadroomBytes),
        static_cast<unsigned long>(gDiagnostics.minStackHeadroomBytes),
        static_cast<unsigned long>(gDiagnostics.deferredCommandPeak),
        static_cast<unsigned long>(gDiagnostics.deferredCommandDrops));
    (void)gUsb.writeLineCritical(line, 120U);
    return;
  }
#ifdef HMI_TEST_HOOKS
  if (!strcmp(command, "TEST:USB:TX_SILENT")) {
    if (!motionSafeForHeavyMaintenance()) {
      (void)gUsb.writeLineCritical("ERR:TEST:WAIT_SAFE", 120U);
      return;
    }
    // Fault injection only: RX remains active, TX is withheld for at most 8 s.
    // A USB session reset clears this immediately. No actuator command is sent.
    gUsb.testSuppressTx(8000U);
    return;
  }
  if (!strcmp(command, "TEST:SPI:REINIT_FAIL")) {
    if (!motionSafeForHeavyMaintenance()) {
      (void)gUsb.writeLineCritical("ERR:TEST:WAIT_SAFE", 120U);
      return;
    }
    Board_TestInjectSpiInitFailureOnce();
    const bool unexpectedSuccess = tft.testForceRuntimeSpiReinit();
    (void)gUsb.writeLineCritical(unexpectedSuccess
                                     ? "ERR:TEST:REINIT_NOT_INJECTED"
                                     : "ACK:TEST:REINIT_FAIL_INJECTED",
                                 120U);
    return;
  }
  if (!strcmp(command, "TEST:SPI:TX_FAIL")) {
    if (!motionSafeForHeavyMaintenance()) {
      (void)gUsb.writeLineCritical("ERR:TEST:WAIT_SAFE", 120U);
      return;
    }
    tft.testInjectTxFailureOnce();
    (void)gUsb.writeLineCritical("ACK:TEST:TX_FAIL_ARMED", 120U);
    markUiDirty();
    drawUiNow(true);
    return;
  }
#endif
  if (!strcmp(command, "TFT:VERIFY")) {
    if (gUiDrawActive || !splashComplete || !tft.displayReady() ||
        tft.displayFaulted()) {
      (void)gUsb.writeLineCritical("ERR:TFT:BUSY_OR_FAULT", 120U);
      return;
    }
    if (!motionSafeForHeavyMaintenance()) {
      (void)gUsb.writeLineCritical("ERR:TFT:WAIT_SAFE", 120U);
      return;
    }
    gOperatorUi.cancelTouch();
    const bool verified = tft.verifyCurrentWriteClock();
    sampleDiagnostics(HAL_GetTick());
    (void)gUsb.writeLineCritical(verified ? "ACK:TFT:VERIFY" : "ERR:TFT:VERIFY",
                                 120U);
    markUiDirty();
    drawUiNow(true);
    return;
  }
  if (!strcmp(command, "TFT:TEST")) {
    if (gUiDrawActive || !splashComplete || !tft.displayReady() ||
        tft.displayFaulted()) {
      (void)gUsb.writeLineCritical("ERR:TFT:BUSY_OR_FAULT", 120U);
      return;
    }
    if (!motionSafeForHeavyMaintenance()) {
      (void)gUsb.writeLineCritical("ERR:TFT:MOTION_OR_NAV_ACTIVE", 120U);
      return;
    }
    const bool ok = runTftSelfTest();
    (void)gUsb.writeLineCritical(ok ? "ACK:TFT:TEST" : "ERR:TFT:ABORTED", 120U);
    return;
  }
#endif
#if defined(BOARD_F103C8) && !defined(BOARD_F103_256K)
  if (!std::strncmp(command, "BOOT:DFU", 8)) {
    (void)gUsb.writeLineCritical("ERR:DFU:UNSUPPORTED:F103C8", 120U);
    return;
  }
#else
  if (!strcmp(command, "BOOT:DFU:ARM")) {
    gDfuArmDeadlineMs = HAL_GetTick() + 5000U;
    printBoth("ACK:DFU:ARMED");
    return;
  }
  if (!strcmp(command, "BOOT:DFU:CONFIRM")) {
    if (gUiDrawActive) {
      (void)gUsb.writeLineCritical("ERR:DFU:BUSY", 120U);
      return;
    }
    const uint32_t now = HAL_GetTick();
    if (gDfuArmDeadlineMs == 0U ||
        static_cast<int32_t>(gDfuArmDeadlineMs - now) <= 0) {
      gDfuArmDeadlineMs = 0U;
      printBoth("ERR:DFU:NOT_ARMED");
      return;
    }
    stopAllManualTest();
#if BTS_WINCH_ENABLED
    // Firmware update is destructive maintenance: cut BTS7960 power first and
    // reject the transition unless the actuator is observably de-energized.
    Bts7960Winch::emergencyStop();
    refreshWinchSafetyInputs();
#endif
    if (!motionSafeForHeavyMaintenance()) {
      (void)gUsb.writeLineCritical("ERR:DFU:WAIT_SAFE");
      return;
    }
    if (!gUsb.writeLineCritical("ACK:DFU"))
      return;
    gDfuArmDeadlineMs = 0U;
    gUsb.flush(150U);
    HAL_Delay(20U);
    enterSystemDfu();
    return;
  }
  if (!strcmp(command, "BOOT:DFU")) {
    printBoth("ERR:DFU:TWO_STEP_REQUIRED");
    return;
  }
#endif
  if (!strncmp(command, "GOTO:", 5)) {
    setExternalMenu(command + 5);
    return;
  }

  const uint32_t telemetryNow = HAL_GetTick();
  const ExtendedTelemetryParseResult extended =
      parseExtendedTelemetryLine(command, gTelemetry, telemetryNow,
                                 gUsb.hostSessionToken());
  if (extended.recognized) {
    if (!extended.accepted) {
      if (extended.outOfOrder) ++gDiagnostics.extendedTelemetryOutOfOrder;
      else ++gDiagnostics.extendedTelemetryMalformed;
      if (extended.crcError) ++gDiagnostics.extendedTelemetryCrcErrors;
      if (extended.lengthError) ++gDiagnostics.extendedTelemetryLengthErrors;
      if (extended.versionError) ++gDiagnostics.extendedTelemetryVersionErrors;
      if (extended.sessionError) ++gDiagnostics.extendedTelemetrySessionErrors;
      return;
    }
    ++gDiagnostics.extendedTelemetryAccepted;
    if (extended.v3) ++gDiagnostics.extendedTelemetryV3Accepted;
    else ++gDiagnostics.extendedTelemetryLegacyAccepted;
    if (extended.v3) {
      if (extended.domain == ExtendedTelemetryDomain::ESC) {
        lastEscDomainMs = telemetryNow; seenEscDomain = true;
      } else if (extended.domain == ExtendedTelemetryDomain::PERCEPTION) {
        lastPerceptionDomainMs = telemetryNow; seenPerceptionDomain = true;
      } else if (extended.domain == ExtendedTelemetryDomain::NAVIGATION) {
        lastNavigationDomainMs = telemetryNow; seenNavigationDomain = true;
      }
    }
    updateExtendedFreshness(gTelemetry, telemetryNow);
    markUiDirty();
    enforceManualMotionGate();
    return;
  }

  if (!legacyTelemetryPayloadValid(command)) {
    ++gDiagnostics.legacyTelemetryMalformed;
    return;
  }

  bool recognized = true;
  if (!strncmp(command, "ROS:", 4)) {
    bool value = false;
    (void)parseBoolStrict(command + 4, value);
    if (value) markRosHeartbeat(); else forceRosOffline();
  } else if (!strncmp(command, "SYS:", 4)) {
    parseSystemStatus(command + 4);
  } else if (!strncmp(command, "MODE:", 5)) {
    gTelemetry.mode =
        eqIgnoreCase(command + 5, "MANUAL") ? MODE_MANUAL : MODE_AUTO;
  } else if (!strncmp(command, "STATE:", 6)) {
    parseVehicleState(command + 6);
  } else if (!strncmp(command, "SPD:", 4)) {
    gTelemetry.speedKmh = static_cast<float>(parseDoubleOrZero(command + 4));
  } else if (!strncmp(command, "DRIVE_TGT:", 10)) {
    gTelemetry.driveTargetMps = static_cast<float>(parseDoubleOrZero(command + 10));
  } else if (!strncmp(command, "DRIVE_ACT:", 10)) {
    gTelemetry.driveActualMps = static_cast<float>(parseDoubleOrZero(command + 10));
  } else if (!strncmp(command, "ERPM:", 5)) {
    gTelemetry.motorErpm = static_cast<float>(parseDoubleOrZero(command + 5));
  } else if (!strncmp(command, "VBUS:", 5)) {
    gTelemetry.vbusV = static_cast<float>(parseDoubleOrZero(command + 5));
    gTelemetry.vbusValid = std::isfinite(gTelemetry.vbusV) && gTelemetry.vbusV > 0.0F;
  } else if (!strncmp(command, "STEER_TARGET:", 13)) {
    gTelemetry.steeringTargetDeg = static_cast<float>(parseDoubleOrZero(command + 13));
    gTelemetry.steeringErrorDeg =
        gTelemetry.steeringTargetDeg - gTelemetry.steeringActualDeg;
  } else if (!strncmp(command, "STEER_ACTUAL:", 13)) {
    gTelemetry.steeringActualDeg = static_cast<float>(parseDoubleOrZero(command + 13));
    gTelemetry.steeringErrorDeg =
        gTelemetry.steeringTargetDeg - gTelemetry.steeringActualDeg;
  } else if (!strncmp(command, "STEER_ERR:", 10)) {
    gTelemetry.steeringErrorDeg = static_cast<float>(parseDoubleOrZero(command + 10));
  } else if (!strncmp(command, "STEERTEST:", 10)) {
    snprintf(gTelemetry.steeringTestState, sizeof(gTelemetry.steeringTestState),
             "%.11s", command + 10);
  } else if (!strncmp(command, "ESC:", 4)) {
    bool value = false; (void)parseBoolStrict(command + 4, value); gTelemetry.escReady = value;
  } else if (!strncmp(command, "ENC:", 4)) {
    bool value = false; (void)parseBoolStrict(command + 4, value); gTelemetry.encoderReady = value;
  } else if (!strncmp(command, "VESC_LINK:", 10)) {
    bool value = false; (void)parseBoolStrict(command + 10, value); gTelemetry.vescConnected = value;
  } else if (!strncmp(command, "VESC_DRIVE:", 11)) {
    bool value = false; (void)parseBoolStrict(command + 11, value); gTelemetry.vescDriveConnected = value;
  } else if (!strncmp(command, "VESC_STEER:", 11)) {
    bool value = false; (void)parseBoolStrict(command + 11, value); gTelemetry.vescSteerConnected = value;
  } else if (!strcmp(command, "ESTOP:RESET")) {
    if (resetLocalEmergencyStop())
      (void)gUsb.writeLineHighPriority("ACK:ESTOP:RESET");
    else
      (void)gUsb.writeLineHighPriority("ERR:ESTOP:RESET:UNSAFE");
  } else if (!strncmp(command, "ESTOP:", 6)) {
    bool value = false;
    if (!parseBoolStrict(command + 6, value)) {
      (void)gUsb.writeLineHighPriority("ERR:ESTOP:ARGS");
      return;
    }
    setHostEmergencyStop(value);
  } else if (!strncmp(command, "MANUAL_SPEED:", 13)) {
    gTelemetry.manualSpeedPct = static_cast<uint8_t>(
        std::clamp(parseIntOrZero(command + 13), MANUAL_SPEED_MIN, MANUAL_SPEED_MAX));
  } else if (!strncmp(command, "CFGSTEERTEST:", 13)) {
    gTelemetry.steeringTestAngleDeg = static_cast<float>(parseDoubleOrZero(command + 13));
  } else if (!strncmp(command, "CFGERPMMPS:", 13)) {
    gTelemetry.driveErpmPerMps = static_cast<float>(parseDoubleOrZero(command + 13));
  } else if (!strncmp(command, "CFGPERINF:", 10)) {
    gTelemetry.perceptionInference = parseBool(command + 10);
  } else if (!strncmp(command, "GPS:", 4)) {
    gTelemetry.gpsReady = parseBool(command + 4);
  } else if (!strncmp(command, "FIX:", 4)) {
    const int fix = parseIntOrZero(command + 4);
    if (fix <= 1)
      gTelemetry.gpsFix = GPS_NO_FIX;
    else if (fix == 2)
      gTelemetry.gpsFix = GPS_2D_FIX;
    else if (fix == 3)
      gTelemetry.gpsFix = GPS_3D_FIX;
    else
      gTelemetry.gpsFix = GPS_DEGRADED;
  } else if (!strncmp(command, "LAT:", 4)) {
    gTelemetry.latitude = parseDoubleOrZero(command + 4);
  } else if (!strncmp(command, "LON:", 4)) {
    gTelemetry.longitude = parseDoubleOrZero(command + 4);
  } else if (!strncmp(command, "SAT:", 4)) {
    gTelemetry.satellites =
        static_cast<uint8_t>(std::clamp(parseIntOrZero(command + 4), 0, 99));
  } else if (!strncmp(command, "HDOP:", 5)) {
    gTelemetry.hdop = static_cast<float>(parseDoubleOrZero(command + 5));
  } else if (!strncmp(command, "HACC:", 5)) {
    gTelemetry.haccM = static_cast<float>(parseDoubleOrZero(command + 5));
  } else if (!strncmp(command, "GAGE:", 5)) {
    gTelemetry.gnssAgeSec = static_cast<float>(parseDoubleOrZero(command + 5));
  } else if (!strncmp(command, "HEAD:", 5)) {
    gTelemetry.headingDeg = static_cast<float>(parseDoubleOrZero(command + 5));
  } else if (!strncmp(command, "IMU:", 4)) {
    gTelemetry.imuReady = parseBool(command + 4);
  } else if (!strncmp(command, "GYROZ:", 6)) {
    gTelemetry.gyroZRps = static_cast<float>(parseDoubleOrZero(command + 6));
  } else if (!strncmp(command, "MAG:", 4)) {
    gTelemetry.magReady = parseBool(command + 4);
  } else if (!strncmp(command, "CAM:", 4)) {
    gTelemetry.cameraReady = parseBool(command + 4);
  } else if (!strncmp(command, "PER:", 4)) {
    gTelemetry.perceptionReady = parseBool(command + 4);
  } else if (!strncmp(command, "FPS:", 4)) {
    gTelemetry.cameraFps = static_cast<float>(parseDoubleOrZero(command + 4));
  } else if (!strncmp(command, "OBJ:", 4)) {
    snprintf(gTelemetry.detectedObject, sizeof(gTelemetry.detectedObject),
             "%.23s", command + 4);
  } else if (!strncmp(command, "DIST:", 5)) {
    gTelemetry.objectDistanceM = static_cast<float>(parseDoubleOrZero(command + 5));
  } else if (!strncmp(command, "CONF:", 5)) {
    gTelemetry.confidencePct = static_cast<float>(parseDoubleOrZero(command + 5));
  } else if (!strncmp(command, "DRV:", 4)) {
    gTelemetry.drivableAreaClear = parseBool(command + 4);
  } else if (!strncmp(command, "OBS:", 4)) {
    gTelemetry.obstacleDetected = parseBool(command + 4);
  } else if (!strncmp(command, "LANE:", 5)) {
    snprintf(gTelemetry.laneState, sizeof(gTelemetry.laneState), "%.19s",
             command + 5);
  } else if (!strncmp(command, "MOTION:", 7)) {
    gTelemetry.motionReady = parseBool(command + 7);
  } else if (!strncmp(command, "NAV2:", 5)) {
    gTelemetry.nav2Ready = parseBool(command + 5);
  } else if (!strncmp(command, "GOAL_DIST:", 10)) {
    const float value = static_cast<float>(parseDoubleOrZero(command + 10));
    gTelemetry.goalRemainingDistanceValid =
        std::isfinite(value) && value >= 0.0F;
    gTelemetry.goalRemainingDistanceM =
        gTelemetry.goalRemainingDistanceValid ? value : 0.0F;
  } else if (!strncmp(command, "LOCSTATE:", 9)) {
    snprintf(gTelemetry.localizationState, sizeof(gTelemetry.localizationState),
             "%.23s", command + 9);
  } else if (!strncmp(command, "GNSSSTATUS:", 11)) {
    snprintf(gTelemetry.gnssStatus, sizeof(gTelemetry.gnssStatus), "%.19s",
             command + 11);
  } else if (!strncmp(command, "IMUSTATUS:", 10)) {
    snprintf(gTelemetry.imuStatus, sizeof(gTelemetry.imuStatus), "%.19s",
             command + 10);
  } else if (!strncmp(command, "EKFLOCAL:", 9)) {
    snprintf(gTelemetry.ekfLocalStatus, sizeof(gTelemetry.ekfLocalStatus),
             "%.19s", command + 9);
  } else if (!strncmp(command, "EKFGLOBAL:", 10)) {
    snprintf(gTelemetry.ekfGlobalStatus, sizeof(gTelemetry.ekfGlobalStatus),
             "%.19s", command + 10);
  } else if (!strncmp(command, "WPSEL:", 6)) {
    gTelemetry.selectedWaypoint = static_cast<uint8_t>(
        std::clamp(parseIntOrZero(command + 6), 0, HMI_WAYPOINT_COUNT - 1));
  } else if (!strncmp(command, "TARGET:", 7)) {
    snprintf(gTelemetry.activeTarget, sizeof(gTelemetry.activeTarget), "%.19s",
             command + 7);
  } else if (!strncmp(command, "MISSION:", 8)) {
    snprintf(gTelemetry.missionState, sizeof(gTelemetry.missionState), "%.23s",
             command + 8);
  } else if (!strncmp(command, "NAV:", 4)) {
    const char *state = command + 4;
    if (eqIgnoreCase(state, "SELECTED"))
      gTelemetry.navigationStatus = NAV_SELECTED;
    else if (eqIgnoreCase(state, "QUEUED") || eqIgnoreCase(state, "SENDING"))
      gTelemetry.navigationStatus = NAV_QUEUED;
    else if (eqIgnoreCase(state, "NAVIGATING") || eqIgnoreCase(state, "ACTIVE"))
      gTelemetry.navigationStatus = NAV_NAVIGATING;
    else if (eqIgnoreCase(state, "ARRIVED") || eqIgnoreCase(state, "SUCCEEDED"))
      gTelemetry.navigationStatus = NAV_ARRIVED;
    else if (eqIgnoreCase(state, "STOPPED") || eqIgnoreCase(state, "CANCELED"))
      gTelemetry.navigationStatus = NAV_STOPPED;
    else if (eqIgnoreCase(state, "FAILED") || eqIgnoreCase(state, "ABORTED") ||
             eqIgnoreCase(state, "REJECTED"))
      gTelemetry.navigationStatus = NAV_FAILED;
    else
      gTelemetry.navigationStatus = NAV_IDLE;
  } else if (!strncmp(command, "WP", 2) && command[2] >= '0' &&
             command[2] <= '3' && command[3] == ':') {
    const uint8_t index = static_cast<uint8_t>(command[2] - '0');
    char *payload = command + 4;
    char *colon = strchr(payload, ':');
    if (colon != nullptr) {
      *colon = '\0';
      gTelemetry.waypointSaved[index] = parseBool(payload);
      snprintf(gTelemetry.waypointName[index], HMI_WAYPOINT_NAME_LEN, "%.19s",
               colon + 1);
    }
  } else {
    recognized = false;
  }

  if (!recognized) {
    ++gDiagnostics.unknownCommands;
    char line[224];
    std::snprintf(line, sizeof(line), "ERR:UNKNOWN_COMMAND:%s", command);
    (void)gUsb.writeLine(line);
    return;
  }
  sanitizeTelemetry();
  updateDomainFreshness(HAL_GetTick());
  enforceManualMotionGate();
  markUiForCommand(command);
}

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
    if (splashComplete)
      drawUiNow(true);
    else
      drawSplashScreen();
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
   * asynchronously so safety/VESC service continues each main-loop iteration.
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

static void restartSplash() {
  stopDriveTest();
  gUi = UiState{};
  gUi.menu = UiMenuId::SPLASH;
  splashComplete = false;
  splashProgress = 0U;
  splashReadyText = false;
  lastFrameMs = 0U;
  gOperatorUi.cancelTouch();
    gUiDirty = false;
  lastUiRefreshMs = 0U;
  gTelemetry.systemStatus = SYS_INITIALIZING;
  if (tft.displayReady() && !tft.displayFaulted())
    drawSplashScreen();
  splashStartMs = HAL_GetTick();
  lastUiInteractionMs = splashStartMs;
  publishPage();
}

static bool updateProgressBar() {
  if (splashComplete)
    return false;
  const uint32_t now = HAL_GetTick();
  if (now - lastFrameMs < FRAME_MS)
    return true;
  lastFrameMs = now;
  const uint32_t elapsed = now - splashStartMs;
  const bool canDraw = tft.displayReady() && !tft.displayFaulted();
  if (elapsed < PROGRESS_MS) {
    const uint8_t target =
        static_cast<uint8_t>((elapsed * 100UL) / PROGRESS_MS);
    if (target != splashProgress) {
      splashProgress = target;
      if (canDraw) {
        const int innerW = PB_W - 4;
        const int fillW = static_cast<int>((splashProgress * innerW) / 100);
        tft.fillRoundRect(PB_X + 2, PB_Y + 2, innerW, PB_H - 4, PB_R - 2, C_BG);
        if (fillW > 0)
          tft.fillRoundRect(PB_X + 2, PB_Y + 2, fillW, PB_H - 4, PB_R - 2,
                            C_ACCENT);
      }
    }
    return true;
  }
  if (!splashReadyText) {
    splashReadyText = true;
    splashReadyMs = now;
    if (canDraw) {
      tft.fillRoundRect(PB_X + 2, PB_Y + 2, PB_W - 4, PB_H - 4, PB_R - 2,
                        C_READY);
      tft.fillRect(70, 186, 180, 18, C_BG);
      drawUiText("HMI ready", W / 2, 187, C_READY, C_BG, MC_DATUM);
    }
  }
  if (now - splashReadyMs >= READY_HOLD) {
    splashComplete = true;
    Board_ResetServiceGapStats();
    gOperatorUi.cancelTouch();
    if (gTelemetry.systemStatus == SYS_INITIALIZING)
      gTelemetry.systemStatus = SYS_NOT_READY;
    gUi.menu = UiMenuId::HOME;
    gUi.pageIndex = 0U;
    gUi.detailViewIndex = 0U;
    gUi.serviceSession = false;
    gOperatorUi.reset();
    drawUiNow(true);
    publishPage();
    return false;
  }
  return true;
}

int main() {
  // Capture reset flags before board/HAL initialization mutates clock/reset state.
  gResetCauseFlags = RCC->CSR;
  __HAL_RCC_CLEAR_RESET_FLAGS();
  Board_Init();
  // TIM11 supervises every subsystem after the base clock/peripheral bring-up:
  // persistent config, winch/touch init, USB, SPI/display init, then main loop.
  startAppWatchdog();
  noteWatchdogProgress();
  gPersistentConfigReady = gPersistentConfig.begin();
  noteWatchdogProgress();
#if BTS_WINCH_ENABLED
  Bts7960Winch::begin(gPersistentConfigReady ? &gPersistentConfig : nullptr);
  HmiTouchService::begin(emitServiceLine);
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
  restartSplash();
  if (!usbInitOk)
    gTelemetry.systemStatus = SYS_FAULT;

  while (true) {
    /* Service deferred UART recovery/queue work every loop. Motor-link TX also
     * chains in its ISR, so this is a fallback rather than the realtime clock.
     */
    Board_Service();
  #if BTS_WINCH_ENABLED
    refreshWinchSafetyInputs();
    Bts7960Winch::update();
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

    if (!splashComplete) {
      (void)updateProgressBar();
    } else {
      const uint32_t now = HAL_GetTick();
      if (static_cast<uint32_t>(now - lastTouchPollMs) >= TOUCH_POLL_MS) {
        lastTouchPollMs = now;
        handleTouch();
      }
      if (gUiDirty && !gOperatorUi.touchDown() &&
          static_cast<uint32_t>(now - lastUiRefreshMs) >= DISPLAY_REFRESH_MS) {
        drawUiNow(false);
      }
    }

    static uint32_t ledMs = 0U;
    if (static_cast<uint32_t>(HAL_GetTick() - ledMs) >= 500U) {
      ledMs = HAL_GetTick();
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
    }
      noteWatchdogProgress();
  }
}
