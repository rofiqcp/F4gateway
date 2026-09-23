#if !defined(F103_BUILD_BOOTLOADER)
#include "Bts7960Winch.h"
#include "BoardSupport.h"
#include "PersistentConfigStore.h"
#include <cstring>

namespace {
constexpr uint16_t kPersistKey = 0x0201U;
constexpr uint32_t kReverseDeadtimeMs = 50U;
constexpr uint32_t kHomeWatchdogMs = 30000U;
constexpr uint32_t kLimitConfirmMs = 30U;
constexpr uint32_t kLimitHardStopMs = 2U;
constexpr uint32_t kLocalUpKickMs = 150U;

PersistentConfigStore *gStore = nullptr;
bool gInitialized = false;
Bts7960WinchState gState = Bts7960WinchState::STOPPED;
uint16_t gConfiguredPwm = Bts7960Winch::PWM_DEFAULT;
uint16_t gAppliedPwm = 0U;
int8_t gDirection = 0;
uint32_t gMotionStartedMs = 0U;
uint32_t gTimedDurationMs = 0U;

bool gPendingMotion = false;
int8_t gPendingDirection = 0;
Bts7960WinchState gPendingState = Bts7960WinchState::STOPPED;
uint32_t gPendingDurationMs = 0U;
uint32_t gPendingSinceMs = 0U;

bool gTopTracking = false;
bool gBottomTracking = false;
uint32_t gTopSinceMs = 0U;
uint32_t gBottomSinceMs = 0U;
bool gTopConfirmed = false;
bool gBottomConfirmed = false;
bool gLimitFaultLatched = false;
bool gMovementTimeoutLatched = false;
bool gMotionRequiresHost = true;
bool gPendingRequiresHost = true;
Bts7960Winch::SafetyInputs gSafety{};

bool baseSafetyValid(bool requireHost = true) {
  // Physical/local safety is authoritative for the on-panel operator controls.
  // A ROS/global health fault must not disable a stationary local fork service
  // action; remote commands still require the full system/host health gate.
  const bool physical = !gSafety.emergencyStop &&
                        gSafety.physicalSafetyValid &&
                        gSafety.vehicleSafe;
  if (!physical) return false;
  if (!requireHost) return true;
  return !gSafety.systemFault &&
         gSafety.hostSessionValid &&
         gSafety.commandSessionValid;
}

void drivePwm(uint16_t rpwm, uint16_t lpwm) {
#if BTS_WINCH_ENABLED
  Board_BtsSetPwm(rpwm, lpwm);
#else
  (void)rpwm;
  (void)lpwm;
#endif
}

bool readTopRaw() {
  return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_6) == GPIO_PIN_RESET;
}

bool readBottomRaw() {
  return HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7) == GPIO_PIN_RESET;
}

void updateConfirmWindow(bool rawActive, bool &tracking, uint32_t &since,
                         bool &confirmed, uint32_t now) {
  if (rawActive) {
    if (!tracking) {
      tracking = true;
      since = now;
    }
    confirmed = static_cast<uint32_t>(now - since) >= kLimitConfirmMs;
  } else {
    tracking = false;
    since = 0U;
    confirmed = false;
  }
}

bool fastLimitActive(bool rawActive, bool tracking, uint32_t since, uint32_t now) {
  return rawActive && tracking &&
         static_cast<uint32_t>(now - since) >= kLimitHardStopMs;
}

bool topHardStopActive(uint32_t now) {
  return fastLimitActive(readTopRaw(), gTopTracking, gTopSinceMs, now);
}

bool bottomHardStopActive(uint32_t now) {
  return fastLimitActive(readBottomRaw(), gBottomTracking, gBottomSinceMs, now);
}

void rawStop() {
  drivePwm(0U, 0U);
  gAppliedPwm = 0U;
  gDirection = 0;
}
void cancelPendingMotion() {
  gPendingMotion = false;
  gPendingDirection = 0;
  gPendingState = Bts7960WinchState::STOPPED;
  gPendingDurationMs = 0U;
  gPendingSinceMs = 0U;
  gPendingRequiresHost = true;
}

void setState(Bts7960WinchState state) { gState = state; }

void hardStop(Bts7960WinchState state) {
  cancelPendingMotion();
  rawStop();
  gTimedDurationMs = 0U;
  gMotionRequiresHost = true;
  setState(state);
}

bool applyDirection(int8_t direction, uint16_t pwm) {
  if (direction > 0) {
    if (topHardStopActive(HAL_GetTick())) return false;
    drivePwm(pwm, 0U);
    gAppliedPwm = pwm;
    gDirection = 1;
    return true;
  }
  if (direction < 0) {
    if (bottomHardStopActive(HAL_GetTick())) return false;
    drivePwm(0U, pwm);
    gAppliedPwm = pwm;
    gDirection = -1;
    return true;
  }
  rawStop();
  return true;
}
bool startMotionNow(int8_t direction, Bts7960WinchState state, uint32_t durationMs,
                    bool requireHost) {
  if (!baseSafetyValid(requireHost)) {
    hardStop(Bts7960WinchState::STOPPED);
    return false;
  }
  if (gConfiguredPwm == 0U) {
    hardStop(Bts7960WinchState::STOPPED);
    return false;
  }
  if (direction > 0 && topHardStopActive(HAL_GetTick())) {
    hardStop(Bts7960WinchState::TOP_LIMIT);
    return false;
  }
  if (direction < 0 && bottomHardStopActive(HAL_GetTick())) {
    hardStop(Bts7960WinchState::BOTTOM_LIMIT);
    return false;
  }
  // Native-hardware PWM start policy. DOWN follows the v2 behavior and starts
  // immediately at the configured PWM. Local UP keeps its short full-duty kick
  // because that is already proven useful for loaded-fork breakaway.
  uint16_t startPwm = gConfiguredPwm;
  if (!requireHost && direction > 0)
    startPwm = Bts7960Winch::PWM_MAX;
  if (!applyDirection(direction, startPwm)) return false;

  gMotionStartedMs = HAL_GetTick();
  gTimedDurationMs = durationMs;
  gMotionRequiresHost = requireHost;
  setState(state);
  return true;
}

void requestMotion(int8_t direction, Bts7960WinchState state, uint32_t durationMs,
                   bool requireHost = true) {
  if (!gInitialized) return;
  cancelPendingMotion();
  gMovementTimeoutLatched = false;
  if (!baseSafetyValid(requireHost)) {
    hardStop(Bts7960WinchState::STOPPED);
    return;
  }
  if (direction > 0 && topHardStopActive(HAL_GetTick())) {
    hardStop(Bts7960WinchState::TOP_LIMIT);
    return;
  }
  if (direction < 0 && bottomHardStopActive(HAL_GetTick())) {
    hardStop(Bts7960WinchState::BOTTOM_LIMIT);
    return;
  }
  if (gDirection != 0 && gDirection != direction) {
    rawStop();
    gTimedDurationMs = 0U;
    setState(Bts7960WinchState::STOPPED);
    gPendingMotion = true;
    gPendingDirection = direction;
    gPendingState = state;
    gPendingDurationMs = durationMs;
    gPendingSinceMs = HAL_GetTick();
    gPendingRequiresHost = requireHost;
    return;
  }

  (void)startMotionNow(direction, state, durationMs, requireHost);
}

bool isUpState() {
  return gState == Bts7960WinchState::UP_HOME ||
         gState == Bts7960WinchState::UP_TIMED_1 ||
         gState == Bts7960WinchState::UP_TIMED_2;
}

bool isDownState() {
  return gState == Bts7960WinchState::DOWN_HOME ||
         gState == Bts7960WinchState::DOWN_TIMED_1 ||
         gState == Bts7960WinchState::DOWN_TIMED_2;
}
} // namespace

namespace Bts7960Winch {
void begin(PersistentConfigStore *store) {
#if BTS_WINCH_ENABLED
  gStore = store;
  rawStop();
  cancelPendingMotion();
  uint16_t stored = 0U;
  if (gStore != nullptr && gStore->read(kPersistKey, &stored, sizeof(stored)) &&
      stored <= PWM_MAX) {
    gConfiguredPwm = stored;
  } else {
    gConfiguredPwm = PWM_DEFAULT;
  }

  gTopTracking = gBottomTracking = false;
  gTopConfirmed = gBottomConfirmed = false;
  gTopSinceMs = gBottomSinceMs = 0U;
  gLimitFaultLatched = false;
  gMovementTimeoutLatched = false;
  gMotionRequiresHost = true;
  gPendingRequiresHost = true;
  gSafety = SafetyInputs{};
  gMotionStartedMs = gTimedDurationMs = 0U;
  setState(Bts7960WinchState::STOPPED);
  gInitialized = true;
#else
  (void)store;
#endif
}

void setSafetyInputs(const SafetyInputs &inputs) {
#if BTS_WINCH_ENABLED
  gSafety = inputs;
  const bool requireHost = gDirection != 0 ? gMotionRequiresHost :
                           (gPendingMotion ? gPendingRequiresHost : true);
  if (gInitialized && !baseSafetyValid(requireHost) &&
      (gDirection != 0 || gPendingMotion))
    hardStop(Bts7960WinchState::STOPPED);
#else
  (void)inputs;
#endif
}

void update() {
#if BTS_WINCH_ENABLED
  if (!gInitialized) return;
  const uint32_t now = HAL_GetTick();
  const bool topRaw = readTopRaw();
  const bool bottomRaw = readBottomRaw();
  updateConfirmWindow(topRaw, gTopTracking, gTopSinceMs, gTopConfirmed, now);
  updateConfirmWindow(bottomRaw, gBottomTracking, gBottomSinceMs,
                      gBottomConfirmed, now);

  // Directional safety uses a 2 ms fast-confirm window. This is much faster
  // than the 30 ms status/fault debounce, but rejects single-sample EMI/bounce
  // that would otherwise falsely stop motion. The opposite direction remains
  // available for recovery from an active end-stop.
  const bool topFast = fastLimitActive(topRaw, gTopTracking, gTopSinceMs, now);
  const bool bottomFast = fastLimitActive(bottomRaw, gBottomTracking, gBottomSinceMs, now);
  const bool rawTopHit = (gDirection > 0 && topFast) ||
                         (gPendingMotion && gPendingDirection > 0 && topFast);
  const bool rawBottomHit = (gDirection < 0 && bottomFast) ||
                            (gPendingMotion && gPendingDirection < 0 && bottomFast);
  if (rawTopHit)
    hardStop(Bts7960WinchState::TOP_LIMIT);
  else if (rawBottomHit)
    hardStop(Bts7960WinchState::BOTTOM_LIMIT);

  // Only a persistent contradictory TOP+BOTTOM state is a fault. A normal
  // single active end-stop must never lock out motion away from that end-stop.
  if (gTopConfirmed && gBottomConfirmed) {
    gLimitFaultLatched = true;
    if (gState != Bts7960WinchState::FAULT || gDirection != 0 || gPendingMotion)
      hardStop(Bts7960WinchState::FAULT);
    return;
  }
  if (gLimitFaultLatched && !(topRaw && bottomRaw) &&
      gDirection == 0 && !gPendingMotion) {
    gLimitFaultLatched = false;
    if (gState == Bts7960WinchState::FAULT)
      setState(Bts7960WinchState::STOPPED);
  }
  if (rawTopHit || rawBottomHit)
    return;
  const bool requireHost = gDirection != 0 ? gMotionRequiresHost :
                           (gPendingMotion ? gPendingRequiresHost : true);
  if (!baseSafetyValid(requireHost)) {
    if (gDirection != 0 || gPendingMotion)
      hardStop(Bts7960WinchState::STOPPED);
    return;
  }
  if (gPendingMotion && static_cast<uint32_t>(now - gPendingSinceMs) >= kReverseDeadtimeMs) {
    const int8_t direction = gPendingDirection;
    const Bts7960WinchState state = gPendingState;
    const uint32_t duration = gPendingDurationMs;
    const bool pendingRequireHost = gPendingRequiresHost;
    cancelPendingMotion();
    (void)startMotionNow(direction, state, duration, pendingRequireHost);
  }

  if (gDirection != 0 && gTimedDurationMs > 0U &&
      static_cast<uint32_t>(now - gMotionStartedMs) >= gTimedDurationMs) {
    hardStop(Bts7960WinchState::STOPPED);
    return;
  }

  const bool home = gState == Bts7960WinchState::UP_HOME ||
                    gState == Bts7960WinchState::DOWN_HOME;
  if (gDirection != 0 && home &&
      static_cast<uint32_t>(now - gMotionStartedMs) >= kHomeWatchdogMs) {
    // HOME timeout cuts power and returns to STOPPED.
    // Keep a diagnostic latch, but do not turn a recoverable timeout into a
    // permanent actuator fault.
    gMovementTimeoutLatched = true;
    hardStop(Bts7960WinchState::STOPPED);
    return;
  }
  if (!gPendingMotion && gConfiguredPwm != 0U && gDirection != 0) {
    uint16_t desiredPwm = gConfiguredPwm;
    if (!gMotionRequiresHost && isUpState() && gDirection > 0) {
      const uint32_t elapsed = static_cast<uint32_t>(now - gMotionStartedMs);
      desiredPwm = elapsed < kLocalUpKickMs ? PWM_MAX : gConfiguredPwm;
    }
    if (isUpState() && gDirection > 0 && gAppliedPwm != desiredPwm) {
      drivePwm(desiredPwm, 0U);
      gAppliedPwm = desiredPwm;
    } else if (isDownState() && gDirection < 0 && gAppliedPwm != desiredPwm) {
      drivePwm(0U, desiredPwm);
      gAppliedPwm = desiredPwm;
    }
  }
#endif
}

void stop() {
#if BTS_WINCH_ENABLED
  if (!gInitialized) return;
  hardStop(Bts7960WinchState::STOPPED);
#endif
}

void emergencyStop() { stop(); }
void upHome() { requestMotion(+1, Bts7960WinchState::UP_HOME, 0U); }
void downHome() { requestMotion(-1, Bts7960WinchState::DOWN_HOME, 0U); }
void upTimed1() { requestMotion(+1, Bts7960WinchState::UP_TIMED_1, 1000U); }
void upTimed2() { requestMotion(+1, Bts7960WinchState::UP_TIMED_2, 5000U); }
void downTimed1() { requestMotion(-1, Bts7960WinchState::DOWN_TIMED_1, 1000U); }
void downTimed2() { requestMotion(-1, Bts7960WinchState::DOWN_TIMED_2, 5000U); }
void upHomeLocal() { requestMotion(+1, Bts7960WinchState::UP_HOME, 0U, false); }
void downHomeLocal() { requestMotion(-1, Bts7960WinchState::DOWN_HOME, 0U, false); }
void upTimed1Local() { requestMotion(+1, Bts7960WinchState::UP_TIMED_1, 1000U, false); }
void upTimed2Local() { requestMotion(+1, Bts7960WinchState::UP_TIMED_2, 5000U, false); }
void downTimed1Local() { requestMotion(-1, Bts7960WinchState::DOWN_TIMED_1, 1000U, false); }
void downTimed2Local() { requestMotion(-1, Bts7960WinchState::DOWN_TIMED_2, 5000U, false); }
void timedLocal(int8_t direction, uint32_t durationMs) {
  if (durationMs < 1000U || durationMs > 30000U) return;
  if (direction > 0)
    requestMotion(+1, Bts7960WinchState::UP_TIMED_1, durationMs, false);
  else if (direction < 0)
    requestMotion(-1, Bts7960WinchState::DOWN_TIMED_1, durationMs, false);
}

bool processCommand(const char *command) {
  if (command == nullptr) return false;
  if (!std::strcmp(command, "UP HOME") || !std::strcmp(command, "UP") ||
      !std::strcmp(command, "WINCH UP") || !std::strcmp(command, "WINCH UP HOME")) {
    upHome(); return true;
  }
  if (!std::strcmp(command, "UP 1") || !std::strcmp(command, "WINCH UP 1") ||
      !std::strcmp(command, "WINCH 1")) {
    upTimed1(); return true;
  }
  if (!std::strcmp(command, "UP 5") || !std::strcmp(command, "WINCH UP 5") ||
      !std::strcmp(command, "WINCH 5") ||
      !std::strcmp(command, "UP 2") || !std::strcmp(command, "WINCH UP 2") ||
      !std::strcmp(command, "WINCH 2")) {
    upTimed2(); return true;
  }
  if (!std::strcmp(command, "DOWN HOME") || !std::strcmp(command, "DOWN") ||
      !std::strcmp(command, "WINCH DOWN") || !std::strcmp(command, "WINCH DOWN HOME") ||
      !std::strcmp(command, "WINCH 0")) {
    downHome(); return true;
  }
  if (!std::strcmp(command, "DOWN 1") || !std::strcmp(command, "WINCH DOWN 1")) {
    downTimed1(); return true;
  }
  if (!std::strcmp(command, "DOWN 5") || !std::strcmp(command, "WINCH DOWN 5") ||
      !std::strcmp(command, "DOWN 2") || !std::strcmp(command, "WINCH DOWN 2")) {
    downTimed2(); return true;
  }
  if (!std::strcmp(command, "STOP") || !std::strcmp(command, "WINCH STOP")) {
    stop(); return true;
  }
  return false;
}
bool setPwm(uint16_t pwm, bool persist) {
  if (pwm > PWM_MAX) return false;
  if (gConfiguredPwm == pwm) return true;

  // Flash programming is never attempted while the actuator is energized.
  if (persist && (gDirection != 0 || gPendingMotion)) return false;
  if (persist && gStore != nullptr &&
      !gStore->write(kPersistKey, &pwm, sizeof(pwm)))
    return false;

  // Commit runtime state only after the persistent transaction succeeds.
  gConfiguredPwm = pwm;
  if (pwm == 0U && (gDirection != 0 || gPendingMotion))
    stop();
  return true;
}

bool resetPwm(bool persist) {
  return setPwm(PWM_DEFAULT, persist);
}
uint16_t configuredPwm() { return gConfiguredPwm; }
uint16_t appliedPwm() { return gAppliedPwm; }
bool topLimitActive() { return gTopConfirmed; }
bool bottomLimitActive() { return gBottomConfirmed; }
bool topLimitRaw() { return readTopRaw(); }
bool bottomLimitRaw() { return readBottomRaw(); }
bool limitFault() { return gLimitFaultLatched; }
bool movementTimedOut() { return gMovementTimeoutLatched; }
bool limitInputsPlausible() {
  return gInitialized && !(gTopConfirmed && gBottomConfirmed);
}
bool limitsReady() {
  return limitInputsPlausible() && !gLimitFaultLatched;
}
bool initialized() { return gInitialized; }
bool motionAllowed(int8_t direction) {
  if (!gInitialized || !baseSafetyValid()) return false;
  if (direction > 0 && topHardStopActive(HAL_GetTick())) return false;
  if (direction < 0 && bottomHardStopActive(HAL_GetTick())) return false;
  return direction >= -1 && direction <= 1;
}
bool clearFault() {
  // Clear only a recovered/stale actuator fault. One legitimate end-stop may
  // remain active (for example BOTTOM while the fork is parked); the unsafe
  // condition is the contradictory TOP+BOTTOM state, which
  // limitInputsPlausible() rejects.
  //
  // Local HMI operation intentionally does not depend on ROS/global system
  // health, but it must still be stationary and free of E-stop before a winch
  // fault can be cleared.
  if (!gInitialized || gDirection != 0 || gPendingMotion ||
      !limitInputsPlausible() || gSafety.emergencyStop ||
      !gSafety.vehicleSafe)
    return false;
  gLimitFaultLatched = false;
  gMovementTimeoutLatched = false;
  setState(Bts7960WinchState::STOPPED);
  return true;
}
int8_t direction() { return gDirection; }
bool motionPending() { return gPendingMotion; }
Bts7960WinchState state() { return gState; }
bool faulted() {
  return gState == Bts7960WinchState::FAULT || gLimitFaultLatched;
}

const char *stateName() {
  switch (gState) {
    case Bts7960WinchState::STOPPED: return "STOPPED";
    case Bts7960WinchState::UP_HOME: return "UP_HOME";
    case Bts7960WinchState::DOWN_HOME: return "DOWN_HOME";
    case Bts7960WinchState::UP_TIMED_1: return "UP_TIMED_1";
    case Bts7960WinchState::UP_TIMED_2: return "UP_TIMED_5";
    case Bts7960WinchState::DOWN_TIMED_1: return "DOWN_TIMED_1";
    case Bts7960WinchState::DOWN_TIMED_2: return "DOWN_TIMED_5";
    case Bts7960WinchState::TOP_LIMIT: return "TOP_LIMIT";
    case Bts7960WinchState::BOTTOM_LIMIT: return "BOTTOM_LIMIT";
    case Bts7960WinchState::FAULT: return "FAULT";
  }
  return "FAULT";
}
} // namespace Bts7960Winch

#endif
