#pragma once

#include "FeatureConfig.h"
#include <cstdint>

class PersistentConfigStore;

enum class BtsWinchState : uint8_t {
  STOPPED = 0U,
  UP_HOME,
  DOWN_HOME,
  UP_TIMED_1,
  UP_TIMED_2,
  DOWN_TIMED_1,
  DOWN_TIMED_2,
  TOP_LIMIT,
  BOTTOM_LIMIT,
  FAULT
};

namespace BtsWinch {
constexpr uint16_t PWM_MAX = 1023U;
constexpr uint16_t PWM_DEFAULT = 500U;

struct SafetyInputs {
  bool emergencyStop{true};
  bool physicalSafetyValid{false};
  bool systemFault{true};
  bool hostSessionValid{false};
  bool commandSessionValid{false};
  bool vehicleSafe{false};
};

void begin(PersistentConfigStore *store);
void setSafetyInputs(const SafetyInputs &inputs);
void update();
void stop();
void emergencyStop();
void upHome();
void downHome();
void upTimed1();
void upTimed2();
void downTimed1();
void downTimed2();
bool processCommand(const char *command);

bool setPwm(uint16_t pwm, bool persist = true);
bool resetPwm(bool persist = true);
uint16_t configuredPwm();
uint16_t appliedPwm();

bool topLimitActive();
bool bottomLimitActive();
bool topLimitRaw();
bool bottomLimitRaw();
bool limitFault();
bool movementTimedOut();
bool motionAllowed(int8_t direction = 0);
bool clearFault();
int8_t direction();
bool motionPending();

BtsWinchState state();
const char *stateName();
bool faulted();
} // namespace BtsWinch
