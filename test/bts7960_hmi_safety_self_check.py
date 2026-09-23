#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
main = (root / "src/main.cpp").read_text()
winch = (root / "src/Bts7960Winch.cpp").read_text()
header = (root / "src/Bts7960Winch.h").read_text()
ui = (root / "src/HmiOperatorUi.h").read_text()
handler = (root / "src/F103MinimalCommandHandler.inc").read_text()

def need(condition, message):
    if not condition:
        raise SystemExit("FAIL: " + message)
    print("PASS:", message)

need("HmiOperatorUi gOperatorUi" in main,
     "HmiOperatorUi is the single production renderer")
need("Bts7960Winch::emergencyStop();" in
     main[main.index("static void forceRosOffline()"):
          main.index("static void checkRosLinkTimeout")],
     "ROS/session loss stops the winch")
need("struct SafetyInputs" in header and "baseSafetyValid()" in winch,
     "central winch safety gate exists")
need("gTopConfirmed && gBottomConfirmed" in winch and
     "gLimitFaultLatched = true" in winch,
     "contradictory limits latch a fault")
need("kLimitConfirmMs" in winch and "updateConfirmWindow" in winch,
     "limit inputs are debounced")
need("gMovementTimeoutLatched = true" in winch and
     "kHomeWatchdogMs" in winch,
     "HOME timeout latches a fault")
need("if (persist && (gDirection != 0 || gPendingMotion)) return false;" in winch and
     winch.index("gStore->write") < winch.index("gConfiguredPwm = pwm"),
     "persistent PWM commits before runtime PWM")
need('"FORK UP"' in ui and '"FORK DOWN"' in ui and '"WINCH UP"' in ui and
     '"1 SECOND"' in ui and '"5 SECOND"' in ui,
     "forklift HMI exposes only the requested primary controls and duration selectors")
need("Bts7960Winch::upTimed1Local();" in ui and
     "Bts7960Winch::upTimed2Local();" in ui and
     "Bts7960Winch::downTimed1Local();" in ui and
     "Bts7960Winch::downTimed2Local();" in ui,
     "HMI duration selector reuses the existing non-blocking timed actuator paths")
need("upTimed1Local" in header and "downTimed1Local" in header and
     "baseSafetyValid(requireHost)" in winch,
     "local timed HMI motion bypasses ROS authority only while retaining common safety")
need("if(actuatorBusy()) return;" in ui and "motionPending()" in ui,
     "HMI blocks overlapping actuator actions")
need("processCommand" in winch and "upHome(); return true" in winch,
     "remote winch commands retain host-authority path")
dfu = handler[handler.index('if (!std::strcmp(command,"BOOT:DFU:CONFIRM"))'):
              handler.index('if (!std::strcmp(command,"USB:RECOVER"))')]
need("Bts7960Winch::emergencyStop();" in dfu and
     "firmwareUpdateSafe()" in dfu,
     "DFU transition cuts winch power and requires de-energized update safety")
outputs_start = main.index("static bool maintenanceOutputsStopped()")
outputs_end = main.index("static bool firmwareUpdateSafe()", outputs_start)
outputs = main[outputs_start:outputs_end]
need("Bts7960Winch::direction() == 0" in outputs and
     "!Bts7960Winch::motionPending()" in outputs and
     "Bts7960Winch::appliedPwm() == 0U" in outputs and
     "return maintenanceOutputsStopped();" in
       main[outputs_end:main.index("#endif", outputs_end)],
     "firmware update uses the shared de-energized output gate")
print("F103C8_WINCH_SAFETY_HMI_SELF_CHECK_PASS")
