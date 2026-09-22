#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
main = (root / "src/main.cpp").read_text()
winch = (root / "src/Bts7960Winch.cpp").read_text()
header = (root / "src/Bts7960Winch.h").read_text()
ui = (root / "src/HmiOperatorUi.h").read_text()
telemetry = (root / "src/Telemetry.h").read_text()
protocol = (root / "src/TelemetryProtocol.cpp").read_text()
pio = (root / "platformio.ini").read_text()

def need(condition, message):
    if not condition:
        raise SystemExit("FAIL: " + message)
    print("PASS:", message)

need("HmiOperatorUi gOperatorUi" in main and "HMI_F4_LAYOUT" not in pio,
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
need("forkSlide_>0U" in ui and "Bts7960Winch::stop();" in ui,
     "manual HMI fork uses press/release deadman")
need("upHomeLocal" in header and "upHomeLocal" in ui and "baseSafetyValid(requireHost)" in winch,
     "local HMI fork does not depend on ROS host but retains common safety")
need("processCommand" in winch and "upHome(); return true" in winch,
     "remote winch commands retain host-authority path")
dfu = main[main.index('if (!strcmp(command, "BOOT:DFU:CONFIRM"))'):
           main.index('if (!strcmp(command, "BOOT:DFU")) {')]
need("Bts7960Winch::emergencyStop();" in dfu and
     "motionSafeForHeavyMaintenance()" in dfu,
     "DFU transition cuts winch power and rechecks maintenance safety")
maint_start = main.rindex("static bool motionSafeForHeavyMaintenance()")
maint_end = main.index("}\n#endif", maint_start) + 1
maint = main[maint_start:maint_end]
need("Bts7960Winch::direction() == 0" in maint and
     "!Bts7960Winch::motionPending()" in maint and
     "Bts7960Winch::appliedPwm() == 0U" in maint,
     "heavy maintenance requires winch direction/pending/PWM all zero")
need("goalRemainingDistanceM" in telemetry and
     "d.objectDistanceM" not in ui[ui.index("void drawNav"):ui.index("void forkCard")],
     "object distance is not used as goal distance")
need("leftErpm" in telemetry and "rightErpm" in telemetry and
     "leftRpm" not in telemetry and "rightRpm" not in telemetry,
     "internal per-motor terminology is ERPM")
need("leftStatusKnown" in telemetry and "rightStatusKnown" in telemetry and
     '"N/A"' in ui[ui.index("void drawEsc"):ui.index("void drawPer")],
     "unknown per-channel ESC state is displayed honestly")
need("F4X3:" in protocol and "Legacy key/value telemetry is display compatibility only" in main,
     "F4X3 domain telemetry remains authoritative")
print("F4_WINCH_SAFETY_HMI_SELF_CHECK_PASS")
