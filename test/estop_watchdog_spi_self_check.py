#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
main = (ROOT / "src/main.cpp").read_text()
ui = (ROOT / "src/HmiOperatorUi.h").read_text()
display = (ROOT / "src/HmiDisplay.cpp").read_text()
display_h = (ROOT / "src/HmiDisplay.h").read_text()
board = (ROOT / "src/BoardSupport.cpp").read_text()

def need(ok, msg):
    if not ok:
        raise SystemExit("FAIL: " + msg)
    print("PASS:", msg)

need("K_ESTOP" in ui and "if(hit(x,y,248,0,72,30)) return K_ESTOP;" in ui,
     "E-stop hit zone is global and checked before page controls")
need("consumeEmergencyStopRequest" in ui and
     "gOperatorUi.consumeEmergencyStopRequest()" in main,
     "touch E-stop request reaches application safety path")

trigger = main.split("static void triggerLocalEmergencyStop()", 1)[1].split(
    "static void setHostEmergencyStop", 1)[0]
for token in ("gLocalEStopLatched = true", "enforceEmergencyStopOutputs()"):
    need(token in trigger, "local E-stop trigger contains " + token)

enforce = main.split("static void enforceEmergencyStopOutputs()", 1)[1].split(
    "static void triggerLocalEmergencyStop", 1)[0]
for token in ("zeroCommandedMotionState()", "Bts7960Winch::emergencyStop()",
              "stopAllManualTest()", "latchAllSafetyStops()",
              "serviceSafetyControlTx()"):
    need(token in enforce, "E-stop output path contains " + token)

hold = main.split("static void serviceEmergencyStopHold()", 1)[1].split(
    "static bool motionSafeForHeavyMaintenance", 1)[0]
need("ESTOP_STOP_REFRESH_MS = 100U" in main and "latchAllSafetyStops()" in hold,
     "latched E-stop refreshes all safety STOPs every 100 ms")
for wire in ("CMD:ESTOP:1", "CMD:ESTOP:0", "CMD:DRIVE:STOP", "CMD:STEER:STOP", "CMD:NAV:STOP"):
    need(wire in main, "fail-safe wire command " + wire)
need("HmiTouchService::active()" in main and "triggerLocalEmergencyStop();" in main,
     "full-screen touch maintenance is interlocked by the global E-stop")

need("commandLinearMps = 0.0F" in main and
     "commandAngularRps = 0.0F" in main and
     "driveTargetMps = 0.0F" in main,
     "local motion command state is forced to zero")
need("gLocalEStopLatched = false" in main and "gHostEStopActive = false" in main,
     "local and host E-stop sources are independent")
need("gTelemetry.eStop = gLocalEStopLatched || gHostEStopActive" in main,
     "effective E-stop is OR of local and host sources")
need('!strcmp(command, "ESTOP:RESET")' in main and
     'ERR:ESTOP:RESET:UNSAFE' in main,
     "local E-stop needs explicit stationary reset")
need('gUsb.writeLineHighPriority("ERR:ESTOP:ARGS")' in main,
     "malformed ESTOP input cannot silently clear stop state")

watch = main.split("static void appWatchdogIsr()", 1)[1].split(
    "static void startAppWatchdog()", 1)[0]
need("gMainLoopHeartbeatEpoch" in watch and "gAppWatchdogStallTicks" in watch,
     "watchdog uses independent TIM11 progress epochs")
need("HAL_GetTick() - gMainLoopHeartbeatMs" not in watch,
     "watchdog timeout does not depend on SysTick")
need(main.count("++gMainLoopHeartbeatEpoch;") == 1,
     "watchdog heartbeat epoch commits only at full loop completion")
need("TIM1_TRG_COM_TIM11_IRQHandler" in board and
     "HAL_TIM_PeriodElapsedCallback" in board,
     "TIM11 watchdog interrupt is present")
need("HAL_NVIC_SetPriority(TIM1_TRG_COM_TIM11_IRQn, 1U, 0U)" in board,
     "watchdog IRQ preempts USB and lower-priority peripheral IRQs")
need(main.find("startAppWatchdog();") < main.find("gPersistentConfigReady = gPersistentConfig.begin();"),
     "watchdog is armed immediately after Board_Init before config/USB/HMI startup")

need("kSpiWaitTimeoutUs = 2500U" in display_h,
     "SPI busy wait is time-bounded")
need("kSpiHalTimeoutMs = 8U" in display_h,
     "SPI HAL transfers are time-bounded")
need("Board_SpiOwner() != BoardSpiOwner::NONE" in display and
     "recoverSpi()" in display,
     "stale SPI owner is recovered")
need("SPI_SR_OVR" in display and "Board_ReinitSpi1" in display,
     "SPI OVR cleanup and peripheral reinit are present")

print("ESTOP_WATCHDOG_SPI_SELF_CHECK_PASS")
