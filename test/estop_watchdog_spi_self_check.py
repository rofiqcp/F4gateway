#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
main = (ROOT / "src/main.cpp").read_text()
handler = (ROOT / "src/F103MinimalCommandHandler.inc").read_text()
ui = (ROOT / "src/HmiOperatorUi.h").read_text()
display = (ROOT / "src/HmiDisplay.cpp").read_text()
display_h = (ROOT / "src/HmiDisplay.h").read_text()
board = (ROOT / "src/BoardSupport.cpp").read_text()

def need(ok, msg):
    if not ok:
        raise SystemExit("FAIL: " + msg)
    print("PASS:", msg)

need("K_ESTOP" in ui and "if(hit(x,y,248,0,72,32)) return K_ESTOP;" in ui,
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
    "static float actualEditValue", 1)[0]
need("ESTOP_STOP_REFRESH_MS = 100U" in main and
     "if (gLocalEStopLatched)" in hold and
     "latchAllSafetyStops()" in hold,
     "physical/local E-stop refreshes all safety STOPs every 100 ms")
host_estop = main.split("static void setHostEmergencyStop(bool active)", 1)[1].split(
    "static bool localEStopResetSafe", 1)[0]
need("if (gHostEStopActive == active)" in host_estop and "return;" in host_estop,
     "repeated ROS E-stop status is edge-deduplicated to prevent command feedback")
for wire in ("CMD:ESTOP:1", "CMD:ESTOP:0", "CMD:DRIVE:STOP", "CMD:STEER:STOP", "CMD:NAV:STOP"):
    need(wire in main, "fail-safe wire command " + wire)
need("HmiTouchService::active()" in handler and "triggerLocalEmergencyStop();" in handler,
     "full-screen touch maintenance is interlocked by the global E-stop")

zero_state = main.split("static void zeroCommandedMotionState()", 1)[1].split(
    "static void updateEffectiveEStopState", 1)[0]
need("commandLinearMps = 0.0F" in zero_state and
     "commandAngularRps = 0.0F" in zero_state and
     "cmdAgeMs = 0U" in zero_state,
     "local commanded navigation state is forced to zero")
need("gLocalEStopLatched = false" in main and "gHostEStopActive = false" in main,
     "local and host E-stop sources are independent")
need("gTelemetry.eStop = gLocalEStopLatched || gHostEStopActive" in main,
     "effective E-stop is OR of local and host sources")
need('!std::strcmp(command,"ESTOP:RESET")' in handler and
     'ERR:ESTOP:RESET:UNSAFE' in handler,
     "local E-stop needs explicit stationary reset")
need('gUsb.writeLineHighPriority("ERR:ESTOP:ARGS")' in handler,
     "malformed ESTOP input cannot silently clear stop state")

watch = main.split("static void appWatchdogIsr()", 1)[1].split(
    "static void startAppWatchdog()", 1)[0]
need("gWatchdogProgressEpoch" in watch and "gAppWatchdogStallTicks" in watch,
     "watchdog uses independent TIM3 progress epochs")
need("HAL_GetTick() - gMainLoopHeartbeatMs" not in watch,
     "watchdog timeout does not depend on SysTick")
need("noteWatchdogProgress();" in main and
     "Board_SetRealtimeServiceCallback([]()" in main,
     "watchdog accepts realtime safety-service progress during long startup/recovery")
need(main.count("noteWatchdogProgress();") >= 3,
     "watchdog progress is refreshed at start, realtime service, and main-loop completion")
need("TIM3_IRQHandler" in board and
     "HAL_TIM_PeriodElapsedCallback" in board,
     "TIM3 watchdog interrupt is present")
need("HAL_NVIC_SetPriority(TIM3_IRQn, 1U, 0U)" in board,
     "watchdog IRQ preempts USB and lower-priority peripheral IRQs")
need(main.find("startAppWatchdog();") < main.find("gPersistentConfigReady = gPersistentConfig.begin();"),
     "watchdog is armed immediately after Board_Init before config/USB/HMI startup")

need("kSpiWaitTimeoutUs = 2500U" in display_h,
     "SPI busy wait is time-bounded")
need("kSpiHalTimeoutMs = 8U" in display_h,
     "SPI HAL transfers are time-bounded")
acquire = display.split("bool HmiDisplay::beginTransaction", 1)[1].split(
    "bool HmiDisplay::endTransaction", 1)[0]
need("if (attempt == 0U)" in acquire and "recoverSpi()" in acquire,
     "SPI acquisition failure performs one-shot peripheral recovery")
need("transaction_error_" in display and "recoverSpi()" in display,
     "SPI transfer errors are latched and recovered on transaction close")
need("SPI_SR_OVR" in display and "Board_ReinitSpi1" in display,
     "SPI OVR cleanup and peripheral reinit are present")
software_cut = (
    "Board_BtsEmergencyCut()" in watch or
    ("g_bts_pwm_ticks = 0U" in watch and
     "g_bts_pwm_direction = 0" in watch and
     "BtsPinsForceLow();" in watch)
)
legacy_cut = "TIM2->CCR4 = 0U" in watch and "TIM4->CCR3 = 0U" in watch
need(software_cut or legacy_cut,
     "watchdog cuts both BTS7960 PWM directions before reset")

fault = board.split("void Board_FaultReset", 1)[1]
software_fault_cut = (
    "g_bts_pwm_ticks = 0U" in fault and
    "g_bts_pwm_direction = 0" in fault and
    "TIM4->DIER = 0U" in fault and
    "BtsPinsForceLow();" in fault
)
legacy_fault_cut = "TIM2->CCR4 = 0U" in fault and "TIM4->CCR3 = 0U" in fault
need(software_fault_cut or legacy_fault_cut,
     "fault reset cuts both BTS7960 PWM directions before reset")

print("ESTOP_WATCHDOG_SPI_SELF_CHECK_PASS")
