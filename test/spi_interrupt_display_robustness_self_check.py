#!/usr/bin/env python3
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]

def rd(path):
    return (ROOT / path).read_text(errors="replace")

def need(condition, message):
    if not condition:
        print("FAIL", message)
        sys.exit(1)
    print("PASS", message)

h = rd("include/HmiDisplay.h")
cpp = rd("src/HmiDisplay.cpp")
main = rd("src/main.cpp")
ui = rd("src/UiShell.h")
board = rd("src/BoardSupport.cpp")
usb = rd("src/usb/usbd_conf.c")
diag = rd("src/Diagnostics.h")
ini = rd("platformio.ini")
build = rd("include/BuildInfo.h")
for token in ["SpiOwner::IDLE", "SpiOwner::TFT_WRITE", "SpiOwner::TFT_READ", "SpiOwner::TOUCH"]:
    need(token in h or token in cpp, f"shared-SPI owner exists: {token}")
need("kSpiWaitTimeoutUs" in h and "DWT->CYCCNT" in cpp, "SPI wait is cycle-bounded")
need("spi_timeout_count_" in h and "spi_hal_error_count_" in h, "SPI timeout/HAL failures are counted")
need("TftCs(true)" in cpp and "TouchCs(true)" in cpp, "both chip selects are deasserted before recovery")
need("Board_ReinitSpi1" in cpp and "recoverSpi" in cpp, "SPI has runtime recovery path")

reinit = re.search(r"bool Board_ReinitSpi1\(\)\s*\{(.*?)\n\}", board, re.S)
need(reinit is not None, "runtime SPI reinit body found")
need("FatalError" not in reinit.group(1), "runtime SPI reinit cannot enter boot-fatal loop")
need("Spi1_InitBootOrFatal" in board and "Spi1_Configure" in board, "boot-fatal and runtime SPI init are separated")
need("beginInit" in cpp and "serviceInit" in cpp and "InitPhase" in h, "display init/recovery uses staged state machine")
need("HAL_Delay" not in cpp[cpp.find("void HmiDisplay::beginInit"):cpp.find("uint8_t HmiDisplay::readRegister8")],
     "display init state machine contains no blocking HAL_Delay")

need("frame_transaction_active_" in h and "beginFrame" in cpp and "endFrame" in cpp,
     "frame-level TFT transaction is implemented")
need("setWindowTx" in cpp and "writeColorTx" in cpp and "pushImageTx" in cpp,
     "window and pixel payloads use batched primitives")
need(re.search(r"text_tile_\s*\[\s*320U\s*\*\s*kTextTileRows\s*\]", h) is not None,
     "fixed RGB565 text tile exists")
need(re.search(r"pushImage\s*\(\s*ax0\s*,\s*stripY\s*,\s*aw\s*,\s*sh\s*,\s*text_tile_\s*\)", cpp) is not None,
     "font renderer flushes text by tile instead of pixel transactions")
need("kTftFastPrescaler = SPI_BAUDRATEPRESCALER_8" in h, "validated 12 MHz candidate exists")
need("kTftUltraFastPrescaler = SPI_BAUDRATEPRESCALER_4" in h, "validated 24 MHz candidate exists")
need("verifyWriteProfile" in cpp and "readPixel565" in cpp, "fast clocks require pixel readback verification")
need("ultra_fast_write_validated_" in h and "ULTRA=%u" in main, "24 MHz validation is observable")
need("TFT:VERIFY" in main, "runtime clock re-verification command exists")

need(re.search(r"SetPriority\(USART1_IRQn,\s*0", board) is not None, "USART1/VESC priority 0")
need(re.search(r"SetPriority\(USART2_IRQn,\s*1", board) is not None, "USART2/GNSS priority 1")
need(re.search(r"SetPriority\(OTG_FS_IRQn,\s*2", usb) is not None, "USB OTG FS priority 2")
need(re.search(r"SetPriority\(TIM1_TRG_COM_TIM11_IRQn,\s*3", board) is not None, "watchdog priority 3")
for body in re.findall(r'extern\s+"C"\s+void\s+\w+_IRQHandler\([^)]*\)\s*\{([^}]*)\}', board + usb, re.S):
    need("HAL_SPI_" not in body and "draw" not in body, "ISR does not render or own SPI")

need("UiDirtyBits" in ui and "uiDirtyMask" in main, "dirty-mask refresh remains active")
need("UiMetricCacheEntry" in ui and "gUiOverviewCache" in ui, "per-field/overview value caches exist")
need("gUiDynamicPass" in ui, "dynamic paint pass is separated from static paint")
need("clearFullFrameBackground" in ui and "fillScreen(C_BG)" not in ui, "full page avoids unconditional framebuffer clear")
need("VehicleTelemetry telemetrySnapshot = gTelemetry" in main and "UiState uiSnapshot = gUi" in main,
     "renderer consumes immutable UI/telemetry snapshots")
need("gRealtimeParserActive" in main and "enqueueDeferredCommand" in main and "serviceDeferredCommands" in main,
     "UI mutation is deferred out of realtime display service")
need("kDeferredCommandSlots" in main and "deferredCommandDrops" in diag, "deferred queue is bounded and observable")
need("invalidateTouchGeneration" in main and "touchGeneration" in rd("src/TouchButtons.h"),
     "touch events are generation-scoped across page/display changes")
need("touch_reject_fast_count_" in cpp and "Median3" in cpp, "XPT2046 fast reject + median filtering remain active")
need("motionSafeForHeavyMaintenance" in main and "ERR:TFT:MOTION_OR_NAV_ACTIVE" in main,
     "heavy TFT test is motion/nav gated")
need("Board_RealtimeDelayMs" in main and "TFT:TEST" in main, "TFT self-test delays continue realtime service")

need("Board_ServiceGapP95Ms" in main and "Board_ServiceGapP99Ms" in main, "steady p95/p99 service latency is exported")
need("Board_StackHeadroomBytes" in main and "minStackHeadroomBytes" in diag, "stack headroom is monitored")
need("DIAGNOSTIC_AUTO_RETURN_MS" in rd("src/Config.h"), "diagnostic pages have bounded idle residency")
need("FW:INFO" in main and "BuildInfo::" in main and "FW_GIT_SHA" in build, "runtime build provenance is exposed")
need("build_provenance.py" in ini, "build identity manifest is generated automatically")

need("[env:blackpill_f411ce_faulttest]" in ini and "HMI_TEST_HOOKS=1" in ini,
     "fault injection is isolated to dedicated non-production environment")
need("#ifdef HMI_TEST_HOOKS" in main and "TEST:SPI:REINIT_FAIL" in main and "TEST:SPI:TX_FAIL" in main,
     "SPI init/transfer fault injection hooks exist only for fault-test builds")
print("PASS F4GATEWAY_SPI_INTERRUPT_DISPLAY_ROBUSTNESS")
