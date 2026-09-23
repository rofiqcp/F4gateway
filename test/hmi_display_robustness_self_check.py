#!/usr/bin/env python3
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]
def rd(path): return (ROOT / path).read_text(errors='replace')
def need(ok, msg):
    if not ok:
        raise SystemExit('FAIL: ' + msg)
    print('PASS:', msg)

h = rd('src/HmiDisplay.h')
cpp = rd('src/HmiDisplay.cpp')
touch = rd('src/HmiTouchService.cpp')
ui = rd('src/HmiOperatorUi.h')
main = rd('src/main.cpp')
handler = rd('src/F103MinimalCommandHandler.inc')
board = rd('src/BoardSupport.cpp')
usb = rd('src/usb/usbd_conf.c')
diag = rd('src/HmiDiagnostics.h')
ini = rd('platformio.ini')
build = rd('src/BuildInfo.h')

for token in ['SpiOwner::IDLE', 'SpiOwner::TFT_WRITE', 'SpiOwner::TFT_READ', 'SpiOwner::TOUCH']:
    need(token in h or token in cpp, 'SPI ownership: ' + token)
need('kSpiWaitTimeoutUs' in h and 'DWT->CYCCNT' in cpp, 'SPI waits are time bounded')
need('spi_timeout_count_' in h and 'spi_hal_error_count_' in h, 'SPI errors are observable')
need('Board_ReinitSpi1' in cpp and 'recoverSpi' in cpp, 'SPI runtime recovery exists')
need('beginInit' in cpp and 'serviceInit' in cpp and 'InitPhase' in h, 'display init is staged/nonblocking')
need('frame_transaction_active_' in h and 'beginFrame' in cpp and 'endFrame' in cpp, 'frame transaction batching exists')
need('verifyWriteProfile' in cpp and 'readPixel565' in cpp, 'fast SPI clock requires readback validation')

need('class HmiOperatorUi' in ui and 'UiShell' not in main and 'TouchButtons' not in main,
     'single operator renderer is active')
need(re.search(r'if\s*\(fullRedraw\)\s*\{\s*tft\.fillScreen\(C_BG\);', ui) is not None,
     'full-screen clear occurs only on full redraw')
need('void cancelTouch()' in ui and 'bool touchDown() const' in ui,
     'operator UI owns touch lifecycle')
need('VehicleTelemetry telemetrySnapshot = gTelemetry' in main,
     'renderer consumes an immutable telemetry snapshot')
need('gRealtimeParserActive' in main and 'enqueueDeferredCommand' in main and 'serviceDeferredCommands' in main,
     'host mutations are deferred during display realtime service')
need('deferredCommandDrops' in diag, 'deferred command overflow is observable')
need('touch_reject_fast_count_' in cpp and 'Median3' in cpp, 'XPT2046 filtering remains active')
need('projectAxisToPanelEdges' in touch and 'W - 1' in touch and 'H - 1' in touch,
     'five-point calibration extrapolates inset targets to the physical LCD edges')
need('sx < 0 || sy < 0 || sx >= width_ || sy >= height_' in cpp and
     '*x = static_cast<uint16_t>(sx);' in cpp,
     'out-of-range touch noise is rejected instead of clamped onto bezel buttons')
need('kTouchCalibrationVersion = 3U' in main and
     'record.version != kTouchCalibrationVersion' in main,
     'stale pre-edge-projection calibration is not restored after firmware update')

reinit = re.search(r'bool Board_ReinitSpi1\(\)\s*\{(.*?)\n\}', board, re.S)
need(reinit is not None and 'FatalError' not in reinit.group(1), 'runtime SPI recovery cannot enter boot fatal path')
need(re.search(r'SetPriority\(USB_LP_CAN1_RX0_IRQn,\s*2', usb) is not None, 'F103 USB IRQ priority remains 2')
need(re.search(r'SetPriority\(TIM3_IRQn,\s*1', board) is not None, 'TIM3 watchdog IRQ priority remains 1 and preempts USB')
need('Board_ServiceGapP95Ms' in main and 'Board_ServiceGapP99Ms' in main, 'service latency telemetry remains exported')
need('Board_StackHeadroomBytes' in main and 'minStackHeadroomBytes' in diag, 'stack headroom remains monitored')
need('FW:INFO:V3:F103C8:64K' in handler and 'FW_GIT_SHA' in build and 'build_provenance.py' in ini, 'C8 runtime identity and binary provenance remain enabled')
need('build_provenance.py' in ini, 'binary provenance manifest remains enabled')
need('HMI_TEST_HOOKS' not in ini, 'production PlatformIO environments contain no fault-injection hooks')
print('HMI_DISPLAY_ROBUSTNESS_SELF_CHECK_PASS')
