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
ui = rd('src/HmiOperatorUi.h')
main = rd('src/main.cpp')
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
need('if (fullRedraw) tft.fillScreen(C_BG2);' in ui,
     'full-screen clear occurs only on full redraw')
need('void cancelTouch()' in ui and 'bool touchDown() const' in ui,
     'operator UI owns touch lifecycle')
need('VehicleTelemetry telemetrySnapshot = gTelemetry' in main,
     'renderer consumes an immutable telemetry snapshot')
need('gRealtimeParserActive' in main and 'enqueueDeferredCommand' in main and 'serviceDeferredCommands' in main,
     'host mutations are deferred during display realtime service')
need('deferredCommandDrops' in diag, 'deferred command overflow is observable')
need('touch_reject_fast_count_' in cpp and 'Median3' in cpp, 'XPT2046 filtering remains active')

reinit = re.search(r'bool Board_ReinitSpi1\(\)\s*\{(.*?)\n\}', board, re.S)
need(reinit is not None and 'FatalError' not in reinit.group(1), 'runtime SPI recovery cannot enter boot fatal path')
need(re.search(r'SetPriority\(OTG_FS_IRQn,\s*2', usb) is not None, 'USB IRQ priority remains 2')
need(re.search(r'SetPriority\(TIM1_TRG_COM_TIM11_IRQn,\s*3', board) is not None, 'watchdog IRQ priority remains 3')
need('Board_ServiceGapP95Ms' in main and 'Board_ServiceGapP99Ms' in main, 'service latency telemetry remains exported')
need('Board_StackHeadroomBytes' in main and 'minStackHeadroomBytes' in diag, 'stack headroom remains monitored')
need('FW:INFO' in main and 'BuildInfo::' in main and 'FW_GIT_SHA' in build, 'runtime build identity remains exposed')
need('build_provenance.py' in ini, 'binary provenance manifest remains enabled')
need('[env:blackpill_f411ce_faulttest]' in ini and 'HMI_TEST_HOOKS=1' in ini,
     'fault injection stays isolated from production')
print('HMI_DISPLAY_ROBUSTNESS_SELF_CHECK_PASS')
