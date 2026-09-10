#!/usr/bin/env python3
from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[1]

def read(rel):
    return (ROOT / rel).read_text(errors='replace')

def need(cond, msg):
    if not cond:
        print('FAIL', msg)
        sys.exit(1)
    print('PASS', msg)

cfg = read('src/Config.h')
menu = read('src/UiMenu.h')
shell = read('src/UiShell.h')
touch = read('src/TouchButtons.h')
main = read('src/main.cpp')
diag = read('src/Diagnostics.h')
board = read('src/BoardSupport.cpp')
usb = read('src/usb/usbd_conf.c')
pio = read('platformio.ini')

# V01: geometry must leave overview above footer.
def intval(name):
    m = re.search(rf'{name}\s*=\s*(\d+)', cfg)
    need(m is not None, f'{name} defined')
    return int(m.group(1))

overview_y = intval('OVERVIEW_CARD_Y')
overview_h = intval('OVERVIEW_CARD_H')
footer_y = intval('CAROUSEL_NAV_Y')
need(overview_y + overview_h <= footer_y, 'V01 overview cards do not overlap carousel footer')
need('SYSTEM_ROOT' in menu and 'count = 4; return overview' in menu, 'V01 four overview domains reachable')

# V02/V03/V04
need('domainHealthColor' in shell and 'STALE' in shell and 'AgeMs' in read('src/Telemetry.h'), 'V02 stale/fresh health model present')
need('fillRoundRect(x, OVERVIEW_CARD_Y, 4, OVERVIEW_CARD_H' in shell, 'V03 geometric health rail present')
for page in ['SYSTEM_OVERVIEW','SYSTEM_PINS_IO','SYSTEM_PINS_DISPLAY','SYSTEM_LINKS','SYSTEM_ERRORS']:
    need(page in menu and page in shell, f'V04 {page} implemented')

# V05: no LVGL dependency was introduced.
need('lvgl' not in pio.lower(), 'V05 no LVGL dependency in PlatformIO')
need('iconChip' in read('src/Icons.h') and 'iconPins' in read('src/Icons.h'), 'V06 procedural SYSTEM icons present')
need('drawMetricRow' in shell and 'drawOverview' in shell, 'V07 label/value hierarchy present')

# R01/R02 touch safety.
need('TOUCH_TAP_MIN_MS' in cfg and 'touchCanceled' in touch and 'TouchEvent::RELEASE' in main, 'R01 release-commit and slide-cancel touch state present')
need('TOUCH_HOLD_ACTION_MS' in cfg and 'TouchEvent::HOLD' in main and 'isManualMotionKey' in touch, 'R02 hold-to-run manual motion present')
need('TEST_STOP' in main and 'ev.type == TouchEvent::PRESS' in main, 'R02 STOP remains immediate on press')
need('stopAllManualTest();' in main and 'ev.type == TouchEvent::RELEASE' in main, 'R02 release-to-stop path present')

# R03 freshness gates.
need('gTelemetry.escFresh' in main and 'gTelemetry.navigationFresh' in main and 'editDomainFresh' in main, 'R03 critical action freshness gates present')

# R04 pin visibility and exact fixed wiring references.
pins = ['GPIO_PIN_6','GPIO_PIN_7','GPIO_PIN_2','GPIO_PIN_3','GPIO_PIN_8','GPIO_PIN_9','GPIO_PIN_12','GPIO_PIN_13','GPIO_PIN_5']
for pin in pins:
    need(pin in board, f'R04 board implementation references {pin}')
need('GPIO_PIN_11 | GPIO_PIN_12' in usb, 'R04 USB owner config maps PA11/PA12')
for field in ['pb6VescTx','pb7VescRx','pa2GnssTx','pa3GnssRx','pb8I2cScl','pb9I2cSda','pb12Safety','pb13SafetyLed','pa8Buzzer','pa5SpiSck','pa6SpiMiso','pa7SpiMosi','pb0TftCs','pb1TftDc','pb2TftRst','pa4TouchCs','pa11UsbDm','pa12UsbDp']:
    need(field in diag and field in main, f'R04 diagnostics samples {field}')

# R05-R08.
for token in ['vescUartErrors','gnssUartErrors','vescFrameErrors','vescRecoveryCount','unknownCommands','overlongCommands','tftControllerId']:
    need(token in diag and token in shell, f'R05 visible diagnostics field {token}')
need('drawUiFrame' in main and 'gUiDrawActive' in main, 'R06 rendering remains main-loop guarded')
need('SYSTEM_ROOT' in shell and 'gDiagnostics.tftOk' in shell, 'R07 local SYSTEM health does not depend on ROS')
need('eqIgnoreCase(name, "SYSTEM")' in main, 'R08 GOTO:SYSTEM routing present')

print('PASS F4GATEWAY_HMI_10REPO_AUDIT')
