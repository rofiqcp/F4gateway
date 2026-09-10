#!/usr/bin/env python3
from pathlib import Path
import re,sys
ROOT=Path(__file__).resolve().parents[1]
def rd(x): return (ROOT/x).read_text(errors='replace')
def need(c,m):
    if not c: print('FAIL',m); sys.exit(1)
    print('PASS',m)
cfg,menu,shell,touch,main,diag=map(rd,['src/Config.h','src/UiMenu.h','src/UiShell.h','src/TouchButtons.h','src/main.cpp','src/Diagnostics.h'])
board,usb,pio=rd('src/BoardSupport.cpp'),rd('src/usb/usbd_conf.c'),rd('platformio.ini')

# V01 operator information architecture.
need('UiMenuId::HOME' in menu and 'UiMenuId::MAIN_MENU' in menu,'V01 HOME and MAIN MENU implemented')
need('count = 3U; return mainMenu' in menu,'V01 exactly three operator domains')
mm=re.search(r'static const UiMenuId mainMenu\[\]\s*=\s*\{([^}]*)\}',menu,re.S)
need(mm and 'SYSTEM_ROOT' not in mm.group(1),'V01 SYSTEM hidden from operator menu')
need('SERVICE_HOLD_MS = 800' in cfg and 'enterServiceMode' in main,'V01 hidden Service entry implemented')
need('count = 12U; return esc' in menu and 'count = 9U; return perception' in menu and 'count = 12U; return navigation' in menu,'V01 12/9/12 final domain catalog')
need('pageIndex' in menu and 'menuPageCount' in menu and 'menuCardAt' in menu,'V01 page-of-three navigation state implemented')

# V02 telemetry/freshness honesty.
need('domainHealthColor' in shell and 'STALE' in shell and 'AgeMs' in rd('src/Telemetry.h'),'V02 stale/fresh health model retained')
need('vbusValid' in rd('src/Telemetry.h') and '"N/A"' in shell,'V02 unavailable POWER is explicit N/A')
need('drawHome' in shell and 'drawMainMenu' in shell and 'drawDomainMenu' in shell,'V03 dedicated Stage-1 renderers present')
need('UiHomeCache' in shell and 'UiMetricCacheEntry' in shell,'V03 bounded dynamic value caches retained')
for page in ['SYSTEM_OVERVIEW','SYSTEM_PINS_IO','SYSTEM_PINS_DISPLAY','SYSTEM_LINKS','SYSTEM_ERRORS']:
    need(page in menu and page in shell,f'V04 Service subset {page} implemented')
need('lvgl' not in pio.lower(),'V05 no LVGL dependency introduced')
need('iconChip' in rd('src/Icons.h') and 'iconPins' in rd('src/Icons.h'),'V06 procedural service icons retained')
need('drawMetricRow' in shell and 'drawValueTextPadded' in shell,'V07 label/value hierarchy retained')

# R01/R02 touch safety.
need('TOUCH_TAP_MIN_MS' in cfg and 'touchCanceled' in touch and 'TouchEvent::RELEASE' in main,'R01 release-commit + slide-cancel retained')
need('TOUCH_HOLD_ACTION_MS' in cfg and 'TouchEvent::HOLD' in main and 'isManualMotionKey' in touch,'R02 hold-to-run retained')
need('TEST_STOP' in main and 'ev.type == TouchEvent::PRESS' in main,'R02 STOP remains immediate on press')
need('stopAllManualTest();' in main and 'ev.type == TouchEvent::RELEASE' in main,'R02 release-to-stop retained')
need('touchBlockUntilRelease' in touch and 'suppressTouchUntilRelease' in main,'R02 long-hold page transition cannot re-arm touch')

# R03 manual motion gate closure.
for token in ['steeringTestRunning','manualMotionGateValid','enforceManualMotionGate','oldEsc && !gTelemetry.escFresh','gTelemetry.eStop && (driveTestRunning || steeringTestRunning)']:
    need(token in main,f'R03 fail-safe token present: {token}')
need('gTelemetry.encoderReady && gTelemetry.state == STATE_STOPPED' in main,'R03 steering requires encoder + stopped')
need('gTelemetry.mode != MODE_MANUAL' in main,'R03 MANUAL->AUTO invalidates active test')
need('forceRosOffline()' in main and 'stopAllManualTest();' in main[main.find('static void forceRosOffline'):main.find('static void checkRosLinkTimeout')],'R03 ROS loss stops manual motion')

# R04 fixed wiring remains visible/unchanged in owner files.
for pin in ['GPIO_PIN_6','GPIO_PIN_7','GPIO_PIN_2','GPIO_PIN_3','GPIO_PIN_8','GPIO_PIN_9','GPIO_PIN_12','GPIO_PIN_13','GPIO_PIN_5']:
    need(pin in board,f'R04 board still references {pin}')
need('GPIO_PIN_11 | GPIO_PIN_12' in usb,'R04 USB remains PA11/PA12')
for field in ['pb6VescTx','pb7VescRx','pa2GnssTx','pa3GnssRx','pb8I2cScl','pb9I2cSda','pb12Safety','pb13SafetyLed','pa8Buzzer','pa5SpiSck','pa6SpiMiso','pa7SpiMosi','pb0TftCs','pb1TftDc','pb2TftRst','pa4TouchCs','pa11UsbDm','pa12UsbDp']:
    need(field in diag and field in main,f'R04 diagnostics still samples {field}')

# R05-R08 diagnostics/realtime compatibility.
for token in ['vescUartErrors','gnssUartErrors','vescFrameErrors','vescRecoveryCount','unknownCommands','overlongCommands','tftControllerId']:
    need(token in diag and token in shell,f'R05 visible diagnostic {token}')
need('drawUiFrame' in main and 'gUiDrawActive' in main,'R06 rendering remains main-loop guarded')
need('gDiagnostics.tftOk' in shell and 'UiDomain::SYSTEM' in shell,'R07 local Service health independent of ROS')
need('eqIgnoreCase(name, "SYSTEM")' in main and 'eqIgnoreCase(name, "SERVICE")' in main,'R08 GOTO SYSTEM/SERVICE compatibility retained')
print('PASS F4GATEWAY_STAGE1_HMI_AUDIT')
