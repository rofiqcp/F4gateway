#!/usr/bin/env python3
from pathlib import Path
import re, sys
root=Path(__file__).resolve().parents[1]
main=(root/'src/main.cpp').read_text()
board=(root/'src/BoardSupport.cpp').read_text()
boardh=(root/'src/BoardSupport.h').read_text()
usb=(root/'src/usb/UsbCdcPort.cpp').read_text()
usbh=(root/'src/usb/UsbCdcPort.h').read_text()

def need(ok,msg):
    if not ok: raise SystemExit('FAIL: '+msg)
    print('PASS:',msg)

# Direct-ESC production: F411 contains no motor transport implementation at all.
need('F4_ESC_GATEWAY' not in main+board+boardh,'legacy F4 ESC feature switch removed')
need('gVesc' not in main+board+boardh,'legacy F4 VESC objects removed')
need('USART1_IRQHandler' not in board and 'huart1' not in board+boardh,'F411 USART1 motor ownership removed')
need(not (root/'src/VescGateway.cpp').exists() and not (root/'src/VescGateway.h').exists(),'legacy VescGateway sources deleted')
need('ERR:VESC:DIRECT_ESC_ONLY' in main,'stale VESC-over-F411 commands fail closed')

# Every destructive USB queue reset advances a generation observed by safety.
reset=usb[usb.index('void UsbCdcPort::resetSessionState'):usb.index('bool UsbCdcPort::startUsbStack')]
need('if (drop_queues)' in reset and '++usb_session_generation_' in reset,'queue-destructive reset advances transport generation')
repair=usb[usb.index('void UsbCdcPort::service()'):usb.index('void UsbCdcPort::poll()')]
need('hcdc->TxState == 0U' in repair and 'resetSessionState(true, true);' in repair,'endpoint split-brain repair is queue-destructive')
state=main[main.index('static void serviceUsbTransportState'):main.index('static void serviceSafetyControlTx')]
need('transportGeneration()' in state and 'latchAllSafetyStops();' in state,'transport generation re-latches all STOP controls')

# USB physical/session loss revokes local authority without heartbeat grace.
need('hostSessionEstablished()' in usbh and 'confirmHostSession()' in usbh and 'invalidateHostSession()' in usbh,'explicit host-session authority state exists')
for fn in ('onUsbClassInit','onUsbClassDeInit','softRestartUsb'):
    start=usb.index('UsbCdcPort::'+fn)
    tail=usb[start:start+500]
    need('invalidateHostSession();' in tail,f'{fn} invalidates host session')
manual=main[main.index('static bool manualMotionGateValid'):main.index('static void enforceManualMotionGate')]
need('gUsb.connected()' in manual and 'gUsb.hostSessionEstablished()' in manual,'manual motion requires physical USB + established host session')
need('!transport_authoritative' in state and 'forceRosOffline();' in state,'transport authority loss revokes ROS authority immediately')
off=main[main.index('static void forceRosOffline() {'):main.index('static void checkRosLinkTimeout')]
need('gNavStopPending = true;' in off and 'gTelemetry.navigationStatus = NAV_STOPPED;' in off,'transport/ROS loss closes autonomous state fail-safe')

hello=main[main.index('if (!std::strncmp(command, "HOST:HELLO:"'):main.index('if (!strcmp(command, "GET:STATE"')]
need('gUsb.confirmHostSession();' in hello,'host session becomes authoritative only after ACK enqueue')
need('gUsb.invalidateHostSession();' in hello,'failed ACK leaves host session invalid')
need('host_est=%u' in main,'host-session authority is externally observable')
print('F4_STAGE1_TRANSPORT_SAFETY_SELF_CHECK_PASS')
