#!/usr/bin/env python3
from pathlib import Path
import re, sys
root=Path(__file__).resolve().parents[1]
main=(root/'src/main.cpp').read_text()
board=(root/'src/BoardSupport.cpp').read_text()
boardh=(root/'include/BoardSupport.h').read_text()
usb=(root/'src/usb/UsbCdcPort.cpp').read_text()
usbh=(root/'include/UsbCdcPort.h').read_text()
vesc=(root/'src/VescGateway.cpp').read_text()

def need(ok,msg):
    if not ok: raise SystemExit('FAIL: '+msg)
    print('PASS:',msg)

# Direct-ESC production must have no executable legacy transport path.
need('DisabledVescGateway' not in main,'no misleading runtime-disabled VESC stub remains')
for call in ('gVesc.begin();','gVesc.poll();','gVesc.setSafetyStop(','gVesc.maintenanceMode()'):
    positions=[m.start() for m in re.finditer(re.escape(call),main)]
    for pos in positions:
        guard=main.rfind('#if F4_ESC_GATEWAY',0,pos)
        endif=main.rfind('#endif',0,pos)
        need(guard > endif, f'{call} guarded by F4_ESC_GATEWAY')
need(re.search(r'#if F4_ESC_GATEWAY\s*\nUART_HandleTypeDef huart1',board) is not None,'USART1 handle compile-time gated')
need(re.search(r'#if F4_ESC_GATEWAY\s*\nHalUartPort gVescUart',board) is not None,'legacy VESC UART object compile-time gated')
need(re.search(r'#if F4_ESC_GATEWAY\s*\nextern "C" void USART1_IRQHandler',board) is not None,'USART1 IRQ compile-time gated')
need('#if F4_ESC_GATEWAY' in vesc[:200] and vesc.rstrip().endswith('#endif // F4_ESC_GATEWAY'),'legacy VESC translation unit hard-gated')
need('#if F4_ESC_GATEWAY\nextern UART_HandleTypeDef huart1;' in boardh,'legacy UART declaration hard-gated')

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
