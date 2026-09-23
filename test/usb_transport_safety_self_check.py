#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
main = (root/'src/main.cpp').read_text()
handler = (root/'src/F103MinimalCommandHandler.inc').read_text()
board = (root/'src/BoardSupport.cpp').read_text()
boardh = (root/'src/BoardSupport.h').read_text()
usb = (root/'src/usb/UsbCdcPort.cpp').read_text()
usbh = (root/'src/usb/UsbCdcPort.h').read_text()

def need(ok, msg):
    if not ok:
        raise SystemExit('FAIL: ' + msg)
    print('PASS:', msg)

production = main + handler + board + boardh
need('gVesc' not in production and 'VESC:' not in handler,
     'obsolete VESC gateway command path removed')
need('USART1_IRQHandler' not in board and 'huart1' not in board + boardh,
     'gateway owns no obsolete USART1 motor transport')
need(not (root/'src/VescGateway.cpp').exists() and not (root/'src/VescGateway.h').exists(),
     'legacy VescGateway sources are absent')

reset = usb[usb.index('void UsbCdcPort::resetSessionState'):
            usb.index('bool UsbCdcPort::startUsbStack')]
need('if (drop_queues)' in reset and '++usb_session_generation_' in reset,
     'queue-destructive reset advances transport generation')

need('hostSessionEstablished()' in usbh and
     'confirmHostSession()' in usbh and
     'invalidateHostSession()' in usbh,
     'explicit host-session authority state exists')

hello = handler[handler.index('if (!std::strncmp(command, "HOST:HELLO:"'):
                handler.index('#if defined(BOARD_F103_USBBOOT)')]
need('gUsb.beginHostSession(token);' in hello,
     'host handshake begins an explicit session')
need('gUsb.confirmHostSession();' in hello,
     'host session becomes authoritative only after ACK enqueue')
need('gUsb.invalidateHostSession();' in hello,
     'failed ACK leaves host session invalid')

manual = main[main.index('static bool manualMotionGateValid'):
              main.index('static void serviceManualDriveLease')]
need('gUsb.connected()' in manual and 'gUsb.hostSessionEstablished()' in manual,
     'manual motion requires physical USB and established host session')

print('F103C8_TRANSPORT_SAFETY_SELF_CHECK_PASS')
