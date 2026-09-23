#!/usr/bin/env python3
from pathlib import Path

root=Path(__file__).resolve().parents[1]
main=(root/'src/main.cpp').read_text()
handler=(root/'src/F103MinimalCommandHandler.inc').read_text()
usb=(root/'src/usb/UsbCdcPort.cpp').read_text()
usbh=(root/'src/usb/UsbCdcPort.h').read_text()
tft=(root/'src/HmiDisplay.cpp').read_text()
diag=(root/'src/HmiDiagnostics.h').read_text()
tp=(root/'src/TelemetryProtocolF103.cpp').read_text()

def need(ok,msg):
    if not ok: raise SystemExit('FAIL: '+msg)
    print('PASS:',msg)

hello=handler[handler.index('if (!std::strncmp(command, "HOST:HELLO:"'):
              handler.index('#if defined(BOARD_F103_USBBOOT)')]
need(hello.index('writeLineHighPriority(ack)') < hello.index('forceRosOffline()'),
     'host ACK is queued before fail-safe ROS reset')
need('gNavStopPending = true;' in main[main.index('static void forceRosOffline'):
                                        main.index('static void checkRosLinkTimeout')],
     'ROS/session loss latches NAV STOP')
need('serviceUsbTransportState' in main and 'transportGeneration()' in main,
     'USB generation change re-latches safety STOP')
need('trim_after_active_message' in usb and
     'tx_high_' in usb[usb.index('void UsbCdcPort::beginHostSession'):
                        usb.index('uint32_t UsbCdcPort::txBusyAgeMs')],
     'session purge preserves message boundary')
need('highSessionPurgeCount' in usbh, 'high-priority session purge is observable')
need('sessionChanged = previous != 0U && previous != session' in tp and 'result.sessionChanged=ok && changed' in tp,
     'telemetry session transition remains explicit')
need('sx = (width_ - 1) - sx;' in tft and 'sy = (height_ - 1) - sy;' in tft,
     'touch inversion has no edge off-by-one')
need('pb12McpCs' not in diag and 'pb10McpInt' not in diag,
     'obsolete MCP diagnostics are absent')
need('Board_RealtimeService();' in usb[usb.index('void UsbCdcPort::flush'):
                                      usb.index('void UsbCdcPort::onReceive')],
     'USB flush remains cooperative')

need('"WINCH:CMD:"' in main and 'handleTransactionalWinchCommand(command)' in handler,
     'transactional winch command path exists')
need('ACK:WINCH:%lu:%s' in main and 'ERR:WINCH:%lu:%s' in main,
     'transactional winch ACK/ERR preserves request ID')
need('executeWinchCommand(command,0U,false)' in handler,
     'legacy and transactional winch paths share one safety executor')
need(handler.index('if (!gUsb.hostSessionEstablished())') <
     handler.index('if (handleTransactionalWinchCommand(command))'),
     'transactional winch mutation requires established host session')
print('F103C8_GATEWAY_REGRESSION_SELF_CHECK_PASS')
