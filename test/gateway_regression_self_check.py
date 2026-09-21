#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).resolve().parents[1]
main=(root/'src/main.cpp').read_text(); usb=(root/'src/usb/UsbCdcPort.cpp').read_text()
usbh=(root/'src/usb/UsbCdcPort.h').read_text(); board=(root/'src/BoardSupport.cpp').read_text()
tft=(root/'src/HmiDisplay.cpp').read_text(); diag=(root/'src/HmiDiagnostics.h').read_text(); tp=(root/'src/TelemetryProtocol.cpp').read_text()

def need(ok,msg):
    if not ok: raise SystemExit('FAIL: '+msg)
    print('PASS:',msg)

start=main.index('if (!std::strncmp(command, "HOST:HELLO:"'); end=main.index('if (!strcmp(command, "GET:STATE"', start); h=main[start:end]
need(h.index('writeLineHighPriority(ack)') < h.index('forceRosOffline()'),'host ACK queued before fail-safe reset')
need('gNavStopPending = true;' in main[main.index('static void forceRosOffline'):main.index('static void checkRosLinkTimeout')],'ROS/session loss latches NAV STOP')
need('serviceUsbTransportState' in main and 'transportGeneration()' in main,'USB generation change re-latches safety STOP')
need('trim_after_active_message' in usb and 'tx_high_' in usb[usb.index('void UsbCdcPort::beginHostSession'):usb.index('uint32_t UsbCdcPort::txBusyAgeMs')],'session purge preserves message boundary')
need('highSessionPurgeCount' in usbh,'high-priority session purge observable')
need('legacyTelemetryPayloadValid' in main and 'legacyTelemetryMalformed' in main,'legacy telemetry validation retained')
need('parseFiniteLegacyRealStrict' in main and 'parseBoolStrict' in main,'parser rejects malformed scalar/bool values')
need('previous_session != h.session' in tp and 'result.sessionChanged = ok && is_v3' in tp,'telemetry session transition remains explicit')
need('sx = (width_ - 1) - sx;' in tft and 'sy = (height_ - 1) - sy;' in tft,'touch inversion has no edge off-by-one')
need('pb12McpCs' not in diag and 'pb10McpInt' not in diag,'obsolete MCP diagnostics removed')
need('safety_retry=' in main and 'safety_queued=' in main,'safety retry/queue counters observable')
need('Board_RealtimeService();' in usb[usb.index('void UsbCdcPort::flush'):usb.index('void UsbCdcPort::onReceive')],'USB flush remains cooperative')
print('F4_POST_AUDIT_REGRESSION_SELF_CHECK_PASS')
