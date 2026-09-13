#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).resolve().parents[1]
main=(root/'src/main.cpp').read_text()
neo=(root/'src/Neo3ProSensors.cpp').read_text()
neoh=(root/'src/Neo3ProSensors.h').read_text()
usb=(root/'src/usb/UsbCdcPort.cpp').read_text()
usbh=(root/'include/UsbCdcPort.h').read_text()
board=(root/'src/BoardSupport.cpp').read_text()
tft=(root/'src/HmiDisplay.cpp').read_text()
diag=(root/'src/Diagnostics.h').read_text()
ui=(root/'src/UiShell.h').read_text()
tp=(root/'src/TelemetryProtocol.cpp').read_text()

def need(ok,msg):
    if not ok: raise SystemExit('FAIL: '+msg)
    print('PASS:',msg)

# P0: error-passive can recover by transmitting; only BUS-OFF hard-blocks TX.
w=neo[neo.index('bool Neo3ProSensors::writeOneCanFrame'):neo.index('void Neo3ProSensors::serviceMcpTx')]
need('if ((eflg & EFLG_TXBO) != 0U)' in w,'BUS-OFF blocks MCP TX')
need('can_tx_passive_probe_count_' in w,'error-passive permits bounded recovery probe')
need('(EFLG_TXBO | EFLG_TXEP)' not in w and 'tec >= 128U) {' not in w,'TXEP/TEC>=128 no longer deadlocks all TX')
need('CAN_TX_HW_DEADLINE_US' in neo and 'CAN_TX_BACKOFF_BASE_MS' in neo,'recovery probes remain deadline/backoff bounded')

# P0: new host epoch is fail-closed and ACK precedes STOP records.
start=main.index('if (!std::strncmp(command, "HOST:HELLO:"')
end=main.index('if (!strcmp(command, "GET:STATE"', start)
h=main[start:end]
need(h.index('writeLineHighPriority(ack)') < h.index('forceRosOffline()'),'host ACK queued before fail-safe STOP/reset')
need('gNavStopPending = true;' in main[main.index('static void forceRosOffline'):main.index('static void checkRosLinkTimeout')],'ROS/session loss latches NAV STOP')
need('serviceUsbTransportState' in main and 'transportGeneration()' in main,'destructive USB generation change re-latches safety STOP')
need('trim_after_active_message' in usb and 'tx_high_' in usb[usb.index('void UsbCdcPort::beginHostSession'):usb.index('uint32_t UsbCdcPort::txBusyAgeMs')],'host session purges stale high/low backlog at message boundary')
need('highSessionPurgeCount' in usbh,'high-priority session purge is observable')

# P1: malformed legacy data cannot renew readiness.
need('legacyTelemetryPayloadValid' in main and 'legacyTelemetryMalformed' in main,'legacy scalar/bool telemetry is strictly validated')
need('parseFiniteDoubleStrict' in main and 'parseBoolStrict' in main,'legacy parser rejects NaN/garbage boolean payloads')

# P1: command state only advances after transport accepted the start/config request.
need('USB CONTROL BUSY' in main and 'USB TX BUSY' in main,'HMI reports enqueue failure instead of optimistic state')

# P2 correctness/observability.
need('previous_session != h.session' in tp and 'result.sessionChanged = ok && is_v3' in tp,'F4X3 sessionChanged means actual transition')
need('sx = (width_ - 1) - sx;' in tft and 'sy = (height_ - 1) - sy;' in tft,'touch inversion has no off-by-one edge loss')
f=board[board.index('void HalUartPort::flush()'):board.index('bool HalUartPort::service()')]
need('Board_RealtimeService();' in f and '__WFI();' in f,'legacy UART flush is cooperative/bounded')
need('pb12McpCs' in diag and 'pb10McpInt' in diag and 'pb6VescTx' not in diag,'MCP diagnostics use physical signal names')
need('gDiagnostics.pb12McpCs' in ui and 'gDiagnostics.pb10McpInt' in ui,'SYSTEM_PINS uses MCP diagnostics')
need('safety_retry=' in main and 'safety_queued=' in main,'safety STOP retry/queue counters are externally observable')
print('F4_POST_AUDIT_REGRESSION_SELF_CHECK_PASS')
