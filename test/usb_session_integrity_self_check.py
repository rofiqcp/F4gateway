#!/usr/bin/env python3
from pathlib import Path
import re
ROOT=Path(__file__).resolve().parents[1]
main=(ROOT/'src/main.cpp').read_text()
usb=(ROOT/'src/usb/UsbCdcPort.cpp').read_text()
usbh=(ROOT/'src/usb/UsbCdcPort.h').read_text()
proto=(ROOT/'src/TelemetryProtocol.cpp').read_text()
protoh=(ROOT/'src/TelemetryProtocol.h').read_text()
diag=(ROOT/'src/HmiDiagnostics.h').read_text()
bridge_path=ROOT.parent/'src/stmf4/src/stmf4_hmi_bridge.cpp'
bridge=bridge_path.read_text() if bridge_path.exists() else ''
def need(ok,msg):
    if not ok: raise SystemExit('FAIL: '+msg)
    print('PASS:',msg)

# Host-session handshake: strict nonzero token + same-token idempotency.
hello=main[main.index('if (!std::strncmp(command, "HOST:HELLO:"'):main.index('if (!strcmp(command, "GET:STATE"')]
need('parseU32Strict(command + 11, token)' in hello and 'token == 0U' in hello,
     'HOST HELLO token is strict uint32 and nonzero')
need('hostSessionEstablished() && gUsb.hostSessionToken() == token' in hello,
     'same-token HELLO has explicit idempotent branch')
dup=hello[hello.index('hostSessionEstablished() && gUsb.hostSessionToken() == token'):hello.index('gUsb.beginHostSession(token)')]
need('writeLineHighPriority(ack)' in dup and 'forceRosOffline()' not in dup and 'beginHostSession' not in dup,
     'same-token HELLO only re-ACKs without epoch reset')
need(hello.index('gUsb.beginHostSession(token)') < hello.index('gUsb.confirmHostSession()'),
     'changed-token HELLO establishes a new session only after queue reset')
need(hello.index('writeLineHighPriority(ack)') < hello.index('gUsb.confirmHostSession()'),
     'new host session becomes authoritative only after ACK enqueue')

# Trust boundary before authoritative commands.
need('commandAllowedBeforeHostSession' in main and 'ERR:HOST:SESSION:REQUIRED' in main,
     'pre-session trust boundary exists')
need('!gUsb.hostSessionEstablished() && !commandAllowedBeforeHostSession(command)' in main,
     'pre-session telemetry/control is rejected')
for safe in ['PING','GET:STATE','USB:STATUS','USB:RECOVER','FAULT:STATUS','FW:INFO','BOOT:DFU:ARM','BOOT:DFU:CONFIRM']:
    need(f'"{safe}"' in main[main.index('static bool commandAllowedBeforeHostSession'):main.index('static void handleSerialCommand(char *command) {', main.index('static bool commandAllowedBeforeHostSession'))],
         f'pre-session safe/recovery command retained: {safe}')

# RX overflow must fail closed at both USB ring and line parser layers.
rx=usb[usb.index('void UsbCdcPort::onReceive'):usb.index('void UsbCdcPort::onTransmitComplete')]
need('rx_discard_until_newline_' in rx and 'rx_tail_ = rx_head_' in rx,
     'USB RX overflow purges buffered fragment and enters newline resync')
need("if (value == '\\n')" in rx and 'rx_resync_complete_count_' in rx,
     'USB RX resumes only after a newline boundary')
need('rxResyncCount()' in usbh and 'gUsb.rxResyncCount()' in main and 'serialRxLen = 0U' in main,
     'main parser clears partial command on USB RX resync event')

# Critical host state bypasses best-effort deferred queue during TFT service.
crit=main[main.index('static bool hostCommandRealtimeCritical'):main.index('static bool commandAllowedBeforeHostSession')]
for prefix in ['HOST:HELLO:','F4X3:','ROS:','MODE:','STATE:','ESTOP:','ESC:','ENC:','VESC_LINK:','MOTION:','NAV2:','NAV:']:
    need(f'"{prefix}"' in crit, f'critical realtime bypass includes {prefix}')
need('gRealtimeParserActive && !transportRealtime' in main and 'enqueueDeferredCommand(command)' in main,
     'best-effort commands remain bounded in deferred queue')

# Domain freshness is v3-only and bound to the current HELLO token.
need('expectedSession' in protoh and 'h.session != expectedSession' in proto and 'sessionError' in protoh,
     'F4X3 parser rejects CRC-valid wrong-session frames')
need('gUsb.hostSessionToken()' in main[main.index('parseExtendedTelemetryLine'):main.index('legacyTelemetryPayloadValid', main.index('parseExtendedTelemetryLine'))],
     'F4X3 parser is bound to current host token')
need('if (extended.v3)' in main and 'markDomainForCommand' not in main,
     'only F4X3 v3 can refresh domain authority')
need('extendedTelemetrySessionErrors' in diag and 'extended.sessionError' in main,
     'wrong-session telemetry is observable')

# Config ACK parser must not wrap uint32 into uint16.
cfg=main[main.index('static void parseConfigResult'):main.index('static void markRosHeartbeat')]
need('parseU32Strict' in cfg and 'txn32 <= 0xFFFFU' in cfg and 'configAckMalformed' in cfg,
     'config ACK transaction id is strict uint16 without wrap')

# Bridge side: F4X3 session == HELLO token and critical legacy state self-heals.
if bridge:
    need('resetExtendedTelemetrySession(host_session_token_)' in bridge,
         'bridge binds F4X3 session to acknowledged HOST token')
    need('parseU32DecimalStrict(generation_text, transport_generation)' in bridge and 'transport_generation == 0U' in bridge,
         'bridge strictly validates ACK transport generation')
    reset=bridge[bridge.index('void resetExtendedTelemetrySession'):bridge.index('void extendedTelemetryTick')]
    need('extended_session_id_ = session' in reset and 'escx_power_seq_' in reset and 'navx_ctrl_seq_' in reset,
         'bridge resets all F4X3 sequences on host epoch change')
    need('if (awaiting_host_session_ || extended_session_id_ == 0U) return;' in bridge,
         'bridge never emits F4X3 before host synchronization')
    tele=bridge[bridge.index('void telemetryTick()'):bridge.index('void publishConnected')]
    need('force_critical' in tele and 'std::chrono::seconds(1)' in tele,
         'critical legacy state has low-rate periodic refresh')
    for key in ['SYS','MODE','STATE','ESTOP','VESC_LINK','ESC','ENC','MOTION','NAV2']:
        need(re.search(rf'sendState\("{key}".*force_critical\);', tele) is not None,
             f'periodic critical refresh covers {key}')
    need('mirrorWaypointState(force, force_critical)' in tele,
         'navigation state receives periodic critical refresh')

else:
    print('PASS: external ROS bridge not present; firmware session contract remains self-contained')
print('F4_STAGE2_SESSION_INTEGRITY_SELF_CHECK_PASS')
