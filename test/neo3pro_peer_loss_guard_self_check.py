#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).resolve().parents[1]
cpp=(root/'src/Neo3ProSensors.cpp').read_text()
h=(root/'src/Neo3ProSensors.h').read_text()

def need(ok,msg):
    if not ok: raise SystemExit('FAIL: '+msg)
    print('PASS:',msg)

verify=cpp[cpp.index('void Neo3ProSensors::serviceDnaVerification'):cpp.index('bool Neo3ProSensors::mcpTransfer')]
req=cpp[cpp.index('bool Neo3ProSensors::requestService'):cpp.index('bool Neo3ProSensors::requestGetNodeInfo')]
peer=cpp[cpp.index('void Neo3ProSensors::updatePeerFailsafe'):cpp.index('void Neo3ProSensors::pollSafetyIo')]
need('dna_last_seen_ms_[next]' in verify and 'CAN_NODE_STALE_MS' in verify,'periodic verification skips stale peers')
need('dna_verified_[dna_curr_verifying_node_] = false' not in verify,'missed maintenance response does not erase persistent identity')
need('dna_last_seen_ms_[destination_node_id]' in req and 'CAN_NODE_STALE_MS' in req,'all host service TX is freshness-gated')
need('abortMcpTxBounded()' in peer and 'clearCanardTxQueue()' in peer,'PEER_LOST aborts and purges stale TX')
need('can_peer_loss_tx_purge_count_' in peer and 'can_silent_recovery_count_' in peer,'peer-loss containment is observable')
need('recoverCan();' in peer and 'if (next == PeerState::PEER_LOST)' in peer,'silent-bus controller recovery is bounded to loss transition')
need('SENS:CANGUARD:' in cpp,'CAN guard telemetry is externally observable')
need('dna_last_uptime_[source] > next.uptime_sec' in cpp and 'dna_verified_[source] = false' in cpp,'real node reboot still forces identity re-verification')
need('can_peer_loss_tx_purge_count_' in h and 'can_silent_recovery_count_' in h,'guard counters persist in driver state')
print('NEO3PRO_PEER_LOSS_GUARD_SELF_CHECK_PASS')
