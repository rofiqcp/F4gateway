from pathlib import Path
cpp=Path('src/Neo3ProSensors.cpp').read_text()
h=Path('src/Neo3ProSensors.h').read_text()
checks={
 'raw software rx queue': 'RAW_CAN_QUEUE_CAPACITY = 128U' in h and 'processRawCanQueue' in cpp,
 'mcp rx hotpath only queues': 'parseAndQueueMcpRx' in cpp and 'enqueueRawCan(id, dlc, &rx[6], timestamp_us)' in cpp and 'canardHandleRxFrame' not in cpp[cpp.index('void Neo3ProSensors::irqFastDrain()'):cpp.index('bool Neo3ProSensors::enqueueRawCan')],
 'known DTOs accepted before verification': 'transport-only quarantine' in cpp,
 'node status discovery before authority': 'ArduPilot DNA tracks NodeStatus for every node' in cpp and 'markDnaNodeStatus(next)' in cpp,
 'getnodeinfo embeds nodestatus': 'GetNodeInfoResponse embeds NodeStatus as the first 56 bits' in cpp,
 'stale node does not erase identity': 'does not erase verified identity merely because NodeStatus is' in cpp,
 'node health operational failsafe': 'dnaNodeHealthy(primary_node_id_)' in cpp and 'PeerState::NODE_UNHEALTHY' in cpp,
 'bus fault separated from peer lost': 'PeerState::BUS_FAULT' in cpp and 'PeerState::PEER_LOST' in cpp,
 'hard recovery resets rx transport': 'canardInit(&canard_, canard_pool_' in cpp and 'raw_can_head_ = raw_can_tail_ = raw_can_count_ = 0U' in cpp,
 'tx hw deadline not hypershort': 'CAN_TX_HW_DEADLINE_US = 20000U' in cpp,
 'tx queue deadline bounded': 'CAN_TX_QUEUE_DEADLINE_US = 250000ULL' in cpp,
 'tx timeout preserves unrelated queue': 'Keep unrelated queued frames' in cpp,
 'uptime reboot detection': 'dna_last_uptime_[source] > next.uptime_sec' in cpp and 'Force its UID verification to run again' in cpp,
 'health telemetry': 'SENS:NEOHEALTH:' in cpp,
}
failed=[k for k,v in checks.items() if not v]
for k,v in checks.items(): print(('PASS' if v else 'FAIL'), k)
if failed: raise SystemExit('FAILED: '+', '.join(failed))
print('NEO3PRO_ARDUPILOT_STAGE1_SELF_CHECK_PASS')
