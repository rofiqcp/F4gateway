#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).resolve().parents[1]
cpp=(root/'src/Neo3ProSensors.cpp').read_text()
h=(root/'src/Neo3ProSensors.h').read_text()
db=(root/'src/DroneCanDnaDatabase.cpp').read_text()
dbh=(root/'src/DroneCanDnaDatabase.h').read_text()
boot=(root/'bootloader/src/main.c').read_text()
ld=(root/'linker/STM32F411CEUX_APP.ld').read_text()
checks={
'APP stops before DNA sector':'LENGTH = 0x38000' in ld,
'DNA sector reserved':'kStorageBase = 0x08040000UL' in dbh and 'kStorageLimit = 0x08060000UL' in dbh,
'boot updater preserves sector6':'FLASH_SECTOR_5' in boot and 'FLASH_SECTOR_6' not in boot.split('static bool begin_update',1)[1].split('static bool program_chunk',1)[0],
'true separate commit word':'commit_word' in dbh and 'address + 16U' in db and 'memcmp(reinterpret_cast<const void *>(address), &jr, 16U)' in db,
'ArduPilot FNV offset':'14695981039346656037ULL' in dbh,
'ArduPilot FNV prime':'1099511628211ULL' in dbh,
'ArduPilot 56bit historical fold':'hash >> 56U' in db and '1ULL << 56U' in db,
'ArduPilot CRC8 poly07':'0x07U' in db,
'allocates highest free':'for (int id = kMaxNodeId; id > 0; --id)' in db,
'preferred node ignored':'(void)requested' in cpp,
'UID mapping persistent':'handleAllocation(dna_unique_id_)' in cpp,
'duplicate node fails closed':'DnaServerState::DUPLICATE_NODES' in cpp and 'PeerState::DUPLICATE_NODE' in cpp,
'node unhealthy fails closed':'DnaServerState::NODE_STATUS_UNHEALTHY' in cpp,
'periodic verification 5 sec':'5000U' in cpp and 'serviceDnaVerification(now_ms)' in cpp,
'seen verified healthy separated':'dna_seen_' in h and 'dna_verified_' in h and 'dna_healthy_' in h,
'GetNodeInfo UID checked':'uidMatchesNode(source, next.unique_id' in cpp,
'CAN reset not peer failsafe':'next = PeerState::PEER_LOST' in cpp and 'next = PeerState::BUS_FAULT' in cpp,
'DNA diagnostics':'SENS:DNASRV:' in cpp,
}
failed=[k for k,v in checks.items() if not v]
for k,v in checks.items(): print(('PASS' if v else 'FAIL'),k)
if failed: raise SystemExit('FAILED: '+', '.join(failed))
print('NEO3PRO_ARDUPILOT_STAGE2_SELF_CHECK_PASS')
