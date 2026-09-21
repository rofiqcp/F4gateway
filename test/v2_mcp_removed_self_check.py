#!/usr/bin/env python3
from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]

def need(ok, msg):
    if not ok:
        raise SystemExit('FAIL ' + msg)
    print('PASS ' + msg)

production = []
for base in ('src',):
    for p in (ROOT / base).rglob('*'):
        if p.is_file() and p.suffix in {'.h','.hpp','.c','.cpp'}:
            production.append(p)
production.append(ROOT / 'platformio.ini')
text = '\n'.join(p.read_text(errors='replace') for p in production)
for token in ('MCP2515', 'MCP2515_ENABLED', 'NEO3PRO', 'Neo3ProSensors', 'DroneCanDna', 'libcanard', 'canard'):
    need(token not in text, f'production has no {token}')
need(not (ROOT/'lib').exists(), 'legacy lib directory removed; sources live under src')
need(not (ROOT/'src/Neo3ProSensors.cpp').exists(), 'Neo3Pro MCP driver removed')
need(not (ROOT/'src/DroneCanDnaDatabase.cpp').exists(), 'DroneCAN DNA database removed')
pio=(ROOT/'platformio.ini').read_text()
need('default_envs = bluepill_f103c8' in pio and '[env:bluepill_f103c8]' in pio and '[env:blackpill_f411ce_romdfu]' in pio and 'extends = env:blackpill_f411ce_v2' in pio, 'v3 defaults to F103C8 and retains the v2 F411 ROM-DFU environment')
need('scripts/cdc_boot_upload.py $SOURCE' in pio, 'normal upload remains USB CDC bootloader')
need('-DVECT_TAB_OFFSET=0x00004000U' in pio and 'board_upload.maximum_size = 376832' in pio, 'application bootloader layout unchanged')
f4=(ROOT/'src/HmiOperatorUi.h').read_text(); main=(ROOT/'src/main.cpp').read_text()
need('if (fullRedraw) tft.fillScreen(C_BG2);' in f4, 'HMI full clear is conditional')
need('gOperatorUi.draw(telemetrySnapshot, full);' in main, 'telemetry redraw does not force full-screen clear')
print('V2_MCP_REMOVED_SELF_CHECK_PASS')
