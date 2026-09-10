#!/usr/bin/env python3
from pathlib import Path
import sys
root=Path(__file__).resolve().parents[1]
menu=(root/'src/UiMenu.h').read_text(); shell=(root/'src/UiShell.h').read_text(); cfg=(root/'src/Config.h').read_text(); proto=(root/'src/TelemetryProtocol.cpp').read_text(); bridge=Path('/home/sirobo/agv/src/stmf4/src/stmf4_hmi_bridge.cpp').read_text()
def need(c,m):
    if not c: raise SystemExit('FAIL '+m)
    print('PASS',m)
need('count = 10U; return system' in menu,'10-card Service catalog')
for x in ['SYSTEM_SPI_BUS','SYSTEM_UART_STATUS','SYSTEM_POWER_STATUS','SYSTEM_TOUCH_PANEL','SYSTEM_TFT_TEST']:
    need(x in menu and x in shell,'Service renderer '+x)
need('SERVICE_HOLD_MS = 800' in cfg,'responsive 0.8 s Service hold')
need('VisualAssets' not in (root/'src/Icons.h').read_text(),'large RGB565 assets removed')
need('WAIT STAGE 2' not in shell and 'READ ONLY STAGE 1' not in shell,'no Stage-1/2 placeholders remain')
for pfx in ['ESCX','PERX','NAVX']:
    need(pfx in proto and ('send_ext("'+pfx+'"') in bridge,'bridge/parser contract '+pfx)
for topic in ['/esc/foc/telemetry','/yolop/lane_metrics','/perception/obstacle_metrics','/odometry/filtered','/odometry/filtered_map','/local_costmap/costmap','/navigation/mppi_closed_loop/status','/navigation/trajectory_safety_state']:
    need(topic in bridge,'authoritative source '+topic)
need('encoded.size() <= 220U' in bridge and 'kMaxExtendedLine = 220U' in proto,'220-byte wire bound symmetric')
print('PASS F4GATEWAY_STAGE2_CONTRACT')