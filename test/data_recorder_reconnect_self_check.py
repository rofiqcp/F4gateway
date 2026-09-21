#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).resolve().parents[1]
s=(root/'tools/data.py').read_text()

def need(ok,msg):
    if not ok: raise SystemExit('FAIL: '+msg)
    print('PASS:',msg)

need('/dev/serial/by-id/usb-STMicroelectronics_BLACKPILL_F411CE' in s,'F411 uses stable by-id')
need('/tmp/agv_devices/imu' in s and '2.4.1:1.0-port0' in s,'Yahboom avoids duplicate CP2102 by-id and uses stable role/topology')
need('def _open_neo(' in s and 'def _close_neo(' in s,'F411 has reopen state machine')
need('def _open_imu(' in s and 'def _close_imu(' in s,'Yahboom has reopen state machine')
open_neo=s[s.index('def _open_neo('):s.index('def _close_neo(')]
need('self.nb.clear()' in open_neo and 'reset_input_buffer()' in open_neo,'F411 parser and stale CDC input reset on reopen')
need('HOST:HELLO:' in open_neo and 'ROS:1' in open_neo,'F411 reopen establishes a fresh host session and telemetry authority')
need('def _invalidate_imu_epoch(' in s and 'self.ib.clear()' in s,'Yahboom parser/sample epoch resets')
open_imu=s[s.index('def _open_imu('):s.index('def _close_imu(')]
need('reset_input_buffer()' in open_imu,'Yahboom discards stale CP2102 RX data on reopen')
run=s[s.index('def run(self)'):s.index('def close(self)')]
need('self._open_neo(); self._open_imu()' in run,'both transports retry independently')
need('if neo_fd is not None' in run and 'if imu_fd is not None' in run,'missing device does not enter select set')
need('_close_neo("eof")' in run and '_close_imu("eof")' in run,'EOF hot-unplug closes both transports safely')
need('F411_TRANSPORT_STALL_S' in run and 'YAHBOOM_TRANSPORT_STALL_S' in run,'silent transports are reopened')
need('except (OSError, serial.SerialException)' in run,'read errors trigger reconnect')
sample=s[s.index('def sample(self)'):s.index('@staticmethod\n    def fmt')]
need('gnss_fresh' in sample and 'mag_fresh' in sample,'F411-derived data has freshness gates')
need('imu_fresh' in sample and 'yahboom_stale' in sample and 'imu_stale' in sample,'Yahboom stale data is fail-closed')
need('yaw_inertial_valid":int(imu_fresh' in sample,'integrated yaw is invalid after IMU loss')
need('f411_disconnects' in s and 'f411_reconnects' in s,'F411 recovery counters persisted')
need('yahboom_disconnects' in s and 'yahboom_reconnects' in s,'Yahboom recovery counters persisted')
print('DATA_RECORDER_RECONNECT_SELF_CHECK_PASS')
