#!/usr/bin/env python3
"""Direct F411 + Yahboom acquisition logger.

Transport:
  F411 CDC : /dev/ttyACM0 @ 1,000,000 baud (SENS:* lines)
  Yahboom  : /dev/ttyUSB0 @ 921,600 baud (WIT 0x55 frames)

The summary.csv schema is shared with ``ros2 run navigation data`` so direct
hardware and ROS-pipeline captures can be compared column-for-column.
All original F411 lines and Yahboom frames are also retained in raw.jsonl.
"""
from __future__ import annotations
import argparse, csv, fcntl, json, math, os, select, shutil, signal, socket, subprocess, sys, time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict
import serial
import yaml

AGV_ROOT = Path(os.environ.get("AGV_ROOT", str(Path(__file__).resolve().parents[2]))).expanduser().resolve()
F411_BY_ID = "/dev/serial/by-id/usb-STMicroelectronics_BLACKPILL_F411CE_CDC_in_FS_Mode_338133833134-if00"
YAHBOOM_BY_ID = "/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0"
F411_REOPEN_BACKOFF_S = 0.50
F411_DATA_STALE_S = 1.50
F411_TRANSPORT_STALL_S = 3.00
YAHBOOM_REOPEN_BACKOFF_S = 0.50
YAHBOOM_DATA_STALE_S = 1.00
YAHBOOM_TRANSPORT_STALL_S = 3.00
MAG_DATA_STALE_S = 1.00
sys.path.insert(0, str(AGV_ROOT / "tools"))
from data_common import (  # noqa: E402
    UNIFIED_FIELDS, G, NAN, apply_lut, crc16_ccitt, deg, enu_from_origin,
    finite, geodetic_to_ecef, norm_angle, parse_v2_payload, wrap360,
)


def i16(lo: int, hi: int) -> int:
    v = lo | (hi << 8)
    return v - 65536 if v & 0x8000 else v


def iso_utc(ns: int) -> str:
    return datetime.fromtimestamp(ns / 1e9, tz=timezone.utc).isoformat(timespec="microseconds")


def sha256_file(path: Path) -> str:
    import hashlib
    h = hashlib.sha256()
    with path.open("rb") as f:
        for chunk in iter(lambda: f.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


class DirectRecorder:
    def __init__(self, args: argparse.Namespace):
        self.a = args
        self.running = True
        self.root = AGV_ROOT
        self.f4root = self.root / "F4gateway"
        self.nav_cfg = self.root / "src/navigation/config"
        self.mag_cfg = self._load_yaml_params(self.nav_cfg / "mag_heading.yaml", "mag_heading_fusion")
        self.imu_cfg = self._load_yaml_params(self.nav_cfg / "imu.yaml", "data_imu_node")
        self.start_wall_ns = time.time_ns(); self.start_mono_ns = time.monotonic_ns(); self.sample_seq = 0
        self.lock = open("/tmp/agv_f4_direct_data.lock", "w")
        try:
            fcntl.flock(self.lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            raise SystemExit("F4 direct data logger sudah berjalan")
        if args.takeover:
            self._takeover(args.neo_port); self._takeover(args.imu_port)
        # Both USB transports are reopenable. F411 may re-enumerate during
        # DFU/recovery and Yahboom's CP2102 may be unplugged/replugged during a
        # long field capture. Missing hardware must not terminate acquisition.
        self.neo = None
        self.imu = None
        self.nb = bytearray(); self.ib = bytearray()
        self.outdir = self._create_outdir(args.output_dir)
        self.raw_path = self.outdir / "raw.jsonl"; self.summary_path = self.outdir / "summary.csv"; self.meta_path = self.outdir / "metadata.json"
        self.raw_f = self.raw_path.open("w", encoding="utf-8", buffering=1)
        self.sum_f = self.summary_path.open("w", encoding="utf-8", newline="", buffering=1)
        self.writer = csv.DictWriter(self.sum_f, fieldnames=UNIFIED_FIELDS); self.writer.writeheader()
        self._snapshot_configs(); self._write_meta(True)
        self.origin = None; self.gnss_enu = (NAN, NAN, NAN)
        self.gnss = {}; self.gnsspro = {}; self.gnss_heading = NAN
        self.neo_mag = [NAN, NAN, NAN]; self.neo_yaw = NAN; self.neo_valid = False; self.neo_norm = NAN
        self.imu_acc_raw = [None]*3; self.imu_acc = [NAN]*3
        self.imu_gyro_raw = [None]*3; self.imu_gyro_dps = [NAN]*3; self.imu_gyro_rps = [NAN]*3
        self.imu_angle_raw = [None]*3; self.imu_rpy = [NAN]*3
        self.yah_mag_raw = [None]*3; self.yah_q = [NAN, NAN]; self.yah_norm = NAN; self.yah_yaw = NAN; self.yah_valid = False
        self.inertial_yaw = NAN; self.inertial_last_t = None; self.inertial_source = "none"
        self.baro_pressure = NAN; self.baro_temp = NAN
        self.last_neo_crc_ok = ""; self.last_yah_checksum_ok = ""
        self.bad_neo_crc = 0; self.bad_yah_checksum = 0
        self.counts = dict(f4=0, raw=0, gnss=0, neo_mag=0, acc=0, gyro=0, angle=0, imu_mag=0)
        self.last_display = 0.0
        self.neo_last_rx_mono = 0.0
        self.neo_last_gnss_mono = 0.0
        self.neo_last_mag_mono = 0.0
        self.neo_next_open_mono = 0.0
        self.neo_disconnects = 0
        self.neo_reconnects = 0
        self.imu_last_rx_mono = 0.0
        self.imu_last_valid_mono = 0.0
        self.imu_next_open_mono = 0.0
        self.imu_disconnects = 0
        self.imu_reconnects = 0
        self._open_neo(initial=True)
        self._open_imu(initial=True)

    @staticmethod
    def _load_yaml_params(path: Path, key: str) -> Dict[str, Any]:
        try:
            return yaml.safe_load(path.read_text())[key]["ros__parameters"]
        except Exception:
            return {}

    @staticmethod
    def _takeover(dev: str) -> None:
        subprocess.run(["fuser", "-k", dev], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.3)

    def _open_neo(self, initial: bool = False) -> bool:
        if self.neo is not None:
            return True
        now = time.monotonic()
        if now < self.neo_next_open_mono:
            return False
        try:
            self.neo = serial.Serial(self.a.neo_port, self.a.neo_baud, timeout=0, exclusive=True)
            # A CDC endpoint can retain bytes queued before this logger became the
            # host. Start every acquisition/reconnect at a clean transport epoch
            # so stale/partial telemetry cannot be mistaken for live sensor data.
            self.neo.reset_input_buffer()
            time.sleep(0.03)
            self.neo.reset_input_buffer()
            self.nb.clear()
            self.neo_last_rx_mono = time.monotonic()
            self.neo_last_gnss_mono = 0.0
            self.neo_last_mag_mono = 0.0
            token = int(time.monotonic_ns() & 0xFFFFFFFF) or 1
            self.neo_session_token = token
            # Match the firmware transport contract: resync any partial command,
            # establish a fresh host authority epoch, then explicitly enable ROS
            # telemetry. Never depend on ROS:1 persisting from a previous host.
            hello = f"\nHOST:HELLO:{token}\nROS:1\nPING\nNEO:STATUS\nUSB:STATUS\n".encode()
            self.neo.write(hello)
            self.neo.flush()
            if not initial:
                self.neo_reconnects += 1
            self._raw("F411", "TRANSPORT", "CONNECTED", 1)
            return True
        except (OSError, serial.SerialException) as exc:
            self.neo = None
            self.neo_next_open_mono = now + F411_REOPEN_BACKOFF_S
            if not initial:
                self._raw("F411", "TRANSPORT", f"OPEN_FAIL:{type(exc).__name__}", 0)
            return False

    def _close_neo(self, reason: str) -> None:
        if self.neo is None:
            return
        try:
            self.neo.close()
        except Exception:
            pass
        self.neo = None
        self.nb.clear()
        self.neo_disconnects += 1
        self.neo_next_open_mono = time.monotonic() + F411_REOPEN_BACKOFF_S
        self._raw("F411", "TRANSPORT", f"DISCONNECTED:{reason}", 0)

    def _invalidate_imu_epoch(self) -> None:
        self.ib.clear()
        self.imu_acc_raw = [None] * 3; self.imu_acc = [NAN] * 3
        self.imu_gyro_raw = [None] * 3; self.imu_gyro_dps = [NAN] * 3; self.imu_gyro_rps = [NAN] * 3
        self.imu_angle_raw = [None] * 3; self.imu_rpy = [NAN] * 3
        self.yah_mag_raw = [None] * 3; self.yah_q = [NAN, NAN]
        self.yah_norm = NAN; self.yah_yaw = NAN; self.yah_valid = False
        self.inertial_yaw = NAN; self.inertial_last_t = None; self.inertial_source = "none"
        self.last_yah_checksum_ok = ""

    def _open_imu(self, initial: bool = False) -> bool:
        if self.imu is not None:
            return True
        now = time.monotonic()
        if now < self.imu_next_open_mono:
            return False
        try:
            self.imu = serial.Serial(self.a.imu_port, self.a.imu_baud, timeout=0, exclusive=True)
            # CP2102/ttyUSB can retain a small receive burst across ownership
            # changes. Discard it so the first decoded WIT frame always belongs
            # to this acquisition epoch rather than the previous reader.
            self.imu.reset_input_buffer()
            self._invalidate_imu_epoch()
            self.imu_last_rx_mono = time.monotonic()
            self.imu_last_valid_mono = 0.0
            if not initial:
                self.imu_reconnects += 1
            self._raw("YAHBOOM", "TRANSPORT", "CONNECTED", 1)
            return True
        except (OSError, serial.SerialException) as exc:
            self.imu = None
            self.imu_next_open_mono = now + YAHBOOM_REOPEN_BACKOFF_S
            if not initial:
                self._raw("YAHBOOM", "TRANSPORT", f"OPEN_FAIL:{type(exc).__name__}", 0)
            return False

    def _close_imu(self, reason: str) -> None:
        if self.imu is None:
            return
        try:
            self.imu.close()
        except Exception:
            pass
        self.imu = None
        self._invalidate_imu_epoch()
        self.imu_disconnects += 1
        self.imu_next_open_mono = time.monotonic() + YAHBOOM_REOPEN_BACKOFF_S
        self._raw("YAHBOOM", "TRANSPORT", f"DISCONNECTED:{reason}", 0)

    def _create_outdir(self, arg: str) -> Path:
        base = Path(arg).expanduser() if arg else self.f4root / "tools/records"
        base.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        out = base / f"data_direct_{stamp}"; n = 1
        while out.exists():
            out = base / f"data_direct_{stamp}_{n:02d}"; n += 1
        out.mkdir(parents=True)
        latest = base / "latest_direct"
        try:
            if latest.is_symlink() or latest.exists(): latest.unlink()
            latest.symlink_to(out.name)
        except Exception: pass
        return out

    def _snapshot_configs(self) -> None:
        for p in (self.nav_cfg / "mag_heading.yaml", self.nav_cfg / "yahboom_mag_calibration.yaml", self.nav_cfg / "imu.yaml",
                  self.f4root / ".pio/build/blackpill_f411ce_neo3pro/firmware.identity.json"):
            if p.exists():
                try: shutil.copy2(p, self.outdir / p.name)
                except Exception: pass

    def _write_meta(self, started: bool) -> None:
        snaps = {}
        if hasattr(self, "outdir"):
            for p in self.outdir.iterdir():
                if p.is_file() and p.name not in ("raw.jsonl", "summary.csv", "metadata.json"):
                    try: snaps[p.name] = sha256_file(p)
                    except Exception: pass
        meta = {
            "schema": "agv-data-unified-v2", "source_mode": "DIRECT_SERIAL", "started": bool(started),
            "start_wall_utc_ns": self.start_wall_ns, "start_utc_iso8601": iso_utc(self.start_wall_ns),
            "stop_wall_utc_ns": None if started else time.time_ns(), "hostname": socket.gethostname(),
            "f411_port": self.a.neo_port, "f411_baud": self.a.neo_baud,
            "f411_disconnects": getattr(self, "neo_disconnects", 0),
            "f411_reconnects": getattr(self, "neo_reconnects", 0),
            "yahboom_port": self.a.imu_port, "yahboom_baud": self.a.imu_baud,
            "yahboom_disconnects": getattr(self, "imu_disconnects", 0),
            "yahboom_reconnects": getattr(self, "imu_reconnects", 0),
            "summary_fields": UNIFIED_FIELDS,
            "timestamp_contract": "DroneCAN Fix2 gnss_us becomes measurement_stamp_ns only when time_standard=2 UTC; otherwise host UTC receive time",
            "xy_contract": "local WGS84 ENU from the first valid direct GNSS fix",
            "raw_contract": "raw.jsonl retains every F411 line and every Yahboom 11-byte frame with host wall/monotonic timestamps",
            "snapshot_sha256": snaps,
        }
        self.meta_path.write_text(json.dumps(meta, indent=2, sort_keys=True))

    def _raw(self, source: str, packet: str, payload: Any, checksum_ok: Any = "") -> None:
        wall = time.time_ns(); mono = time.monotonic_ns(); self.counts["raw"] += 1
        rec = {"source": source, "packet_type": packet, "wall_utc_ns": wall, "utc_iso8601": iso_utc(wall),
               "monotonic_ns": mono, "checksum_ok": checksum_ok, "payload": payload}
        self.raw_f.write(json.dumps(rec, separators=(",", ":"), allow_nan=True) + "\n")

    def _parse_v2_line(self, line: str, prefix: str, expected_min: int) -> list[str] | None:
        fields = parse_v2_payload(line[len(prefix):])
        if fields is None or len(fields) < expected_min:
            self.bad_neo_crc += 1; self.last_neo_crc_ok = 0
            return None
        self.last_neo_crc_ok = 1
        return fields

    def parse_f4_line(self, line: str) -> None:
        line = line.strip()
        if not line: return
        self.neo_last_rx_mono = time.monotonic()
        self.counts["f4"] += 1; self._raw("F411", line.split(":", 2)[0:2].__str__(), line)
        try:
            if line.startswith("SENS:GNSS:"):
                f = self._parse_v2_line(line, "SENS:GNSS:", 23)
                if f and len(f) == 23:
                    v = list(map(float, f)); self.counts["gnss"] += 1
                    self.neo_last_gnss_mono = time.monotonic()
                    self.gnss = dict(mcu_ms=int(v[1]), itow_ms=int(v[2]), fix_type=int(v[3]), fix_valid=int(v[4] > .5 and v[5] < .5),
                        satellites=int(v[6]), lat=v[7], lon=v[8], alt=v[9], hacc=v[10], vacc=v[11], vn=v[12], ve=v[13], vd=v[14],
                        gs=v[15], course=v[16], sacc=v[17], course_acc=v[18], pdop=v[19], rate=v[20], mode=int(v[21]), submode=int(v[22]))
                    if self.gnss["fix_valid"] and abs(v[7]) <= 90 and abs(v[8]) <= 180 and not (v[7] == 0 and v[8] == 0):
                        if self.origin is None: self.origin = (v[7], v[8], v[9], geodetic_to_ecef(v[7], v[8], v[9]))
                        self.gnss_enu = enu_from_origin(v[7], v[8], v[9], self.origin)
            elif line.startswith("SENS:GNSSPRO:"):
                f = self._parse_v2_line(line, "SENS:GNSSPRO:", 24)
                if f and len(f) == 24:
                    self.neo_last_gnss_mono = time.monotonic()
                    v = list(map(float, f)); self.gnsspro = dict(mcu_ms=int(v[1]), node=int(v[2]), network_us=int(v[3]), gnss_us=int(v[4]),
                        time_std=int(v[5]), leap=int(v[6]), status=int(v[7]), mode=int(v[8]), submode=int(v[9]), sats=int(v[10]), sats_visible=int(v[11]),
                        alt_msl=v[12]*1e-3, alt_ellipsoid=v[13]*1e-3, gdop=v[14], pdop=v[15], hdop=v[16], vdop=v[17], tdop=v[18], ndop=v[19], edop=v[20], rate=v[21])
            elif line.startswith("SENS:MAGPRO:"):
                f = self._parse_v2_line(line, "SENS:MAGPRO:", 9)
                if f:
                    self.neo_last_mag_mono = time.monotonic()
                    v = list(map(float, f)); self.neo_mag = [v[5], v[6], v[7]]; self.counts["neo_mag"] += 1; self._update_neo_yaw()
                    if not self.gnsspro.get("node"): self.gnsspro["node"] = int(v[2])
            elif line.startswith("SENS:GNSSHEAD:"):
                f = self._parse_v2_line(line, "SENS:GNSSHEAD:", 7)
                if f:
                    v = list(map(float, f)); self.gnss_heading = v[5] if v[3] > .5 else NAN
            elif line.startswith("SENS:BARO:"):
                f = self._parse_v2_line(line, "SENS:BARO:", 5)
                if f: self.baro_pressure = float(f[3])
            elif line.startswith("SENS:TEMP:"):
                f = self._parse_v2_line(line, "SENS:TEMP:", 5)
                if f: self.baro_temp = float(f[3])
        except Exception:
            # Raw line is always retained even if a single diagnostic parser fails.
            return

    def _update_neo_yaw(self) -> None:
        mx, my, mz = self.neo_mag
        if not all(math.isfinite(x) for x in (mx, my, mz)): return
        self.neo_norm = math.sqrt(mx*mx + my*my + mz*mz)
        p = self.mag_cfg; bx, by = p.get("neo3_mag_bias_xy_ut", [0.0, 0.0])[:2]; m = p.get("neo3_mag_matrix_xy", [1.,0.,0.,1.])
        qx = float(m[0])*(mx-float(bx)) + float(m[1])*(my-float(by)); qy = float(m[2])*(mx-float(bx)) + float(m[3])*(my-float(by))
        roll = math.radians(self.imu_rpy[0]) if math.isfinite(self.imu_rpy[0]) else 0.0
        pitch = math.radians(self.imu_rpy[1]) if math.isfinite(self.imu_rpy[1]) else 0.0
        cr,sr,cp,sp = math.cos(roll),math.sin(roll),math.cos(pitch),math.sin(pitch)
        xh = qx*cp + mz*sp; yh = qx*sr*sp + qy*cr - mz*sr*cp
        if math.hypot(xh,yh) < 1e-9: return
        yaw = norm_angle(float(p.get("neo3_mag_yaw_sign",1.0))*math.atan2(yh,xh) + float(p.get("neo3_mag_yaw_offset_rad",0.0)) - float(p.get("magnetic_declination_rad",0.0)))
        if p.get("neo3_heading_lut_enabled", False): yaw = apply_lut(yaw, p.get("neo3_heading_lut_input_rad",[]), p.get("neo3_heading_lut_correction_rad",[]))
        self.neo_yaw = yaw; max_tilt=float(p.get("neo3_planar_max_tilt_rad",0.2617993878))
        self.neo_valid = 15.0 <= self.neo_norm <= 100.0 and abs(roll) <= max_tilt and abs(pitch) <= max_tilt
        if not math.isfinite(self.inertial_yaw) or self.inertial_source != "neo3_calibrated_direct":
            self.inertial_yaw = yaw; self.inertial_source = "neo3_calibrated_direct"; self.inertial_last_t = time.monotonic()

    def _update_yah_yaw(self) -> None:
        if any(x is None for x in self.yah_mag_raw): return
        p=self.mag_cfg; x,y,z=map(float,self.yah_mag_raw); bx,by=p.get("imu_mag_bias_xy_lsb",[0.,0.])[:2]; m=p.get("imu_mag_matrix_xy_per_lsb",[1.,0.,0.,1.])
        qx=float(m[0])*(x-float(bx))+float(m[1])*(y-float(by)); qy=float(m[2])*(x-float(bx))+float(m[3])*(y-float(by)); self.yah_q=[qx,qy]; self.yah_norm=math.hypot(qx,qy)
        yaw=norm_angle(float(p.get("imu_mag_yaw_sign",1.0))*math.atan2(qy,qx)+float(p.get("imu_mag_yaw_offset_rad",0.0))-float(p.get("magnetic_declination_rad",0.0)))
        if p.get("imu_heading_lut_enabled",False): yaw=apply_lut(yaw,p.get("imu_heading_lut_input_rad",[]),p.get("imu_heading_lut_correction_rad",[]))
        self.yah_yaw=yaw; lo=float(p.get("imu_corrected_norm_min",0.75)); hi=float(p.get("imu_corrected_norm_max",1.25)); mt=float(p.get("imu_planar_max_tilt_rad",0.2617993878))
        roll=math.radians(self.imu_rpy[0]) if math.isfinite(self.imu_rpy[0]) else 0.; pitch=math.radians(self.imu_rpy[1]) if math.isfinite(self.imu_rpy[1]) else 0.
        self.yah_valid=lo <= self.yah_norm <= hi and abs(roll)<=mt and abs(pitch)<=mt

    def parse_imu_frame(self, fr: bytes, now: float) -> None:
        ok=((sum(fr[:10]) & 0xff) == fr[10]); self.last_yah_checksum_ok=int(ok)
        self._raw("YAHBOOM", f"0x{fr[1]:02X}", fr.hex(" "), int(ok))
        if not ok: self.bad_yah_checksum += 1; return
        self.imu_last_valid_mono = now
        typ=fr[1]; v=[i16(fr[2],fr[3]),i16(fr[4],fr[5]),i16(fr[6],fr[7])]
        if typ==0x51:
            self.imu_acc_raw=v; self.imu_acc=[x/32768.0*self.a.accel_fsr_g*G for x in v]; self.counts["acc"]+=1
        elif typ==0x52:
            self.imu_gyro_raw=v; self.imu_gyro_dps=[x/32768.0*self.a.gyro_fsr_dps for x in v]; self.imu_gyro_rps=[math.radians(x) for x in self.imu_gyro_dps]; self.counts["gyro"]+=1
            bias=self.imu_cfg.get("gyro_bias",[0.,0.,0.]); bz=float(bias[2]) if len(bias)>=3 else 0.0
            if self.inertial_last_t is None: self.inertial_last_t=now
            dt=now-self.inertial_last_t; self.inertial_last_t=now
            if math.isfinite(self.inertial_yaw) and 0.0 < dt <= .2: self.inertial_yaw=norm_angle(self.inertial_yaw+(self.imu_gyro_rps[2]-bz)*dt)
        elif typ==0x53:
            self.imu_angle_raw=v; self.imu_rpy=[x/32768.0*180.0 for x in v]; self.imu_rpy[2]=wrap360(self.imu_rpy[2]); self.counts["angle"]+=1; self._update_neo_yaw(); self._update_yah_yaw()
        elif typ==0x54:
            self.yah_mag_raw=v; self.counts["imu_mag"]+=1; self._update_yah_yaw()

    @staticmethod
    def _n(v: Any) -> float:
        return finite(v)

    def sample(self) -> None:
        self.sample_seq += 1; wall=time.time_ns(); mono=time.monotonic_ns(); now=time.monotonic()
        f411_fresh = self.neo is not None and (now - self.neo_last_rx_mono) <= F411_DATA_STALE_S
        gnss_fresh = f411_fresh and self.neo_last_gnss_mono > 0.0 and (now - self.neo_last_gnss_mono) <= F411_DATA_STALE_S
        mag_fresh = f411_fresh and self.neo_last_mag_mono > 0.0 and (now - self.neo_last_mag_mono) <= MAG_DATA_STALE_S
        imu_fresh = self.imu is not None and self.imu_last_valid_mono > 0.0 and (now - self.imu_last_valid_mono) <= YAHBOOM_DATA_STALE_S
        g = self.gnss if gnss_fresh else {}
        gp = self.gnsspro if gnss_fresh else {}
        if not mag_fresh:
            self.neo_valid = False
        if not imu_fresh:
            self.yah_valid = False
        utc_ok=gnss_fresh and gp.get("time_std",-1)==2 and gp.get("gnss_us",0)>0
        meas=int(gp.get("gnss_us",0))*1000 if utc_ok else wall
        accnorm=math.sqrt(sum(x*x for x in self.imu_acc)) if imu_fresh and all(math.isfinite(x) for x in self.imu_acc) else NAN
        row={k:"" for k in UNIFIED_FIELDS}
        row.update({
            "source_mode":"DIRECT_SERIAL", "sample_seq":self.sample_seq, "wall_utc_ns":wall, "utc_iso8601":iso_utc(wall), "ros_now_ns":-1, "monotonic_ns":mono,
            "measurement_stamp_ns":meas, "measurement_stamp_sec":meas//1_000_000_000, "measurement_stamp_nanosec":meas%1_000_000_000,
            "f411_mcu_ms":gp.get("mcu_ms",g.get("mcu_ms",-1)), "dronecan_network_us":gp.get("network_us",-1), "dronecan_gnss_us":gp.get("gnss_us",-1),
            "dronecan_time_standard":gp.get("time_std",-1), "timestamp_source_code":4 if utc_ok else 3,
            "x_m":self.gnss_enu[0] if gnss_fresh else NAN, "y_m":self.gnss_enu[1] if gnss_fresh else NAN,
            "xy_source":"gnss_local_enu_direct" if gnss_fresh else "gnss_stale",
            "gnss_east_m":self.gnss_enu[0] if gnss_fresh else NAN, "gnss_north_m":self.gnss_enu[1] if gnss_fresh else NAN,
            "longitude_deg":g.get("lon",NAN), "latitude_deg":g.get("lat",NAN), "altitude_m":g.get("alt",gp.get("alt_msl",NAN)), "hacc_m":g.get("hacc",NAN), "vacc_m":g.get("vacc",NAN),
            "fix_type":g.get("fix_type",-1), "fix_valid":g.get("fix_valid",0), "satellites":g.get("satellites",gp.get("sats",-1)), "satellites_visible":gp.get("sats_visible",-1),
            "gnss_ground_speed_mps":g.get("gs",NAN), "gnss_course_deg":g.get("course",NAN), "gnss_vel_n_mps":g.get("vn",NAN), "gnss_vel_e_mps":g.get("ve",NAN), "gnss_vel_d_mps":g.get("vd",NAN),
            "gnss_sacc_mps":g.get("sacc",NAN), "gnss_course_acc_deg":g.get("course_acc",NAN), "gnss_pdop":g.get("pdop",gp.get("pdop",NAN)), "gnss_rate_hz":g.get("rate",gp.get("rate",NAN)),
            "yaw_neo3_deg":deg(self.neo_yaw) if mag_fresh else NAN,
            "yaw_neo3_source":"neo3_calibrated_direct" if mag_fresh else "neo3_stale",
            "yaw_neo3_valid":int(self.neo_valid and mag_fresh),
            "neo3_mag_x_ut":self.neo_mag[0] if mag_fresh else NAN, "neo3_mag_y_ut":self.neo_mag[1] if mag_fresh else NAN,
            "neo3_mag_z_ut":self.neo_mag[2] if mag_fresh else NAN, "neo3_mag_norm_ut":self.neo_norm if mag_fresh else NAN,
            "neo3_node_id":gp.get("node",-1),
            "yaw_yahboom_deg":deg(self.yah_yaw) if imu_fresh else NAN, "yaw_yahboom_source":"yahboom_calibrated_direct" if imu_fresh else "yahboom_stale", "yaw_yahboom_valid":int(self.yah_valid and imu_fresh),
            "yah_mag_raw_x_lsb":self.yah_mag_raw[0] if imu_fresh else NAN, "yah_mag_raw_y_lsb":self.yah_mag_raw[1] if imu_fresh else NAN, "yah_mag_raw_z_lsb":self.yah_mag_raw[2] if imu_fresh else NAN, "yah_mag_corrected_x":self.yah_q[0] if imu_fresh else NAN, "yah_mag_corrected_y":self.yah_q[1] if imu_fresh else NAN, "yahboom_corrected_norm":self.yah_norm if imu_fresh else NAN,
            "yaw_inertial_deg":deg(self.inertial_yaw) if imu_fresh else NAN, "yaw_inertial_source":self.inertial_source+":gyro_integral" if imu_fresh else "imu_stale", "yaw_inertial_valid":int(imu_fresh and math.isfinite(self.inertial_yaw)), "gyro_integrated_yaw_deg":deg(self.inertial_yaw) if imu_fresh else NAN,
            "yaw_gnss_heading_deg":deg(self.gnss_heading) if gnss_fresh else NAN, "yaw_imu_orientation_deg":self.imu_rpy[2] if imu_fresh else NAN, "map_yaw_from_enu_deg":0.0,
            "imu_acc_raw_x":self.imu_acc_raw[0] if imu_fresh else NAN, "imu_acc_raw_y":self.imu_acc_raw[1] if imu_fresh else NAN, "imu_acc_raw_z":self.imu_acc_raw[2] if imu_fresh else NAN, "imu_acc_x_mps2":self.imu_acc[0] if imu_fresh else NAN, "imu_acc_y_mps2":self.imu_acc[1] if imu_fresh else NAN, "imu_acc_z_mps2":self.imu_acc[2] if imu_fresh else NAN, "imu_acc_norm_mps2":accnorm,
            "imu_gyro_raw_x":self.imu_gyro_raw[0] if imu_fresh else NAN, "imu_gyro_raw_y":self.imu_gyro_raw[1] if imu_fresh else NAN, "imu_gyro_raw_z":self.imu_gyro_raw[2] if imu_fresh else NAN, "imu_gyro_x_dps":self.imu_gyro_dps[0] if imu_fresh else NAN, "imu_gyro_y_dps":self.imu_gyro_dps[1] if imu_fresh else NAN, "imu_gyro_z_dps":self.imu_gyro_dps[2] if imu_fresh else NAN,
            "imu_gyro_x_rps":self.imu_gyro_rps[0] if imu_fresh else NAN, "imu_gyro_y_rps":self.imu_gyro_rps[1] if imu_fresh else NAN, "imu_gyro_z_rps":self.imu_gyro_rps[2] if imu_fresh else NAN, "gyro_z_rps":self.imu_gyro_rps[2] if imu_fresh else NAN,
            "imu_angle_raw_roll":self.imu_angle_raw[0] if imu_fresh else NAN, "imu_angle_raw_pitch":self.imu_angle_raw[1] if imu_fresh else NAN, "imu_angle_raw_yaw":self.imu_angle_raw[2] if imu_fresh else NAN, "imu_roll_deg":self.imu_rpy[0] if imu_fresh else NAN, "imu_pitch_deg":self.imu_rpy[1] if imu_fresh else NAN, "imu_accgyro_yaw_deg":self.imu_rpy[2] if imu_fresh else NAN,
            "baro_pressure_pa":self.baro_pressure, "baro_temperature_k":self.baro_temp,
            "neo3_crc_ok":self.last_neo_crc_ok, "yahboom_checksum_ok":self.last_yah_checksum_ok, "bad_neo3_crc":self.bad_neo_crc, "bad_yahboom_checksum":self.bad_yah_checksum,
            "f4_line_count":self.counts["f4"], "gnss_count":self.counts["gnss"], "neo3_mag_count":self.counts["neo_mag"], "imu_acc_count":self.counts["acc"], "imu_gyro_count":self.counts["gyro"], "imu_angle_count":self.counts["angle"], "imu_mag_count":self.counts["imu_mag"],
            "raw_message_count":self.counts["raw"], "raw_reference":f"F4line={self.counts['f4']};YAH={self.counts['acc']+self.counts['gyro']+self.counts['angle']+self.counts['imu_mag']}",
        })
        self.writer.writerow(row); self.dashboard(row)

    @staticmethod
    def fmt(v: Any, n=3) -> str:
        try:
            x=float(v); return f"{x:.{n}f}" if math.isfinite(x) else "--"
        except Exception: return "--"

    def dashboard(self, r: Dict[str,Any]) -> None:
        now=time.monotonic()
        if not sys.stdout.isatty() and now-self.last_display<1.0: return
        self.last_display=now
        if sys.stdout.isatty(): print("\033[2J\033[H",end="")
        print(f"AGV DATA [DIRECT SERIAL] {r['utc_iso8601']}  save={self.outdir}")
        print(f"x={self.fmt(r['x_m'])} y={self.fmt(r['y_m'])} | lon={self.fmt(r['longitude_deg'],8)} lat={self.fmt(r['latitude_deg'],8)} hAcc={self.fmt(r['hacc_m'])}m sat={r['satellites']} fix={r['fix_type']}")
        print(f"yaw NEO3={self.fmt(r['yaw_neo3_deg'])} [{r['yaw_neo3_valid']}] | Yahboom={self.fmt(r['yaw_yahboom_deg'])} [{r['yaw_yahboom_valid']}] | inertia={self.fmt(r['yaw_inertial_deg'])}")
        print(f"stamp={r['measurement_stamp_sec']}.{int(r['measurement_stamp_nanosec']):09d} F411={r['f411_mcu_ms']}ms Fix2UTC={r['dronecan_gnss_us']} time_std={r['dronecan_time_standard']}")
        print(f"raw F4={r['f4_line_count']} YAH acc/gyr/ang/mag={r['imu_acc_count']}/{r['imu_gyro_count']}/{r['imu_angle_count']}/{r['imu_mag_count']} crc_bad={r['bad_neo3_crc']} checksum_bad={r['bad_yahboom_checksum']}",flush=True)

    def run(self) -> None:
        next_sample=time.monotonic(); deadline=time.monotonic()+self.a.duration if self.a.duration>0 else None
        while self.running and (deadline is None or time.monotonic()<deadline):
            self._open_neo(); self._open_imu()
            readers=[]
            neo_fd=self.neo.fileno() if self.neo is not None else None
            imu_fd=self.imu.fileno() if self.imu is not None else None
            if neo_fd is not None: readers.append(neo_fd)
            if imu_fd is not None: readers.append(imu_fd)
            try:
                ready,_,_=select.select(readers,[],[],0.02) if readers else ([],[],[])
                if not readers: time.sleep(0.02)
            except (OSError, ValueError) as exc:
                # EBADF does not identify which hot-unplugged fd failed. Closing
                # both epochs is deterministic and both reopen independently.
                self._close_neo(f"select:{type(exc).__name__}")
                self._close_imu(f"select:{type(exc).__name__}")
                ready=[]
            if neo_fd is not None and neo_fd in ready and self.neo is not None:
                try:
                    chunk=os.read(neo_fd,8192)
                    if not chunk:
                        self._close_neo("eof")
                    else:
                        self.neo_last_rx_mono=time.monotonic()
                        self.nb.extend(chunk)
                        while b"\n" in self.nb:
                            raw,self.nb=self.nb.split(b"\n",1)
                            self.parse_f4_line(raw.decode("utf-8","replace").rstrip("\r"))
                except (OSError, serial.SerialException) as exc:
                    self._close_neo(f"read:{type(exc).__name__}")
            if imu_fd is not None and imu_fd in ready and self.imu is not None:
                try:
                    chunk=os.read(imu_fd,8192)
                    if not chunk:
                        self._close_imu("eof")
                    else:
                        self.imu_last_rx_mono=time.monotonic()
                        self.ib.extend(chunk)
                        while len(self.ib)>=11:
                            if self.ib[0]!=0x55 or not (0x51<=self.ib[1]<=0x59): del self.ib[0]; continue
                            fr=bytes(self.ib[:11]); del self.ib[:11]; self.parse_imu_frame(fr,time.monotonic())
                except (OSError, serial.SerialException) as exc:
                    self._close_imu(f"read:{type(exc).__name__}")
            now=time.monotonic()
            if self.neo is not None and (now-self.neo_last_rx_mono)>F411_TRANSPORT_STALL_S:
                self._close_neo("rx_stall")
            if self.imu is not None and (now-self.imu_last_rx_mono)>YAHBOOM_TRANSPORT_STALL_S:
                self._close_imu("rx_stall")
            if now>=next_sample:
                self.sample(); next_sample=now+1.0/max(1.0,self.a.rate)
        self.close()

    def close(self) -> None:
        if not self.running: return
        self.running=False
        try: self.raw_f.flush(); self.sum_f.flush(); os.fsync(self.raw_f.fileno()); os.fsync(self.sum_f.fileno())
        except Exception: pass
        try:
            if self.neo is not None: self.neo.close()
            if self.imu is not None: self.imu.close()
        except Exception: pass
        self._write_meta(False)
        try: self.raw_f.close(); self.sum_f.close()
        except Exception: pass
        print(f"\nSAVED summary: {self.summary_path}\nSAVED raw:     {self.raw_path}\nSAVED meta:    {self.meta_path}")


def main() -> int:
    ap=argparse.ArgumentParser(description="Direct F411 CDC + Yahboom serial unified AGV data logger")
    ap.add_argument("--neo-port",default=F411_BY_ID); ap.add_argument("--neo-baud",type=int,default=1000000)
    ap.add_argument("--imu-port",default=YAHBOOM_BY_ID); ap.add_argument("--imu-baud",type=int,default=921600)
    ap.add_argument("--rate",type=float,default=10.0); ap.add_argument("--duration",type=float,default=0.0)
    ap.add_argument("--accel-fsr-g",type=float,default=16.0); ap.add_argument("--gyro-fsr-dps",type=float,default=2000.0)
    ap.add_argument("--output-dir",default=""); ap.add_argument("--takeover",action="store_true",help="terminate current owners of both serial ports before opening")
    a=ap.parse_args(); rec=None
    try:
        rec=DirectRecorder(a); signal.signal(signal.SIGTERM,lambda *_: setattr(rec,"running",False)); signal.signal(signal.SIGINT,lambda *_: setattr(rec,"running",False)); rec.run(); return 0
    except KeyboardInterrupt:
        if rec: rec.close()
        return 0
    except Exception as exc:
        print(f"ERROR: {exc}",file=sys.stderr)
        if rec:
            try: rec.close()
            except Exception: pass
        return 2

if __name__=="__main__": raise SystemExit(main())
