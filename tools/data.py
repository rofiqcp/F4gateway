#!/usr/bin/env python3
"""AGV navigation sensor recorder.

Live terminal fields:
  x, y, longitude, latitude, GNSS hAcc,
  NEO3 yaw, Yahboom yaw, inertial yaw.

Recording starts immediately.  Ctrl+C closes the files cleanly.
Every discovered topic under /gnss, /neo3, /neo3pro, /imu, /heading and
/odometry is written to raw.jsonl, while synchronized operator values are
written to summary.csv.

Timestamp contract:
  * ROS header stamps are preserved exactly as sec/nanosec for Nav2.
  * wall_utc_ns is Unix UTC nanoseconds.
  * monotonic_ns is host monotonic time for latency/order analysis.
  * NEO3PRO Fix2 network_us/gnss_us/time_standard and F411 mcu_ms are retained.
    The F4 bridge only promotes Fix2 gnss_us to ROS header time when
    time_standard == 2 (UTC), matching the ArduPilot DroneCAN authority rule.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import shutil
import signal
import socket
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, Optional, Tuple

import yaml
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.convert import message_to_ordereddict
from rosidl_runtime_py.utilities import get_message


PI = math.pi
WGS84_A = 6378137.0
WGS84_F = 1.0 / 298.257223563
WGS84_E2 = WGS84_F * (2.0 - WGS84_F)
RAW_PREFIXES = ("/gnss", "/neo3", "/neo3pro", "/imu", "/heading", "/odometry", "/localization/map_yaw_from_enu")


def norm_angle(v: float) -> float:
    if not math.isfinite(v):
        return float("nan")
    return math.atan2(math.sin(v), math.cos(v))


def deg(v: float) -> float:
    return math.degrees(v) if math.isfinite(v) else float("nan")


def finite_or_none(v: Any) -> Optional[float]:
    try:
        x = float(v)
        return x if math.isfinite(x) else None
    except Exception:
        return None


def stamp_ns(msg: Any) -> int:
    h = getattr(msg, "header", None)
    s = getattr(h, "stamp", None) if h is not None else None
    if s is None:
        return 0
    return int(s.sec) * 1_000_000_000 + int(s.nanosec)


def yaw_from_q(q: Any) -> float:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    return norm_angle(math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))


def roll_pitch_from_q(q: Any) -> Tuple[float, float]:
    x, y, z, w = float(q.x), float(q.y), float(q.z), float(q.w)
    sinr = 2.0 * (w * x + y * z)
    cosr = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr, cosr)
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(PI / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    return roll, pitch


def geodetic_to_ecef(lat_deg: float, lon_deg: float, alt_m: float) -> Tuple[float, float, float]:
    lat = math.radians(lat_deg)
    lon = math.radians(lon_deg)
    sl, cl = math.sin(lat), math.cos(lat)
    so, co = math.sin(lon), math.cos(lon)
    n = WGS84_A / math.sqrt(1.0 - WGS84_E2 * sl * sl)
    return ((n + alt_m) * cl * co,
            (n + alt_m) * cl * so,
            (n * (1.0 - WGS84_E2) + alt_m) * sl)


def enu_from_origin(lat: float, lon: float, alt: float,
                    origin: Tuple[float, float, float, Tuple[float, float, float]]) -> Tuple[float, float, float]:
    lat0, lon0, _alt0, ecef0 = origin
    x, y, z = geodetic_to_ecef(lat, lon, alt)
    dx, dy, dz = x - ecef0[0], y - ecef0[1], z - ecef0[2]
    la, lo = math.radians(lat0), math.radians(lon0)
    sl, cl, so, co = math.sin(la), math.cos(la), math.sin(lo), math.cos(lo)
    east = -so * dx + co * dy
    north = -sl * co * dx - sl * so * dy + cl * dz
    up = cl * co * dx + cl * so * dy + sl * dz
    return east, north, up


def apply_lut(yaw: float, inputs: list, corrections: list) -> float:
    if len(inputs) < 2 or len(inputs) != len(corrections):
        return norm_angle(yaw)
    pairs = sorted((norm_angle(float(x)), norm_angle(float(c))) for x, c in zip(inputs, corrections))
    y = norm_angle(yaw)
    hi = 0
    while hi < len(pairs) and pairs[hi][0] < y:
        hi += 1
    yy = y
    if hi == 0:
        x0, c0 = pairs[-1]
        x1, c1 = pairs[0][0] + 2.0 * PI, pairs[0][1]
        yy += 2.0 * PI
    elif hi == len(pairs):
        x0, c0 = pairs[-1]
        x1, c1 = pairs[0][0] + 2.0 * PI, pairs[0][1]
    else:
        x0, c0 = pairs[hi - 1]
        x1, c1 = pairs[hi]
    f = min(1.0, max(0.0, (yy - x0) / max(1.0e-9, x1 - x0)))
    return norm_angle(y + c0 + f * norm_angle(c1 - c0))


def sha256_file(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def git_head(root: Path) -> str:
    try:
        return subprocess.check_output(["git", "-C", str(root), "rev-parse", "HEAD"], text=True).strip()
    except Exception:
        return "unknown"


class Recorder(Node):
    SUMMARY_FIELDS = [
        "wall_utc_ns", "utc_iso8601", "ros_now_ns", "monotonic_ns",
        "measurement_stamp_ns", "measurement_stamp_sec", "measurement_stamp_nanosec",
        "f411_mcu_ms", "dronecan_network_us", "dronecan_gnss_us", "dronecan_time_standard",
        "timestamp_source_code", "x_m", "y_m", "xy_source", "gnss_east_m", "gnss_north_m",
        "longitude_deg", "latitude_deg", "altitude_m", "hacc_m", "fix_type", "satellites",
        "yaw_neo3_deg", "yaw_neo3_source", "yaw_neo3_valid",
        "yaw_yahboom_deg", "yaw_yahboom_source", "yaw_yahboom_valid",
        "yaw_inertial_deg", "yaw_inertial_source", "yaw_inertial_valid",
        "yaw_gnss_heading_deg", "yaw_imu_orientation_deg", "gyro_z_rps",
        "neo3_mag_norm_ut", "yahboom_corrected_norm", "map_yaw_from_enu_deg",
    ]

    def __init__(self, args: argparse.Namespace):
        super().__init__("agv_sensor_data_recorder")
        self.args = args
        self.ws = Path("/home/sirobo/agv")
        self.f4 = self.ws / "F4gateway"
        self.nav_cfg = self.ws / "src/navigation/config"
        self.mag_params = self._load_mag_params()
        self.latest: Dict[str, Tuple[Any, int]] = {}
        self.subs: Dict[str, Any] = {}
        self.origin = None
        self.map_yaw = 0.0
        self.map_yaw_seen = False
        self.roll = self.pitch = 0.0
        self.imu_orientation_yaw = float("nan")
        self.gyro_z = float("nan")
        self.inertial_fallback = float("nan")
        self.inertial_last_stamp_ns = 0
        self.inertial_seed_source = "none"
        self.neo_diag = float("nan")
        self.neo_diag_valid = False
        self.neo_norm = float("nan")
        self.yah_diag = float("nan")
        self.yah_diag_valid = False
        self.yah_norm = float("nan")
        self.fix = None
        self.quality = []
        self.gnss_meta = []
        self.gnss_heading = float("nan")
        self.gnss_enu = (float("nan"), float("nan"), float("nan"))
        self.last_dashboard_ns = 0
        self.start_wall_ns = time.time_ns()
        self.start_mono_ns = time.monotonic_ns()
        self.outdir = self._create_output_dir(args.output_dir)
        self.raw_path = self.outdir / "raw.jsonl"
        self.summary_path = self.outdir / "summary.csv"
        self.meta_path = self.outdir / "metadata.json"
        self.raw_f = self.raw_path.open("w", encoding="utf-8", buffering=1)
        self.sum_f = self.summary_path.open("w", encoding="utf-8", newline="", buffering=1)
        self.sum_w = csv.DictWriter(self.sum_f, fieldnames=self.SUMMARY_FIELDS)
        self.sum_w.writeheader()
        self._snapshot_configs()
        self._write_metadata(started=True)
        self.qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=100,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_timer(0.5, self.discover_topics)
        self.create_timer(1.0 / max(1.0, args.rate), self.sample)
        self.create_timer(1.0, self.flush)
        self.get_logger().info(f"Recording immediately -> {self.outdir}")

    def _load_mag_params(self) -> Dict[str, Any]:
        path = self.nav_cfg / "mag_heading.yaml"
        try:
            return yaml.safe_load(path.read_text())["mag_heading_fusion"]["ros__parameters"]
        except Exception as exc:
            print(f"WARNING: cannot load {path}: {exc}", file=sys.stderr)
            return {}

    def _create_output_dir(self, base_arg: str) -> Path:
        base = Path(base_arg).expanduser() if base_arg else self.f4 / "tools/records"
        base.mkdir(parents=True, exist_ok=True)
        stamp = datetime.now().astimezone().strftime("%Y%m%d_%H%M%S")
        out = base / f"agv_nav_{stamp}"
        n = 1
        while out.exists():
            out = base / f"agv_nav_{stamp}_{n:02d}"
            n += 1
        out.mkdir(parents=True)
        latest = base / "latest"
        try:
            if latest.is_symlink() or latest.exists():
                latest.unlink()
            latest.symlink_to(out.name)
        except Exception:
            pass
        return out

    def _snapshot_configs(self) -> None:
        for p in [self.nav_cfg / "mag_heading.yaml", self.nav_cfg / "yahboom_mag_calibration.yaml",
                  self.nav_cfg / "imu.yaml", self.f4 / ".pio/build/blackpill_f411ce_neo3pro/firmware.identity.json"]:
            if p.exists():
                try:
                    shutil.copy2(p, self.outdir / p.name)
                except Exception:
                    pass

    def _write_metadata(self, started: bool) -> None:
        files = {}
        for p in self.outdir.iterdir():
            if p.is_file() and p.name not in ("raw.jsonl", "summary.csv", "metadata.json"):
                try:
                    files[p.name] = sha256_file(p)
                except Exception:
                    pass
        meta = {
            "schema": "agv-nav-recorder-v1",
            "started": bool(started),
            "start_wall_utc_ns": self.start_wall_ns,
            "start_utc_iso8601": datetime.fromtimestamp(self.start_wall_ns / 1e9, tz=timezone.utc).isoformat(timespec="microseconds"),
            "stop_wall_utc_ns": None if started else time.time_ns(),
            "hostname": socket.gethostname(),
            "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", ""),
            "workspace_git_head": git_head(self.ws),
            "f4gateway_git_head": git_head(self.f4),
            "raw_prefixes": RAW_PREFIXES,
            "timestamp_contract": {
                "ros_header": "exact ROS message header stamp, sec+nanosec; Nav2/REP-105 consumer time",
                "wall_utc_ns": "Unix UTC nanoseconds from host CLOCK_REALTIME",
                "monotonic_ns": "host monotonic nanoseconds for ordering/latency",
                "neo3pro": "raw Fix2 mcu_ms/network_us/gnss_us/time_standard preserved from /neo3pro/gnss/meta",
                "authority": "F4 bridge promotes Fix2 gnss_us to ROS stamp only when time_standard=2 UTC; otherwise mapped MCU/reception time",
            },
            "xy_contract": "prefer /odometry/filtered_map, then /odometry/gnss_map, else WGS84 local ENU from first valid fix",
            "yaw_contract": "prefer official fusion topics; calibrated diagnostic fallback is displayed but explicitly source-tagged and never promoted to Nav2 authority",
            "snapshot_sha256": files,
        }
        self.meta_path.write_text(json.dumps(meta, indent=2, sort_keys=True))

    def discover_topics(self) -> None:
        for topic, types in self.get_topic_names_and_types():
            if topic in self.subs or not any(topic == p or topic.startswith(p + "/") for p in RAW_PREFIXES):
                continue
            if len(types) != 1:
                continue
            try:
                cls = get_message(types[0])
                self.subs[topic] = self.create_subscription(cls, topic, lambda m, t=topic: self.on_raw(t, m), self.qos)
            except Exception as exc:
                self.get_logger().warning(f"Cannot subscribe {topic} ({types}): {exc}")

    def on_raw(self, topic: str, msg: Any) -> None:
        recv_wall = time.time_ns()
        recv_mono = time.monotonic_ns()
        recv_ros = self.get_clock().now().nanoseconds
        hns = stamp_ns(msg)
        try:
            payload = message_to_ordereddict(msg)
        except Exception:
            payload = {"repr": str(msg)}
        rec = {
            "topic": topic,
            "type": msg.__class__.__module__.replace(".", "/") + "/" + msg.__class__.__name__,
            "wall_utc_ns": recv_wall,
            "utc_iso8601": datetime.fromtimestamp(recv_wall / 1e9, tz=timezone.utc).isoformat(timespec="microseconds"),
            "ros_receive_ns": recv_ros,
            "monotonic_ns": recv_mono,
            "header_stamp_ns": hns,
            "msg": payload,
        }
        try:
            self.raw_f.write(json.dumps(rec, separators=(",", ":"), allow_nan=True) + "\n")
        except Exception as exc:
            self.get_logger().error(f"raw write failed: {exc}")
        self.latest[topic] = (msg, recv_mono)
        self._dispatch(topic, msg, hns)

    def _dispatch(self, topic: str, msg: Any, hns: int) -> None:
        if topic == "/gnss/fix_raw":
            lat, lon, alt = float(msg.latitude), float(msg.longitude), float(msg.altitude)
            if math.isfinite(lat) and math.isfinite(lon) and abs(lat) <= 90.0 and abs(lon) <= 180.0 and not (abs(lat) < 1e-12 and abs(lon) < 1e-12):
                self.fix = msg
                if self.origin is None:
                    self.origin = (lat, lon, alt, geodetic_to_ecef(lat, lon, alt))
                self.gnss_enu = enu_from_origin(lat, lon, alt, self.origin)
        elif topic == "/gnss/quality":
            self.quality = list(msg.data)
        elif topic == "/neo3pro/gnss/meta":
            self.gnss_meta = list(msg.data)
        elif topic == "/neo3pro/gnss/heading_rad":
            self.gnss_heading = float(msg.data)
        elif topic == "/localization/map_yaw_from_enu":
            if math.isfinite(float(msg.data)):
                self.map_yaw = norm_angle(float(msg.data)); self.map_yaw_seen = True
        elif topic == "/imu/data":
            self.roll, self.pitch = roll_pitch_from_q(msg.orientation)
            self.imu_orientation_yaw = yaw_from_q(msg.orientation)
            self.gyro_z = float(msg.angular_velocity.z)
            self._integrate_inertial(hns or self.get_clock().now().nanoseconds)
        elif topic == "/neo3/mag":
            self._neo_raw_heading(msg)
        elif topic == "/imu/mag_raw_lsb":
            self._yah_raw_heading(msg)

    def _integrate_inertial(self, current_ns: int) -> None:
        if not math.isfinite(self.inertial_fallback):
            if math.isfinite(self.neo_diag):
                self.inertial_fallback = self.neo_diag; self.inertial_seed_source = "neo3_diag"
            elif math.isfinite(self.imu_orientation_yaw):
                self.inertial_fallback = self.imu_orientation_yaw; self.inertial_seed_source = "imu_orientation"
            self.inertial_last_stamp_ns = current_ns
            return
        if self.inertial_seed_source != "neo3_diag" and math.isfinite(self.neo_diag):
            self.inertial_fallback = self.neo_diag
            self.inertial_seed_source = "neo3_diag"
            self.inertial_last_stamp_ns = current_ns
            return
        if self.inertial_last_stamp_ns and math.isfinite(self.gyro_z):
            dt = (current_ns - self.inertial_last_stamp_ns) * 1.0e-9
            if 0.0 < dt <= 0.2:
                bias = 0.0
                try:
                    imu_cfg = yaml.safe_load((self.nav_cfg / "imu.yaml").read_text())["data_imu_node"]["ros__parameters"]
                    bias = float(imu_cfg.get("gyro_bias", [0.0, 0.0, 0.0])[2])
                except Exception:
                    pass
                self.inertial_fallback = norm_angle(self.inertial_fallback + (self.gyro_z - bias) * dt)
        self.inertial_last_stamp_ns = current_ns

    def _neo_raw_heading(self, msg: Any) -> None:
        p = self.mag_params
        mx = float(msg.magnetic_field.x) * 1e6
        my = float(msg.magnetic_field.y) * 1e6
        mz = float(msg.magnetic_field.z) * 1e6
        self.neo_norm = math.sqrt(mx * mx + my * my + mz * mz)
        bx, by = p.get("neo3_mag_bias_xy_ut", [0.0, 0.0])[:2]
        mat = p.get("neo3_mag_matrix_xy", [1.0, 0.0, 0.0, 1.0])
        qx = float(mat[0]) * (mx - float(bx)) + float(mat[1]) * (my - float(by))
        qy = float(mat[2]) * (mx - float(bx)) + float(mat[3]) * (my - float(by))
        cr, sr, cp, sp = math.cos(self.roll), math.sin(self.roll), math.cos(self.pitch), math.sin(self.pitch)
        xh = qx * cp + mz * sp
        yh = qx * sr * sp + qy * cr - mz * sr * cp
        if math.hypot(xh, yh) < 1e-9:
            return
        yaw = norm_angle(float(p.get("neo3_mag_yaw_sign", 1.0)) * math.atan2(yh, xh) +
                         float(p.get("neo3_mag_yaw_offset_rad", 0.0)) - float(p.get("magnetic_declination_rad", 0.0)))
        if p.get("neo3_heading_lut_enabled", False):
            yaw = apply_lut(yaw, p.get("neo3_heading_lut_input_rad", []), p.get("neo3_heading_lut_correction_rad", []))
        self.neo_diag = norm_angle(yaw + self.map_yaw)
        max_tilt = float(p.get("neo3_planar_max_tilt_rad", 0.2617993878))
        self.neo_diag_valid = (15.0 <= self.neo_norm <= 100.0 and abs(self.roll) <= max_tilt and abs(self.pitch) <= max_tilt)

    def _yah_raw_heading(self, msg: Any) -> None:
        if len(msg.data) < 3:
            return
        p = self.mag_params
        x, y = float(msg.data[0]), float(msg.data[1])
        bx, by = p.get("imu_mag_bias_xy_lsb", [0.0, 0.0])[:2]
        m = p.get("imu_mag_matrix_xy_per_lsb", [1.0, 0.0, 0.0, 1.0])
        qx = float(m[0]) * (x - float(bx)) + float(m[1]) * (y - float(by))
        qy = float(m[2]) * (x - float(bx)) + float(m[3]) * (y - float(by))
        self.yah_norm = math.hypot(qx, qy)
        yaw = norm_angle(float(p.get("imu_mag_yaw_sign", 1.0)) * math.atan2(qy, qx) +
                         float(p.get("imu_mag_yaw_offset_rad", 0.0)) - float(p.get("magnetic_declination_rad", 0.0)))
        if p.get("imu_heading_lut_enabled", False):
            yaw = apply_lut(yaw, p.get("imu_heading_lut_input_rad", []), p.get("imu_heading_lut_correction_rad", []))
        self.yah_diag = norm_angle(yaw + self.map_yaw)
        lo = float(p.get("imu_corrected_norm_min", 0.75)); hi = float(p.get("imu_corrected_norm_max", 1.25))
        max_tilt = float(p.get("imu_planar_max_tilt_rad", 0.2617993878))
        self.yah_diag_valid = lo <= self.yah_norm <= hi and abs(self.roll) <= max_tilt and abs(self.pitch) <= max_tilt

    def _fresh(self, topic: str, max_age: float = 0.75) -> Optional[Any]:
        item = self.latest.get(topic)
        if not item or (time.monotonic_ns() - item[1]) * 1e-9 > max_age:
            return None
        return item[0]

    def _yaw_choice(self, topic: str, fallback: float, fallback_source: str, valid: bool) -> Tuple[float, str, bool]:
        m = self._fresh(topic)
        if m is not None:
            try:
                return yaw_from_q(m.pose.pose.orientation), topic, True
            except Exception:
                pass
        return fallback, fallback_source, bool(valid and math.isfinite(fallback))

    def _xy(self) -> Tuple[float, float, str]:
        for topic, source in (("/odometry/filtered_map", "nav2_filtered_map"), ("/odometry/gnss_map", "gnss_map")):
            m = self._fresh(topic, 1.5)
            if m is not None:
                return float(m.pose.pose.position.x), float(m.pose.pose.position.y), source
        return float(self.gnss_enu[0]), float(self.gnss_enu[1]), "gnss_local_enu"

    def sample(self) -> None:
        now_wall = time.time_ns(); now_mono = time.monotonic_ns(); ros_now = self.get_clock().now().nanoseconds
        x, y, xy_src = self._xy()
        neo_yaw, neo_src, neo_valid = self._yaw_choice("/neo3/mag_heading_fusion", self.neo_diag, "neo3_calibrated_diag", self.neo_diag_valid)
        yah_yaw, yah_src, yah_valid = self._yaw_choice("/imu/mag_heading_fusion", self.yah_diag, "yahboom_calibrated_diag", self.yah_diag_valid)
        inert_msg = self._fresh("/imu/inertial_heading")
        if inert_msg is not None:
            inert_yaw, inert_src, inert_valid = yaw_from_q(inert_msg.pose.pose.orientation), "/imu/inertial_heading", True
        else:
            inert_yaw, inert_src, inert_valid = self.inertial_fallback, "logger_gyro_integral:" + self.inertial_seed_source, math.isfinite(self.inertial_fallback)

        lat = float(self.fix.latitude) if self.fix is not None else float("nan")
        lon = float(self.fix.longitude) if self.fix is not None else float("nan")
        alt = float(self.fix.altitude) if self.fix is not None else float("nan")
        fix_stamp = stamp_ns(self.fix) if self.fix is not None else 0
        q = self.quality
        hacc = float(q[2]) if len(q) > 2 else float("nan")
        fix_type = int(q[3]) if len(q) > 3 and math.isfinite(q[3]) else -1
        sats = int(q[0]) if len(q) > 0 and math.isfinite(q[0]) else -1
        ts_source = int(q[24]) if len(q) > 24 and math.isfinite(q[24]) else -1
        meta = self.gnss_meta
        mcu_ms = int(meta[1]) if len(meta) > 1 and math.isfinite(meta[1]) else -1
        network_us = int(meta[3]) if len(meta) > 3 and math.isfinite(meta[3]) else -1
        gnss_us = int(meta[4]) if len(meta) > 4 and math.isfinite(meta[4]) else -1
        time_std = int(meta[5]) if len(meta) > 5 and math.isfinite(meta[5]) else -1
        measurement_ns = fix_stamp or ros_now
        row = {
            "wall_utc_ns": now_wall,
            "utc_iso8601": datetime.fromtimestamp(now_wall / 1e9, tz=timezone.utc).isoformat(timespec="microseconds"),
            "ros_now_ns": ros_now, "monotonic_ns": now_mono,
            "measurement_stamp_ns": measurement_ns,
            "measurement_stamp_sec": measurement_ns // 1_000_000_000,
            "measurement_stamp_nanosec": measurement_ns % 1_000_000_000,
            "f411_mcu_ms": mcu_ms, "dronecan_network_us": network_us,
            "dronecan_gnss_us": gnss_us, "dronecan_time_standard": time_std,
            "timestamp_source_code": ts_source,
            "x_m": x, "y_m": y, "xy_source": xy_src,
            "gnss_east_m": self.gnss_enu[0], "gnss_north_m": self.gnss_enu[1],
            "longitude_deg": lon, "latitude_deg": lat, "altitude_m": alt,
            "hacc_m": hacc, "fix_type": fix_type, "satellites": sats,
            "yaw_neo3_deg": deg(neo_yaw), "yaw_neo3_source": neo_src, "yaw_neo3_valid": int(neo_valid),
            "yaw_yahboom_deg": deg(yah_yaw), "yaw_yahboom_source": yah_src, "yaw_yahboom_valid": int(yah_valid),
            "yaw_inertial_deg": deg(inert_yaw), "yaw_inertial_source": inert_src, "yaw_inertial_valid": int(inert_valid),
            "yaw_gnss_heading_deg": deg(self.gnss_heading), "yaw_imu_orientation_deg": deg(self.imu_orientation_yaw),
            "gyro_z_rps": self.gyro_z, "neo3_mag_norm_ut": self.neo_norm,
            "yahboom_corrected_norm": self.yah_norm, "map_yaw_from_enu_deg": deg(self.map_yaw),
        }
        self.sum_w.writerow(row)
        self._dashboard(row)

    @staticmethod
    def _fmt(v: Any, n: int = 3) -> str:
        try:
            x = float(v)
            return f"{x:.{n}f}" if math.isfinite(x) else "--"
        except Exception:
            return "--"

    def _dashboard(self, r: Dict[str, Any]) -> None:
        if sys.stdout.isatty():
            print("\033[2J\033[H", end="")
        else:
            if time.monotonic_ns() - self.last_dashboard_ns < 1_000_000_000:
                return
        self.last_dashboard_ns = time.monotonic_ns()
        utc = r["utc_iso8601"]
        print(f"AGV SENSOR RECORDER  [RECORDING]  {utc}")
        print(f"save: {self.outdir}")
        print(f"x={self._fmt(r['x_m'])} m   y={self._fmt(r['y_m'])} m   source={r['xy_source']}")
        print(f"lon={self._fmt(r['longitude_deg'],8)}   lat={self._fmt(r['latitude_deg'],8)}   hAcc={self._fmt(r['hacc_m'])} m   sat={r['satellites']} fix={r['fix_type']}")
        print(f"yaw NEO3    = {self._fmt(r['yaw_neo3_deg'],3)} deg   valid={r['yaw_neo3_valid']}   [{r['yaw_neo3_source']}]")
        print(f"yaw Yahboom = {self._fmt(r['yaw_yahboom_deg'],3)} deg   valid={r['yaw_yahboom_valid']}   [{r['yaw_yahboom_source']}]")
        print(f"yaw inertia = {self._fmt(r['yaw_inertial_deg'],3)} deg   valid={r['yaw_inertial_valid']}   [{r['yaw_inertial_source']}]")
        print(f"ROS stamp={r['measurement_stamp_sec']}.{int(r['measurement_stamp_nanosec']):09d}  F411={r['f411_mcu_ms']}ms  Fix2 UTC us={r['dronecan_gnss_us']}  time_std={r['dronecan_time_standard']} src={r['timestamp_source_code']}")
        print(f"raw topics subscribed={len(self.subs)}   Ctrl+C = save & close", flush=True)

    def flush(self) -> None:
        try:
            self.raw_f.flush(); self.sum_f.flush()
        except Exception:
            pass

    def close_files(self) -> None:
        try:
            self.flush()
            os.fsync(self.raw_f.fileno()); os.fsync(self.sum_f.fileno())
        except Exception:
            pass
        try:
            self.raw_f.close(); self.sum_f.close()
        except Exception:
            pass
        self._write_metadata(started=False)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="Record AGV NEO3PRO/Yahboom/Nav2 navigation data")
    p.add_argument("--output-dir", default="", help="base directory; default F4gateway/tools/records")
    p.add_argument("--rate", type=float, default=10.0, help="summary/display rate Hz (default 10)")
    p.add_argument("--duration", type=float, default=0.0, help="optional seconds; 0=until Ctrl+C")
    return p.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = Recorder(args)
    deadline = time.monotonic() + args.duration if args.duration > 0 else None
    try:
        while rclpy.ok() and (deadline is None or time.monotonic() < deadline):
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_files()
        print(f"\nSAVED summary: {node.summary_path}")
        print(f"SAVED raw:     {node.raw_path}")
        print(f"SAVED meta:    {node.meta_path}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
