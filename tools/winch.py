#!/usr/bin/env python3
"""Standalone STM32 winch terminal over USB CDC. No ROS is used."""
import argparse
import glob
import fcntl
import os
import re
import select
import signal
import subprocess
import sys
import termios
import time
import tty
from pathlib import Path

import serial

RUNTIME_GLOBS = (
    "/dev/serial/by-id/usb-STMicroelectronics_BLUEPILL_F103_CDC_in_FS_Mode*-if00",
    "/dev/serial/by-path/platform-3610000.usb-usb-0:2.2:1.0",
    "/dev/ttyACM*",
)
LIMIT_RE = re.compile(
    r"LIMITS:READY=(\d+):TOP=(\d+):BOTTOM=(\d+)"
    r"(?::TOP_RAW=(\d+):BOTTOM_RAW=(\d+))?"
)
STATUS_RE = re.compile(
    r"WINCH:STATE=([^:]+):PWM=(\d+):APPLIED=(\d+):TOP=(\d+):BOTTOM=(\d+)"
    r":SW_TICKS=(\d+):SW_DIR=(-?\d+)"
)
GATEWAY_BIN = "/home/otomasi2/forclift/install/f4gateway/lib/f4gateway/f4gateway_node"
CDC_OWNER_LOCK = "/tmp/f4gateway_cdc_owner.lock"


def repo_gateway_pids():
    pids = []
    proc = Path("/proc")
    for entry in proc.iterdir():
        if not entry.name.isdigit():
            continue
        try:
            cmd = (entry / "cmdline").read_bytes().replace(b"\0", b" ").decode(errors="replace")
        except OSError:
            continue
        if cmd.startswith(GATEWAY_BIN + " ") or cmd == GATEWAY_BIN:
            pids.append(int(entry.name))
    return pids


def stop_repo_gateway():
    pids = repo_gateway_pids()
    for pid in pids:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic() + 0.75
    while time.monotonic() < deadline:
        alive = repo_gateway_pids()
        if not alive:
            return
        time.sleep(0.03)
    alive = repo_gateway_pids()
    if alive:
        raise RuntimeError(
            "f4gateway_node tidak berhenti secara graceful: PID " +
            ",".join(str(x) for x in alive)
        )

def find_device(explicit=None):
    if explicit:
        return explicit if os.path.exists(explicit) else None
    seen = set()
    for pattern in RUNTIME_GLOBS:
        for path in sorted(glob.glob(pattern)):
            real = os.path.realpath(path)
            if real in seen:
                continue
            seen.add(real)
            if "BOOT_CDC" in path:
                continue
            return path
    return None


def port_holders(path):
    real = os.path.realpath(path)
    cp = subprocess.run(
        ["fuser", real], stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL, text=True, check=False
    )
    return [int(x) for x in cp.stdout.split() if x.isdigit()]


def pid_cmdline(pid):
    try:
        return Path(f"/proc/{pid}/cmdline").read_bytes().replace(
            b"\0", b" "
        ).decode(errors="replace")
    except OSError:
        return ""
def release_gateway_holder(path):
    # Only terminate the known F4 gateway when it actually owns this CDC port.
    # Do not wait for the respawned process itself to disappear; once the port
    # is free the caller immediately claims it with exclusive=True.
    deadline = time.monotonic() + 2.0
    signaled = set()
    while time.monotonic() < deadline:
        holders = [p for p in port_holders(path) if p != os.getpid()]
        if not holders:
            return
        unknown = []
        for pid in holders:
            cmd = pid_cmdline(pid)
            if cmd.startswith(GATEWAY_BIN + " ") or cmd == GATEWAY_BIN:
                if pid not in signaled:
                    try:
                        os.kill(pid, signal.SIGTERM)
                        signaled.add(pid)
                    except ProcessLookupError:
                        pass
            else:
                unknown.append((pid, cmd))
        if unknown:
            detail = "; ".join(f"PID {pid}: {cmd}" for pid, cmd in unknown)
            raise RuntimeError(f"CDC dipakai proses lain, tidak dihentikan: {detail}")
        time.sleep(0.04)
    holders = [p for p in port_holders(path) if p != os.getpid()]
    detail = "; ".join(f"PID {pid}: {pid_cmdline(pid)}" for pid in holders)
    raise RuntimeError(f"f4gateway_node belum melepas CDC setelah 2 detik: {detail}")


class DirectWinch:
    def __init__(self, device=None):
        self.requested_device = device
        self.device = None
        self.ser = None
        self._lock_fd = None
        self.connected = False
        self.state = "UNKNOWN"
        self.configured_pwm = 0
        self.applied_pwm = 0
        self.sw_dir = 0
        self.limit_ready = None
        self.top = None
        self.bottom = None
        self.top_raw = None
        self.bottom_raw = None
        self.last_error = ""
        self.last_line = ""
        self._token = (
            ((os.getpid() << 12) ^ int(time.monotonic() * 1000) ^ 0x57494E43)
            & 0xFFFFFFFF
        ) or 1

    def _close_serial(self):
        if self.ser is not None:
            try:
                self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connected = False

    def _acquire_owner_lock(self):
        if self._lock_fd is not None:
            return
        fd = os.open(CDC_OWNER_LOCK, os.O_CREAT | os.O_RDWR, 0o644)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError:
            os.close(fd)
            raise RuntimeError("CDC ownership lock sedang dipakai")
        self._lock_fd = fd

    def _release_owner_lock(self):
        if self._lock_fd is None:
            return
        try:
            fcntl.flock(self._lock_fd, fcntl.LOCK_UN)
        finally:
            os.close(self._lock_fd)
            self._lock_fd = None

    def close(self):
        self._close_serial()
        self._release_owner_lock()

    def connect(self, timeout=15.0):
        self.close()
        deadline = time.monotonic() + timeout
        last = "runtime CDC belum muncul"
        recovery_sent = False
        while time.monotonic() < deadline:
            path = find_device(self.requested_device)
            if not path:
                time.sleep(0.10)
                continue
            try:
                release_gateway_holder(path)
                self._acquire_owner_lock()
                self.ser = serial.Serial(
                    path, 1000000, timeout=0.05, write_timeout=0.5,
                    exclusive=True,
                )
                self.device = path
                time.sleep(0.15)
                self._resync()
                self._handshake()
                self.connected = True
                self.refresh_status()
                return True
            except Exception as exc:
                last = str(exc)
                # HOST:HELLO timeout commonly means CDC OUT still works while
                # the IN endpoint/session is stale. USB:RECOVER is intentionally
                # accepted before a host session and forces physical re-enumeration.
                if self.ser is not None and "HOST:HELLO" in last and not recovery_sent:
                    try:
                        self.ser.write(b"USB:RECOVER\n")
                        self.ser.flush()
                        recovery_sent = True
                        time.sleep(0.10)
                    except Exception:
                        pass
                self._close_serial()
                time.sleep(0.12)
        self.last_error = last
        self._release_owner_lock()
        return False

    def _resync(self):
        for _ in range(3):
            self.ser.write(b"\n")
            self.ser.flush()
            time.sleep(0.03)
        quiet = time.monotonic()
        deadline = quiet + 0.8
        while time.monotonic() < deadline:
            if self.ser.in_waiting:
                self.ser.read(self.ser.in_waiting)
                quiet = time.monotonic()
            elif time.monotonic() - quiet >= 0.15:
                break
            time.sleep(0.01)
        self.ser.reset_input_buffer()
    def _handshake(self):
        expected = f"ACK:HOST:SESSION:{self._token}:"
        self._write(f"HOST:HELLO:{self._token}")
        line = self._wait_for((expected,), 2.5)
        if not line:
            raise RuntimeError("timeout HOST:HELLO")
        self._write("PING")
        line = self._wait_for(("ACK:PONG",), 1.5)
        if not line:
            raise RuntimeError("timeout ACK:PONG")

    def _write(self, command):
        if self.ser is None:
            raise RuntimeError("CDC belum terbuka")
        self.ser.write((command + "\n").encode())
        self.ser.flush()

    def _read_line(self):
        raw = self.ser.readline()
        if not raw:
            return None
        line = raw.decode(errors="replace").strip()
        if line:
            self.last_line = line
            self._parse(line)
        return line

    def _parse(self, line):
        m = LIMIT_RE.search(line)
        if m:
            self.limit_ready = bool(int(m.group(1)))
            self.top = bool(int(m.group(2)))
            self.bottom = bool(int(m.group(3)))
            if m.group(4) is not None:
                self.top_raw = bool(int(m.group(4)))
                self.bottom_raw = bool(int(m.group(5)))
            return
        m = STATUS_RE.search(line)
        if m:
            self.state = m.group(1)
            self.configured_pwm = int(m.group(2))
            self.applied_pwm = int(m.group(3))
            self.top = bool(int(m.group(4)))
            self.bottom = bool(int(m.group(5)))
            self.sw_dir = int(m.group(7))

    def _wait_for(self, prefixes, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            line = self._read_line()
            if not line:
                continue
            if any(line.startswith(p) for p in prefixes):
                return line
            if line.startswith("ERR:"):
                return line
        return None

    def transact(self, command, prefixes, timeout=1.5):
        if not self.connected and not command.startswith("HOST:HELLO:"):
            raise RuntimeError("CDC belum connected")
        try:
            self._write(command)
            line = self._wait_for(prefixes, timeout)
            if line is None:
                raise RuntimeError(f"timeout menunggu response: {command}")
            return line
        except (OSError, serial.SerialException) as exc:
            self.close()
            raise RuntimeError(f"USB CDC terputus: {exc}") from exc

    def refresh_status(self):
        if self.ser is None:
            return False
        try:
            self._write("LIMITS")
            self._wait_for(("LIMITS:",), 0.25)
            self._write("WINCH STATUS")
            self._wait_for(("WINCH:STATE=",), 0.25)
            return True
        except Exception as exc:
            self.last_error = str(exc)
            self.close()
            return False

    def status_text(self):
        port = os.path.basename(os.path.realpath(self.device)) if self.device else "-"
        top = "?" if self.top is None else ("ON" if self.top else "OFF")
        bottom = "?" if self.bottom is None else ("ON" if self.bottom else "OFF")
        top_raw = "?" if self.top_raw is None else ("LOW" if self.top_raw else "HIGH")
        bottom_raw = "?" if self.bottom_raw is None else ("LOW" if self.bottom_raw else "HIGH")
        if self.top and self.bottom:
            ready = "FAULT(BOTH_ACTIVE)"
        else:
            ready = "?" if self.limit_ready is None else ("OK" if self.limit_ready else "FAULT")
        direction = {1: "UP", -1: "DOWN", 0: "STOP"}.get(self.sw_dir, str(self.sw_dir))
        active_pin = {1: "PB8(RPWM)", -1: "PA3(LPWM)", 0: "-"}.get(self.sw_dir, "?")
        pct = (100.0 * self.applied_pwm / 1023.0) if self.applied_pwm else 0.0
        return (
            f"CDC={'ON' if self.connected else 'OFF'}({port})  "
            f"STATE={self.state}  DIR={direction} PIN={active_pin}  "
            f"PWM={self.applied_pwm}/{self.configured_pwm}({pct:.1f}%)  "
            f"LIMIT_TOP={top}  LIMIT_BOTTOM={bottom}  "
            f"RAW_PB6={top_raw} RAW_PB7={bottom_raw}  LIMITS={ready}"
        )

    def local_command(self, payload):
        line = self.transact(
            "LOCAL " + payload,
            ("ACK:WINCH:LOCAL:", "ERR:WINCH:LOCAL:"),
            1.0,
        )
        self.refresh_status()
        return line
    def stop(self):
        try:
            return self.local_command("STOP")
        except Exception:
            return "STOP gagal"

    def timed_move(self, direction, seconds):
        if seconds < 1 or seconds > 30 or int(seconds) != seconds:
            return "Durasi harus bilangan bulat 1..30 detik."
        total = int(seconds)
        name = "UP" if direction > 0 else "DOWN"
        if direction > 0 and self.top:
            return "Ditolak: limit atas sudah aktif."
        if direction < 0 and self.bottom:
            return "Ditolak: limit bawah sudah aktif."

        reply = self.local_command(f"{name} {total}")
        if not reply.startswith("ACK:"):
            return reply

        started = time.monotonic()
        end = started + total + 0.15
        next_poll = 0.0
        motion_seen = False
        while time.monotonic() < end:
            now = time.monotonic()
            if now >= next_poll:
                self.refresh_status()
                if self.sw_dir == direction and self.applied_pwm > 0:
                    motion_seen = True
                elapsed = min(total, int(now - started) + 1)
                sys.stdout.write(
                    "\r\033[2K" + self.status_text()
                    + f"  | {name} {elapsed}/{total} s"
                )
                sys.stdout.flush()
                next_poll = now + 0.20
            if direction > 0 and self.top:
                self.stop()
                return "Selesai: limit atas aktif."
            if direction < 0 and self.bottom:
                self.stop()
                return "Selesai: limit bawah aktif."
            if self.sw_dir == 0 and (now - started) > 0.25:
                break
            time.sleep(0.03)
        self.refresh_status()
        if not motion_seen:
            return (f"GAGAL: {name} tidak pernah mengaktifkan DIR/PWM. "
                    "Cek status limit/fault dan jalur output.")
        return f"{name} {total} detik selesai."

    def execute(self, text):
        words = text.strip().lower().split()
        if not words:
            return True, ""
        if words[0] in ("q", "quit", "exit"):
            self.stop()
            return False, "STOP / keluar."
        if words[0] == "help":
            return True, "Commands: up N | down N | up home | down home | stop | status | fault clear | quit"
        if words[0] == "status":
            self.refresh_status()
            return True, self.status_text()
        if words[0] == "stop":
            return True, self.stop()
        if words == ["fault", "clear"]:
            line = self.transact(
                "WINCH FAULT CLEAR",
                ("ACK:WINCH:FAULT_CLEAR", "ERR:WINCH:FAULT_CLEAR"),
                1.0,
            )
            self.refresh_status()
            return True, line
        if len(words) == 2 and words[0] in ("up", "down") and words[1] == "home":
            direction = "UP" if words[0] == "up" else "DOWN"
            if direction == "UP" and self.top:
                return True, "Ditolak: limit atas sudah aktif."
            if direction == "DOWN" and self.bottom:
                return True, "Ditolak: limit bawah sudah aktif."
            return True, self.local_command(f"{direction} HOME")
        if len(words) == 2 and words[0] in ("up", "down"):
            try:
                seconds = float(words[1])
            except ValueError:
                return True, "Format salah. Contoh: up 1"
            msg = self.timed_move(1 if words[0] == "up" else -1, seconds)
            return True, msg
        return True, "Command tidak dikenal. Ketik: help"
def interactive(winch):
    print("Winch USB CDC standalone - TANPA ROS.")
    print("Commands: up N | down N | up home | down home | stop | status | fault clear | help | quit")
    print(winch.status_text())
    if not sys.stdin.isatty():
        for line in sys.stdin:
            keep, msg = winch.execute(line)
            if msg:
                print(msg)
            if not keep:
                break
        return

    old = termios.tcgetattr(sys.stdin.fileno())
    buffer = ""
    last_poll = 0.0
    try:
        tty.setcbreak(sys.stdin.fileno())
        while True:
            now = time.monotonic()
            if now - last_poll >= 0.35:
                if not winch.refresh_status():
                    sys.stdout.write("\r\033[2KCDC terputus, reconnect...")
                    sys.stdout.flush()
                    winch.connect(10.0)
                last_poll = now
            sys.stdout.write(
                "\r\033[2K" + winch.status_text() + "  |  winch> " + buffer
            )
            sys.stdout.flush()
            ready, _, _ = select.select([sys.stdin], [], [], 0.10)
            if not ready:
                continue
            ch = sys.stdin.read(1)
            if ch in ("\r", "\n"):
                command, buffer = buffer, ""
                sys.stdout.write("\r\033[2K")
                sys.stdout.flush()
                keep, msg = winch.execute(command)
                if msg:
                    print(msg)
                if not keep:
                    break
            elif ch == "\x03":
                raise KeyboardInterrupt
            elif ch == "\x04" and not buffer:
                break
            elif ch in ("\x7f", "\b"):
                buffer = buffer[:-1]
            elif ch.isprintable():
                buffer += ch
    finally:
        termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old)
        sys.stdout.write("\r\033[2K")
        sys.stdout.flush()


def main():
    ap = argparse.ArgumentParser(description="Standalone USB CDC winch terminal")
    ap.add_argument("--device", default=None)
    ap.add_argument("--command", default=None, help='e.g. --command "up 1"')
    args = ap.parse_args()

    winch = DirectWinch(args.device)
    print("Mencari STM32 USB CDC...")
    if not winch.connect(15.0):
        print(f"ERROR: STM32 tidak terkoneksi: {winch.last_error}", file=sys.stderr)
        return 2
    print(f"CONNECTED: {winch.device}")
    print(winch.status_text())
    try:
        if args.command:
            _, msg = winch.execute(args.command)
            if msg:
                print(msg)
            print(winch.status_text())
            return 0 if not msg.startswith(("ERR:", "Ditolak", "STOP gagal")) else 3
        interactive(winch)
        return 0
    except KeyboardInterrupt:
        print("\nCtrl-C: kirim STOP...")
        try:
            print(winch.stop())
        except Exception as exc:
            print(f"STOP warning: {exc}")
        return 130
    finally:
        winch.close()


if __name__ == "__main__":
    raise SystemExit(main())
