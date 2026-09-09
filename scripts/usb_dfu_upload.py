Import("env")

import fcntl
import glob
import os
import signal
import subprocess
import time

DFU_VID = "0483"
DFU_PID = "df11"
MANUAL_DFU_WAIT_S = float(os.environ.get("F411_DFU_MANUAL_WAIT_S", "60.0"))
UPLOAD_LOCK = "/tmp/adv_f411_dfu_upload.lock"
USB_ONLY = os.environ.get("F411_USB_ONLY", "0").strip().lower() in ("1", "true", "yes", "on")
_upload_lock_fd = None

AGV_ROOT = os.path.abspath(os.environ.get("AGV_ROOT", os.path.join(os.path.expanduser("~"), "agv")))
ROS_PROCESS_MARKERS = (
    "/opt/ros/",
    os.path.join(AGV_ROOT, "install") + os.sep,
    "ros2 launch ",
    "ros2 run ",
)

def _protected_pids():
    protected = {os.getpid()}
    pid = os.getppid()
    while pid > 1 and pid not in protected:
        protected.add(pid)
        try:
            with open(f"/proc/{pid}/stat", "r", encoding="ascii") as f:
                pid = int(f.read().split()[3])
        except Exception:
            break
    return protected

def _proc_cmdline(pid):
    try:
        raw = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ")
        return raw.decode(errors="replace").strip()
    except OSError:
        return ""

def _stop_ros_processes():
    protected = _protected_pids()
    victims = []
    for name in os.listdir("/proc"):
        if not name.isdigit():
            continue
        pid = int(name)
        if pid in protected:
            continue
        cmd = _proc_cmdline(pid)
        if cmd and any(marker in cmd for marker in ROS_PROCESS_MARKERS):
            victims.append(pid)
    if not victims:
        print("[USB-DFU] ROS stack already stopped")
        return
    print(f"[USB-DFU] stopping ROS processes: {victims}")
    for pid in victims:
        try: os.kill(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError): pass
    deadline = time.monotonic() + 4.0
    while time.monotonic() < deadline:
        alive = [pid for pid in victims if os.path.exists(f"/proc/{pid}")]
        if not alive:
            return
        time.sleep(0.10)
    alive = [pid for pid in victims if os.path.exists(f"/proc/{pid}")]
    if alive:
        print(f"[USB-DFU] force-stopping remaining ROS processes: {alive}")
    for pid in alive:
        try: os.kill(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError): pass
    time.sleep(0.20)


def _acquire_upload_lock():
    global _upload_lock_fd
    if _upload_lock_fd is not None:
        return
    fd = os.open(UPLOAD_LOCK, os.O_CREAT | os.O_RDWR, 0o660)
    try:
        fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        os.close(fd)
        raise RuntimeError("another F411 DFU upload is already active")
    os.ftruncate(fd, 0)
    os.write(fd, f"pid={os.getpid()} started={time.time():.3f}\n".encode())
    os.fsync(fd)
    _upload_lock_fd = fd



def _prepare_dfu_tool():
    root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    script = os.path.join(root, "scripts", "build_dfu_util_blackpill.sh")
    if not os.path.isfile(script):
        raise RuntimeError(f"DFU tool build script missing: {script}")
    print("[USB-DFU] preparing verified DFU utility while runtime application is still alive")
    cp = subprocess.run([script], stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                        text=True, timeout=180.0, check=False)
    if cp.returncode != 0:
        print(cp.stdout)
        raise RuntimeError(f"DFU utility preparation failed rc={cp.returncode}")

def _port_holders(port):
    real = os.path.realpath(port)
    try:
        result = subprocess.run(
            ["fuser", real], stdout=subprocess.PIPE, stderr=subprocess.DEVNULL,
            text=True, check=False, timeout=2.0)
    except Exception:
        return []
    holders = []
    for token in result.stdout.split():
        if not token.isdigit():
            continue
        pid = int(token)
        if pid == os.getpid():
            continue
        try:
            raw = open(f"/proc/{pid}/cmdline", "rb").read().replace(b"\0", b" ")
            cmd = raw.decode(errors="replace").strip()
        except OSError:
            cmd = ""
        holders.append((pid, cmd))
    return holders


def _release_cdc_holders(port):
    holders = _port_holders(port)
    if not holders:
        return
    protected = _protected_pids()
    holders = [(pid, cmd) for pid, cmd in holders if pid not in protected]
    if not holders:
        return
    pids = [pid for pid, _ in holders]
    print(f"[USB-DFU] releasing F411 CDC holders pid={pids}")
    for pid in pids:
        try: os.kill(pid, signal.SIGTERM)
        except (ProcessLookupError, PermissionError): pass
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        if not any(pid in pids for pid, _ in _port_holders(port)):
            return
        time.sleep(0.05)
    for pid in pids:
        try: os.kill(pid, signal.SIGKILL)
        except (ProcessLookupError, PermissionError): pass
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline:
        if not any(pid in pids for pid, _ in _port_holders(port)):
            return
        time.sleep(0.05)
    raise RuntimeError(f"F411 CDC still held after stopping pid={pids}")


def _usb_nodes(vid, pid):
    nodes = []
    for vendor_file in glob.glob("/sys/bus/usb/devices/*/idVendor"):
        devdir = os.path.dirname(vendor_file)
        try:
            with open(vendor_file, "r", encoding="ascii") as f:
                vendor = f.read().strip().lower()
            with open(os.path.join(devdir, "idProduct"), "r", encoding="ascii") as f:
                product = f.read().strip().lower()
            if vendor != vid.lower() or product != pid.lower():
                continue
            with open(os.path.join(devdir, "busnum"), "r", encoding="ascii") as f:
                busnum = int(f.read().strip())
            with open(os.path.join(devdir, "devnum"), "r", encoding="ascii") as f:
                devnum = int(f.read().strip())
            nodes.append(f"/dev/bus/usb/{busnum:03d}/{devnum:03d}")
        except (OSError, ValueError):
            continue
    return nodes


def _find_cdc_port():
    configured = env.GetProjectOption("custom_usb_cdc_port", "").strip()
    candidates = []
    if configured:
        candidates.append(configured)
    candidates.extend(sorted(glob.glob(
        "/dev/serial/by-id/usb-STMicroelectronics_BLACKPILL_F411CE_CDC_in_FS_Mode*-if00"
    )))
    # Intentionally no generic /dev/ttyACM* fallback: another CDC device may
    # be present. Only the configured/by-id BlackPill endpoint is safe to trigger.
    for path in candidates:
        if os.path.exists(path):
            return path
    return None


def _assert_dfu_permission(nodes, settle_s=3.0):
    deadline = time.monotonic() + settle_s
    while True:
        writable = [n for n in nodes if os.path.exists(n) and os.access(n, os.R_OK | os.W_OK)]
        if writable:
            return
        if time.monotonic() >= deadline:
            break
        time.sleep(0.10)
    joined = ", ".join(nodes) if nodes else "unknown USB node"
    raise RuntimeError(
        "STM32 ROM DFU is active, but Linux cannot write " + joined + ". "
        "Run ./scripts/install_stm32_udev_rules.sh once with sudo, "
        "then reconnect/reset the board into DFU and rerun pio run -t upload."
    )


def _wait_for_dfu(seconds, manual_hint=False):
    if manual_hint:
        print("[USB-DFU] Recovery otomatis belum berhasil memunculkan ROM DFU.")
        print("[USB-DFU] Masuk ROM DFU sekali ini via USB:")
        print("          1) tahan tombol BOOT0")
        print("          2) tekan-lepas NRST/RESET")
        print("          3) lepas BOOT0")
        print(f"[USB-DFU] Menunggu 0483:df11 sampai {int(seconds)} detik ...")

    deadline = time.time() + seconds
    last_tick = -1
    while time.time() < deadline:
        nodes = _usb_nodes(DFU_VID, DFU_PID)
        if nodes:
            print("[USB-DFU] STM32 ROM DFU ready:", ", ".join(nodes))
            _assert_dfu_permission(nodes)
            time.sleep(0.35)
            return True
        if manual_hint:
            remaining = int(deadline - time.time())
            tick = remaining // 10
            if tick != last_tick and remaining > 0:
                last_tick = tick
                print(f"[USB-DFU] waiting... {remaining}s")
        time.sleep(0.10)
    return False


def _request_software_dfu(port):
    print(f"[USB-DFU] Requesting ROM DFU through {port} ...")
    try:
        import serial

        def collect_until(ser, needle, seconds):
            deadline = time.monotonic() + seconds
            chunks = []
            text = ""
            while time.monotonic() < deadline:
                try:
                    waiting = ser.in_waiting
                    if waiting:
                        chunks.append(ser.read(min(waiting, 1024)))
                        text = b"".join(chunks).decode(errors="replace")
                        if needle in text:
                            return text
                except OSError:
                    break
                time.sleep(0.01)
            return text

        with serial.Serial(port, 1000000, timeout=0.03, write_timeout=1.0) as ser:
            time.sleep(0.15)
            ser.reset_input_buffer()
            ser.write(b"\n\n\n")
            ser.flush()
            time.sleep(0.05)
            ser.reset_input_buffer()

            reply = ""
            for _ in range(3):
                ser.write(b"BOOT:DFU:ARM\n")
                ser.flush()
                reply = collect_until(ser, "ACK:DFU:ARMED", 0.55)
                if "ACK:DFU:ARMED" in reply:
                    break
                ser.reset_input_buffer()
            if reply:
                tail = reply[-800:].replace("\r", " ").replace("\n", " | ")
                print("[USB-DFU] CDC reply tail:", tail)
            if "ACK:DFU:ARMED" in reply:
                ser.write(b"BOOT:DFU:CONFIRM\n")
                ser.flush()
                confirm = collect_until(ser, "ACK:DFU", 0.75)
                if "ACK:DFU" in confirm:
                    print("[USB-DFU] DFU confirm ACK seen")
                    time.sleep(0.20)
                    return True
                if confirm:
                    tail = confirm[-800:].replace("\r", " ").replace("\n", " | ")
                    print("[USB-DFU] DFU confirm rejected:", tail)
                if "ERR:DFU:WAIT_SAFE" in confirm:
                    raise RuntimeError("F411 rejected DFU: vehicle/navigation state is not confirmed safe")
                if "ERR:DFU:NOT_ARMED" in confirm:
                    raise RuntimeError("F411 rejected DFU: two-step arm window expired")
                return False

            if "UNKNOWN_COMMAND" in reply:
                ser.write(b"BOOT:DFU\n")
                ser.flush()
                time.sleep(0.20)
                return True
            return False
    except Exception as exc:
        print(f"[USB-DFU] CDC trigger detail: {exc}")
        return False



def _blind_software_dfu(port):
    """Best-effort transition when CDC RX still works but TX/ACK path is wedged."""
    print(f"[USB-DFU] Blind DFU handshake through {port} ...")
    try:
        import serial
        with serial.Serial(port, 1000000, timeout=0.03, write_timeout=1.0) as ser:
            time.sleep(0.12)
            for attempt in range(4):
                ser.write(b"\nBOOT:DFU:ARM\n")
                ser.flush()
                time.sleep(0.06)
                ser.write(b"BOOT:DFU:CONFIRM\n")
                ser.flush()
                print(f"[USB-DFU] blind arm/confirm {attempt + 1}/4 sent")
                time.sleep(0.18)
        return True
    except Exception as exc:
        print(f"[USB-DFU] blind CDC trigger detail: {exc}")
        return False


def _wait_for_cdc(seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        port = _find_cdc_port()
        if port:
            return port
        time.sleep(0.10)
    return None


def _reset_runtime_usb_device():
    """Reset only the exact BlackPill USB device; never a generic ttyACM node."""
    nodes = _usb_nodes("0483", "5740")
    if not nodes:
        return False
    USBDEVFS_RESET = (ord('U') << 8) | 20
    for node in nodes:
        try:
            fd = os.open(node, os.O_WRONLY)
            try:
                fcntl.ioctl(fd, USBDEVFS_RESET, 0)
            finally:
                os.close(fd)
            print(f"[USB-DFU] USB bus reset issued to {node}")
            time.sleep(0.50)
            return True
        except PermissionError:
            print(f"[USB-DFU] USB reset skipped: no permission for {node}")
        except OSError as exc:
            print(f"[USB-DFU] USB reset failed for {node}: {exc}")
    return False


def _openocd_paths():
    root = os.path.join(os.path.expanduser("~"), ".platformio", "packages", "tool-openocd")
    binary = os.path.join(root, "bin", "openocd")
    scripts = os.path.join(root, "scripts")
    return binary, scripts


def _probe_stlink_dbgmcu_id():
    binary, scripts = _openocd_paths()
    if not os.path.isfile(binary):
        return None
    cmd = [
        binary, "-s", scripts,
        "-f", "interface/stlink.cfg",
        "-f", "target/stm32f4x.cfg",
        "-c", "adapter speed 500",
        "-c", "init", "-c", "halt",
        "-c", "echo AGV_DBGMCU_ID=[format 0x%08X [mrw 0xE0042000]]",
        "-c", "resume", "-c", "shutdown",
    ]
    try:
        cp = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, timeout=8.0, check=False)
    except Exception as exc:
        print(f"[USB-DFU] ST-Link probe skipped: {exc}")
        return None
    import re
    m = re.search(r"AGV_DBGMCU_ID=(0x[0-9A-Fa-f]+)", cp.stdout)
    if not m:
        return None
    return int(m.group(1), 16)


def _try_stlink_force_dfu():
    """Use ST-Link only if the attached target identifies exactly as STM32F411.

    A connected F103 programmer is deliberately rejected, preventing the F103
    actuator MCU from ever being reset or modified by the F411 recovery path.
    """
    chip_id = _probe_stlink_dbgmcu_id()
    if chip_id is None:
        print("[USB-DFU] no usable ST-Link recovery target detected")
        return False
    dev_id = chip_id & 0xFFF
    if dev_id != 0x431:
        print(f"[USB-DFU] ST-Link target rejected: DBGMCU=0x{chip_id:08X}, DEV_ID=0x{dev_id:03X} (not STM32F411)")
        return False
    binary, scripts = _openocd_paths()
    # STM32F411: RCC_APB1ENR.PWREN, PWR_CR.DBP, RTC_BKP0R. The resident
    # bootloader consumes DFUB and jumps to immutable system-memory USB DFU.
    command = (
        "adapter speed 500; init; halt; "
        "set r [mrw 0x40023840]; mww 0x40023840 [expr {$r | 0x10000000}]; "
        "set p [mrw 0x40007000]; mww 0x40007000 [expr {$p | 0x00000100}]; "
        "mww 0x40002850 0x42465544; reset run; sleep 300; shutdown"
    )
    cmd = [binary, "-s", scripts, "-f", "interface/stlink.cfg",
           "-f", "target/stm32f4x.cfg", "-c", command]
    print("[USB-DFU] STM32F411 ST-Link recovery: requesting resident bootloader -> ROM DFU")
    try:
        cp = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                            text=True, timeout=10.0, check=False)
        if cp.returncode != 0:
            print("[USB-DFU] ST-Link recovery command failed")
            return False
        return True
    except Exception as exc:
        print(f"[USB-DFU] ST-Link recovery failed: {exc}")
        return False


def _cdc_to_dfu_with_recovery(port):
    _release_cdc_holders(port)
    if _request_software_dfu(port):
        if _wait_for_dfu(6.0, manual_hint=False):
            return True
    # Some failures leave CDC TX silent while RX/parser is still alive.
    port = _find_cdc_port()
    if port:
        _release_cdc_holders(port)
        if _blind_software_dfu(port) and _wait_for_dfu(4.0, manual_hint=False):
            return True
    # A USB bus reset can recover a wedged CDC peripheral without resetting an
    # otherwise healthy MCU. Permission depends on installed udev rules.
    if _reset_runtime_usb_device():
        port = _wait_for_cdc(4.0)
        if port:
            _release_cdc_holders(port)
            if _request_software_dfu(port) and _wait_for_dfu(6.0, manual_hint=False):
                return True
            port = _find_cdc_port()
            if port:
                _release_cdc_holders(port)
                if _blind_software_dfu(port) and _wait_for_dfu(4.0, manual_hint=False):
                    return True
    return False

def _before_upload(source, target, env):
    _acquire_upload_lock()
    _prepare_dfu_tool()
    _stop_ros_processes()

    # State A: already in immutable STM32 ROM DFU. Program immediately.
    nodes = _usb_nodes(DFU_VID, DFU_PID)
    if nodes:
        print("[USB-DFU] STM32 ROM DFU already active:", ", ".join(nodes))
        _assert_dfu_permission(nodes)
        return

    # State B/C: runtime CDC exists. Try acknowledged transition first, then
    # blind RX-only recovery and USB peripheral reset.
    port = _find_cdc_port()
    if port and _cdc_to_dfu_with_recovery(port):
        return

    # State D: runtime is crashed/silent. If a programmer is attached, it is
    # used ONLY when DBGMCU proves the target is an STM32F411 (DEV_ID 0x431).
    # A programmer connected to the F103 actuator MCU is explicitly rejected.
    if USB_ONLY:
        print("[USB-DFU] F411_USB_ONLY=1: ST-Link recovery intentionally disabled for this proof run")
    elif _try_stlink_force_dfu() and _wait_for_dfu(8.0, manual_hint=False):
        return

    # State E: allow time for a hardware watchdog/power-cycle/manual BOOT0 reset
    # to expose ROM DFU. Once 0483:df11 appears, the same transactional uploader
    # takes over automatically; no separate service/programmer script is needed.
    if _wait_for_dfu(MANUAL_DFU_WAIT_S, manual_hint=True):
        return

    raise RuntimeError(
        "F411 tidak dapat dipindahkan ke ROM DFU. Uploader sudah mencoba: "
        "existing DFU, CDC ACK, blind CDC, USB reset, dan F411-only ST-Link recovery. "
        "Jika USB/power/NRST secara fisik tidak terhubung, software tidak dapat memaksa reset MCU."
    )


env.AddPreAction("upload", _before_upload)
