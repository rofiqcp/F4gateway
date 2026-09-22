#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="/usr/bin/python3"
OPENOCD="$HOME/.platformio/packages/tool-openocd/bin/openocd"
OPENOCD_SCRIPTS="$HOME/.platformio/packages/tool-openocd/openocd/scripts"
BOOT="$ROOT/bootloader/.pio-user-build/f103_256k_recovery_boot/firmware.bin"
APP="$ROOT/.pio/build/f103_256k_usbboot/firmware.bin"
CDC_GLOB="/dev/serial/by-id/usb-STMicroelectronics_BLUEPILL_F103_CDC_in_FS_Mode_1A5B1F830000-if00"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT INT TERM
BOOT_LOCKED="$TMP/bootloader.bin"
APP_LOCKED="$TMP/application.bin"
MANIFEST="$TMP/manifest.bin"

[[ -x "$PYTHON" ]] || { echo "[F103-STLINK] python missing" >&2; exit 2; }
[[ -x "$OPENOCD" ]] || { echo "[F103-STLINK] openocd missing" >&2; exit 2; }

probe="$("$OPENOCD" -s "$OPENOCD_SCRIPTS" -f interface/stlink.cfg -c 'transport select swd' -f target/stm32f1x.cfg \
  -c 'adapter speed 480; init; halt; echo AGV_DBGMCU_ID=[format 0x%08X [mrw 0xE0042000]]; echo AGV_FLASH_KB=[mrh 0x1FFFF7E0]; resume; shutdown' 2>&1 || true)"
id="$(printf '%s\n' "$probe" | sed -n 's/.*AGV_DBGMCU_ID=\(0x[0-9A-Fa-f]*\).*/\1/p' | tail -1)"
flash_raw="$(printf '%s\n' "$probe" | sed -n 's/.*AGV_FLASH_KB=\(0x[0-9A-Fa-f]*\|[0-9][0-9]*\).*/\1/p' | tail -1)"
if [[ -z "$id" || -z "$flash_raw" ]]; then
  echo "[F103-STLINK] cannot identify target" >&2
  printf '%s\n' "$probe" >&2
  exit 3
fi
dev=$(( id & 0xFFF ))
flash_kb=$(( flash_raw ))
[[ "$dev" -eq $((0x414)) ]] || {
  printf '[F103-STLINK] REFUSED DEV_ID=0x%03X; STM32F103 high-density 0x414 required\n' "$dev" >&2
  exit 4
}
[[ "$flash_kb" -eq 256 ]] || {
  printf '[F103-STLINK] REFUSED flash=%s KiB; exactly 256 KiB required\n' "$flash_kb" >&2
  exit 4
}
echo "[F103-STLINK] verified STM32F103RC-class target, flash=256 KiB"

# Recovery provisioning must have sole ownership of the runtime transport.
# If ROS or another uploader owns CDC, fail before any erase/write operation.
if [[ -e "$CDC_GLOB" ]]; then
  runtime_real="$(readlink -f "$CDC_GLOB")"
  runtime_holders="$(fuser "$runtime_real" 2>/dev/null || true)"
  if [[ -n "$runtime_holders" ]]; then
    echo "[F103-STLINK] REFUSED: runtime CDC is in use by pid=$runtime_holders" >&2
    exit 10
  fi
fi

PERSIST_BEFORE="$TMP/persistent.before.bin"
PERSIST_AFTER="$TMP/persistent.after.bin"
"$OPENOCD" -s "$OPENOCD_SCRIPTS" -f interface/stlink.cfg -c 'transport select swd' -f target/stm32f1x.cfg \
  -c "adapter speed 480; init; halt; dump_image $PERSIST_BEFORE 0x0803E800 0x1800; resume; shutdown"
persist_sha_before="$(sha256sum "$PERSIST_BEFORE" | awk '{print $1}')"
echo "[F103-STLINK] persistent SHA256 before=$persist_sha_before"

(cd "$ROOT/bootloader" && pio run -e f103_256k_recovery_boot)
(cd "$ROOT" && pio run -e f103_256k_usbboot)

# Freeze exactly the images that were just built. Other agents/developers may
# build another PlatformIO environment in the same repository; flashing only
# these private copies prevents a build-dir race between validation and write.
cp -- "$BOOT" "$BOOT_LOCKED"
cp -- "$APP" "$APP_LOCKED"
"$PYTHON" "$ROOT/scripts/make_app_manifest.py" "$APP_LOCKED" "$MANIFEST" --target f103
echo "[F103-STLINK] locked boot SHA256=$(sha256sum "$BOOT_LOCKED" | awk '{print $1}')"
echo "[F103-STLINK] locked app  SHA256=$(sha256sum "$APP_LOCKED" | awk '{print $1}')"

[[ $(stat -c %s "$BOOT_LOCKED") -le $((0x4000)) ]] || {
  echo "[F103-STLINK] bootloader exceeds 16 KiB" >&2; exit 5;
}
[[ $(stat -c %s "$APP_LOCKED") -le $((0x3A000)) ]] || {
  echo "[F103-STLINK] application exceeds app region" >&2; exit 6;
}
[[ $(stat -c %s "$MANIFEST") -eq 32 ]] || {
  echo "[F103-STLINK] invalid manifest size" >&2; exit 7;
}
echo "[F103-STLINK] flashing boot/app/manifest; persistent 0x0803E800..0x08040000 is preserved"
"$OPENOCD" -s "$OPENOCD_SCRIPTS" -f interface/stlink.cfg -c 'transport select swd' -f target/stm32f1x.cfg \
  -c "adapter speed 480; init; reset halt; \
      flash erase_address 0x08000000 0x4000; \
      flash write_image $BOOT_LOCKED 0x08000000 bin; verify_image $BOOT_LOCKED 0x08000000 bin; \
      flash erase_address 0x08004000 0x3A000; \
      flash write_image $APP_LOCKED 0x08004000 bin; verify_image $APP_LOCKED 0x08004000 bin; \
      flash erase_address 0x0803E000 0x800; \
      flash write_image $MANIFEST 0x0803E000 bin; verify_image $MANIFEST 0x0803E000 bin; \
      reset run; shutdown"

"$OPENOCD" -s "$OPENOCD_SCRIPTS" -f interface/stlink.cfg -c 'transport select swd' -f target/stm32f1x.cfg \
  -c "adapter speed 480; init; halt; dump_image $PERSIST_AFTER 0x0803E800 0x1800; resume; shutdown"
persist_sha_after="$(sha256sum "$PERSIST_AFTER" | awk '{print $1}')"
echo "[F103-STLINK] persistent SHA256 after=$persist_sha_after"
if [[ "$persist_sha_before" != "$persist_sha_after" ]]; then
  echo "[F103-STLINK] FATAL persistent configuration changed during provisioning" >&2
  exit 9
fi
echo "[F103-STLINK] persistent partition verified byte-for-byte unchanged"

CDC=""
for _ in $(seq 1 150); do
  if [[ -e "$CDC_GLOB" ]]; then CDC="$CDC_GLOB"; break; fi
  sleep 0.10
done

if [[ -z "$CDC" ]]; then
  echo "[F103-STLINK] runtime CDC absent; validating application execution via SWD"
  exec_probe="$("$OPENOCD" -s "$OPENOCD_SCRIPTS" -f interface/stlink.cfg -c 'transport select swd' -f target/stm32f1x.cfg \
    -c 'adapter speed 480; init; reset run; sleep 1000; halt; shutdown' 2>&1 || true)"
  pc_hex="$(printf '%s\n' "$exec_probe" | sed -n 's/.*pc: \(0x[0-9A-Fa-f]*\).*/\1/p' | tail -1)"
  [[ -n "$pc_hex" ]] || { printf '%s\n' "$exec_probe" >&2; exit 8; }
  pc=$(( pc_hex ))
  if (( pc < 0x08004000 || pc >= 0x0803E000 )); then
    printf '[F103-STLINK] application not running, PC=%s\n' "$pc_hex" >&2
    exit 8
  fi
  echo "[F103-STLINK] application executes via SWD PC=$pc_hex, but runtime CDC did not enumerate" >&2
  echo "[F103-STLINK] REFUSED success: executable app is not sufficient proof of USB upload/send/receive health" >&2
  exit 8
fi
real="$(readlink -f "$CDC")"
holders="$(fuser "$real" 2>/dev/null || true)"
if [[ -n "$holders" ]]; then
  echo "[F103-STLINK] runtime CDC is owned by pid=$holders; cannot prove heartbeat/session exclusively" >&2
  echo "[F103-STLINK] REFUSED success: stop the port owner and rerun verification" >&2
  exit 10
fi

"$PYTHON" - "$CDC" <<'PY'
import os, sys, time, serial
port = sys.argv[1]
deadline = time.monotonic() + 15.0
last_error = None
while time.monotonic() < deadline:
    try:
        with serial.Serial(port, 1000000, timeout=.05, write_timeout=1.0, exclusive=True) as ser:
            time.sleep(.15)
            for _ in range(3):
                ser.write(b'\n')
                ser.flush()
                time.sleep(.04)
            ser.reset_input_buffer()
            token = ((os.getpid() << 12) ^ int(time.monotonic() * 1000) ^ 0xF103256) & 0xFFFFFFFF
            token = token or 1
            ser.write((f'HOST:HELLO:{token}\n').encode())
            ser.flush()
            session_prefix = f'ACK:HOST:SESSION:{token}:'.encode()
            session_deadline = time.monotonic() + 2.0
            data = bytearray()
            while time.monotonic() < session_deadline:
                chunk = ser.read(512)
                if chunk:
                    data += chunk
                if session_prefix in data:
                    break
            if session_prefix not in data:
                raise RuntimeError('HOST:HELLO acknowledgement missing')
            ser.write(b'PING\nFW:INFO\nUSB:STATUS\n')
            ser.flush()
            verify_deadline = time.monotonic() + 2.0
            while time.monotonic() < verify_deadline:
                chunk = ser.read(512)
                if chunk:
                    data += chunk
                if (b'ACK:PONG' in data and
                    b'FW:INFO:V3:F103RC:256K' in data and
                    b'USB:STAT:host=1' in data):
                    print(data.decode(errors='replace'))
                    raise SystemExit(0)
            raise RuntimeError('runtime heartbeat/build/session verification incomplete')
    except (OSError, serial.SerialException, RuntimeError) as exc:
        last_error = exc
        time.sleep(.15)
raise SystemExit(f'[F103-STLINK] CDC verification failed after re-enumeration retries: {last_error}')
PY

echo "[F103-STLINK] provision complete"
