#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DFU="$ROOT_DIR/.pio/tools/dfu-util-blackpill/bin/dfu-util"
IMAGE="${1:-}"
[[ -f "$IMAGE" ]] || { echo "Firmware not found: $IMAGE" >&2; exit 2; }
"$ROOT_DIR/scripts/build_dfu_util_blackpill.sh"
TMP="$(mktemp -d)"
cleanup() { rm -rf "$TMP"; }
trap cleanup EXIT INT TERM
MANIFEST="$TMP/manifest.bin"; RAW="$TMP/app_raw.bin"; READBACK="$TMP/readback.bin"
PERSIST_OLD="$TMP/persistent_old.bin"; PERSIST_NEW="$TMP/persistent_new.bin"; PERSIST_READBACK="$TMP/persistent_readback.bin"
OLD_SECTOR6="$TMP/old_sector6.bin"
BOOT_SNAPSHOT="$TMP/boot_sector0.bin"
python3 - <<PY2
from pathlib import Path
d=Path("$IMAGE").read_bytes()
if len(d)>=16 and d[-8:-5] == b"UFD" and d[-5] == 16:
    d=d[:-16]
Path("$RAW").write_bytes(d)
print(f"[USB-DFU] normalized raw app bytes={len(d)}")
PY2
python3 "$ROOT_DIR/scripts/make_app_manifest.py" "$RAW" "$MANIFEST"
SIZE=$(stat -c %s "$RAW")

wait_dfu_ready() {
  local ticks="${1:-80}" i
  for i in $(seq 1 "$ticks"); do
    if lsusb -d 0483:df11 >/dev/null 2>&1; then return 0; fi
    sleep 0.10
  done
  return 1
}

run_dfu() {
  local label="$1"; shift
  local attempt rc=1
  for attempt in 1 2 3; do
    echo "[USB-DFU] ${label} attempt=${attempt}/3"
    if "$DFU" "$@"; then return 0; else rc=$?; fi
    if (( attempt < 3 )); then
      wait_dfu_ready 100 || true
      sleep "0.$((attempt * 3))"
    fi
  done
  echo "[USB-DFU] ${label} failed after retries rc=${rc}" >&2
  return "$rc"
}

echo "[USB-DFU] recovery update: preserve Sector 7 -> app -> verify -> restore persistent journal"
run_dfu verify-resident-layout -a 0 -d 0483:df11 -s "0x08000000:16384" -U "$BOOT_SNAPSHOT"
python3 - "$BOOT_SNAPSHOT" <<'PYBOOT'
import sys
from pathlib import Path
data=Path(sys.argv[1]).read_bytes()
marker=b'AGVBL3-04000-60000'
if len(data)!=0x4000 or marker not in data:
    raise SystemExit('[USB-DFU] incompatible/legacy resident bootloader; run ST-Link provisioning once before ROM-DFU recovery')
print('[USB-DFU] resident layout marker verified')
PYBOOT
# ROM DFU is the emergency path. Preserve all persistent bytes before any write
# to Sector 7, and snapshot legacy Sector 6 for one-time 224K->368K migration.
run_dfu backup-persistent -a 0 -d 0483:df11 -s "0x08060000:131072" -U "$PERSIST_OLD"
run_dfu backup-legacy-sector6 -a 0 -d 0483:df11 -s "0x08040000:131072" -U "$OLD_SECTOR6"
python3 "$ROOT_DIR/scripts/compose_persistent_image.py" --persistent "$PERSIST_OLD" --legacy-sector6 "$OLD_SECTOR6" --manifest "$MANIFEST" --output "$PERSIST_NEW"
run_dfu write-app -a 0 -d 0483:df11 -s 0x08004000 -D "$RAW"
run_dfu readback-app -a 0 -d 0483:df11 -s "0x08004000:${SIZE}" -U "$READBACK"
cmp "$RAW" "$READBACK"
echo "[USB-DFU] app readback verified (${SIZE} bytes)"
run_dfu restore-persistent -a 0 -d 0483:df11 -s 0x08060000 -D "$PERSIST_NEW"
run_dfu readback-persistent -a 0 -d 0483:df11 -s "0x08060000:131072" -U "$PERSIST_READBACK"
cmp "$PERSIST_NEW" "$PERSIST_READBACK"
echo "[USB-DFU] persistent journal restored and verified"

CDC="/dev/serial/by-id/usb-STMicroelectronics_BLACKPILL_F411CE_CDC_in_FS_Mode_338133833134-if00"
leave_bootloader() {
  local attempt rc=1 tick
  for attempt in 1 2 3; do
    echo "[USB-DFU] leave-via-bootloader attempt=${attempt}/3 address=0x08000000"
    set +e
    # DfuSe command mode: SET_ADDRESS(0x08000000) + zero-length DNLOAD leave.
    # No -D/-U file is supplied, therefore bootloader flash is never rewritten.
    "$DFU" -a 0 -d 0483:df11 -s 0x08000000:leave
    rc=$?
    set -e
    if (( rc == 0 )); then return 0; fi
    for tick in $(seq 1 30); do
      if [[ -e "$CDC" ]]; then
        echo "[USB-DFU] leave returned rc=${rc}, but runtime CDC is back; accepting reset success"
        return 0
      fi
      sleep 0.10
    done
    # If DFU is already gone, do not blindly issue another command to a new device.
    if ! lsusb -d 0483:df11 >/dev/null 2>&1; then
      echo "[USB-DFU] ROM DFU disappeared after leave; waiting for runtime CDC"
      return 0
    fi
    if (( attempt < 3 )); then sleep "0.$((attempt * 3))"; fi
  done
  echo "[USB-DFU] leave-via-bootloader failed after retries rc=${rc}" >&2
  return "$rc"
}
leave_bootloader

wait_runtime_cdc() {
  local ticks="${1:-150}" i
  for i in $(seq 1 "$ticks"); do
    if [[ -e "$CDC" ]]; then return 0; fi
    sleep 0.10
  done
  return 1
}

verify_runtime_app() {
  python3 - "$CDC" <<'PYAPP'
import os, sys, time
port=sys.argv[1]
if not os.path.exists(port):
    raise SystemExit(2)
try:
    import serial
    with serial.Serial(port, 1000000, timeout=0.05, write_timeout=1.0) as ser:
        time.sleep(0.15)
        ser.reset_input_buffer()
        for _ in range(4):
            ser.write(b"PING\n")
            ser.flush()
            deadline=time.monotonic()+0.65
            data=b""
            while time.monotonic()<deadline:
                n=ser.in_waiting
                if n:
                    data += ser.read(min(n, 2048))
                    if b"ACK:PONG" in data or b"ADV HMI realtime menu firmware - boot" in data:
                        print("[USB-DFU] runtime application heartbeat verified")
                        raise SystemExit(0)
                time.sleep(0.01)
        print("[USB-DFU] runtime CDC present but application heartbeat missing")
        raise SystemExit(3)
except Exception as exc:
    print(f"[USB-DFU] runtime verification error: {exc}")
    raise SystemExit(4)
PYAPP
}

if ! wait_runtime_cdc 150; then
  # Some ROM revisions report leave success before the USB reset has propagated.
  # If DFU is still visible, issue one explicit USB reset request and wait again.
  if lsusb -d 0483:df11 >/dev/null 2>&1; then
    echo "[USB-DFU] runtime CDC absent; ROM DFU still visible, requesting USB reset"
    set +e
    "$DFU" -a 0 -d 0483:df11 -R
    set -e
  fi
  if ! wait_runtime_cdc 100; then
    echo "[USB-DFU] ERROR: flash/readback/manifest succeeded but runtime CDC did not return" >&2
    exit 12
  fi
fi

# Do not equate USB enumeration with firmware health. The freshly programmed
# application has an independent watchdog, so allow one watchdog recovery cycle
# before declaring the upload bad.
APP_OK=0
for pass in 1 2 3; do
  if verify_runtime_app; then APP_OK=1; break; fi
  echo "[USB-DFU] runtime verify retry ${pass}/3"
  sleep 4
  wait_runtime_cdc 40 || true
done
if (( APP_OK == 0 )); then
  echo "[USB-DFU] ERROR: CDC returned but final application did not answer PING" >&2
  exit 13
fi

echo "[USB-DFU] manifest committed last; DFU reset complete; final application healthy"
