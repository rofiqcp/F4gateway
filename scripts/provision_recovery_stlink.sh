#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OPENOCD="$HOME/.platformio/packages/tool-openocd/bin/openocd"
BOOT="$ROOT/bootloader/.pio/build/f411_recovery_boot/firmware.bin"
APP="$ROOT/.pio/build/blackpill_f411ce_stlink/firmware.bin"
MANIFEST="$ROOT/.pio/build/blackpill_f411ce_stlink/manifest.bin"
CDC="/dev/serial/by-id/usb-STMicroelectronics_BLACKPILL_F411CE_CDC_in_FS_Mode_338133833134-if00"
[[ -x "$OPENOCD" ]] || { echo "[STLINK] openocd not found: $OPENOCD" >&2; exit 2; }

probe="$($OPENOCD -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c 'adapter speed 500; init; halt; echo AGV_DBGMCU_ID=[format 0x%08X [mrw 0xE0042000]]; resume; shutdown' 2>&1 || true)"
id="$(printf '%s\n' "$probe" | sed -n 's/.*AGV_DBGMCU_ID=\(0x[0-9A-Fa-f]*\).*/\1/p' | tail -1)"
[[ -n "$id" ]] || { echo "[STLINK] cannot identify target MCU" >&2; printf '%s\n' "$probe" >&2; exit 3; }
dev=$(( id & 0xFFF ))
[[ "$dev" -eq $((0x431)) ]] || { printf '[STLINK] REFUSED target %s DEV_ID=0x%03X is not STM32F411\n' "$id" "$dev" >&2; exit 4; }
echo "[STLINK] target verified STM32F411 DBGMCU=$id"

cd "$ROOT/bootloader"; pio run -e f411_recovery_boot
cd "$ROOT"; pio run -e blackpill_f411ce_stlink
python3 scripts/make_app_manifest.py "$APP" "$MANIFEST"
[[ $(stat -c %s "$BOOT") -le $((0x8000)) ]] || { echo '[STLINK] bootloader exceeds 32 KiB' >&2; exit 5; }
[[ $(stat -c %s "$APP") -le $((0x58000)) ]] || { echo '[STLINK] application exceeds app region' >&2; exit 6; }
[[ $(stat -c %s "$MANIFEST") -eq 32 ]] || { echo '[STLINK] invalid manifest size' >&2; exit 7; }

echo "[STLINK] transactional provision boot@08000000 app@08008000 manifest@08060000"
"$OPENOCD" -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "adapter speed 500; init; reset halt; \
      flash erase_address 0x08000000 0x8000; \
      flash write_image $BOOT 0x08000000 bin; verify_image $BOOT 0x08000000 bin; \
      flash write_image erase $APP 0x08008000 bin; verify_image $APP 0x08008000 bin; \
      flash write_image erase $MANIFEST 0x08060000 bin; verify_image $MANIFEST 0x08060000 bin; \
      reset run; shutdown"

wait_cdc() {
  local i
  for i in $(seq 1 120); do [[ -e "$CDC" ]] && return 0; sleep 0.10; done
  return 1
}
if ! wait_cdc; then
  # ST-Link is often the only physical cable attached during recovery. In that
  # state a missing runtime CDC is not evidence of a bad flash. Prove that the
  # resident bootloader accepted the manifest and that the CPU executes inside
  # the relocated application region instead.
  echo "[STLINK] runtime CDC not present; validating application execution via SWD"
  exec_probe="$($OPENOCD -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c 'adapter speed 500; init; reset run; sleep 800; halt; shutdown' 2>&1 || true)"
  pc_hex="$(printf '%s\n' "$exec_probe" | sed -n 's/.*pc: \(0x[0-9A-Fa-f]*\).*/\1/p' | tail -1)"
  if [[ -z "$pc_hex" ]]; then
    echo '[STLINK] ERROR cannot prove runtime execution through SWD' >&2
    printf '%s\n' "$exec_probe" >&2
    exit 8
  fi
  pc=$((pc_hex))
  if (( pc < 0x08008000 || pc >= 0x08060000 )); then
    printf '[STLINK] ERROR CPU PC=%s is outside application region\n' "$pc_hex" >&2
    printf '%s\n' "$exec_probe" >&2
    exit 8
  fi
  echo "[STLINK] application execution verified via SWD PC=$pc_hex (USB runtime cable not enumerated)"
  echo "[STLINK] provision complete"
  exit 0
fi

# Verify application heartbeat when the port is free. If ROS already owns the
# exact CDC endpoint, enumeration is accepted here and ROS hot-plug validation
# remains authoritative; we never steal bytes from a live bridge.
real="$(readlink -f "$CDC")"
holders="$(fuser "$real" 2>/dev/null || true)"
if [[ -z "$holders" ]]; then
  python3 - "$CDC" <<'PY'
import sys,time,serial
port=sys.argv[1]
with serial.Serial(port,1000000,timeout=.05,write_timeout=1.0,exclusive=True) as s:
    time.sleep(.12); s.reset_input_buffer()
    for _ in range(4):
        s.write(b'PING\n'); s.flush(); end=time.monotonic()+.7; data=b''
        while time.monotonic()<end:
            if s.in_waiting:
                data += s.read(min(2048,s.in_waiting))
                if b'ACK:PONG' in data:
                    print('[STLINK] runtime heartbeat ACK:PONG verified'); raise SystemExit(0)
            time.sleep(.01)
raise SystemExit('[STLINK] ERROR CDC enumerated but ACK:PONG missing')
PY
else
  echo "[STLINK] runtime CDC enumerated; heartbeat delegated to existing holder pid=${holders}"
fi
echo "[STLINK] provision complete"
