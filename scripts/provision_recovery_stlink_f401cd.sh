#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON="/usr/bin/python3"
OPENOCD="$HOME/.platformio/packages/tool-openocd/bin/openocd"
BOOT="$ROOT/bootloader/.pio-user-build/f401cd_recovery_boot/firmware.bin"
APP="$ROOT/.pio/build/blackpill_f401cd_stlink/firmware.bin"
MANIFEST="$ROOT/.pio/build/blackpill_f401cd_stlink/manifest.bin"
CDC_GLOB="/dev/serial/by-id/usb-STMicroelectronics_BLACKPILL_F401CD_CDC_in_FS_Mode*-if00"
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT INT TERM
PERSIST="$TMP/sector6.bin"

[[ -x "$OPENOCD" ]] || { echo "[STLINK-CD] openocd missing" >&2; exit 2; }
[[ -x "$PYTHON" ]] || { echo "[STLINK-CD] /usr/bin/python3 missing" >&2; exit 2; }

probe="$("$OPENOCD" -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c 'adapter speed 100; init; halt; echo AGV_DBGMCU_ID=[format 0x%08X [mrw 0xE0042000]]; echo AGV_FLASH_KB=[mrh 0x1FFF7A22]; shutdown' 2>&1 || true)"
id="$(printf '%s\n' "$probe" | sed -n 's/.*AGV_DBGMCU_ID=\(0x[0-9A-Fa-f]*\).*/\1/p' | tail -1)"
flash_raw="$(printf '%s\n' "$probe" | sed -n 's/.*AGV_FLASH_KB=\(0x[0-9A-Fa-f]*\|[0-9][0-9]*\).*/\1/p' | tail -1)"
[[ -n "$id" && -n "$flash_raw" ]] || { printf '%s\n' "$probe" >&2; exit 3; }
dev=$(( id & 0xFFF ))
flash_kb=$((flash_raw))
[[ "$dev" -eq $((0x433)) ]] || {
  printf '[STLINK-CD] REFUSED DBGMCU=%s DEV_ID=0x%03X; STM32F401 requires 0x433\n' "$id" "$dev" >&2
  exit 4
}
[[ "$flash_kb" -eq 384 ]] || {
  printf '[STLINK-CD] REFUSED flash=%s KiB; STM32F401CDU6 requires 384 KiB\n' "$flash_kb" >&2
  exit 4
}
echo "[STLINK-CD] verified STM32F401CDU6-class DEV_ID=0x433 FLASH=384KiB"

cd "$ROOT/bootloader"
pio run -e f401cd_recovery_boot
cd "$ROOT"
pio run -e blackpill_f401cd_stlink
"$PYTHON" scripts/make_app_manifest.py --target f401cd "$APP" "$MANIFEST"

[[ $(stat -c %s "$BOOT") -le $((0x4000)) ]] || { echo "[STLINK-CD] bootloader >16KiB" >&2; exit 5; }
[[ $(stat -c %s "$APP") -le $((0x3C000)) ]] || { echo "[STLINK-CD] app >240KiB" >&2; exit 6; }
[[ $(stat -c %s "$MANIFEST") -eq 32 ]] || { echo "[STLINK-CD] bad manifest" >&2; exit 7; }

"$PYTHON" - "$MANIFEST" "$PERSIST" <<'PY'
import sys
from pathlib import Path
manifest=Path(sys.argv[1]).read_bytes()
image=bytearray(b'\xff' * 0x20000)
image[:len(manifest)] = manifest
Path(sys.argv[2]).write_bytes(image)
PY
echo "[STLINK-CD] flashing boot@08000000 app@08004000 sector6@08040000"
"$OPENOCD" -f interface/stlink.cfg -f target/stm32f4x.cfg \
  -c "adapter speed 100; init; reset halt; \
      flash erase_address 0x08000000 0x4000; \
      flash write_image $BOOT 0x08000000 bin; verify_image $BOOT 0x08000000 bin; \
      flash erase_address 0x08004000 0x3C000; \
      flash write_image $APP 0x08004000 bin; verify_image $APP 0x08004000 bin; \
      flash erase_address 0x08040000 0x20000; \
      flash write_image $PERSIST 0x08040000 bin; verify_image $PERSIST 0x08040000 bin; \
      reset run; shutdown"

sleep 1
matches=( $CDC_GLOB )
if [[ ${#matches[@]} -eq 1 && -e "${matches[0]}" ]]; then
  CDC="${matches[0]}"
  "$PYTHON" - "$CDC" <<'PY'
import sys,time,serial
p=sys.argv[1]
with serial.Serial(p,1000000,timeout=.05,write_timeout=1,exclusive=True) as s:
    time.sleep(.15); s.reset_input_buffer()
    for _ in range(5):
        s.write(b'PING\n'); s.flush()
        end=time.monotonic()+.8; data=b''
        while time.monotonic()<end:
            if s.in_waiting:
                data += s.read(min(2048,s.in_waiting))
                if b'ACK:PONG' in data:
                    print('[STLINK-CD] runtime USB CDC ACK:PONG verified')
                    raise SystemExit(0)
            time.sleep(.01)
raise SystemExit('[STLINK-CD] runtime CDC present but ACK:PONG missing')
PY
else
  echo "[STLINK-CD] USB runtime cable not present; checking app execution via SWD"
  exec_probe="$("$OPENOCD" -f interface/stlink.cfg -f target/stm32f4x.cfg \
    -c 'adapter speed 100; init; reset run; sleep 800; halt; shutdown' 2>&1 || true)"
  pc_hex="$(printf '%s\n' "$exec_probe" | sed -n 's/.*pc: \(0x[0-9A-Fa-f]*\).*/\1/p' | tail -1)"
  [[ -n "$pc_hex" ]] || { printf '%s\n' "$exec_probe" >&2; exit 8; }
  pc=$((pc_hex))
  if (( pc < 0x08004000 || pc >= 0x08040000 )); then
    printf '[STLINK-CD] app execution check failed PC=%s\n' "$pc_hex" >&2
    exit 8
  fi
  echo "[STLINK-CD] application execution verified PC=$pc_hex"
fi

echo "[STLINK-CD] provision complete"
echo "[STLINK-CD] future USB-only upload:"
echo "  pio run -e blackpill_f401cd -t upload"
