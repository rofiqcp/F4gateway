#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
APP="${1:?application binary is required}"
BOOT="$ROOT/.pio/build/f103_bootloader/firmware.bin"
OPENOCD="$HOME/.platformio/packages/tool-openocd/bin/openocd"
OPENOCD_SCRIPTS="$HOME/.platformio/packages/tool-openocd/openocd/scripts"
APP_BASE=0x08002000
APP_LIMIT=0x0800F7F0
META_BASE=0x0800F7F0
PERSIST_BASE=0x0800F800
PERSIST_SIZE=0x800
TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT INT TERM

[[ -x "$OPENOCD" ]] || { echo "[F103C8-STLINK] OpenOCD missing" >&2; exit 2; }
[[ -f "$APP" ]] || { echo "[F103C8-STLINK] app binary missing: $APP" >&2; exit 2; }

probe=""
id=""
flash_raw=""
for attempt in 1 2 3 4 5; do
  echo "[F103C8-STLINK] probe $attempt/5"
  probe="$("$OPENOCD" -s "$OPENOCD_SCRIPTS" -f interface/stlink.cfg \
    -c 'transport select swd' -f target/stm32f1x.cfg \
    -c 'adapter speed 125; init; reset halt; echo AGV_DBGMCU_ID=[format 0x%08X [mrw 0xE0042000]]; echo AGV_FLASH_KB=[mrh 0x1FFFF7E0]; shutdown' 2>&1 || true)"
  id="$(printf '%s\n' "$probe" | sed -n 's/.*AGV_DBGMCU_ID=\(0x[0-9A-Fa-f]*\).*/\1/p' | tail -1)"
  flash_raw="$(printf '%s\n' "$probe" | sed -n 's/.*AGV_FLASH_KB=\(0x[0-9A-Fa-f]*\|[0-9][0-9]*\).*/\1/p' | tail -1)"
  [[ -n "$id" && -n "$flash_raw" ]] && break
  printf '%s\n' "$probe" | tail -n 8
  sleep 1
done
[[ -n "$id" && -n "$flash_raw" ]] || { echo "[F103C8-STLINK] target probe failed" >&2; exit 3; }

dev=$(( id & 0xFFF ))
flash_kb=$(( flash_raw ))
if [[ "$dev" -ne $((0x410)) && "$dev" -ne $((0x414)) ]]; then
  printf '[F103C8-STLINK] REFUSED DEV_ID=0x%03X; expected C8-compatible/clone ID 0x410 or 0x414\n' "$dev" >&2
  exit 4
fi
if [[ "$flash_kb" -ne 64 && "$flash_kb" -ne 128 && "$flash_kb" -ne 256 ]]; then
  printf '[F103C8-STLINK] REFUSED flash-size register=%s KiB\n' "$flash_kb" >&2
  exit 4
fi
printf '[F103C8-STLINK] target DEV_ID=0x%03X reports %u KiB; firmware layout is forcibly capped at official C8 64 KiB\n' "$dev" "$flash_kb"

cd "$ROOT"
pio run -e f103_bootloader

boot_size="$(stat -c %s "$BOOT")"
app_size="$(stat -c %s "$APP")"
(( boot_size <= 8192 )) || { echo "[F103C8-STLINK] bootloader exceeds 8 KiB: $boot_size" >&2; exit 5; }
(( app_size <= APP_LIMIT-APP_BASE )) || { echo "[F103C8-STLINK] app exceeds 55280-byte slot: $app_size" >&2; exit 6; }

META="$TMP/app.meta.bin"
python3 - "$APP" "$META" <<'PY'
import pathlib,struct,sys,zlib
data=pathlib.Path(sys.argv[1]).read_bytes()
head=struct.pack("<III",0x34475641,len(data),zlib.crc32(data)&0xffffffff)
pathlib.Path(sys.argv[2]).write_bytes(head+struct.pack("<I",zlib.crc32(head)&0xffffffff))
PY

BEFORE="$TMP/persist.before.bin"
AFTER="$TMP/persist.after.bin"

run_openocd_retry() {
  local command="$1"
  local attempt
  for attempt in 1 2 3 4 5; do
    echo "[F103C8-STLINK] SWD step $attempt/5"
    if "$OPENOCD" -s "$OPENOCD_SCRIPTS" -f interface/stlink.cfg \
      -c 'transport select swd' -f target/stm32f1x.cfg \
      -c "$command"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

run_openocd_retry "adapter speed 125; init; reset halt; dump_image $BEFORE $PERSIST_BASE $PERSIST_SIZE; shutdown"

run_openocd_retry "adapter speed 125; init; reset halt; \
    flash erase_address 0x08000000 0xF800; \
    flash write_image $BOOT 0x08000000 bin; verify_image $BOOT 0x08000000 bin; \
    flash write_image $APP $APP_BASE bin; verify_image $APP $APP_BASE bin; \
    flash write_image $META $META_BASE bin; verify_image $META $META_BASE bin; \
    mmw 0x4002101C 0x18000000 0x0; mmw 0x40007000 0x00000100 0x0; \
    mwh 0x40006C04 0x0000; mwh 0x40006C28 0x0000; \
    reset run; shutdown"

# Final persistent verification must not leave the MCU halted.
run_openocd_retry "adapter speed 125; init; reset halt; dump_image $AFTER $PERSIST_BASE $PERSIST_SIZE; reset run; shutdown"

cmp -s "$BEFORE" "$AFTER" || {
  echo "[F103C8-STLINK] FATAL persistent area changed" >&2
  exit 7
}

echo "[F103C8-STLINK] SUCCESS boot=$boot_size app=$app_size persistent-preserved"
echo "[F103C8-STLINK] app base 0x08002000; USB updates use f103_usb"
