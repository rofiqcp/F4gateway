#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
IMAGE="${1:?bootloader binary is required}"
OPENOCD="$HOME/.platformio/packages/tool-openocd/bin/openocd"
OPENOCD_SCRIPTS="$HOME/.platformio/packages/tool-openocd/openocd/scripts"
MAX_BOOT=8192

[[ -x "$OPENOCD" ]] || { echo "[F103C8-BOOT] OpenOCD missing" >&2; exit 2; }
[[ -f "$IMAGE" ]] || { echo "[F103C8-BOOT] image missing: $IMAGE" >&2; exit 2; }
size="$(stat -c %s "$IMAGE")"
(( size <= MAX_BOOT )) || { echo "[F103C8-BOOT] image exceeds 8 KiB: $size" >&2; exit 3; }

openocd_once() {
  "$OPENOCD" -s "$OPENOCD_SCRIPTS" -f interface/stlink.cfg     -c 'transport select swd' -f target/stm32f1x.cfg     -c "$1"
}

recover_stlink() {
  # Do not usbreset ST-Link/V2 here. Several clone programmers disappear from
  # the bus after a userspace USB reset. Let the interface settle and retry SWD.
  sleep 1
}

probe_ok=0
for attempt in 1 2 3 4 5 6; do
  echo "[F103C8-BOOT] probe $attempt/6"
  out="$(openocd_once 'adapter speed 125; init; reset halt; echo AGV_ID=[format 0x%08X [mrw 0xE0042000]]; echo AGV_FLASH=[mrh 0x1FFFF7E0]; shutdown' 2>&1 || true)"
  id="$(printf '%s\n' "$out" | sed -n 's/.*AGV_ID=\(0x[0-9A-Fa-f]*\).*/\1/p' | tail -1)"
  flash="$(printf '%s\n' "$out" | sed -n 's/.*AGV_FLASH=\(0x[0-9A-Fa-f]*\|[0-9][0-9]*\).*/\1/p' | tail -1)"
  if [[ -n "$id" && -n "$flash" ]]; then
    dev=$(( id & 0xFFF ))
    kb=$(( flash ))
    if [[ "$dev" -eq $((0x410)) || "$dev" -eq $((0x414)) ]]; then
      printf '[F103C8-BOOT] target DEV_ID=0x%03X flash-report=%uKiB; layout capped at 64KiB\n' "$dev" "$kb"
      probe_ok=1
      break
    fi
  fi
  printf '%s\n' "$out" | tail -n 8
  recover_stlink
done
[[ "$probe_ok" -eq 1 ]] || { echo "[F103C8-BOOT] target probe failed" >&2; exit 4; }

flash_ok=0
for attempt in 1 2 3 4; do
  echo "[F103C8-BOOT] flash $attempt/4 size=$size"
  if openocd_once "adapter speed 125; init; reset halt; flash write_image erase $IMAGE 0x08000000 bin; verify_image $IMAGE 0x08000000 bin; mmw 0x4002101C 0x18000000 0x0; mmw 0x40007000 0x00000100 0x0; mwh 0x40006C04 0x0000; mwh 0x40006C28 0x0000; reset run; shutdown"; then
    flash_ok=1
    break
  fi
  recover_stlink
done
[[ "$flash_ok" -eq 1 ]] || { echo "[F103C8-BOOT] flash/verify failed" >&2; exit 5; }

echo "[F103C8-BOOT] SUCCESS bootloader=$size bytes at 0x08000000"
