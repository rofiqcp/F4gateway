#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "$0")/.." && pwd)"
cd "$ROOT"

wait_stlink() {
  local n
  for n in $(seq 1 15); do
    if lsusb -d 0483:3748 >/dev/null 2>&1; then
      echo "[PIPELINE] ST-Link detected"
      return 0
    fi
    sleep 1
  done
  echo "[PIPELINE] ST-Link 0483:3748 is not enumerated; replug/power-cycle programmer first" >&2
  return 1
}

wait_runtime() {
  local n
  for n in $(seq 1 20); do
    if [[ -e /dev/f4gateway ]] || compgen -G "/dev/serial/by-id/*BLUEPILL_F103_CDC_in_FS_Mode*" >/dev/null; then
      echo "[PIPELINE] runtime CDC detected"
      return 0
    fi
    sleep 1
  done
  echo "[PIPELINE] runtime CDC did not enumerate" >&2
  return 1
}

wait_stlink

echo "[PIPELINE] 1/3 bootloader upload"
pio run -e f103_bootloader -t upload

echo "[PIPELINE] 2/3 ST-Link provision bootloader + app"
pio run -e f103_stlink -t upload
wait_runtime

echo "[PIPELINE] 3/3 resident USB application update"
pio run -e f103_usb -t upload
wait_runtime

echo "[PIPELINE] SUCCESS bootloader + ST-Link provision + USB update verified"
