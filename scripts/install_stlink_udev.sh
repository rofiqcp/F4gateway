#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RULE_SRC="$ROOT/scripts/49-stlinkv2.rules"
RULE_DST="/etc/udev/rules.d/49-stlinkv2.rules"
[[ $EUID -eq 0 ]] || { echo "Run with sudo: sudo $0" >&2; exit 2; }
install -m 0644 "$RULE_SRC" "$RULE_DST"
udevadm control --reload-rules
udevadm trigger --subsystem-match=usb --attr-match=idVendor=0483 --attr-match=idProduct=3748 || true
sleep 1
echo "[STLINK-UDEV] installed $RULE_DST"
