#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RULE_SRC="$ROOT_DIR/99-blackpill-stm32.rules"
RULE_DST="/etc/udev/rules.d/99-blackpill-stm32.rules"

if [[ ! -f "$RULE_SRC" ]]; then
  echo "Rule file not found: $RULE_SRC" >&2
  exit 1
fi

sudo install -m 0644 "$RULE_SRC" "$RULE_DST"
sudo udevadm control --reload-rules
sudo udevadm trigger

echo "STM32 udev rules installed. Reconnect ST-Link/BlackPill if permissions do not refresh immediately."
