#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "$0")/.." && pwd)"
RULE_SRC="$ROOT_DIR/99-f103c8-gateway.rules"
RULE_DST="/etc/udev/rules.d/99-f103c8-gateway.rules"

[[ -f "$RULE_SRC" ]] || { echo "Rule file not found: $RULE_SRC" >&2; exit 1; }

for spec in   '5740|BLUEPILL_F103 CDC in FS Mode|f4gateway'   '5741|BLUEPILL_F103 BOOT CDC|f4gateway-boot'
do
  IFS='|' read -r pid product alias_name <<< "$spec"
  grep -Fq "ATTR{product}==\"$product\"" "$RULE_SRC" ||
    { echo "Missing product-scoped USB rule for PID $pid" >&2; exit 1; }
  grep -Fq "ATTRS{product}==\"$product\"" "$RULE_SRC" ||
    { echo "Missing product-scoped TTY rule for PID $pid" >&2; exit 1; }
  grep -Fq "SYMLINK+=\"$alias_name\"" "$RULE_SRC" ||
    { echo "Missing stable alias $alias_name" >&2; exit 1; }
done

sudo install -m 0644 "$RULE_SRC" "$RULE_DST"
sudo udevadm control --reload-rules
sudo udevadm trigger
cmp -s "$RULE_SRC" "$RULE_DST" ||
  { echo "Installed udev rule differs from repository source" >&2; exit 1; }

echo "STM32F103C8 gateway udev rules installed."
