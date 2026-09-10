#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RULE_SRC="$ROOT_DIR/99-blackpill-stm32.rules"
RULE_DST="/etc/udev/rules.d/99-blackpill-stm32.rules"

if [[ ! -f "$RULE_SRC" ]]; then
  echo "Rule file not found: $RULE_SRC" >&2
  exit 1
fi

# The runtime/boot CDC permissions are deliberately scoped to this exact
# BlackPill. Never grant USBDEVFS reset access to arbitrary 0483:5740 devices.
BLACKPILL_SERIAL="338133833134"
for pid in 5740 5741; do
  if ! grep -Eq "SUBSYSTEM==\"usb\".*idProduct}==\"${pid}\".*ATTR\{serial\}==\"${BLACKPILL_SERIAL}\"" "$RULE_SRC"; then
    echo "Refusing to install non identity-scoped BlackPill rule for PID ${pid}" >&2
    exit 1
  fi
done

sudo install -m 0644 "$RULE_SRC" "$RULE_DST"
sudo udevadm control --reload-rules
sudo udevadm trigger
if ! cmp -s "$RULE_SRC" "$RULE_DST"; then
  echo "Installed udev rule does not match repository source" >&2
  exit 1
fi

echo "STM32 udev rules installed for exact BlackPill serial ${BLACKPILL_SERIAL}. Reconnect F411 if permissions do not refresh immediately."
