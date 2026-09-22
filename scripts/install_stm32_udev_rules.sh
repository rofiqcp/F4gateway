#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
RULE_SRC="$ROOT_DIR/99-blackpill-stm32.rules"
RULE_DST="/etc/udev/rules.d/99-blackpill-stm32.rules"

if [[ ! -f "$RULE_SRC" ]]; then
  echo "Rule file not found: $RULE_SRC" >&2
  exit 1
fi

# Runtime/boot CDC permissions are deliberately scoped to approved board
# identities. Never grant USBDEVFS access to arbitrary 0483:5740/5741 devices.
STM32_CDC_SERIALS=("33A433673134" "319435643038" "1A5B1F830000")
for serial in "${STM32_CDC_SERIALS[@]}"; do
  for pid in 5740 5741; do
    if ! grep -Eq "SUBSYSTEM==\"usb\".*idProduct}==\"${pid}\".*ATTR\{serial\}==\"${serial}\"" "$RULE_SRC"; then
      echo "Refusing to install non identity-scoped STM32 CDC rule for serial ${serial} PID ${pid}" >&2
      exit 1
    fi
  done
done

sudo install -m 0644 "$RULE_SRC" "$RULE_DST"
sudo udevadm control --reload-rules
sudo udevadm trigger
if ! cmp -s "$RULE_SRC" "$RULE_DST"; then
  echo "Installed udev rule does not match repository source" >&2
  exit 1
fi

echo "STM32 udev rules installed for approved gateway serials. Reconnect the board USB if permissions do not refresh immediately."
