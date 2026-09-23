#!/usr/bin/env python3
from pathlib import Path

root = Path(__file__).resolve().parents[1]
rules = (root / "99-f103c8-gateway.rules").read_text()
installer = (root / "scripts/install_stm32_udev_rules.sh").read_text()

expected = (
    ("5740", "BLUEPILL_F103 CDC in FS Mode", "f4gateway"),
    ("5741", "BLUEPILL_F103 BOOT CDC", "f4gateway-boot"),
)
for pid, product, alias_name in expected:
    usb = [x for x in rules.splitlines()
           if 'SUBSYSTEM=="usb"' in x and f'idProduct}}=="{pid}"' in x]
    tty = [x for x in rules.splitlines()
           if 'SUBSYSTEM=="tty"' in x and f'idProduct}}=="{pid}"' in x]
    if len(usb) != 1 or len(tty) != 1:
        raise SystemExit(f"FAIL role rule count PID={pid}")
    if f'ATTR{{product}}=="{product}"' not in usb[0]:
        raise SystemExit(f"FAIL USB product scope PID={pid}")
    if f'ATTRS{{product}}=="{product}"' not in tty[0]:
        raise SystemExit(f"FAIL TTY product scope PID={pid}")
    if 'ID_MM_DEVICE_IGNORE' not in usb[0] or 'ID_MM_DEVICE_IGNORE' not in tty[0]:
        raise SystemExit(f"FAIL ModemManager isolation PID={pid}")
    if f'SYMLINK+="{alias_name}"' not in tty[0]:
        raise SystemExit(f"FAIL alias {alias_name}")

if 'cmp -s "$RULE_SRC" "$RULE_DST"' not in installer:
    raise SystemExit("FAIL installer verification")
if "ATTR{serial}" in rules or "ATTRS{serial}" in rules:
    raise SystemExit("FAIL clone support: hardcoded MCU serial remains")
print("UDEV_IDENTITY_SCOPE_SELF_CHECK_PASS")
