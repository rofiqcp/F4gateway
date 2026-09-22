#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).resolve().parents[1]
r=(root/'99-blackpill-stm32.rules').read_text()
i=(root/'scripts/install_stm32_udev_rules.sh').read_text()
serials=('33A433673134','319435643038','1A5B1F830000')
for pid in ('5740','5741'):
    usb=[x for x in r.splitlines() if 'SUBSYSTEM=="usb"' in x and f'idProduct}}=="{pid}"' in x]
    tty=[x for x in r.splitlines() if 'SUBSYSTEM=="tty"' in x and f'idProduct}}=="{pid}"' in x]
    if len(usb)!=len(serials): raise SystemExit(f'FAIL USB {pid} count')
    if len(tty)!=len(serials): raise SystemExit(f'FAIL TTY {pid} count')
    for serial in serials:
        if not any(f'ATTR{{serial}}=="{serial}"' in x for x in usb): raise SystemExit(f'FAIL USB {pid} serial {serial}')
        if not any(f'ATTRS{{serial}}=="{serial}"' in x for x in tty): raise SystemExit(f'FAIL TTY {pid} serial {serial}')
f103=[x for x in r.splitlines() if '1A5B1F830000' in x]
if len(f103)!=4: raise SystemExit('FAIL F103 identity rule count')
if not all('ID_MM_DEVICE_IGNORE' in x for x in f103): raise SystemExit('FAIL F103 ModemManager isolation')
if not all('power/control' in x for x in f103 if 'SUBSYSTEM=="usb"' in x): raise SystemExit('FAIL F103 USB power policy')
if 'SYMLINK+="f4gateway"' not in r or 'SYMLINK+="f4gateway-boot"' not in r: raise SystemExit('FAIL F103 stable role aliases')
if 'cmp -s "$RULE_SRC" "$RULE_DST"' not in i: raise SystemExit('FAIL installer verification')
if 'STM32_CDC_SERIALS=("33A433673134" "319435643038" "1A5B1F830000")' not in i: raise SystemExit('FAIL installer serial allowlist')
print('UDEV_IDENTITY_SCOPE_SELF_CHECK_PASS')
