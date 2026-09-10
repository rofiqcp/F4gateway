#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).resolve().parents[1]
r=(root/'99-blackpill-stm32.rules').read_text()
i=(root/'scripts/install_stm32_udev_rules.sh').read_text()
serial='338133833134'
for pid in ('5740','5741'):
    usb=[x for x in r.splitlines() if 'SUBSYSTEM=="usb"' in x and f'idProduct}}=="{pid}"' in x]
    tty=[x for x in r.splitlines() if 'SUBSYSTEM=="tty"' in x and f'idProduct}}=="{pid}"' in x]
    if len(usb)!=1 or f'ATTR{{serial}}=="{serial}"' not in usb[0]: raise SystemExit(f'FAIL USB {pid} scope')
    if len(tty)!=1 or f'ATTRS{{serial}}=="{serial}"' not in tty[0]: raise SystemExit(f'FAIL TTY {pid} scope')
if 'cmp -s "$RULE_SRC" "$RULE_DST"' not in i: raise SystemExit('FAIL installer verification')
if 'BLACKPILL_SERIAL="338133833134"' not in i: raise SystemExit('FAIL installer serial guard')
print('UDEV_IDENTITY_SCOPE_SELF_CHECK_PASS')
