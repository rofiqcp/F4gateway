#!/usr/bin/env python3
from pathlib import Path
ROOT=Path(__file__).resolve().parents[1]
s=(ROOT/'src/usb/UsbCdcPort.cpp').read_text()
start=s.index('void UsbCdcPort::flush')
end=s.index('void UsbCdcPort::onReceive', start)
block=s[start:end]
for token in ('service();','Board_RealtimeService();','__WFI();'):
    assert token in block, token
assert 'poll();' not in block, 'flush must not be a raw poll busy-loop'
assert 'HAL_Delay' not in block, 'flush must not block with HAL_Delay'
print('F103C8_COOPERATIVE_USB_FLUSH_SELF_CHECK_PASS')
