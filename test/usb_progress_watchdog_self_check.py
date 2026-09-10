#!/usr/bin/env python3
from pathlib import Path
h=Path(__file__).parents[1]/'include/UsbCdcPort.h'
c=Path(__file__).parents[1]/'src/usb/UsbCdcPort.cpp'
m=Path(__file__).parents[1]/'src/main.cpp'
s=h.read_text()+c.read_text()+m.read_text()
checks={
 'busy age':'txBusyAgeMs() const',
 'tx age':'lastTxCompleteAgeMs() const',
 'rx age':'lastRxAgeMs() const',
 'queue depth':'txHighQueueDepth() const',
 'progress counter':'tx_progress_stall_count_',
 'middleware state':'hcdc->TxState == 0U',
 'explicit soft restart':'if (explicit_recovery)',
 'no timed soft restart':'kTxStallSoftRestartMs',
 'reset cause':'reset_csr=%08lX',
}
for name,tok in checks.items():
    if name=='no timed soft restart':
        assert tok not in s, 'timed autonomous soft restart reintroduced'
    else:
        assert tok in s, f'missing {name}'
print('USB_PROGRESS_WATCHDOG_SELF_CHECK_PASS')
