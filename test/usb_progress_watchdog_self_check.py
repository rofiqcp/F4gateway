#!/usr/bin/env python3
from pathlib import Path
h=Path(__file__).parents[1]/'src/usb/UsbCdcPort.h'
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
 'explicit soft restart':'restart_requested = explicit_recovery',
 'sustained link-loss recovery':'kUsbLossRecoveryMs = 2000U',
 'single-shot link-loss latch':'usb_seen_configured_ = false',
 'no tx-stall timed soft restart':'kTxStallSoftRestartMs',
 'reset cause':'reset_csr=%08lX',
}
for name,tok in checks.items():
    if name=='no tx-stall timed soft restart':
        assert tok not in s, 'TX-stall autonomous soft restart reintroduced'
    else:
        assert tok in s, f'missing {name}'
print('USB_PROGRESS_WATCHDOG_SELF_CHECK_PASS')
