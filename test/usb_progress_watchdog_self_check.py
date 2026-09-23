#!/usr/bin/env python3
from pathlib import Path
root=Path(__file__).parents[1]
h=(root/'src/usb/UsbCdcPort.h').read_text()
c=(root/'src/usb/UsbCdcPort.cpp').read_text()
cdc=(root/'src/usb/usbd_cdc_if.cpp').read_text()
m=(root/'src/main.cpp').read_text()
s=h+c+cdc+m
checks={
 'busy age':'txBusyAgeMs() const',
 'tx age':'lastTxCompleteAgeMs() const',
 'rx age':'lastRxAgeMs() const',
 'queue depth':'txHighQueueDepth() const',
 'progress counter':'tx_progress_stall_count_',
 'middleware state':'hcdc->TxState == 0U',
 'explicit physical recovery':'const bool explicit_recovery = recovery_pending_',
 'main-context RX rearm':'F103Gateway_CdcTryRearmRxFromMain',
 'RX rearm failure counter':'rx_rearm_failure_count_',
 'RX rearm recovery counter':'rx_rearm_recovery_count_',
}
forbidden={
 'autonomous link-loss restart':'kUsbLossRecoveryMs',
 'autonomous RX-silence restart':'kHostRxSilenceRecoveryMs',
 'legacy RX-silence reason':'last_recovery_reason_ = 4U',
 'legacy initial-enum reason':'last_recovery_reason_ = 5U',
 'tx-stall timed soft restart':'kTxStallSoftRestartMs',
}
assert 'kInitialEnumerationRecoveryMs = 2500U' in s, 'missing bounded initial enumeration recovery'
assert 'kMaxInitialEnumerationRecoveries = 3U' in s, 'initial enumeration recovery must be bounded'
assert 'usb_auto_restart_count_ < kMaxInitialEnumerationRecoveries' in s, 'initial enumeration retry cap not enforced'
for name,tok in checks.items():
    assert tok in s, f'missing {name}'
for name,tok in forbidden.items():
    assert tok not in s, f'{name} reintroduced'
print('USB_PROGRESS_WATCHDOG_SELF_CHECK_PASS')
