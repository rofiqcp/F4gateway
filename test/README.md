# Firmware self-checks

Run every maintained source-contract test with:

```bash
for f in test/*_self_check.py; do python3 "$f" || exit 1; done
```

The checks cover BTS7960/limit-switch safety, parity with `/forclift/f4`, HMI/SPI,
USB session integrity and recovery, flash layout, telemetry, persistence, and the
absence of legacy MCP2515/DroneCAN code. Hardware flashing is verified separately
with `scripts/provision_recovery_stlink.sh` and `scripts/cdc_boot_upload.py`.
