#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
h = (ROOT / "src/PersistentConfigStore.h").read_text()
cpp = (ROOT / "src/PersistentConfigStore.cpp").read_text()
main = (ROOT / "src/main.cpp").read_text()
pio = (ROOT / "platformio.ini").read_text()

checks = {
    "C8 persistent base": "PERSIST_STORAGE_BASE 0x0800F800UL" in h,
    "C8 persistent limit": "PERSIST_STORAGE_LIMIT 0x08010000UL" in h,
    "PlatformIO base synchronized": "-DPERSIST_STORAGE_BASE=0x0800F800UL" in pio,
    "PlatformIO limit synchronized": "-DPERSIST_STORAGE_LIMIT=0x08010000UL" in pio,
    "record fixed 48 bytes": "sizeof(Record) == 48U" in h,
    "payload 28 bytes": "kMaxValueBytes = 28U" in h,
    "CRC32 present": "PersistentConfigStore::crc32" in cpp,
    "two-phase halfword commit": "address + 44U" in cpp and "address + 46U" in cpp and "kCommitWord" in h,
    "payload verified before commit": 'memcmp(reinterpret_cast<const void *>(address), &record, 44U)' in cpp,
    "same-value write elided": "std::memcmp(current.data, data, length) == 0" in cpp,
    "no erase in config store": "HAL_FLASHEx_Erase" not in cpp,
    "torn records ignored": "if (!valid(record))" in cpp,
    "flash lock restored": "HAL_FLASH_Lock" in cpp,
    "store initialized": "gPersistentConfig.begin()" in main,
    "touch calibration uses persistent store": "gPersistentConfig.write(kTouchCalibrationKey" in main and
                                                "gPersistentConfig.read(kTouchCalibrationKey" in main,
    "winch receives persistent store": "Bts7960Winch::begin(gPersistentConfigReady ? &gPersistentConfig : nullptr)" in main,
}

failed = [name for name, ok in checks.items() if not ok]
for name, ok in checks.items():
    print(("PASS" if ok else "FAIL"), name)
if failed:
    raise SystemExit("FAILED: " + ", ".join(failed))

slots = (0x08010000 - 0x0800F800) // 48
assert slots == 42
print(f"PASS C8 persistent geometry slots={slots}, payload=28 bytes")
print("F103C8_PERSISTENT_CONFIG_STORE_SELF_CHECK_PASS")
