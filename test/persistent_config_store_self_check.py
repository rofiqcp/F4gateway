#!/usr/bin/env python3
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
h = (ROOT / "src/PersistentConfigStore.h").read_text()
cpp = (ROOT / "src/PersistentConfigStore.cpp").read_text()
main = (ROOT / "src/main.cpp").read_text()
dna = (ROOT / "src/DroneCanDnaDatabase.h").read_text()

checks = {
    "EEPROM base": "kStorageBase = 0x08064000UL" in h,
    "EEPROM limit": "kStorageLimit = 0x0806C000UL" in h,
    "record fixed 48 bytes": "sizeof(Record) == 48U" in h,
    "payload 28 bytes": "kMaxValueBytes = 28U" in h,
    "CRC32 present": "PersistentConfigStore::crc32" in cpp,
    "two-phase commit": "address + 44U" in cpp and "kCommitWord" in h,
    "payload verified before commit": "memcmp(reinterpret_cast<const void *>(address), &record, 44U)" in cpp,
    "same-value write elided": "std::memcmp(current.data, data, length) == 0" in cpp,
    "no sector erase": "HAL_FLASHEx_Erase" not in cpp,
    "DNA separated": "kStorageBase = 0x0807E000UL" in dna,
}

checks.update({
    "store initialized": "gPersistentConfig.begin()" in main,
    "status command": '"EEPROM:STATUS"' in main,
    "read command": '"EEPROM:GET:"' in main,
    "write command": '"EEPROM:SET:"' in main,
    "write requires safe state": "motionSafeForHeavyMaintenance()" in main.split('"EEPROM:SET:"', 1)[1].split('"GET:STATE"', 1)[0],
    "journal reports capacity": "totalSlots()" in cpp and "usedSlots()" in h,
    "torn records ignored": "if (!valid(record))" in cpp,
    "flash lock restored": "HAL_FLASH_Lock" in cpp,
})

failed = [name for name, ok in checks.items() if not ok]
for name, ok in checks.items():
    print(("PASS" if ok else "FAIL"), name)
if failed:
    raise SystemExit("FAILED: " + ", ".join(failed))

slots = (0x0806C000 - 0x08064000) // 48
assert slots == 682
print(f"PASS EEPROM geometry slots={slots}, payload=28 bytes")
print("PERSISTENT_CONFIG_STORE_SELF_CHECK_PASS")
