#!/usr/bin/env python3
"""Offline contract for the optimized STM32F411 16K/368K/persistent layout."""
import importlib.util
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
app_ini = (ROOT / "platformio.ini").read_text()
boot_ini = (ROOT / "bootloader/platformio.ini").read_text()
linker = (ROOT / "linker/STM32F411CEUX_APP.ld").read_text()
boot = (ROOT / "bootloader/src/main.c").read_text()
isr = (ROOT / "bootloader/src/isr_vectors.S").read_text()
dna = (ROOT / "src/DroneCanDnaDatabase.h").read_text()
cfg = (ROOT / "src/PersistentConfigStore.h").read_text()
usbd = (ROOT / "bootloader/include/usbd_conf.h").read_text()
cdc = (ROOT / "scripts/cdc_boot_upload.py").read_text()
dfu = (ROOT / "scripts/dfu_upload_blackpill.sh").read_text()
stlink = (ROOT / "scripts/provision_recovery_stlink.sh").read_text()

checks = {
    "boot max 16 KiB": "board_upload.maximum_size = 16384" in boot_ini,
    "boot app base": "-DAPP_BASE=0x08004000UL" in boot_ini,
    "boot app limit": "-DAPP_LIMIT=0x08060000UL" in boot_ini,
    "manifest journal 16 KiB": "-DMANIFEST_BASE=0x08060000UL" in boot_ini and "-DMANIFEST_LIMIT=0x08064000UL" in boot_ini,
    "LTO and section GC": "-flto" in boot_ini and "-Wl,--gc-sections" in boot_ini,
    "ISR strong veneers": ".global SysTick_Handler" in isr and ".global OTG_FS_IRQHandler" in isr,
    "app max 368 KiB": "board_upload.maximum_size = 376832" in app_ini,
    "app vector offset": "-DVECT_TAB_OFFSET=0x00004000U" in app_ini,
    "linker app region": "ORIGIN = 0x08004000" in linker and "LENGTH = 0x5C000" in linker,
}
checks.update({
    "EEPROM partition": "kStorageBase = 0x08064000UL" in cfg and "kStorageLimit = 0x0806C000UL" in cfg,
    "DNA partition": "kStorageBase = 0x0807E000UL" in dna and "kStorageLimit = 0x08080000UL" in dna,
    "sector7 never erased": "FLASH_SECTOR_7" not in boot.split("static bool begin_update", 1)[1].split("static bool program_chunk", 1)[0],
    "app sectors1-6 erased": "FLASH_SECTOR_1" in boot and "s <= FLASH_SECTOR_6" in boot,
    "manifest capacity guard": "manifest_next_address() == 0U" in boot,
    "manifest append/readback": "manifest_next_address" in boot and "memcmp((const void *)target, &m, sizeof(m)) == 0" in boot,
    "protocol v2 only": "DATA2:" in boot and '"DATA:"' not in boot and "incompatible resident bootloader layout" in cdc and "AGVBL3-04000-60000" in cdc,
    "no heavy libc parser/formatter": "snprintf" not in boot and "strtoul" not in boot,
    "static USB allocation": "USBD_malloc USBD_static_malloc" in usbd and "#include <stdlib.h>" not in usbd,
    "CDC layout synchronized": "APP_BASE=0x08004000; APP_LIMIT=0x08060000" in cdc,
    "ROM DFU preserves persistent": "backup-persistent" in dfu and "restore-persistent" in dfu and "compose_persistent_image.py" in dfu,
    "ST-Link layout synchronized": "boot@08000000 app@08004000 persistent@08060000" in stlink,
    "ST-Link erases full app region": "flash erase_address 0x08004000 0x5C000" in stlink,
})

failed = [name for name, ok in checks.items() if not ok]
for name, ok in checks.items():
    print(("PASS" if ok else "FAIL"), name)
if failed:
    raise SystemExit("FAILED: " + ", ".join(failed))

spec = importlib.util.spec_from_file_location("persist", ROOT / "scripts/compose_persistent_image.py")
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
manifest = bytes(range(32))
legacy = bytearray(b"\xff" * mod.SECTOR_SIZE)
legacy[:40] = bytes((i * 13 + 7) & 0xFF for i in range(40))

# Already-compact layout must preserve EEPROM, reserved and DNA partitions.
current = bytearray(b"\xff" * mod.SECTOR_SIZE)
current[mod.MANIFEST_REGION_SIZE : mod.MANIFEST_REGION_SIZE + 4] = b"CFG1"
compact_dna = bytearray(legacy[:40]); compact_dna[0:2] = mod.DNA_MAGIC; compact_dna[20:22] = mod.DNA_MAGIC
current[mod.DNA_OFFSET : mod.DNA_OFFSET + 40] = compact_dna
image, mode, migrated = mod.compose(bytes(current), bytes(legacy), manifest)
assert mode == "compact-preserved" and migrated == 0
assert image[mod.MANIFEST_REGION_SIZE : mod.MANIFEST_REGION_SIZE + 4] == b"CFG1"
assert image[mod.DNA_OFFSET : mod.DNA_OFFSET + 40] == compact_dna

# Pre-compact v3: preserve first 16 KiB EEPROM and move DNA from offset 0x8000.
v3 = bytearray(b"\xff" * mod.SECTOR_SIZE)
v3[mod.MANIFEST_REGION_SIZE : mod.MANIFEST_REGION_SIZE + 4] = b"CFG1"
v3_dna = bytearray(legacy[:40]); v3_dna[0:2] = mod.DNA_MAGIC; v3_dna[20:22] = mod.DNA_MAGIC
v3[mod.OLD_V3_DNA_OFFSET : mod.OLD_V3_DNA_OFFSET + 40] = v3_dna
image, mode, migrated = mod.compose(bytes(v3), bytes(legacy), manifest)
assert mode == "v3-dna-compacted" and migrated == 40
assert image[mod.MANIFEST_REGION_SIZE : mod.MANIFEST_REGION_SIZE + 4] == b"CFG1"
assert image[mod.OLD_V3_DNA_OFFSET : mod.DNA_OFFSET] == b"\xff" * (mod.DNA_OFFSET - mod.OLD_V3_DNA_OFFSET)
assert image[mod.DNA_OFFSET : mod.DNA_OFFSET + 40] == v3_dna

# Layout-v2 DNA began immediately after manifest; migrate directly to compact DNA.
v2 = bytearray(b"\xff" * mod.SECTOR_SIZE)
v2_dna = bytearray(legacy[:40]); v2_dna[0:2] = mod.DNA_MAGIC; v2_dna[20:22] = mod.DNA_MAGIC
v2[mod.MANIFEST_REGION_SIZE : mod.MANIFEST_REGION_SIZE + 40] = v2_dna
image, mode, migrated = mod.compose(bytes(v2), bytes(legacy), manifest)
assert mode == "v2-dna-migrated" and migrated == 40
assert image[mod.DNA_OFFSET : mod.DNA_OFFSET + 40] == v2_dna

# Legacy fixed manifest: DNA source lived in Sector 6.
old = bytearray(b"\xff" * mod.SECTOR_SIZE)
import struct
struct.pack_into("<III", old, 0, mod.MANIFEST_MAGIC, mod.MANIFEST_FORMAT, mod.LEGACY_APP_BASE)
legacy_dna = bytearray(legacy); legacy_dna[0:2] = mod.DNA_MAGIC; legacy_dna[20:22] = mod.DNA_MAGIC
image, mode, migrated = mod.compose(bytes(old), bytes(legacy_dna), manifest)
assert mode == "legacy-sector6-migrated" and migrated == 40
assert image[mod.DNA_OFFSET : mod.DNA_OFFSET + 40] == legacy_dna[:40]
assert len(image) == mod.SECTOR_SIZE
assert mod.CONFIG_REGION_SIZE == 0x8000 and mod.RESERVED_SIZE == 0x12000 and mod.DNA_SIZE == 0x2000
print("PASS persistent composer compact+v3+v2+legacy migration")
print("F411_FLASH_LAYOUT_V2_SELF_CHECK_PASS")
