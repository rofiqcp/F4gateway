#!/usr/bin/env python3
import argparse
import struct
from pathlib import Path

SECTOR_SIZE = 0x20000
MANIFEST_REGION_SIZE = 0x4000
MANIFEST_SIZE = 32
CONFIG_REGION_SIZE = 0x8000
RESERVED_OFFSET = MANIFEST_REGION_SIZE + CONFIG_REGION_SIZE
RESERVED_SIZE = 0x12000
DNA_OFFSET = 0x1E000
DNA_SIZE = 0x2000
DNA_RECORD_SIZE = 20
OLD_V3_DNA_OFFSET = 0x8000
MANIFEST_MAGIC = 0x31564741
MANIFEST_FORMAT = 2
LEGACY_APP_BASE = 0x08008000
DNA_MAGIC = b"\x01\xac"


def read_exact(path: Path, size: int, label: str) -> bytes:
    data = path.read_bytes()
    if len(data) != size:
        raise ValueError(f"{label} must be {size} bytes, got {len(data)}")
    return data


def dna_used(data: bytes) -> int:
    erased = b"\xff" * DNA_RECORD_SIZE
    for off in range(0, len(data), DNA_RECORD_SIZE):
        record = data[off:off + DNA_RECORD_SIZE]
        if len(record) < DNA_RECORD_SIZE or record == erased:
            return off
    return len(data)


def copy_dna(image: bytearray, source: bytes, mode: str) -> tuple[bytes, str, int]:
    used = dna_used(source)
    if used > DNA_SIZE:
        raise ValueError(f"DNA journal uses {used} bytes, exceeds compact {DNA_SIZE}-byte partition")
    image[DNA_OFFSET:DNA_OFFSET + DNA_SIZE] = b"\xff" * DNA_SIZE
    image[DNA_OFFSET:DNA_OFFSET + used] = source[:used]
    return bytes(image), mode, used


def compose(persistent: bytes, legacy_sector6: bytes, manifest: bytes) -> tuple[bytes, str, int]:
    image = bytearray(persistent)
    image[:MANIFEST_REGION_SIZE] = b"\xff" * MANIFEST_REGION_SIZE
    image[:MANIFEST_SIZE] = manifest

    # Already compact: keep EEPROM, reserved space and DNA exactly as-is.
    if persistent[DNA_OFFSET:DNA_OFFSET + 2] == DNA_MAGIC:
        return bytes(image), "compact-preserved", 0

    magic, fmt, app_base = struct.unpack_from("<III", persistent, 0)
    if magic == MANIFEST_MAGIC and fmt == MANIFEST_FORMAT and app_base == LEGACY_APP_BASE:
        image[MANIFEST_REGION_SIZE:] = b"\xff" * (SECTOR_SIZE - MANIFEST_REGION_SIZE)
        return copy_dna(image, legacy_sector6, "legacy-sector6-migrated")

    # Earliest optimized layout: DNA directly followed the 16 KiB manifest journal.
    if persistent[MANIFEST_REGION_SIZE:MANIFEST_REGION_SIZE + 2] == DNA_MAGIC:
        source = persistent[MANIFEST_REGION_SIZE:]
        image[MANIFEST_REGION_SIZE:] = b"\xff" * (SECTOR_SIZE - MANIFEST_REGION_SIZE)
        return copy_dna(image, source, "v2-dna-migrated")

    # Current pre-compact layout: 16 KiB EEPROM followed by a 96 KiB DNA journal.
    # Preserve the existing first 16 KiB EEPROM bytes, clear the newly expanded
    # EEPROM half + reserved region, and move only committed DNA records to 8 KiB.
    if persistent[OLD_V3_DNA_OFFSET:OLD_V3_DNA_OFFSET + 2] == DNA_MAGIC:
        source = persistent[OLD_V3_DNA_OFFSET:]
        image[OLD_V3_DNA_OFFSET:DNA_OFFSET] = b"\xff" * (DNA_OFFSET - OLD_V3_DNA_OFFSET)
        return copy_dna(image, source, "v3-dna-compacted")

    # No DNA records found. Preserve existing EEPROM bytes and initialize all
    # newly assigned space erased; this also covers a fresh device.
    image[OLD_V3_DNA_OFFSET:SECTOR_SIZE] = b"\xff" * (SECTOR_SIZE - OLD_V3_DNA_OFFSET)
    return bytes(image), "fresh-or-empty", 0


def main() -> int:
    parser = argparse.ArgumentParser(description="Compose STM32F411 Sector-7 persistent image")
    parser.add_argument("--persistent", required=True, type=Path)
    parser.add_argument("--legacy-sector6", required=True, type=Path)
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    args = parser.parse_args()
    persistent = read_exact(args.persistent, SECTOR_SIZE, "persistent Sector 7")
    legacy = read_exact(args.legacy_sector6, SECTOR_SIZE, "legacy Sector 6")
    manifest = read_exact(args.manifest, MANIFEST_SIZE, "manifest")
    image, mode, migrated = compose(persistent, legacy, manifest)
    args.output.write_bytes(image)
    print(f"[PERSIST] mode={mode} output={len(image)} bytes manifest_slots={MANIFEST_REGION_SIZE // MANIFEST_SIZE} config_bytes={CONFIG_REGION_SIZE} reserved_bytes={RESERVED_SIZE} dna_bytes={DNA_SIZE} dna_slots={DNA_SIZE // DNA_RECORD_SIZE} migrated_bytes={migrated}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
