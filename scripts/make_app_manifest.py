#!/usr/bin/env python3
import argparse
import struct
import time
import zlib
from pathlib import Path

MAGIC = 0x31564741  # AGV1
FORMAT = 2
APP_BASE = 0x08004000
PROFILES = {
    "f411ce": (0x08060000, 0xF411CE01, 0x20020000),
    "f411cc": (0x08020000, 0xF411CC01, 0x20020000),
    "f401cd": (0x08040000, 0xF401CD01, 0x20018000),
    "f103": (0x0803E000, 0xF1030101, 0x20005000),
}
parser = argparse.ArgumentParser()
parser.add_argument("image", type=Path)
parser.add_argument("output", type=Path)
parser.add_argument("--target", choices=sorted(PROFILES), default="f411ce")
args = parser.parse_args()
APP_LIMIT, BOARD_ID, SRAM_END = PROFILES[args.target]
image = args.image.read_bytes()
out = args.output
if len(image) < 8 or len(image) > APP_LIMIT - APP_BASE:
    raise SystemExit(f"invalid application size: {len(image)}")
sp, reset = struct.unpack_from("<II", image, 0)
if not (0x20000000 <= sp <= SRAM_END) or (sp & 3):
    raise SystemExit(f"invalid application MSP: 0x{sp:08X}")
if (reset & 1) == 0:
    raise SystemExit(f"invalid application reset vector (not Thumb): 0x{reset:08X}")
pc = reset & ~1
if not (APP_BASE <= pc < APP_BASE + len(image)):
    raise SystemExit(f"application reset vector outside image: 0x{pc:08X}")
app_crc = zlib.crc32(image) & 0xFFFFFFFF
head = struct.pack("<5I", MAGIC, FORMAT, APP_BASE, len(image), app_crc)
header_crc = zlib.crc32(head) & 0xFFFFFFFF
generation = int(time.time()) & 0xFFFFFFFF
manifest = head + struct.pack("<3I", header_crc, generation, BOARD_ID)
out.write_bytes(manifest)
print(f"manifest size={len(manifest)} app_size={len(image)} app_crc=0x{app_crc:08X} header_crc=0x{header_crc:08X} gen={generation}")
