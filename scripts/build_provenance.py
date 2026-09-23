Import("env")

import datetime
import hashlib
import json
import subprocess
import zlib
from pathlib import Path

PROJECT_DIR = Path(env.subst("$PROJECT_DIR"))
PIO_ENV = env.subst("$PIOENV")
SCHEMA_VERSION = 3

# C++-only size optimizations; keep C compiler command lines warning-free.
env.Append(CXXFLAGS=["-fno-rtti", "-fno-use-cxa-atexit"])

IMAGE_LAYOUTS = {
    "f103_bootloader": (0x08000000, 0x08002000, "STM32F103C8T6-BOOT"),
    "f103_stlink": (0x08002000, 0x0800F7F0, "STM32F103C8T6"),
    "f103_usb": (0x08002000, 0x0800F7F0, "STM32F103C8T6"),
}

if PIO_ENV not in IMAGE_LAYOUTS:
    raise RuntimeError("No firmware image layout declared for PIOENV={}".format(PIO_ENV))

APP_BASE, APP_LIMIT, MCU_TARGET = IMAGE_LAYOUTS[PIO_ENV]


def git_text(*args):
    try:
        return subprocess.check_output(["git", "-C", str(PROJECT_DIR), *args], text=True).strip()
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


git_sha = git_text("rev-parse", "--short=12", "HEAD")
git_full_sha = git_text("rev-parse", "HEAD")
status = git_text("status", "--porcelain", "--untracked-files=normal")
git_dirty = 1 if status not in ("", "unknown") else 0
build_epoch = int(datetime.datetime.now(datetime.timezone.utc).timestamp())

# Quoted C string macros are consumed through src/BuildInfo.h.
env.Append(CPPDEFINES=[
    ("FW_GIT_SHA", '\\"{}\\"'.format(git_sha)),
    ("FW_GIT_FULL_SHA", '\\"{}\\"'.format(git_full_sha)),
    ("FW_GIT_DIRTY", git_dirty),
    ("FW_BUILD_EPOCH", build_epoch),
    ("FW_SCHEMA_VERSION", SCHEMA_VERSION),
])


def write_identity(target, source, env):
    binary = Path(str(target[0]))
    if not binary.exists():
        return
    payload = binary.read_bytes()
    identity = {
        "schema": SCHEMA_VERSION,
        "git_sha": git_sha,
        "git_full_sha": git_full_sha,
        "git_dirty": bool(git_dirty),
        "build_epoch_utc": build_epoch,
        "environment": PIO_ENV,
        "mcu_target": MCU_TARGET,
        "binary": binary.name,
        "size": len(payload),
        "crc32": "{:08X}".format(zlib.crc32(payload) & 0xFFFFFFFF),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "app_base": "0x{:08X}".format(APP_BASE),
        "app_limit": "0x{:08X}".format(APP_LIMIT),
    }
    output = binary.with_name("firmware.identity.json")
    output.write_text(json.dumps(identity, indent=2, sort_keys=True) + "\n")
    print("[PROVENANCE] {}".format(output))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", write_identity)
