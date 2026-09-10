Import("env")

import datetime
import hashlib
import json
import subprocess
import zlib
from pathlib import Path

PROJECT_DIR = Path(env.subst("$PROJECT_DIR"))
SCHEMA_VERSION = 3


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

# Quoted C string macros are consumed through include/BuildInfo.h.
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
        "binary": binary.name,
        "size": len(payload),
        "crc32": "{:08X}".format(zlib.crc32(payload) & 0xFFFFFFFF),
        "sha256": hashlib.sha256(payload).hexdigest(),
        "app_base": "0x08008000",
        "app_limit": "0x08060000",
    }
    output = binary.with_name("firmware.identity.json")
    output.write_text(json.dumps(identity, indent=2, sort_keys=True) + "\n")
    print("[PROVENANCE] {}".format(output))


env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", write_identity)
