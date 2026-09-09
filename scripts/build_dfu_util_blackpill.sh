#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
TOOLS_DIR="$ROOT_DIR/.pio/tools"
SRC_DIR="$TOOLS_DIR/dfu-util-blackpill-src"
PREFIX_DIR="$TOOLS_DIR/dfu-util-blackpill"
BIN="$PREFIX_DIR/bin/dfu-util"
REPO="https://github.com/azorg/dfu-util.git"
COMMIT="61ab4ee"

if [[ -x "$BIN" ]]; then
  exit 0
fi

mkdir -p "$TOOLS_DIR"
echo "[USB-DFU] Building STM32F411-safe dfu-util (QUIRK_POLLTIMEOUT)..."
rm -rf "$SRC_DIR" "$PREFIX_DIR"
git clone --quiet "$REPO" "$SRC_DIR"
cd "$SRC_DIR"
git checkout --quiet "$COMMIT"
./autogen.sh >/dev/null
./configure --prefix="$PREFIX_DIR" >/dev/null
make -j2 >/dev/null
make install >/dev/null
[[ -x "$BIN" ]] || { echo "dfu-util build failed" >&2; exit 1; }
echo "[USB-DFU] Patched uploader ready: $BIN"
