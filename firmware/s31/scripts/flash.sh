#!/usr/bin/env bash
# Flash the already-built s31 firmware via esptool directly.
#
# BUILD.md notes idf.py flash's "--diff-with" optimization is unreliable on
# this board (pyserial "Input/output error" on the second connect attempt),
# so this reuses build/s31.bin etc. directly instead, matching the verified
# recipe in BUILD.md's "Flashing" section.
#
# Usage: scripts/flash.sh [port]
#   S31_PORT / S31_BAUD env vars override the port/baud (default
#   /dev/ttyACM0 @ 460800).
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$S31_DIR"

PORT="${1:-$S31_PORT}"

if [ ! -f build/s31.bin ]; then
    echo "error: build/s31.bin not found — run scripts/build.sh first" >&2
    exit 1
fi

python3 -m esptool --chip esp32s31 -p "$PORT" -b "$S31_BAUD" \
    --before=default-reset --after=hard-reset write-flash \
    --flash-mode dio --flash-freq 40m --flash-size 16MB \
    0x2000 build/bootloader/bootloader.bin \
    0x8000 build/partition_table/partition-table.bin \
    0x10000 build/s31.bin
