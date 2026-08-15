#!/usr/bin/env bash
# Build the s31 firmware via idf.py directly (the verified-working path —
# see BUILD.md; `pio run` is currently not recommended for this target).
#
# Usage: scripts/build.sh [extra idf.py args...]
#   scripts/build.sh              normal build
#   scripts/build.sh fullclean    e.g. to force a clean rebuild
set -euo pipefail

source "$(dirname "${BASH_SOURCE[0]}")/env.sh"
cd "$S31_DIR"

idf.py -DIDF_TARGET=esp32s31 "${@:-build}"
