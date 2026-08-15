#!/usr/bin/env bash
# Build then flash the s31 firmware in one step.
#
# Usage: scripts/build_flash.sh [port]
set -euo pipefail

DIR="$(dirname "${BASH_SOURCE[0]}")"
"$DIR/build.sh"
"$DIR/flash.sh" "$@"
