#!/usr/bin/env bash
# Open a serial monitor on the s31 board.
#
# Uses `pio device monitor`, per BUILD.md's "Using PlatformIO" section: it's
# confirmed working independent of `pio run` (which is not recommended for
# this target — see BUILD.md), since it only needs the serial port.
# monitor_speed is already set correctly in platformio.ini.
#
# Needs a real interactive TTY on stdin — won't run from a non-interactive
# shell/agent.
#
# Usage: scripts/monitor.sh [port]
set -euo pipefail

S31_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PORT="${1:-${S31_PORT:-/dev/ttyACM0}}"
PIO="$HOME/.platformio/penv/bin/pio"

cd "$S31_DIR"
exec "$PIO" device monitor -p "$PORT"
