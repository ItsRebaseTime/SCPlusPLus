#!/usr/bin/env bash
# Shared environment for the s31 build/flash scripts. Source this, don't run
# it directly: `source scripts/env.sh`.
#
# Mirrors the verified-working recipe in BUILD.md ("Current recipe: official
# espressif/esp-idf master") — official espressif/esp-idf checkout, not the
# stale pioarduino fork that platformio.ini still points at.

# Prefer the known-good checkout over any inherited $IDF_PATH — a stray
# IDF_PATH from copy-pasting BUILD.md's literal `/path/to/esp-idf-official`
# example into a shell earlier in the session should not silently win here.
DEFAULT_IDF_PATH="$HOME/.espressif/esp-idf-official"
if [ -d "$DEFAULT_IDF_PATH" ]; then
    export IDF_PATH="$DEFAULT_IDF_PATH"
else
    export IDF_PATH="${IDF_PATH:-$DEFAULT_IDF_PATH}"
fi
export IDF_MAINTAINER=1
export ESP_IDF_VERSION="6.2.0"

TOOLCHAIN_BIN="$HOME/.espressif/toolchain/riscv32-esp-elf/bin"

# The build/ directory already configured in this project was cmake-
# configured against PlatformIO's copy of the idf6.1 venv (a symlink to
# .platformio/penv/.espidf-6.0.0), not the .espressif/ one BUILD.md
# describes reusing — cmake hard-fails ("configured with a different
# python") if the active python doesn't match exactly. Match what's
# already configured rather than forcing an unnecessary fullclean.
IDF_PYTHON_ENV_DIR="$HOME/.platformio/python_env/idf6.1_py3.14_env"
IDF_VENV_BIN="$IDF_PYTHON_ENV_DIR/bin"

# cmake/ninja were never installed system-wide or via idf_tools.py on this
# machine — reuse PlatformIO's bundled copies rather than requiring a
# separate system install.
CMAKE_BIN="$HOME/.platformio/packages/tool-cmake/bin"
NINJA_BIN="$HOME/.platformio/packages/tool-ninja"

if [ ! -d "$IDF_PATH" ]; then
    echo "error: IDF_PATH ($IDF_PATH) not found — see BUILD.md 'Current recipe'" >&2
    return 1 2>/dev/null || exit 1
fi
if [ ! -d "$TOOLCHAIN_BIN" ]; then
    echo "error: toolchain not found at $TOOLCHAIN_BIN — see BUILD.md 'Current recipe'" >&2
    return 1 2>/dev/null || exit 1
fi

export IDF_PYTHON_ENV_PATH="$IDF_PYTHON_ENV_DIR"
export PATH="$IDF_VENV_BIN:$TOOLCHAIN_BIN:$CMAKE_BIN:$NINJA_BIN:$IDF_PATH/tools:$PATH"

# idf.py's shebang (`#!/usr/bin/env python`) resolves to bin/python, but the
# already-configured build/ dir's CMakeCache recorded the exact string
# bin/python3 (same file via symlink, but CMake compares it literally and
# refuses to proceed on a mismatch). Invoke through python3 explicitly so
# sys.executable matches what's already configured.
idf.py() {
    "$IDF_VENV_BIN/python3" "$IDF_PATH/tools/idf.py" "$@"
}

S31_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
S31_PORT="${S31_PORT:-/dev/ttyACM0}"
S31_BAUD="${S31_BAUD:-460800}"
