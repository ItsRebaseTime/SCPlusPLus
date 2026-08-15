#!/usr/bin/env bash
# Convert any audio file to firmware/s31/main/startup_audio.h for SC haptic
# PCM streaming (Steam Controller touchpad haptics — unrelated to the
# ES8311/LE-Audio speaker+mic path, despite both saying "audio").
# Usage: ./tools/gen_startup_audio.sh <input_file> [duration_seconds]
# Requires: ffmpeg, python3
#
# Audio processing: none — raw conversion only.

set -euo pipefail

INPUT="${1:?Usage: $0 <input_file> [duration_seconds]}"
DURATION="${2:-}"
OUT_H="firmware/s31/main/startup_audio.h"
TMP_RAW="$(mktemp /tmp/startup_audio_XXXXXX.raw)"
trap 'rm -f "$TMP_RAW"' EXIT

DURATION_ARGS=()
[[ -n "$DURATION" ]] && DURATION_ARGS=(-t "$DURATION")

echo "[1/3] Converting to 8 kHz signed-8-bit stereo PCM..."
ffmpeg -y -i "$INPUT" \
    "${DURATION_ARGS[@]}" \
    -ar 8000 \
    -ac 2 \
    -f s8 \
    "$TMP_RAW"

BYTE_COUNT=$(wc -c < "$TMP_RAW")
DURATION_S=$(python3 -c "print(f'{$BYTE_COUNT/16000:.2f}')")
echo "[2/3] ${BYTE_COUNT} bytes (${DURATION_S}s at 8 kHz stereo)"

echo "[3/3] Generating ${OUT_H}..."
python3 - "$TMP_RAW" "$OUT_H" <<'EOF'
import sys, struct

raw_path, out_path = sys.argv[1], sys.argv[2]

with open(raw_path, 'rb') as f:
    data = f.read()

samples = struct.unpack(f'{len(data)}b', data)
cols = 16

with open(out_path, 'w') as f:
    f.write('#pragma once\n')
    f.write('#include <stdint.h>\n')
    f.write('#include <stddef.h>\n\n')
    f.write('static const int8_t startup_audio[] = {\n')
    for i in range(0, len(samples), cols):
        chunk = samples[i:i+cols]
        f.write('    ' + ', '.join(str(s) for s in chunk))
        f.write(',\n' if i + cols < len(samples) else '\n')
    f.write('};\n')
    f.write(f'static const size_t startup_audio_len = {len(samples)};\n')
EOF

echo "Done: ${OUT_H}"
