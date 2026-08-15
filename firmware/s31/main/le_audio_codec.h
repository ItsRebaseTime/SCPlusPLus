#pragma once
#include <cstdint>
#include <cstddef>

// ES8311 codec + NS4150B amp bring-up for the Function-CoreBoard-1's
// onboard mic/speaker. I2C control path + I2S data path via ESP-IDF's
// esp_codec_dev/es8311 components, following the official
// examples/peripherals/i2s/i2s_codec/i2s_es8311 reference pattern.
//
// This is a standalone audio I/O layer with no BLE/LE-Audio dependency —
// Phase 2 scope is a local mic-to-speaker loopback test to de-risk the
// hardware/pin wiring before Phase 3 wires it to the LE Audio BAP stream.

// Initialize I2C bus, I2S channels, and the ES8311 codec (playback + mic).
// Must be called once before any other codec_* call.
bool codec_init(uint32_t sample_rate_hz = 16000);

// le_audio_bap.c (compiled as plain C — see its file header for why) calls
// these and needs them reachable with C linkage; wrapped separately from
// codec_init() above since that one keeps a default argument, which is
// C++-only.
#ifdef __cplusplus
extern "C" {
#endif

// Reconfigure I2S clocks for a new sample rate (LE Audio negotiates this
// per-connection in Phase 3; not needed for the Phase 2 loopback test).
bool codec_set_sample_rate(uint32_t sample_rate_hz);

// Blocking I2S read/write, 16-bit stereo interleaved PCM. Returns bytes
// actually transferred (mirrors i2s_channel_read/write semantics).
size_t codec_write(const int16_t *samples, size_t byte_len);
size_t codec_read(int16_t *samples, size_t byte_len);

#ifdef __cplusplus
}
#endif

// 0-100.
void codec_set_speaker_volume(int volume_pct);
void codec_set_mic_gain(int gain_pct);

// Simple local loopback: mic -> speaker, blocking, runs forever on the
// calling task. Used to verify the codec/amp wiring end-to-end without any
// BLE involved (Phase 2 verification step).
void codec_loopback_task(void *arg);
