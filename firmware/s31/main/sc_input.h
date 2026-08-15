#pragma once
#include "sc_report.h"
#include <stddef.h>

// Start USB host tasks (lib + client). Must be called from setup() before loop().
void sc_host_begin();

// Copy the latest report into *out and return true if a new one is available.
// Returns false (and leaves *out unchanged) when no new data has arrived.
bool sc_report_poll(SCReport* out);

// Copy the latest touch velocity report into *out and return true if a new one is available.
bool sc_touch_poll(SCTouchReport* out);

// Send haptic rumble to the SC. left/right are 0–255 motor strengths (0 = stop).
// Thread-safe; can be called from any task.
void sc_rumble(uint8_t left, uint8_t right);

// Send haptic tone to SC touchpad LRA actuators (CH_LPAD / CH_RPAD).
// left/right are 0–255 amplitude values (0 = stop). ~200 Hz tone frequency.
// Thread-safe; can be called from any task.
void sc_haptic_tone(uint8_t left, uint8_t right);

// Initialize SC haptic PCM engine for both touchpad channels (~2 s blocking).
// Called automatically on SC USB connect — only call manually for explicit re-init.
void sc_pcm_begin();

// Queue one PCM frame for delivery to the SC touchpad haptic actuators.
// samples: interleaved stereo (L0 R0 L1 R1 …), sample_count: per-channel (1–31).
// Thread-safe and non-blocking; drops the frame silently if the queue is full.
void sc_pcm_enqueue(const int8_t* samples, size_t sample_count);

// Block until the USB host library is installed and ready for client registration.
// Other tasks that need to register USB clients must call this before registering.
void sc_usb_host_wait();
