# SC++

ESP32-S31 firmware that bridges a Steam Controller 2026 to Linux/Windows/console hosts as a **DualSense Edge**, while the same device simultaneously acts as a bidirectional Bluetooth **LE Audio (LC3) headset** — one BLE identity, one GATT server, both roles at once.

## How it works

The host sees a single BLE peripheral that's both a DualSense Edge gamepad and an LE Audio headset. The ESP32-S31 sits in the middle:

```
Steam Controller 2026 (USB host)
         ↓ HID Interrupt IN (54-byte SCReport @ 4ms)
    ESP32-S31 (SC++)
         ├─ Button/stick/touchpad/IMU → BLE HID → DualSense Edge (host)
         ├─ Rumble output reports → SC haptic engine (USB OUT)
         ├─ ES8311 mic → LC3 encode → BAP Source ASE → host (voice chat)
         └─ Host → BAP Sink ASE → LC3 decode → ES8311 speaker
```

Both roles share one NimBLE host, one GATT server, and one BLE connection — the HID-over-GATT service (DualSense Edge emulation) and the LE Audio PACS/ASCS services (BAP Unicast Server) are registered together and advertised as a single combo device.

## Hardware

| Component | Details |
|-----------|---------|
| MCU | ESP32-S31-Function-CoreBoard-1 (dual-core RISC-V, BLE 5.4 with native LE Audio/ISO) |
| Controller | Steam Controller 2026 (USB HID, VID `0x28DE` / PID `0x1302`), via the board's USB-A host port |
| Audio codec | ES8311 (I2C control + I2S data) driving an onboard mic and an NS4150B speaker amp |
| Power | USB-powered only — no PMIC/battery on this board |

### GPIO pinout

| Signal | GPIO |
|--------|------|
| Codec I2C SDA | 51 |
| Codec I2C SCL | 50 |
| Codec I2S MCLK | 52 |
| Codec I2S SCLK (BCLK) | 53 |
| Codec I2S ASDOUT (mic → MCU) | 54 |
| Codec I2S LRCK (word select) | 55 |
| Codec I2S DSDIN (MCU → speaker) | 56 |
| NS4150B amp enable (PA_CTRL) | 57 |
| RGB LED | 60 |
| BOOT button | 61 |

Pins traced directly from the official board schematic — see the comment in [firmware/s31/main/config.h](firmware/s31/main/config.h) for the source and one still-unconfirmed detail (NS4150B enable polarity).

## Features

### DualSense Edge emulation
- Full button mapping: face, bumpers, triggers, paddles (L4/L5/R4/R5), d-pad with diagonals, system buttons
- Analog sticks with coordinate frame correction (SC Y-up → DS Y-down)
- Analog triggers (0–32767 → 0–255)
- Dual touchpad emulation — SC left pad maps to DS left half (0–959 X), right pad to DS right half (960–1919 X)
- IMU pass-through (accelerometer + gyroscope)
- Rumble forwarded from BLE output report back to SC haptic engine
- Adaptive-trigger output reports are parsed (so the host doesn't see malformed responses) but not acted on — this board has no trigger actuator hardware
- Battery Service reports a fixed AC-powered state — no PMIC on this board

### Bidirectional LE Audio (LC3)
- BAP Unicast Server: one Sink ASE (speaker playback) + one Source ASE (mic capture), registered via ESP-IDF's `esp_ble_audio` PACS/ASCS API
- LC3 encode (mic) / decode (speaker) via `espressif/esp_audio_codec`, using the codec parameters actually negotiated per-connection (sample rate, frame duration, octets/frame) rather than fixed assumptions
- Shares the same GATT server, connection, and advertisement as the DualSense Edge HID service

### Startup audio (unrelated to LE Audio)
Startup audio plays through the Steam Controller's own touchpad haptic actuators using its PCM streaming mode (0x88 packets, 31 samples/packet per channel, 8 kHz stereo, 4 ms period) — this is a completely separate path from the ES8311/LE-Audio speaker+mic system above; they share nothing but the word "audio."

The sound is embedded as a C array in [firmware/s31/main/startup_audio.h](firmware/s31/main/startup_audio.h). Convert any audio file to replace it:

```sh
./tools/gen_startup_audio.sh your_sound.mp3
```

Requires `ffmpeg` and `python3`. Writes directly to `firmware/s31/main/startup_audio.h` (8 kHz, signed 8-bit, stereo interleaved PCM). Keep it short — the array is embedded in flash; at 8 kHz stereo s8, 30 seconds is ~480 KB of source.

## Building

`firmware/s31/` is a standalone project (native ESP-IDF, not Arduino) — see **[firmware/s31/BUILD.md](firmware/s31/BUILD.md)** for the full, verified build recipe, including a known `pio run` bug on this brand-new chip target and the `idf.py`-direct workaround.

```sh
cd firmware/s31
idf.py -DIDF_TARGET=esp32s31 build
idf.py -DIDF_TARGET=esp32s31 -p /dev/ttyACM0 flash monitor
```

### Dependencies

Pulled automatically via ESP-IDF's Component Manager (`firmware/s31/main/idf_component.yml`): `espressif/usb`, `espressif/esp_codec_dev`, `espressif/esp_bt_audio` (which transitively brings in `espressif/esp_audio_codec` for LC3). The DualSense Edge HID-over-GATT service is hand-rolled against ESP-IDF's native NimBLE API — no external HID library dependency (unlike this project's discontinued Arduino/ESP32-S3 build, which depended on a forked `ESP32-BLE-CompositeHID`).

## Source layout

```
firmware/s31/
  BUILD.md                    — full build recipe + confirmed/open findings
  main/
    main.cpp                  — boot sequence, 4 ms poll loop
    sc_input.{h,cpp}           — USB host driver, Steam Controller HID, haptic/PCM engine
    sc_report.h                — SCReport/SCTouchReport structs, button flag enum
    ds_report.h                — DualSense Edge wire-format structs/constants
    ds_edge_hid.{h,cpp}        — DualSense Edge HID-over-GATT service (native NimBLE)
    le_audio_bap.{h,c}         — BAP Unicast Server (PACS/ASCS), shared NimBLE/GATT lifecycle, LC3 data path
    le_audio_codec.{h,cpp}     — ES8311/NS4150B I2C+I2S bring-up
    config.h                   — GPIO assignments
    startup_audio.h            — embedded startup PCM (8 kHz, s8, stereo) for SC touchpad haptics
tools/
  gen_startup_audio.sh         — audio file → startup_audio.h converter
boards/
  esp32s31_coreboard1.json     — PlatformIO board definition
```

Note `le_audio_bap.c` is plain C, not C++, unlike the rest of this codebase — see the comment at the top of that file for why (`esp_ble_audio`'s own headers use syntax that's legal C but not legal ISO C++).

## Thread model

| Task | Role |
|---|---|
| `app_main` main loop | Poll SC report → map to DS input report → GATT notify |
| `usb_lib_task` / `usb_client_task` | USB host stack + SC HID client (rumble, haptic PCM) |
| `nimble_host_task` | NimBLE host controller interface + GATT event processing |
| `mic_encode_task` | Mic PCM → LC3 encode → BAP Source ASE send, while streaming |

Speaker decode runs inline in the BAP stream's `recv` callback rather than its own task.

## Status

Full build (USB host + DualSense Edge BLE HID + bidirectional LE Audio BAP, all sharing one GATT server) compiles and links cleanly against ESP-IDF master for the real `esp32s31` target. **Not yet verified on real hardware** — see `firmware/s31/BUILD.md`'s "What's confirmed vs. still open" section for the current state and known unknowns (NS4150B polarity, ISO/CIS timing under concurrent HID+audio load, etc).
