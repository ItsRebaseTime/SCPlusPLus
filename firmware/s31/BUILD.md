# Building for ESP32-S31 (Function-CoreBoard-1)

Phase 0 status (2026-08-13): **confirmed working end-to-end.** A minimal
`app_main()` plus the `espressif/esp_bt_audio` component (BAP Unicast
Server — the LE Audio/LC3 piece this project needs) both build and link
cleanly for the `esp32s31` target against ESP-IDF's `master` branch. This
document records the exact recipe, since none of it works out of the box
yet — the chip is roughly a month into mass production and tooling support
is still catching up.

## Why this isn't a normal `pio run`

Three things are true simultaneously right now:

1. **ESP32-S31 support in ESP-IDF is `master`-only.** No tagged release has
   it yet. You must build against a git checkout of
   `https://github.com/pioarduino/esp-idf` (or `espressif/esp-idf`) at
   `master`, not a released IDF version.
2. **`pio run -e s31coreboard` is currently broken** by a real bug in
   `pioarduino/platform-espressif32` (confirmed at release `55.03.311`):
   `install_tool()` correctly resolves and downloads the right GCC build via
   ESP-IDF's own `idf_tools.py`, but the follow-up step that registers it
   with PlatformIO's own package resolver doesn't stick — `pio run` then
   fails with `Error: Missing toolchain directory 'None'`, even from a
   completely clean package cache. This is worth re-testing against future
   pioarduino releases, but as of this date it reproduces reliably.
3. **The toolchain version matters and isn't what gets auto-installed.**
   ESP-IDF's own `tool_version_check.cmake` requires GCC build
   `esp-15.2.0_20251204` for this target specifically (it has the RISC-V ISA
   extensions this chip's core needs — `zaamo`, `zalrsc`, plus Espressif's
   own `xesploop`/`xespv`). The toolchain that gets installed automatically
   by default is `14.2.0_20260121`, which predates that ISA support and
   fails with `unsupported standard extension` errors on `-march=`. Fetch
   `esp-15.2.0_20251204` explicitly from Espressif's own crosstool-NG
   releases — pioarduino's registry mirror hadn't caught up to it as of this
   writing.

Given (2), the working path for now is to **drive `idf.py` directly**,
bypassing PlatformIO's build glue entirely. `platformio.ini` is still
checked in and may start working again once pioarduino ships a fix — worth
periodically retrying `pio run -e s31coreboard` — but don't rely on it today.

## One-time environment setup

```sh
# 1. Get ESP-IDF master (this pulls in the esp32s31-bt-lib and
#    esp-ble-audio-lib submodules specifically, confirming real S31/LE-Audio
#    support exists at the source level, not just as an announced feature)
git clone --recursive https://github.com/pioarduino/esp-idf.git /path/to/esp-idf
cd /path/to/esp-idf && git checkout master

export IDF_PATH=/path/to/esp-idf

# 2. Get the correct GCC toolchain (NOT the one idf_tools.py installs by
#    default today — see point 3 above)
curl -LO https://github.com/espressif/crosstool-NG/releases/download/esp-15.2.0_20251204/riscv32-esp-elf-15.2.0_20251204-<your-platform>.tar.xz
mkdir -p /path/to/toolchain && tar xf riscv32-esp-elf-15.2.0_20251204-*.tar.xz -C /path/to/toolchain
# binaries end up at /path/to/toolchain/riscv32-esp-elf/bin/

# 3. Set up IDF's Python venv the normal way
cd "$IDF_PATH"
./install.sh esp32s31   # or: python tools/idf_tools.py install-python-env
. ./export.sh
```

If step 3's `install.sh` also stumbles on the toolchain-version mismatch
from point 3 above, you can bootstrap the venv manually instead — this is
what was actually verified during the Phase 0 spike:

```sh
# Create/locate a Python 3.11+ venv, then:
pip install -r "$IDF_PATH/tools/requirements/requirements.core.txt"
touch "$IDF_TOOLS_PATH/espidf.constraints.v6.1.txt"   # skip the online constraints fetch
```

## Building this project

```sh
export IDF_PATH=/path/to/esp-idf
export PATH="/path/to/toolchain/riscv32-esp-elf/bin:$PATH"
export IDF_MAINTAINER=1   # downgrades the (expected, harmless) toolchain
                          # version-mismatch check from a hard error to a
                          # warning, in case anything still resolves the
                          # wrong GCC build first

cd firmware/s31
idf.py -DIDF_TARGET=esp32s31 build
```

A full clean build compiles ~1030 objects (WiFi, BT/NimBLE, LE Audio,
drivers, etc.) and finishes with `Project build complete.` To flash to real
hardware:

```sh
idf.py -DIDF_TARGET=esp32s31 -p /dev/ttyACM0 flash monitor
```

(Port name will vary — this board exposes a dedicated USB-C
Serial/JTOG debug port separate from the USB-A host port the Steam
Controller uses, so flashing shouldn't require the SC to be unplugged.)

### Phase 2 hardware bring-up: isolated audio loopback test

Before wiring the ES8311/NS4150B into LE Audio (Phase 3), verify the codec
wiring in isolation: uncomment the `target_compile_definitions(...
AUDIO_LOOPBACK_TEST)` line near the bottom of `main/CMakeLists.txt`, rebuild
(`rm -rf build && idf.py -DIDF_TARGET=esp32s31 build`), and flash. This
skips USB host + BLE entirely and just loops the onboard mic straight to
the onboard speaker — talk into the mic, you should hear yourself out the
speaker. Confirms real GPIO wiring, I2S timing, and ES8311 register config
all at once. Re-comment the line afterward for normal builds.

## Phase 3 addendum: getting a real LE Audio (BAP) build to link

Getting `le_audio_bap.c` (the PACS/ASCS BAP Unicast Server registration,
sharing one NimBLE host/GATT server with the DualSense HID service in
`ds_edge_hid.cpp`) to actually build required patching the vendored IDF
checkout itself (`~/.platformio/packages/framework-espidf`, NOT this repo) in
three places, plus two sdkconfig.defaults additions. None of this is
committed to this project — it's local toolchain state — so **a fresh
checkout of this toolchain needs these four steps redone** before this
target will link:

1. **`components/soc/esp32s31/include/soc/soc_caps.h`**: uncomment
   `SOC_BLE_ISO_SUPPORTED` and add `SOC_BLE_AUDIO_SUPPORTED` (both `(1)`),
   matching `esp32h4`'s copy of this file exactly. Also add matching
   `config SOC_BLE_ISO_SUPPORTED` / `config SOC_BLE_AUDIO_SUPPORTED` blocks
   (`bool` / `default y`) to the sibling `Kconfig.soc_caps.in` — without
   this the whole "BLE Audio Options" Kconfig menu stays unreachable
   (`depends on SOC_BLE_AUDIO_SUPPORTED`) and `CONFIG_BT_AUDIO`/`CONFIG_BT_ISO`
   can never be set at all, regardless of what's in sdkconfig.defaults.
2. **Update the `esp_ble_audio/lib/lib` submodule**: at
   `~/.platformio/packages/framework-espidf/components/bt/esp_ble_audio/lib/lib`,
   run `git fetch origin main && git checkout origin/main -- esp32s31
   esp32h4`. The commit this whole IDF checkout pins predates Espressif
   publishing an `esp32s31/libble_audio.a` blob for the closed-source LE
   Audio core library — only `esp32h4/libble_audio.a` exists at the pinned
   commit. `origin/main` already has it; the pin is just stale.
3. **`components/bt/CMakeLists.txt`**: the `if(CONFIG_BT_AUDIO)` block only
   has an `if(CONFIG_IDF_TARGET_ESP32H4) add_prebuilt_library(...)` — no
   ESP32-S31 branch exists at all. Add an `elseif(CONFIG_IDF_TARGET_ESP32S31)
   add_prebuilt_library(ble_audio "esp_ble_audio/lib/lib/esp32s31/libble_audio.a")
   target_link_libraries(${COMPONENT_LIB} PRIVATE ble_audio)` mirroring the
   H4 branch exactly.
4. **sdkconfig.defaults**: see the file itself — `CONFIG_BT_BAP_UNICAST_SERVER=y`,
   `CONFIG_BT_NIMBLE_MAX_CCCDS=24`, `CONFIG_BT_LE_ISO_SUPPORT=y`, and
   `CONFIG_BT_NIMBLE_ROLE_OBSERVER=y` (this last one for a genuinely
   surprising reason — NimBLE's own `ble_hs_hci_evt.c` nests the CIS/BIG ISO
   event-handler bodies inside a block that also requires Central or
   Observer role, unrelated to actually wanting to scan; see the in-file
   comment and the plan's Phase 3 section for the full trace).

`le_audio_bap.c` is deliberately plain C, not C++ — see its file header
comment. `esp_ble_audio`'s own public headers use a bare
`enum X;` forward-declaration pattern that's legal C but not legal ISO C++,
and unlike this project's other C-vs-C++ issues, `-fpermissive` doesn't
relax it.

## What's confirmed vs. still open

**Confirmed by actual green builds (Phases 0-2):**
- `esp32s31` target compiles and links cleanly against IDF master — full
  default component set (WiFi, NimBLE/BT, drivers including I2S and I2C,
  HAL, etc.) all build without target-specific errors.
- `espressif/esp_bt_audio` (BAP Unicast Server) builds cleanly for this
  target via the component manager — confirmed in Phase 0, currently
  removed from `idf_component.yml` until Phase 3 needs it again (see
  in-file comment; needs `CONFIG_BT_NIMBLE_EXT_ADV`).
- It pulls in `espressif/esp_audio_codec` automatically as a transitive
  dependency — that component provides the actual LC3 encode/decode, so the
  earlier open question ("does esp_bt_audio bundle LC3 itself?") is
  answered: yes, effectively, via this auto-pulled dependency.
- `esptool.py` 5.3.1 already has native ESP32-S31 image support
  (`Creating ESP32-S31 image...`).
- `sc_input.cpp` (USB host driver, ported from the S3/Arduino build) compiles
  clean against the real `usb_host.h`/`usb_phy.h` APIs on this target.
- A native ESP-IDF NimBLE HID-over-GATT DualSense Edge emulation
  (`ds_report.h`, `ds_edge_hid.h`/`.cpp`) replaces the Arduino-only
  ESP32-BLE-CompositeHID library used by the S3 build — builds and links
  cleanly.
- ES8311/NS4150B GPIO pins extracted directly from the official schematic
  (`config.h`) — not guessed. `le_audio_codec.h`/`.cpp` (I2C control + I2S
  data path via `esp_codec_dev`/`es8311`, following ESP-IDF's own
  `examples/peripherals/i2s/i2s_codec/i2s_es8311` reference) builds clean,
  including a standalone loopback-test configuration.

**Confirmed by an actual green build (Phase 3):**
- Full integration builds and links: USB host + DualSense Edge HID-over-GATT
  + PACS/ASCS BAP Unicast Server registration + shared NimBLE
  host/GATT-server/advertising, all in one image (`s31.bin`, 860KB, 18% of
  the 1MB app partition free). This required patching the vendored IDF
  checkout in several places (see "Phase 3 addendum" above) — none of it
  guessed, all traced to real source/build-system gaps via direct reading
  and, for one, preprocessor bisection.
- Ended up bypassing `espressif/esp_bt_audio` (the component-manager
  wrapper) entirely — its `esp_bt_audio_init()` is monolithic with no hook
  to add a custom GATT service to the same server. `le_audio_bap.c` instead
  calls the lower, documented `esp_ble_audio` API directly
  (`esp_ble_audio_pacs_api.h`/`esp_ble_audio_bap_api.h`), registering PACS +
  ASCS onto the same `ble_gatts_add_svcs()`/`ble_gatts_start()` sequence as
  the HID service.

**Confirmed by an actual green build (Phase 3, LC3 increment):**
- `on_stream_recv()` (speaker) LC3-decodes real ISO frames and writes PCM to
  the codec; `mic_encode_task` (mic) reads PCM, LC3-encodes it, and sends it
  via `esp_ble_audio_bap_stream_send()`. Both use `esp_lc3_dec`/`esp_lc3_enc`
  configured from the actually-negotiated codec params (freq/frame
  duration/octets), not hardcoded assumptions. `s31.bin` now 918KB (8% of
  the 1MB app partition free, down from 18% before this increment — worth
  watching).

**Still open:**
- Nothing has been tested on real hardware yet — no board available in the
  environment these builds ran in. Everything above is "compiles and links
  correctly," not "verified working." See the main plan's Verification
  section for the concrete next steps once hardware is available. This
  specifically includes the LC3 data path above: buffer sizes and the mic
  task's 10ms idle-poll cadence are reasonable first-pass choices, not
  tuned against real ISO/CIS timing.
- NS4150B amp enable polarity (active-high assumed in `le_audio_codec.cpp`)
  — not visible in the schematic crop used, confirm on hardware.
- Whether HID GATT-notify traffic and CIS audio streams coexist cleanly on
  one connection at runtime — needs real hardware, can't be tested by
  compiling alone.
- Whether the freshly-pulled `esp32s31/libble_audio.a` blob (published
  upstream but effectively unreleased/untested outside Espressif at this
  snapshot) actually works correctly at runtime — QoS negotiation, real
  CIS/LC3 data flow timing, all unverified.
