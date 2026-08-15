# Building for ESP32-S31 (Function-CoreBoard-1)

**Status as of 2026-08-15: confirmed working end-to-end on real hardware.**
Full build (USB host + DualSense Edge BLE HID + bidirectional LE Audio BAP,
sharing one GATT server) boots cleanly, advertises, and enumerates a real
Steam Controller over USB. Getting here required switching from pioarduino's
`esp-idf` fork to the **official `espressif/esp-idf` master** checkout (see
below) plus several real, hardware-only bugs fixed along the way — none of
which showed up in any of the earlier build-only verification.

## Current recipe: official `espressif/esp-idf` master

The pioarduino fork this project used through Phase 3 (see "Historical:
pioarduino recipe" below) turned out to be pinned to a commit from
**2026-04-26** — nearly four months stale relative to official
`espressif/esp-idf` master (commits through at least **2026-08-03** as of
this writing). That gap is why real-hardware bring-up hit a silent, zero-
diagnostic boot hang deep inside ESP-IDF's own SPI flash driver: Espressif
had already fixed S31-specific flash/bootloader issues upstream
(`fix(spi_flash): Fix suspend issue on esp32s31, and add trs timing on other
target` among several others) that simply never made it into pioarduino's
mirror. Switching to the real upstream fixed the hang immediately and also
made most of the Phase 3 manual IDF patches (see below) unnecessary, since
official master already has proper `esp32s31` support wired in natively.

```sh
# 1. Clone official ESP-IDF master (shallow + shallow submodules is fine —
#    only need the current tip, not history)
git clone --recursive --shallow-submodules --depth 1 -b master \
    https://github.com/espressif/esp-idf.git /path/to/esp-idf-official
export IDF_PATH=/path/to/esp-idf-official

# 2. Same GCC 15.2.0 toolchain as before — this is chip-architecture-
#    specific, not IDF-version-specific, so it's reusable as-is.
curl -LO https://github.com/espressif/crosstool-NG/releases/download/esp-15.2.0_20251204/riscv32-esp-elf-15.2.0_20251204-<your-platform>.tar.xz
mkdir -p /path/to/toolchain && tar xf riscv32-esp-elf-15.2.0_20251204-*.tar.xz -C /path/to/toolchain

# 3. Python env: the existing IDF 6.1-era venv can be reused, but two
#    packages need upgrading/adding for IDF 6.2's newer component-manager
#    interface version and gdb tooling:
pip install --upgrade idf-component-manager   # need >=3.x (interface_version 5);
                                               # 2.4.x only supports up to 4
pip install -r "$IDF_PATH/tools/requirements/requirements.core.txt"
touch "$IDF_TOOLS_PATH/espidf.constraints.v6.2.txt"   # skip the online constraints fetch
```

## Building this project

```sh
export IDF_PATH=/path/to/esp-idf-official
export PATH="/path/to/toolchain/riscv32-esp-elf/bin:$PATH"
export IDF_MAINTAINER=1
export ESP_IDF_VERSION="6.2.0"   # newer idf_component_manager reads this
                                  # from the environment directly; without
                                  # it, idf.py crashes at startup
                                  # (TypeError: expected string, got NoneType)

cd firmware/s31
idf.py -DIDF_TARGET=esp32s31 build
```

### Flashing

`idf.py ... flash`'s newer "diff with previously-flashed content" 
optimization (`--diff-with`) was unreliable here — hit `Could not configure
port: (5, 'Input/output error')` from pyserial consistently, tracing to a
second connect attempt this optimization makes. Flash via `esptool` directly
instead, which reuses the already-built `build/s31.bin`:

```sh
python3 -m esptool --chip esp32s31 -p /dev/ttyACM0 -b 460800 \
    --before=default-reset --after=hard-reset write-flash \
    --flash-mode dio --flash-freq 40m --flash-size 16MB \
    0x2000 build/bootloader/bootloader.bin \
    0x8000 build/partition_table/partition-table.bin \
    0x10000 build/s31.bin
```

(Port name will vary — this board exposes a dedicated USB-C Serial/JTAG
debug port separate from the USB-A host port the Steam Controller uses, so
flashing doesn't require the SC to be unplugged.)

### Serial monitor

`idf.py monitor` needs a real interactive TTY on stdin and won't run from a
non-interactive shell. A minimal pyserial script works for capturing boot
output in that case — toggle RTS to reset the board, then read:

```python
import serial, time
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.2)
ser.dtr = False; ser.rts = True; time.sleep(0.1); ser.rts = False
time.sleep(0.1)
# ...then read from ser for as long as needed
```

### Phase 2 hardware bring-up: isolated audio loopback test

Uncomment the `target_compile_definitions(... AUDIO_LOOPBACK_TEST)` line
near the bottom of `main/CMakeLists.txt`, rebuild (`rm -rf build && idf.py
-DIDF_TARGET=esp32s31 build`), and flash. Skips USB host + BLE entirely,
just loops the onboard mic straight to the onboard speaker. Re-comment
afterward for normal builds.

## Real hardware bring-up (2026-08-15) — full success, six real bugs found and fixed

First-ever real-hardware run of this project's S31 work. In order encountered:

1. **Silent boot hang, zero diagnostic output, inside ESP-IDF's own SPI
   flash driver** (before any app code ever ran) — root cause turned out to
   be the stale pioarduino IDF fork (see "Current recipe" above); switching
   to official `espressif/esp-idf` master fixed this immediately and
   completely. Two other things were tried and ruled out first (kept
   anyway, since both are independently correct): the flash size was wrongly
   left at IDF's 2MB default against the board's actual 16MB chip
   (`CONFIG_ESPTOOLPY_FLASHSIZE_16MB=y`), and the SPI clock was dropped from
   80MHz to 40MHz as a conservative margin measure
   (`CONFIG_ESPTOOLPY_FLASHFREQ_40M=y`).
2. **Missing `nvs_flash_init()`.** Standard ESP-IDF boilerplate, never added
   in this codebase — `esp_phy_load_cal_data_from_nvs()` needs NVS up before
   the BT controller starts. Its absence didn't just produce a benign log
   warning; it led into a Guru Meditation (Load access fault) shortly after,
   inside `phy_init`. Fixed in `le_audio_bap_begin()`.
3. **Missing `esp_ble_audio_common_init()` call.** Found by decoding the
   crash PC with `riscv32-esp-elf-addr2line` against the built ELF — crashed
   inside `bt_pacs_register_safe` (the closed-source LE Audio blob), a near-
   null pointer access. `esp_ble_audio_common_init()` → `bt_le_host_init()` +
   `bt_le_audio_init()` sets up internal state the PACS/ASCS "_safe" wrapper
   functions depend on; this codebase never called it, since the public API
   surface it belongs to (`esp_ble_audio_common_api.h`) wasn't part of what
   Phase 3's build-only verification ever exercised.
4. **`esp_ble_audio_common_init()` needs the NimBLE host already synced with
   the controller** — confirmed via the exact NimBLE log line
   (`BLE_HS_ENOTSYNCED: Attempt to use the host before it is synced with
   controller`). This meant restructuring `le_audio_bap.c`: PACS/ASCS/HID
   GATT registration and `ble_gatts_start()` — previously done synchronously
   in `le_audio_bap_begin()` before starting the NimBLE host task, the
   standard textbook NimBLE pattern — had to move into a function called
   from `on_sync()` instead, guarded against firing twice on host resync.
   Calling `esp_ble_audio_common_init()` also pulled in NimBLE GATT-*client*
   code (`ble_gattc_read`/`_by_uuid`/`_long`, `ble_gattc_write_flat`/
   `_no_rsp_flat`) at link time, requiring `CONFIG_BT_NIMBLE_ROLE_CENTRAL=y`
   even though this device never actually scans or initiates connections —
   apparently required for esp_ble_audio's internal cross-service discovery
   to link at all.
5. **`PacsInvSuppMask` — "available contexts" must be a subset of
   "supported contexts".** `esp_ble_audio_pacs_set_available_contexts()` was
   being called without ever calling the separate, easy-to-miss
   `esp_ble_audio_pacs_set_supported_contexts()` first, which defaults to
   empty. Fixed by calling both, sink and source, before setting available
   contexts.
6. **Legacy advertising payload over the 31-byte limit.**
   `ble_gap_adv_set_fields()` failed with `rc=4` (`BLE_HS_EMSGSIZE`) once it
   actually had real data to encode — flags + appearance + tx power + 2
   service UUIDs + the full device name together came to ~32 bytes, one over
   the legacy AD limit. Fixed by splitting across the primary advertisement
   (flags + both service UUIDs, ~9 bytes) and the scan response (appearance +
   tx power + name, ~23 bytes) via `ble_gap_adv_rsp_set_fields()` — a
   standard NimBLE pattern for exactly this problem.

Also hit and fixed, unrelated to real-hardware behavior but blocking the
switch to official IDF: newer `-Wall -Werror -Wextra`-by-default build flags
turned `-Wmissing-field-initializers` fatal across every file using partial
designated initializers (this codebase's idiomatic style throughout, matching
ESP-IDF's own) — suppressed centrally via `target_compile_options` in
`main/CMakeLists.txt` rather than touching every struct literal. A `#warning`
about FreeRTOS tick rate for ISO timing became fatal too — fixed by actually
following the recommendation (`CONFIG_FREERTOS_HZ=1000`), not just
suppressing it. Enabling Central role (fix #4 above) then overflowed the
default 1MB app partition by ~88KB — fixed by switching to IDF's built-in
"large" single-app partition table (2MB), given the board's real 16MB flash.

**End result, confirmed by real serial capture:** clean boot, no crashes, no
resets. `SC++ S31 — boot` → USB host ready → BT/NimBLE/PHY init → LE Audio
lib init → GATT services registered → `DualSense Edge BLE HID + LE Audio
ready` → advertising started (`GAP procedure initiated: extended advertise`)
→ **a real Steam Controller enumerated over USB** (`VID=0x28DE PID=0x1302`,
exact match) with its interrupt endpoints found and transfers started. The
device then sits stable in its steady-state loop with no further log output,
exactly as expected (nothing in the 4ms poll loop or the BLE stack's normal
operation logs at INFO level).

## Superseded: manual IDF patches from before the official-master switch

These are no longer needed — official `espressif/esp-idf` master already has
proper `esp32s31` support for all of this. Recorded only for history / in
case anyone is still on the old pioarduino fork:

1. `components/soc/esp32s31/include/soc/soc_caps.h` had `SOC_BLE_ISO_SUPPORTED`
   commented out and no `SOC_BLE_AUDIO_SUPPORTED` at all (both present and
   enabled on official master).
2. The `esp_ble_audio/lib/lib` submodule was pinned to a commit predating
   Espressif publishing an `esp32s31/libble_audio.a` blob (official master's
   own submodule pin already has it).
3. `components/bt/CMakeLists.txt` had no `esp32s31` branch for linking
   `libble_audio.a` at all (official master replaced this whole mechanism
   with a generalized `register_ble_audio_libs()`/`add_subdirectory(esp_ble_audio)`
   approach that just works per-target).

## Still open

- NS4150B amp enable polarity (active-high assumed in `le_audio_codec.cpp`)
  — not visible in the schematic crop used, still unconfirmed. Only testable
  by actually listening to the speaker.
- Whether HID GATT-notify traffic and CIS audio streams coexist cleanly on
  one connection under real load — the device boots and advertises, but no
  BLE central has connected to it yet in testing, so ASE negotiation, LC3
  streaming, and concurrent HID+audio traffic are all still unverified.
- Whether the LC3 encode/decode data path (`le_audio_bap.c`'s
  `on_stream_recv`/`mic_encode_task`) actually produces intelligible audio —
  needs a real LE Audio central to pair and stream.
- General audio/UX quality once actually paired: buffer sizes and the mic
  task's 10ms idle-poll cadence are reasonable first-pass choices, not tuned
  against real ISO/CIS timing.
