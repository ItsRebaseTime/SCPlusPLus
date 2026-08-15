idf_component_register(
    SRCS "main.cpp" "sc_input.cpp" "ds_edge_hid.cpp" "le_audio_codec.cpp" "le_audio_bap.c"
    INCLUDE_DIRS "."
    REQUIRES usb esp_timer bt driver esp_driver_i2s esp_driver_i2c
)

# ESP-IDF's NimBLE porting-layer headers (nimble_npl_os.h and friends) use
# C-style implicit int<->enum conversions that are hard errors under strict
# C++. Arduino's ESP32 core builds with -fpermissive globally for exactly
# this reason (it's how NimBLE-Arduino gets away with being C++); do the
# same narrowly for the one file here that touches the NimBLE host headers
# directly, rather than relaxing it project-wide. le_audio_bap.c doesn't need
# this — it's plain C (see its file header for why), so the int<->enum
# conversions that trip up C++ are unproblematic there.
set_source_files_properties(ds_edge_hid.cpp PROPERTIES COMPILE_OPTIONS "-fpermissive")

# Uncomment for the Phase 2 hardware bring-up build (isolated ES8311/NS4150B
# mic-to-speaker loopback test, no USB host / BLE). See le_audio_codec.h and
# firmware/s31/BUILD.md. Verified to compile+link as its own standalone
# ~245KB image (vs. ~812KB for the full build) since the linker strips the
# then-unused USB host + NimBLE code.
# target_compile_definitions(${COMPONENT_LIB} PRIVATE AUDIO_LOOPBACK_TEST)
