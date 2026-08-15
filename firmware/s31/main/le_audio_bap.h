#pragma once

// Bidirectional LE Audio (BAP Unicast Server: Sink ASE for speaker playback,
// Source ASE for mic capture) sharing one NimBLE GATT server/connection with
// the DualSense Edge HID service (ds_edge_hid.cpp) — the locked-in "single
// BLE identity" requirement.
//
// This file owns the ENTIRE BLE lifecycle (BT controller, NimBLE host,
// GAP/GATT, advertising, ble_gatts_start()) because espressif/esp_bt_audio's
// high-level esp_bt_audio_init() is monolithic with no hook for registering
// an additional custom GATT service into the same server — confirmed by
// reading its actual source. Instead this calls the lower, documented
// public API directly (espressif/esp-idf's components/bt/esp_ble_audio
// api/audio/include headers: esp_ble_audio_pacs_api.h, esp_ble_audio_bap_api.h)
// which DOES expose clean, standalone PACS/ASCS registration functions
// alongside plain ble_gatts_add_svcs()/ble_gatts_start() for everything else.
//
// UNVERIFIED ON HARDWARE: this is the least-trodden integration path in the
// whole project — the ASE state machine callbacks below are structurally
// complete but their correctness (QoS negotiation, actual CIS/LC3 data flow
// timing) has only been confirmed to compile, not to work end-to-end. See
// the plan's Phase 3 verification section.
//
// le_audio_bap.c (not .cpp — see that file's header comment) is compiled as
// plain C, unlike the rest of this project's app code, so main.cpp (C++)
// needs this declared with C linkage to call it correctly.
#ifdef __cplusplus
extern "C" {
#endif

void le_audio_bap_begin(void);

#ifdef __cplusplus
}
#endif
