#pragma once
#include <cstdint>
#include "sc_report.h"

struct ble_gatt_svc_def;

// Native ESP-IDF NimBLE HID-over-GATT implementation of a Sony DualSense
// Edge, replacing the Arduino-only ESP32-BLE-CompositeHID library used by
// the S3/Arduino build's dualsense_output.cpp.
//
// This module does NOT own the NimBLE host/advertising lifecycle — that's
// owned by le_audio_bap.cpp so the HID service and the LE Audio PACS/ASCS
// services can share one GATT server/connection (locked-in "single BLE
// identity" requirement). See le_audio_bap.cpp for the full init sequence;
// this module just supplies its service table and reacts to shared
// connection-state changes.

// le_audio_bap.c (the NimBLE host/GATT/advertising owner) is compiled as
// plain C — see its file header for why — so the three functions it calls
// here need C linkage. The rest of this header (ds_send etc., used only by
// main.cpp) doesn't strictly need it, but wrapping the whole block is
// simpler and extern "C" doesn't restrict the C++-only parameter types below
// (e.g. ds_send's SCReport reference).
#ifdef __cplusplus
extern "C" {
#endif

// Returns the HID GATT service table (NULL-terminated array) to be passed
// to ble_gatts_add_svcs() alongside PACS/ASCS, before ble_gatts_start().
const struct ble_gatt_svc_def* ds_edge_hid_get_services();

// Called by le_audio_bap.c's shared GAP event handler.
void ds_edge_hid_on_connect(uint16_t conn_handle);
void ds_edge_hid_on_disconnect();
void ds_edge_hid_on_subscribe(uint16_t attr_handle, bool notify_enabled);

bool ds_connected();
void ds_send(const SCReport& sc);
void ds_send_idle();
void ds_set_battery(uint8_t level_pct, bool charging);

#ifdef __cplusplus
}
#endif
