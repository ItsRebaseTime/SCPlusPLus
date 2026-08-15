// Native ESP-IDF NimBLE HID-over-GATT DualSense Edge emulation.
//
// Replaces the Arduino-only ESP32-BLE-CompositeHID library (NimBLEHIDDevice)
// used by the S3/Arduino build. Report descriptor bytes, wire structs, and
// the CRC32 algorithm are ported from the sibling project's reference
// implementation (see ds_report.h); the GATT service plumbing below is
// written directly against ESP-IDF's native NimBLE host API since that
// library's convenience wrapper isn't available outside Arduino.
//
// KNOWN GAP (documented in the plan, not a bug): adaptive-trigger effects
// are parsed but discarded — this board build has no servo/solenoid
// actuator hardware (locked-in scope decision). Rumble/haptic-tone output
// is wired through to the Steam Controller via sc_rumble()/sc_haptic_tone().
//
// NOT YET VALIDATED ON HARDWARE: pairing/bonding is currently disabled
// (sm_bonding = 0) for a simpler first pass. Real DualSense Edge BLE HID
// hosts may expect bonding — revisit once this can be tested against a
// real phone/PC/console.
#include "ds_edge_hid.h"
#include "ds_report.h"
#include "sc_input.h"

#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_att.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/bas/ble_svc_bas.h"
#include "services/dis/ble_svc_dis.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_cpu.h"
#include "esp_timer.h"

#include <cstring>
#include <algorithm>

static const char *TAG = "ds_edge_hid";

// ---------------------------------------------------------------------
// UUIDs
// ---------------------------------------------------------------------
static const ble_uuid16_t UUID_HID_SVC       = BLE_UUID16_INIT(0x1812);
static const ble_uuid16_t UUID_HID_INFO      = BLE_UUID16_INIT(0x2A4A);
static const ble_uuid16_t UUID_REPORT_MAP    = BLE_UUID16_INIT(0x2A4B);
static const ble_uuid16_t UUID_HID_CTRL_PT   = BLE_UUID16_INIT(0x2A4C);
static const ble_uuid16_t UUID_REPORT        = BLE_UUID16_INIT(0x2A4D);
static const ble_uuid16_t UUID_PROTOCOL_MODE = BLE_UUID16_INIT(0x2A4E);
static const ble_uuid16_t UUID_REPORT_REF    = BLE_UUID16_INIT(0x2908);

// ---------------------------------------------------------------------
// State
// ---------------------------------------------------------------------
static uint16_t s_conn_handle  = BLE_HS_CONN_HANDLE_NONE;
static bool     s_connected    = false;
static uint16_t s_input_handle = 0;  // for ble_gatts_notify_custom()

static DsInputReport      s_input;
static DsPairingInfoReport s_pairing;
static uint8_t s_minimal_input[DUALSENSE_MINIMAL_INPUT_REPORT_SIZE] = {
    0x80, 0x80, 0x80, 0x80,  // X, Y, Z, Rz (sticks centered)
    0x08,                     // hat = 8 (null)
    0x00, 0x00,               // 14 buttons + padding
    0x00, 0x00                // Rx, Ry (triggers)
};
static uint8_t s_calibration_buf[DUALSENSE_CALIBRATION_REPORT_SIZE];
static uint8_t s_firmware_buf[DUALSENSE_FIRMWARE_INFO_REPORT_SIZE];
static uint8_t s_bt_patch_buf[DUALSENSE_BT_PATCH_REPORT_SIZE];
static uint8_t s_protocol_mode = 0x01;  // Report Protocol

static const uint8_t s_hid_info[4] = { 0x11, 0x01, 0x00, 0x02 };  // bcdHID=0x0111, country=0, flags=NormallyConnectable

// Report Reference descriptor values: [report_id, report_type(1=Input,2=Output,3=Feature)]
static const uint8_t s_ref_input_31[2]   = { DUALSENSE_EDGE_INPUT_REPORT_ID, 0x01 };
static const uint8_t s_ref_input_01[2]   = { DUALSENSE_MINIMAL_INPUT_REPORT_ID, 0x01 };
static const uint8_t s_ref_output_31[2]  = { DUALSENSE_EDGE_OUTPUT_REPORT_ID, 0x02 };
static const uint8_t s_ref_feature_05[2] = { DUALSENSE_CALIBRATION_REPORT_ID, 0x03 };
static const uint8_t s_ref_feature_09[2] = { DUALSENSE_PAIRING_INFO_REPORT_ID, 0x03 };
static const uint8_t s_ref_feature_20[2] = { DUALSENSE_FIRMWARE_INFO_REPORT_ID, 0x03 };
static const uint8_t s_ref_feature_22[2] = { DUALSENSE_BT_PATCH_REPORT_ID, 0x03 };

// ---------------------------------------------------------------------
// CRC32 (same table/algorithm as Linux hid-playstation / DualShock4 BT)
// ---------------------------------------------------------------------
static uint32_t s_crc_table[256];
static bool     s_crc_table_ready = false;

static void generate_crc_table() {
    const uint32_t POLYNOMIAL = 0xEDB88320;
    uint8_t b = 0;
    do {
        uint32_t remainder = b;
        for (unsigned bit = 8; bit > 0; --bit)
            remainder = (remainder & 1) ? (remainder >> 1) ^ POLYNOMIAL : (remainder >> 1);
        s_crc_table[b] = remainder;
    } while (++b != 0);
    s_crc_table_ready = true;
}

uint32_t ds_crc32_le(uint32_t crc, const uint8_t* buf, size_t len) {
    if (!s_crc_table_ready) generate_crc_table();
    for (size_t i = 0; i < len; i++)
        crc = s_crc_table[(crc ^ buf[i]) & 0xff] ^ (crc >> 8);
    return crc;
}

static void build_feature_report_with_crc(uint8_t reportId, const uint8_t* payload, size_t payloadSize,
                                           uint8_t* outBuffer, size_t outSize) {
    if (outSize < payloadSize + 4) return;
    memcpy(outBuffer, payload, payloadSize);
    uint8_t hdr[] = { PS_FEATURE_CRC32_SEED, reportId };
    uint32_t crc = ds_crc32_le(0xFFFFFFFF, hdr, sizeof(hdr));
    crc = ~ds_crc32_le(crc, outBuffer, payloadSize);
    outBuffer[payloadSize + 0] = (uint8_t)(crc & 0xFF);
    outBuffer[payloadSize + 1] = (uint8_t)((crc >> 8) & 0xFF);
    outBuffer[payloadSize + 2] = (uint8_t)((crc >> 16) & 0xFF);
    outBuffer[payloadSize + 3] = (uint8_t)((crc >> 24) & 0xFF);
}

static void rebuild_calibration() {
    build_feature_report_with_crc(DUALSENSE_CALIBRATION_REPORT_ID,
        DualsenseEdge_StockCalibration, DUALSENSE_CALIBRATION_REPORT_SIZE - 5,
        s_calibration_buf, DUALSENSE_CALIBRATION_REPORT_SIZE);
}

static void rebuild_firmware() {
    build_feature_report_with_crc(DUALSENSE_FIRMWARE_INFO_REPORT_ID,
        DualsenseEdge_FirmwareInfo, DUALSENSE_FIRMWARE_INFO_REPORT_SIZE - 5,
        s_firmware_buf, DUALSENSE_FIRMWARE_INFO_REPORT_SIZE);
}

static void rebuild_bt_patch() {
    uint8_t payload[DUALSENSE_BT_PATCH_REPORT_SIZE - 4] = { 0 };
    build_feature_report_with_crc(DUALSENSE_BT_PATCH_REPORT_ID, payload, sizeof(payload),
        s_bt_patch_buf, DUALSENSE_BT_PATCH_REPORT_SIZE);
}

static void rebuild_pairing() {
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_BT);
    memcpy(s_pairing.mac_address, mac, 6);
    memcpy(s_pairing.common, DualsenseEdge_PairInfo_common, 9);
    uint8_t hdr[] = { PS_FEATURE_CRC32_SEED, DUALSENSE_PAIRING_INFO_REPORT_ID };
    uint32_t crc = ds_crc32_le(0xFFFFFFFF, hdr, sizeof(hdr));
    crc = ~ds_crc32_le(crc, (uint8_t*)&s_pairing, offsetof(DsPairingInfoReport, crc32));
    s_pairing.crc32 = crc;
}

static void reset_input_report() {
    s_input = DsInputReport{};
    // Controller at rest: ~1g on the axis facing down (matches reference impl).
    s_input.accel_y = -DUALSENSE_ACC_RES_PER_G;
}

// ---------------------------------------------------------------------
// Button/hat helpers (20-bit logical button space; wire position has a
// 4-bit hat-switch offset in buttons[0]'s low nibble — see reference impl).
// ---------------------------------------------------------------------
static inline void buttons_set(uint8_t b[3], uint32_t mask) {
    uint32_t shifted = mask << 4;
    b[0] |= (uint8_t)(shifted & 0xFF);
    b[1] |= (uint8_t)((shifted >> 8) & 0xFF);
    b[2] |= (uint8_t)((shifted >> 16) & 0xFF);
}
static inline void buttons_clear(uint8_t b[3], uint32_t mask) {
    uint32_t shifted = mask << 4;
    b[0] &= (uint8_t)~(shifted & 0xFF);
    b[1] &= (uint8_t)~((shifted >> 8) & 0xFF);
    b[2] &= (uint8_t)~((shifted >> 16) & 0xFF);
}
static inline void set_button(uint32_t mask, bool pressed) {
    if (pressed) buttons_set(s_input.buttons, mask);
    else         buttons_clear(s_input.buttons, mask);
}
static inline void hat_set(uint8_t b[3], uint8_t dir) {
    b[0] = (uint8_t)((b[0] & 0xF0) | (dir & 0x0F));
}

static uint8_t ds_dpad_hat(uint32_t btns) {
    const bool u = (btns & SC_BTN_DPAD_UP)    != 0;
    const bool d = (btns & SC_BTN_DPAD_DOWN)  != 0;
    const bool l = (btns & SC_BTN_DPAD_LEFT)  != 0;
    const bool r = (btns & SC_BTN_DPAD_RIGHT) != 0;
    if (u && r) return DUALSENSE_BUTTON_DPAD_NORTHEAST;
    if (u && l) return DUALSENSE_BUTTON_DPAD_NORTHWEST;
    if (d && r) return DUALSENSE_BUTTON_DPAD_SOUTHEAST;
    if (d && l) return DUALSENSE_BUTTON_DPAD_SOUTHWEST;
    if (u) return DUALSENSE_BUTTON_DPAD_NORTH;
    if (d) return DUALSENSE_BUTTON_DPAD_SOUTH;
    if (l) return DUALSENSE_BUTTON_DPAD_WEST;
    if (r) return DUALSENSE_BUTTON_DPAD_EAST;
    return DUALSENSE_BUTTON_DPAD_NONE;
}

// ---------------------------------------------------------------------
// Touchpad 2-slot tracking (mirrors touchpadStartTouch/Update/StopTouch)
// ---------------------------------------------------------------------
static uint8_t s_touch_id[2]     = { 0, 0 };
static bool    s_touch_active[2] = { false, false };
static uint8_t s_next_touch_id   = 1;

static int touch_start(uint16_t x, uint16_t y) {
    int slot = -1;
    for (int i = 0; i < 2; i++) if (!s_touch_active[i]) { slot = i; break; }
    if (slot < 0) return -1;
    s_touch_active[slot] = true;
    s_touch_id[slot] = s_next_touch_id;
    s_next_touch_id = (s_next_touch_id + 1) & 0x7F;
    const uint8_t contact = s_touch_id[slot] & 0x7F;
    if (slot == 0) { s_input.touchpoint_0_contact = contact; s_input.touchpoint_0_x = x & 0xFFF; s_input.touchpoint_0_y = y & 0xFFF; }
    else           { s_input.touchpoint_1_contact = contact; s_input.touchpoint_1_x = x & 0xFFF; s_input.touchpoint_1_y = y & 0xFFF; }
    return slot;
}
static void touch_update(int slot, uint16_t x, uint16_t y) {
    if (slot < 0 || slot > 1 || !s_touch_active[slot]) return;
    if (slot == 0) { s_input.touchpoint_0_x = x & 0xFFF; s_input.touchpoint_0_y = y & 0xFFF; }
    else           { s_input.touchpoint_1_x = x & 0xFFF; s_input.touchpoint_1_y = y & 0xFFF; }
}
static void touch_stop(int slot) {
    if (slot < 0 || slot > 1 || !s_touch_active[slot]) return;
    s_touch_active[slot] = false;
    if (slot == 0) s_input.touchpoint_0_contact = 0x80;
    else           s_input.touchpoint_1_contact = 0x80;
}

static bool    s_lpad_active = false, s_rpad_active = false;
static int8_t  s_lpad_slot = -1, s_rpad_slot = -1;

static void update_touchpad(const SCReport& sc) {
    const uint32_t btns = sc_buttons(sc);
    const bool ltouch = (btns & SC_BTN_LPAD_TOUCH) != 0;
    const bool rtouch = (btns & SC_BTN_RPAD_TOUCH) != 0;

    // SC absolute pos (-32768..32767) -> DS touchpad: X 0-1919, Y 0-1079.
    // Left SC pad -> left half (X 0-959), right SC pad -> right half (X 960-1919).
    // SC Y+ = up; DS Y 0 = top (Y+ = down) -- invert Y.
    const uint16_t lx = (uint16_t)((int32_t)(sc.lpad_x + 32768) * 959  / 65535);
    const uint16_t ly = 1079 - (uint16_t)((int32_t)(sc.lpad_y + 32768) * 1079 / 65535);
    const uint16_t rx = 960 + (uint16_t)((int32_t)(sc.rpad_x + 32768) * 959  / 65535);
    const uint16_t ry = 1079 - (uint16_t)((int32_t)(sc.rpad_y + 32768) * 1079 / 65535);

    if (ltouch && !s_lpad_active) {
        s_lpad_slot = (int8_t)touch_start(lx, ly);
        s_lpad_active = (s_lpad_slot >= 0);
    } else if (ltouch && s_lpad_active) {
        touch_update(s_lpad_slot, lx, ly);
    } else if (!ltouch && s_lpad_active) {
        touch_stop(s_lpad_slot);
        s_lpad_active = false; s_lpad_slot = -1;
    }

    if (rtouch && !s_rpad_active) {
        s_rpad_slot = (int8_t)touch_start(rx, ry);
        s_rpad_active = (s_rpad_slot >= 0);
    } else if (rtouch && s_rpad_active) {
        touch_update(s_rpad_slot, rx, ry);
    } else if (!rtouch && s_rpad_active) {
        touch_stop(s_rpad_slot);
        s_rpad_active = false; s_rpad_slot = -1;
    }
}

// ---------------------------------------------------------------------
// Output report -> Steam Controller rumble/haptic forwarding
// ---------------------------------------------------------------------
static uint32_t s_rumble_ts = 0;
static uint8_t  s_last_strong = 0xFF, s_last_weak = 0xFF;
static bool     s_last_haptics_select = false;

static void on_output_report(const DsOutputReport& data) {
    const bool haptics_select = (data.valid_flag0 & DS_OUT_FLAG0_HAPTICS_SELECT) != 0;
    const bool has_rumble = data.hasRumble() || haptics_select;
    if (!has_rumble) return;

    s_rumble_ts = (uint32_t)(esp_timer_get_time() / 1000);
    const uint8_t strong = data.strongMotor();
    const uint8_t weak   = data.weakMotor();
    if (strong == s_last_strong && weak == s_last_weak && haptics_select == s_last_haptics_select) return;
    s_last_strong = strong; s_last_weak = weak; s_last_haptics_select = haptics_select;

    if (haptics_select) {
        sc_haptic_tone(weak, strong);
        sc_rumble(0, 0);
    } else {
        sc_rumble(strong, weak);
        sc_haptic_tone(0, 0);
    }
    // Adaptive-trigger effects (hasLeftTriggerEffect()/hasRightTriggerEffect())
    // are intentionally not acted on further — no actuator hardware on this
    // board build (locked-in scope decision, see plan).
}

static void rumble_timeout_check() {
    const uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (now - s_rumble_ts > 500) {
        sc_rumble(0, 0);
        sc_haptic_tone(0, 0);
    }
}

// ---------------------------------------------------------------------
// GATT access callbacks
// ---------------------------------------------------------------------
static int access_report_ref(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *arg) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_DSC) return BLE_ATT_ERR_UNLIKELY;
    return os_mbuf_append(ctxt->om, arg, 2) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int access_hid_info(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    return os_mbuf_append(ctxt->om, s_hid_info, sizeof(s_hid_info)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int access_report_map(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    return os_mbuf_append(ctxt->om, DualsenseEdge_HIDDescriptor, sizeof(DualsenseEdge_HIDDescriptor)) == 0
        ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int access_hid_ctrl_pt(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    // Suspend/exit-suspend notifications — nothing to do, just accept the write.
    return (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) ? 0 : BLE_ATT_ERR_UNLIKELY;
}

static int access_protocol_mode(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR)
        return os_mbuf_append(ctxt->om, &s_protocol_mode, 1) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        if (len >= 1) ble_hs_mbuf_to_flat(ctxt->om, &s_protocol_mode, 1, nullptr);
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int access_input(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    return os_mbuf_append(ctxt->om, &s_input, sizeof(s_input)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int access_minimal_input(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op != BLE_GATT_ACCESS_OP_READ_CHR) return BLE_ATT_ERR_UNLIKELY;
    return os_mbuf_append(ctxt->om, s_minimal_input, sizeof(s_minimal_input)) == 0
        ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
}

static int access_output(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        // Steam probes readability of the output characteristic on connect;
        // return a valid but empty default report, matching the reference impl.
        static uint8_t def[DS_OUTPUT_REPORT_BT_SIZE] = { 0 };
        def[0] = 0x31; def[2] = 0x10;
        return os_mbuf_append(ctxt->om, def, sizeof(def)) == 0 ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    if (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) {
        uint16_t len = OS_MBUF_PKTLEN(ctxt->om);
        static uint8_t buf[DS_OUTPUT_REPORT_BT_SIZE];
        if (len > sizeof(buf)) len = sizeof(buf);
        if (len < 47) return 0;  // too small to be a real output report; ignore
        if (ble_hs_mbuf_to_flat(ctxt->om, buf, len, nullptr) != 0) return BLE_ATT_ERR_UNLIKELY;
        DsOutputReport rpt;
        if (rpt.load(buf, len)) on_output_report(rpt);
        return 0;
    }
    return BLE_ATT_ERR_UNLIKELY;
}

static int access_calibration(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        rebuild_calibration();
        return os_mbuf_append(ctxt->om, s_calibration_buf, sizeof(s_calibration_buf)) == 0
            ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) ? 0 : BLE_ATT_ERR_UNLIKELY;
}

static int access_pairing(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        rebuild_pairing();
        return os_mbuf_append(ctxt->om, &s_pairing, sizeof(s_pairing)) == 0
            ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) ? 0 : BLE_ATT_ERR_UNLIKELY;
}

static int access_firmware(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        rebuild_firmware();
        return os_mbuf_append(ctxt->om, s_firmware_buf, sizeof(s_firmware_buf)) == 0
            ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) ? 0 : BLE_ATT_ERR_UNLIKELY;
}

static int access_bt_patch(uint16_t, uint16_t, struct ble_gatt_access_ctxt *ctxt, void *) {
    if (ctxt->op == BLE_GATT_ACCESS_OP_READ_CHR) {
        rebuild_bt_patch();
        return os_mbuf_append(ctxt->om, s_bt_patch_buf, sizeof(s_bt_patch_buf)) == 0
            ? 0 : BLE_ATT_ERR_INSUFFICIENT_RES;
    }
    return (ctxt->op == BLE_GATT_ACCESS_OP_WRITE_CHR) ? 0 : BLE_ATT_ERR_UNLIKELY;
}

// ---------------------------------------------------------------------
// GATT table
// ---------------------------------------------------------------------
static struct ble_gatt_dsc_def s_dsc_input[] = {
    { .uuid = &UUID_REPORT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = access_report_ref, .arg = (void*)s_ref_input_31 },
    { 0 }
};
static struct ble_gatt_dsc_def s_dsc_minimal[] = {
    { .uuid = &UUID_REPORT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = access_report_ref, .arg = (void*)s_ref_input_01 },
    { 0 }
};
static struct ble_gatt_dsc_def s_dsc_output[] = {
    { .uuid = &UUID_REPORT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = access_report_ref, .arg = (void*)s_ref_output_31 },
    { 0 }
};
static struct ble_gatt_dsc_def s_dsc_calibration[] = {
    { .uuid = &UUID_REPORT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = access_report_ref, .arg = (void*)s_ref_feature_05 },
    { 0 }
};
static struct ble_gatt_dsc_def s_dsc_pairing[] = {
    { .uuid = &UUID_REPORT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = access_report_ref, .arg = (void*)s_ref_feature_09 },
    { 0 }
};
static struct ble_gatt_dsc_def s_dsc_firmware[] = {
    { .uuid = &UUID_REPORT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = access_report_ref, .arg = (void*)s_ref_feature_20 },
    { 0 }
};
static struct ble_gatt_dsc_def s_dsc_bt_patch[] = {
    { .uuid = &UUID_REPORT_REF.u, .att_flags = BLE_ATT_F_READ, .access_cb = access_report_ref, .arg = (void*)s_ref_feature_22 },
    { 0 }
};

static const struct ble_gatt_chr_def s_hid_chrs[] = {
    { .uuid = &UUID_HID_INFO.u,      .access_cb = access_hid_info,      .flags = BLE_GATT_CHR_F_READ },
    { .uuid = &UUID_REPORT_MAP.u,    .access_cb = access_report_map,    .flags = BLE_GATT_CHR_F_READ },
    { .uuid = &UUID_HID_CTRL_PT.u,   .access_cb = access_hid_ctrl_pt,   .flags = BLE_GATT_CHR_F_WRITE_NO_RSP },
    { .uuid = &UUID_PROTOCOL_MODE.u, .access_cb = access_protocol_mode, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE_NO_RSP },
    { .uuid = &UUID_REPORT.u, .access_cb = access_input,        .descriptors = s_dsc_input,       .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY, .val_handle = &s_input_handle },
    { .uuid = &UUID_REPORT.u, .access_cb = access_minimal_input,.descriptors = s_dsc_minimal,     .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_NOTIFY },
    { .uuid = &UUID_REPORT.u, .access_cb = access_output,       .descriptors = s_dsc_output,      .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE | BLE_GATT_CHR_F_WRITE_NO_RSP },
    { .uuid = &UUID_REPORT.u, .access_cb = access_calibration,  .descriptors = s_dsc_calibration, .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
    { .uuid = &UUID_REPORT.u, .access_cb = access_pairing,      .descriptors = s_dsc_pairing,     .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
    { .uuid = &UUID_REPORT.u, .access_cb = access_firmware,     .descriptors = s_dsc_firmware,    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
    { .uuid = &UUID_REPORT.u, .access_cb = access_bt_patch,     .descriptors = s_dsc_bt_patch,    .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE },
    { 0 }
};

static const struct ble_gatt_svc_def s_gatt_services[] = {
    { .type = BLE_GATT_SVC_TYPE_PRIMARY, .uuid = &UUID_HID_SVC.u, .characteristics = s_hid_chrs },
    { 0 }
};

// ---------------------------------------------------------------------
// Public API
//
// NimBLE host/GAP/GATT/advertising lifecycle is NOT owned here — see
// le_audio_bap.cpp, which registers this service's table alongside
// PACS/ASCS on one shared GATT server (locked-in "single BLE identity"
// requirement) and drives the shared GAP event handler.
// ---------------------------------------------------------------------
const struct ble_gatt_svc_def* ds_edge_hid_get_services() {
    reset_input_report();
    rebuild_calibration();
    rebuild_firmware();
    rebuild_bt_patch();
    rebuild_pairing();
    return s_gatt_services;
}

void ds_edge_hid_on_connect(uint16_t conn_handle) {
    ESP_LOGI(TAG, "connected, handle=%d", conn_handle);
    s_conn_handle = conn_handle;
    s_connected = true;
}

void ds_edge_hid_on_disconnect() {
    ESP_LOGI(TAG, "disconnected");
    s_connected = false;
    s_conn_handle = BLE_HS_CONN_HANDLE_NONE;
}

bool ds_connected() { return s_connected; }

static void notify_input() {
    if (!s_connected || s_input_handle == 0) return;
    uint8_t bthdr[] = { PS_INPUT_CRC32_SEED, DUALSENSE_EDGE_INPUT_REPORT_ID };
    uint32_t crc = ds_crc32_le(0xFFFFFFFF, bthdr, sizeof(bthdr));
    crc = ~ds_crc32_le(crc, (uint8_t*)&s_input, sizeof(s_input) - 4);
    s_input.crc32 = crc;

    struct os_mbuf *om = ble_hs_mbuf_from_flat(&s_input, sizeof(s_input));
    if (!om) return;
    int rc = ble_gatts_notify_custom(s_conn_handle, s_input_handle, om);
    if (rc != 0) ESP_LOGD(TAG, "notify_custom rc=%d", rc);
}

void ds_send(const SCReport& sc) {
    if (!s_connected) return;
    rumble_timeout_check();

    const uint32_t btns = sc_buttons(sc);

    set_button(DUALSENSE_BUTTON_A, btns & SC_BTN_A);
    set_button(DUALSENSE_BUTTON_B, btns & SC_BTN_B);
    set_button(DUALSENSE_BUTTON_X, btns & SC_BTN_X);
    set_button(DUALSENSE_BUTTON_Y, btns & SC_BTN_Y);
    set_button(DUALSENSE_BUTTON_LB, btns & SC_BTN_L1);
    set_button(DUALSENSE_BUTTON_RB, btns & SC_BTN_R1);
    set_button(DUALSENSE_BUTTON_L4, btns & SC_BTN_L4);
    set_button(DUALSENSE_BUTTON_L5, btns & SC_BTN_L5);
    set_button(DUALSENSE_BUTTON_R4, btns & SC_BTN_R4);
    set_button(DUALSENSE_BUTTON_R5, btns & SC_BTN_R5);
    set_button(DUALSENSE_BUTTON_LS, btns & SC_BTN_LSTICK_CLICK);
    set_button(DUALSENSE_BUTTON_RS, btns & SC_BTN_RSTICK_CLICK);
    set_button(DUALSENSE_BUTTON_START, btns & SC_BTN_MENU);
    set_button(DUALSENSE_BUTTON_SHARE, btns & SC_BTN_QUICK_ACCESS);
    set_button(DUALSENSE_BUTTON_SELECT, btns & SC_BTN_VIEW);
    set_button(DUALSENSE_BUTTON_MODE, btns & SC_BTN_STEAM);
    set_button(DUALSENSE_BUTTON_TOUCHPAD, (btns & SC_BTN_LPAD_CLICK) || (btns & SC_BTN_RPAD_CLICK));

    hat_set(s_input.buttons, ds_dpad_hat(btns));

    // Triggers: SC 0-32767 -> DS uint8_t.
    s_input.z  = (uint8_t)(sc.ltrigger >> 7);
    s_input.rz = (uint8_t)(sc.rtrigger >> 7);

    // Sticks: SC +-32767 -> DS +-127; SC Y+ = up, DS Y+ = down -- negate Y.
    {
        const int lx = std::clamp((int)(sc.left_stick_x  >> 8), -127, 127);
        const int ly = std::clamp(-(int)(sc.left_stick_y  >> 8), -127, 127);
        const int rx = std::clamp((int)(sc.right_stick_x >> 8), -127, 127);
        const int ry = std::clamp(-(int)(sc.right_stick_y >> 8), -127, 127);
        s_input.x  = (uint8_t)(lx + DUALSENSE_AXIS_CENTER_OFFSET);
        s_input.y  = (uint8_t)(ly + DUALSENSE_AXIS_CENTER_OFFSET);
        s_input.rx = (uint8_t)(rx + DUALSENSE_AXIS_CENTER_OFFSET);
        s_input.ry = (uint8_t)(ry + DUALSENSE_AXIS_CENTER_OFFSET);
    }

    update_touchpad(sc);

    s_input.accel_x = sc.acc_x; s_input.accel_y = sc.acc_y; s_input.accel_z = sc.acc_z;
    s_input.gyro_x  = sc.gyro_x; s_input.gyro_y = sc.gyro_y; s_input.gyro_z  = sc.gyro_z;

    s_input.timestamp = (uint32_t)(esp_cpu_get_cycle_count() / 1500);
    s_input.seq = (s_input.seq < 254) ? (uint8_t)(s_input.seq + 1) : 0;

    notify_input();
}

void ds_send_idle() {
    if (!s_connected) return;
    rumble_timeout_check();
    s_input.timestamp = (uint32_t)(esp_cpu_get_cycle_count() / 1500);
    s_input.seq = (s_input.seq < 254) ? (uint8_t)(s_input.seq + 1) : 0;
    notify_input();
}

void ds_set_battery(uint8_t level_pct, bool charging) {
    level_pct = std::min<uint8_t>(level_pct, 100);
    const uint8_t internal = level_pct / 10;
    s_input.status = (uint8_t)((s_input.status & 0xF0) | internal);
    s_input.status = (uint8_t)((s_input.status & 0x0F) | ((charging ? 1 : 0) << 4));
    ble_svc_bas_battery_level_set(level_pct);
}
