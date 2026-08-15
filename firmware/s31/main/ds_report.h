#pragma once
// DualSense Edge BLE HID wire format: report IDs, sizes, the raw HID report
// descriptor, and the input/output report byte layouts.
//
// Ported from the sibling ESP32-BLE-CompositeHID project
// (branch dualsense_input_output_field_info, DualsenseDescriptors.h /
// DualsenseGamepadDevice.h) — knowledge only, not compiled code. That
// project's NimBLE-Arduino GATT plumbing is replaced here with ESP-IDF's
// native NimBLE host API (see ds_edge_hid.cpp); the wire-format bytes,
// struct layouts, and CRC32 algorithm below are unchanged since they're
// dictated by the Linux hid-playstation.c-compatible protocol, not by the
// BLE stack.
#include <cstdint>
#include <cstddef>

#define DUALSENSE_VENDOR_ID 0x054C
#define DUALSENSE_EDGE_PRODUCT_ID 0x0DF2
#define DUALSENSE_EDGE_BCD_DEVICE_ID 0x0408

#define DUALSENSE_EDGE_INPUT_REPORT_ID 0x31
#define DUALSENSE_EDGE_OUTPUT_REPORT_ID 0x31
#define DUALSENSE_MINIMAL_INPUT_REPORT_ID 0x01
#define DUALSENSE_MINIMAL_INPUT_REPORT_SIZE 9

#define DS_OUTPUT_REPORT_BT_SIZE 78
#define PS_INPUT_CRC32_SEED 0xA1
#define PS_OUTPUT_CRC32_SEED 0xA2
#define PS_FEATURE_CRC32_SEED 0xA3

#define DUALSENSE_CALIBRATION_REPORT_ID 0x05
#define DUALSENSE_CALIBRATION_REPORT_SIZE 41
#define DUALSENSE_PAIRING_INFO_REPORT_ID 0x09
#define DUALSENSE_PAIRING_INFO_REPORT_SIZE 20
#define DUALSENSE_FIRMWARE_INFO_REPORT_ID 0x20
#define DUALSENSE_FIRMWARE_INFO_REPORT_SIZE 64
#define DUALSENSE_BT_PATCH_REPORT_ID 0x22
#define DUALSENSE_BT_PATCH_REPORT_SIZE 63

// Button bitmasks (20-bit logical button space; wire position has a 4-bit
// hat-switch offset — see buttons_set()/buttons_test() in ds_edge_hid.cpp).
#define DUALSENSE_BUTTON_Y 0x08
#define DUALSENSE_BUTTON_B 0x04
#define DUALSENSE_BUTTON_A 0x02
#define DUALSENSE_BUTTON_X 0x01
#define DUALSENSE_BUTTON_LB 0x10
#define DUALSENSE_BUTTON_RB 0x20
#define DUALSENSE_BUTTON_LT 0x40
#define DUALSENSE_BUTTON_RT 0x80
#define DUALSENSE_BUTTON_SELECT 0x100
#define DUALSENSE_BUTTON_START 0x200
#define DUALSENSE_BUTTON_LS 0x400
#define DUALSENSE_BUTTON_RS 0x800
#define DUALSENSE_BUTTON_MODE 0x1000
#define DUALSENSE_BUTTON_TOUCHPAD 0x2000
#define DUALSENSE_BUTTON_SHARE 0x4000
#define DUALSENSE_BUTTON_MUTE 0x8000
#define DUALSENSE_BUTTON_L4 0x10000
#define DUALSENSE_BUTTON_R4 0x20000
#define DUALSENSE_BUTTON_L5 0x40000
#define DUALSENSE_BUTTON_R5 0x80000

#define DUALSENSE_BUTTON_DPAD_NONE 0x08
#define DUALSENSE_BUTTON_DPAD_NORTH 0x00
#define DUALSENSE_BUTTON_DPAD_NORTHEAST 0x01
#define DUALSENSE_BUTTON_DPAD_EAST 0x02
#define DUALSENSE_BUTTON_DPAD_SOUTHEAST 0x03
#define DUALSENSE_BUTTON_DPAD_SOUTH 0x04
#define DUALSENSE_BUTTON_DPAD_SOUTHWEST 0x05
#define DUALSENSE_BUTTON_DPAD_WEST 0x06
#define DUALSENSE_BUTTON_DPAD_NORTHWEST 0x07

#define DUALSENSE_AXIS_CENTER_OFFSET 0x80
#define DUALSENSE_ACC_RES_PER_G 8192

// Output-report valid-flag bits (host sets a bit to say "apply this field").
#define DS_OUT_FLAG0_COMPATIBLE_VIBRATION    (1 << 0)
#define DS_OUT_FLAG0_HAPTICS_SELECT          (1 << 1)
#define DS_OUT_FLAG0_RIGHT_TRIGGER_EFFECT    (1 << 2)
#define DS_OUT_FLAG0_LEFT_TRIGGER_EFFECT     (1 << 3)
#define DS_OUT_FLAG2_COMPATIBLE_VIBRATION2   (1 << 2)

// Raw HID report descriptor (428 bytes). Byte-for-byte from the reference
// implementation — this defines report IDs 0x01 (minimal Generic-Desktop
// gamepad, required so Windows HIDClass activates the input pipe), 0x31
// (vendor-defined input+output blob matching struct dualsense_input_report
// in Linux hid-playstation.c), and feature reports 0x05/0x08/0x09/0x20/0x22
// plus various others the real DualSense Edge exposes (adaptive-trigger,
// audio, LED feature IDs) that this implementation doesn't populate but
// must still declare so the descriptor's report-count bytes stay accurate.
static const uint8_t DualsenseEdge_HIDDescriptor[] = {
    0x05, 0x01, 0x09, 0x05, 0xA1, 0x01,
    0x85, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x35,
    0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x04, 0x81, 0x02,
    0x09, 0x39, 0x15, 0x00, 0x25, 0x07, 0x35, 0x00, 0x46, 0x3B, 0x01,
    0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x65, 0x00,
    0x05, 0x09, 0x19, 0x01, 0x29, 0x0E, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x0E, 0x81, 0x02, 0x75, 0x06, 0x95, 0x01, 0x81, 0x01,
    0x05, 0x01, 0x09, 0x33, 0x09, 0x34, 0x15, 0x00, 0x26, 0xFF, 0x00,
    0x75, 0x08, 0x95, 0x02, 0x81, 0x02,
    0x06, 0x00, 0xFF, 0x15, 0x00, 0x26, 0xFF, 0x00, 0x75, 0x08, 0x95, 0x4D,
    0x85, 0x31, 0x09, 0x31, 0x91, 0x02, 0x09, 0x3B, 0x81, 0x02,
    0x85, 0x32, 0x09, 0x32, 0x95, 0x8D, 0x91, 0x02,
    0x85, 0x33, 0x09, 0x33, 0x95, 0xCD, 0x91, 0x02,
    0x85, 0x34, 0x09, 0x34, 0x96, 0x0D, 0x01, 0x91, 0x02,
    0x85, 0x35, 0x09, 0x35, 0x96, 0x4D, 0x01, 0x91, 0x02,
    0x85, 0x36, 0x09, 0x36, 0x96, 0x8D, 0x01, 0x91, 0x02,
    0x85, 0x37, 0x09, 0x37, 0x96, 0xCD, 0x01, 0x91, 0x02,
    0x85, 0x38, 0x09, 0x38, 0x96, 0x0D, 0x02, 0x91, 0x02,
    0x85, 0x39, 0x09, 0x39, 0x96, 0x22, 0x02, 0x91, 0x02,
    0x06, 0x80, 0xFF,
    0x85, 0x05, 0x09, 0x33, 0x95, 0x29, 0xB1, 0x02,
    0x85, 0x08, 0x09, 0x34, 0x95, 0x2F, 0xB1, 0x02,
    0x85, 0x09, 0x09, 0x24, 0x95, 0x14, 0xB1, 0x02,
    0x85, 0x20, 0x09, 0x26, 0x95, 0x40, 0xB1, 0x02,
    0x85, 0x22, 0x09, 0x40, 0x95, 0x3F, 0xB1, 0x02,
    0x85, 0x80, 0x09, 0x28, 0x95, 0x3F, 0xB1, 0x02,
    0x85, 0x81, 0x09, 0x29, 0x95, 0x3F, 0xB1, 0x02,
    0x85, 0x82, 0x09, 0x2A, 0x95, 0x09, 0xB1, 0x02,
    0x85, 0x83, 0x09, 0x2B, 0x95, 0x3F, 0xB1, 0x02,
    0x85, 0xF1, 0x09, 0x31, 0x95, 0x3F, 0xB1, 0x02,
    0x85, 0xF2, 0x09, 0x32, 0x95, 0x34, 0xB1, 0x02,
    0x85, 0xF0, 0x09, 0x30, 0x95, 0x3F, 0xB1, 0x02,
    0x85, 0x60, 0x09, 0x41, 0x95, 0x3F, 0xB1, 0x02,
    0x85, 0x61, 0x09, 0x42, 0xB1, 0x02,
    0x85, 0x62, 0x09, 0x43, 0xB1, 0x02,
    0x85, 0x63, 0x09, 0x44, 0xB1, 0x02,
    0x85, 0x64, 0x09, 0x45, 0xB1, 0x02,
    0x85, 0x65, 0x09, 0x46, 0xB1, 0x02,
    0x85, 0x68, 0x09, 0x47, 0xB1, 0x02,
    0x85, 0x70, 0x09, 0x48, 0xB1, 0x02,
    0x85, 0x71, 0x09, 0x49, 0xB1, 0x02,
    0x85, 0x72, 0x09, 0x4A, 0xB1, 0x02,
    0x85, 0x73, 0x09, 0x4B, 0xB1, 0x02,
    0x85, 0x74, 0x09, 0x4C, 0xB1, 0x02,
    0x85, 0x75, 0x09, 0x4D, 0xB1, 0x02,
    0x85, 0x76, 0x09, 0x4E, 0xB1, 0x02,
    0x85, 0x77, 0x09, 0x4F, 0xB1, 0x02,
    0x85, 0x78, 0x09, 0x50, 0xB1, 0x02,
    0x85, 0x79, 0x09, 0x51, 0xB1, 0x02,
    0x85, 0x7A, 0x09, 0x52, 0xB1, 0x02,
    0x85, 0x7B, 0x09, 0x53, 0xB1, 0x02,
    0x85, 0xF4, 0x09, 0x2C, 0x95, 0x3F, 0xB1, 0x02,
    0x85, 0xF5, 0x09, 0x2D, 0x95, 0x07, 0xB1, 0x02,
    0x85, 0xF6, 0x09, 0x2E, 0x96, 0x22, 0x02, 0xB1, 0x02,
    0x85, 0xF7, 0x09, 0x2F, 0x95, 0x07, 0xB1, 0x02,
    0xC0
};
static_assert(sizeof(DualsenseEdge_HIDDescriptor) == 428, "Wrong size");

static const uint8_t DualsenseEdge_PairInfo_common[9] = {
    0x08, 0x25, 0x00, 0x1E, 0x00, 0xEE, 0x74, 0xD0, 0xBC
};

static const uint8_t DualsenseEdge_FirmwareInfo[DUALSENSE_FIRMWARE_INFO_REPORT_SIZE] = {
    0x4A, 0x75, 0x6E, 0x20, 0x31, 0x39, 0x20, 0x32, 0x30, 0x32, 0x33,
    0x31, 0x34, 0x3A, 0x34, 0x37, 0x3A, 0x33, 0x34, 0x03, 0x00, 0x44,
    0x00, 0x08, 0x02, 0x00, 0x01, 0x36, 0x00, 0x00, 0x01, 0xC1, 0xC8,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x24, // Version low byte (2.36 = 0x0224) - must be >= 2.21 for vibration v2
    0x02, // Version high byte
    0x00, 0x00, 0x14, 0x00, 0x00, 0x00, 0x0B, 0x00, 0x01, 0x00, 0x06,
    0x00, 0x00, 0x00, 0x00, 0x82, 0x79, 0xD9, 0x57
};

static const uint8_t DualsenseEdge_StockCalibration[DUALSENSE_CALIBRATION_REPORT_SIZE] = {
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00,             // Gyro Pitch/Yaw/Roll Bias
    0x10, 0x27, 0xF0, 0xD8,                         // Gyro Pitch Plus/Minus
    0x10, 0x27, 0xF0, 0xD8,                         // Gyro Yaw Plus/Minus
    0x10, 0x27, 0xF0, 0xD8,                         // Gyro Roll Plus/Minus
    0xF4, 0x01, 0xF4, 0x01,                         // Gyro Speed Plus/Minus
    0x10, 0x27, 0xF0, 0xD8,                         // Accel X Plus/Minus
    0x10, 0x27, 0xF0, 0xD8,                         // Accel Y Plus/Minus
    0x10, 0x27, 0xF0, 0xD8,                         // Accel Z Plus/Minus
    0x0B, 0x00, 0x00, 0x8D, 0x93, 0xCA, 0x2B
};

// DualSense BLE Input Report (77 bytes, sent after report ID 0x31 prepended
// by the GATT layer). Byte layout matches `struct dualsense_input_report`
// in Linux hid-playstation.c with a leading BT header byte.
#pragma pack(push, 1)
struct DsInputReport {
    uint8_t bt = 0x01;              // byte  0: BLE header (HasHID=1)
    uint8_t x  = 0x80;              // byte  1: left stick X  (0x80 = centered)
    uint8_t y  = 0x80;              // byte  2: left stick Y
    uint8_t rx = 0x80;              // byte  3: right stick X
    uint8_t ry = 0x80;              // byte  4: right stick Y
    uint8_t z  = 0;                 // byte  5: L2 analog trigger
    uint8_t rz = 0;                 // byte  6: R2 analog trigger
    uint8_t seq = 0x20;             // byte  7: sequence number
    // bytes 8-10: hat (low nibble of byte 8) + 20 buttons.
    uint8_t buttons[3] = { 0x08, 0, 0 };
    uint8_t extra_buttons = 0;      // byte 11: Edge paddles / misc
    uint32_t reserved = 0;          // bytes 12-15
    int16_t gyro_x = 0;             // bytes 16-17
    int16_t gyro_y = 0;             // bytes 18-19
    int16_t gyro_z = 0;             // bytes 20-21
    int16_t accel_x = 0;            // bytes 22-23
    int16_t accel_y = 0;            // bytes 24-25
    int16_t accel_z = 0;            // bytes 26-27
    uint32_t timestamp = 0x7621DD40;// bytes 28-31
    uint8_t reserved2 = 0;          // byte 32
    uint8_t touchpoint_0_contact = 0x80;  // byte 33 (0x80 = no contact)
    uint16_t touchpoint_0_x : 12;   // bytes 34-35
    uint16_t touchpoint_0_y : 12;   // byte 36
    uint8_t touchpoint_1_contact = 0x80;  // byte 37
    uint16_t touchpoint_1_x : 12;   // bytes 38-39
    uint16_t touchpoint_1_y : 12;   // byte 40
    uint8_t touchpad_timestamp = 0;         // byte 41
    uint8_t l2_trigger_feedback[3] = {};    // bytes 42-44
    uint8_t r2_trigger_feedback[3] = {};    // bytes 45-47
    uint8_t data_48_52[5] = {};             // bytes 48-52
    uint8_t status = 0x0A;                  // byte 53: low nibble=capacity 0-10, high=charging
    uint8_t status2 = 0x00;                 // byte 54: peripheral/connection status
    uint8_t data_55_71[17] = {};            // bytes 55-71
    uint8_t reserved3 = 0x00;               // byte 72
    uint32_t crc32 = 0;                     // bytes 73-76
} __attribute__((packed));
#pragma pack(pop)
static_assert(sizeof(DsInputReport) == 77, "Input report must be 77 bytes (78 on wire after report ID prepended)");

// DualSense BT Output Report parsed view (wire format is 78 bytes).
// Based on Linux hid-playstation.c dualsense_output_report_bt.
struct DsOutputReport {
    uint8_t report_id = 0;
    uint8_t seq_tag = 0;
    uint8_t tag = 0;

    uint8_t valid_flag0 = 0;
    uint8_t valid_flag1 = 0;
    uint8_t motor_right = 0;
    uint8_t motor_left = 0;

    uint8_t headphone_volume = 0;
    uint8_t speaker_volume = 0;
    uint8_t mic_volume = 0;
    uint8_t audio_control = 0;
    uint8_t mute_button_led = 0;
    uint8_t power_save_control = 0;

    uint8_t right_trigger_motor_mode = 0;
    uint8_t right_trigger_param[10]  = { 0 };
    uint8_t left_trigger_motor_mode  = 0;
    uint8_t left_trigger_param[10]   = { 0 };
    uint8_t reserved_host_timestamp[4] = { 0 };
    uint8_t reduce_motor_power       = 0;
    uint8_t audio_control2 = 0;
    uint8_t valid_flag2 = 0;
    uint8_t reserved3[2] = { 0 };
    uint8_t lightbar_setup = 0;
    uint8_t led_brightness = 0;
    uint8_t player_leds = 0;
    uint8_t lightbar_red = 0;
    uint8_t lightbar_green = 0;
    uint8_t lightbar_blue = 0;
    uint8_t reserved4[24] = { 0 };
    uint32_t crc32 = 0;

    bool hasRumble() const {
        return (valid_flag0 & DS_OUT_FLAG0_COMPATIBLE_VIBRATION)
            || (valid_flag2 & DS_OUT_FLAG2_COMPATIBLE_VIBRATION2)
            || (valid_flag0 == 0 && valid_flag2 == 0);
    }
    bool hasLeftTriggerEffect()  const { return valid_flag0 & DS_OUT_FLAG0_LEFT_TRIGGER_EFFECT;  }
    bool hasRightTriggerEffect() const { return valid_flag0 & DS_OUT_FLAG0_RIGHT_TRIGGER_EFFECT; }
    uint8_t weakMotor()   const { return motor_left;  }
    uint8_t strongMotor() const { return motor_right; }

    // Parses BLE (77/78-byte) or USB-over-BLE (77-byte) output report formats.
    // DSX sends the 77-byte USB-over-BLE variant; a real BLE HOGP write
    // (report ID stripped by the GATT layer) is also 77 bytes but starts
    // with the seq/tag pair instead of a bare sequence counter — disambiguate
    // via value[1] == 0x10 (the DS_OUTPUT_TAG), matching the reference impl.
    bool load(const uint8_t* value, size_t size) {
        if (!value || size < 47) return false;

        int common_offset = 0;
        if (size >= 77) {
            if (size >= 78 && value[0] == 0x31) {
                report_id = value[0]; seq_tag = value[1]; tag = value[2];
                common_offset = 3;
            } else if (value[0] == 0x31) {
                report_id = value[0]; seq_tag = value[1]; tag = value[2];
                common_offset = 3;
            } else if (value[1] == 0x10) {
                report_id = 0x31; seq_tag = value[0]; tag = value[1];
                common_offset = 2;
            } else {
                report_id = 0x02; seq_tag = value[0]; tag = 0;
                common_offset = 1;
            }
        } else if (size >= 62) {
            if (size >= 63 && value[0] == 0x02) {
                report_id = value[0]; seq_tag = 0; tag = 0;
                common_offset = 1;
            } else {
                report_id = 0x02; seq_tag = 0; tag = 0;
                common_offset = 0;
            }
        } else {
            report_id = 0; seq_tag = 0; tag = 0;
            common_offset = 0;
        }

        valid_flag0 = value[common_offset + 0];
        valid_flag1 = value[common_offset + 1];
        motor_right = value[common_offset + 2];
        motor_left  = value[common_offset + 3];
        headphone_volume = value[common_offset + 4];
        speaker_volume   = value[common_offset + 5];
        mic_volume       = value[common_offset + 6];
        audio_control    = value[common_offset + 7];
        mute_button_led  = value[common_offset + 8];
        power_save_control = value[common_offset + 9];

        right_trigger_motor_mode = value[common_offset + 10];
        for (int i = 0; i < 10; ++i) right_trigger_param[i] = value[common_offset + 11 + i];
        left_trigger_motor_mode = value[common_offset + 21];
        for (int i = 0; i < 10; ++i) left_trigger_param[i] = value[common_offset + 22 + i];
        for (int i = 0; i < 4;  ++i) reserved_host_timestamp[i] = value[common_offset + 32 + i];
        reduce_motor_power = value[common_offset + 36];
        audio_control2 = value[common_offset + 37];
        valid_flag2 = value[common_offset + 38];
        lightbar_setup = value[common_offset + 41];
        led_brightness = value[common_offset + 42];
        player_leds = value[common_offset + 43];
        lightbar_red   = value[common_offset + 44];
        lightbar_green = value[common_offset + 45];
        lightbar_blue  = value[common_offset + 46];

        if (size >= (size_t)(common_offset + 47 + 4)) {
            size_t crc_offset = size - 4;
            crc32 = (uint32_t)value[crc_offset]
                | ((uint32_t)value[crc_offset + 1] << 8)
                | ((uint32_t)value[crc_offset + 2] << 16)
                | ((uint32_t)value[crc_offset + 3] << 24);
        } else {
            crc32 = 0;
        }
        return true;
    }
};

#pragma pack(push, 1)
struct DsPairingInfoReport {
    uint8_t mac_address[6];
    uint8_t common[9];
    uint32_t crc32;
    uint8_t _padding;  // BlueZ off-by-one workaround, see reference impl notes
} __attribute__((packed));
#pragma pack(pop)
static_assert(sizeof(DsPairingInfoReport) == DUALSENSE_PAIRING_INFO_REPORT_SIZE, "size mismatch");

// CRC32 (same polynomial/table as Linux hid-playstation / DualShock4 BT).
uint32_t ds_crc32_le(uint32_t crc, const uint8_t* buf, size_t len);
