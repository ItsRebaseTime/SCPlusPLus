#pragma once
// Wire format is hardware-defined (Steam Controller 2026 USB HID protocol),
// not framework-defined — keep this file in sync with ../../../src/sc_report.h.
#include <stdint.h>

// VID/PID: Valve Steam Controller Puck (steam-bridge, PID 0x1304)
static constexpr uint16_t SC_VID       = 0x28DE;
static constexpr uint16_t SC_PID       = 0x1302;
static constexpr uint8_t SC_REPORT_ID          = 0x42;  // full controller state, 54 bytes
static constexpr uint8_t SC_REPORT_ID_TOUCH    = 0x40;  // short capacitive-touch event, 6 bytes
static constexpr uint8_t SC_REPORT_TOUCH_SIZE  = 6;
static constexpr uint8_t SC_REPORT_ID_HW_INFO  = 0x43;  // hardware-info / pre-game-mode status, 15 bytes (ignored)
static constexpr uint8_t SC_REPORT_ID_BTN_SEQ  = 0x41;  // button press event queue, 9 bytes — sends button IDs in the order they were pressed; not useful for polling-based input, ignored
static constexpr uint8_t SC_REPORT_ID_HAPTIC_STOP = 0x82;  // stop haptic motor — use instead of 0x81 to avoid triggering controller reboot
static constexpr uint8_t SC_REPORT_ID_HAPTIC_PLAY = 0x83;  // play haptic tone: [ch, amp, freq_lo, freq_hi, 0xFF, duration, 0, 0, 0]
static constexpr uint8_t SC_REPORT_ID_PCM_INIT    = 0x86;  // enter PCM streaming mode for a channel: [0x86, 0x02, ch, param]
static constexpr uint8_t SC_REPORT_ID_PCM_DATA    = 0x88;  // PCM audio packet: [0x88, 31, L0..L30, R0..R30] at 8 kHz signed 8-bit

// Physical output channels, confirmed via hardware testing.
enum SCChannel : uint8_t {
    SC_CH_LPAD       = 0,  // left  touchpad haptic actuator
    SC_CH_RPAD       = 1,  // right touchpad haptic actuator
    SC_CH_BOTH_PADS  = 2,  // both  touchpad haptic actuators simultaneously
    SC_CH_LRUMBLE    = 3,  // left  rumble motor
    SC_CH_RRUMBLE    = 4,  // right rumble motor
    SC_CH_BOTH_RUMBLE= 5,  // both  rumble motors simultaneously
};

// Button positions within report bytes 2–5 read as LE uint32.
// btns[0]=0x02 → bits 0–7, btns[1]=0x03 → bits 8–15,
// btns[2]=0x04 → bits 16–23, btns[3]=0x05 → bits 24–31.
// All positions confirmed via hardware capture (byte_offset, bit_index).
enum SCButton : uint32_t {
    // Byte 0x02 (bits 0–7)
    SC_BTN_A            = 1u <<  0,   // (0x02,0) → DS Cross
    SC_BTN_B            = 1u <<  1,   // (0x02,1) → DS Circle
    SC_BTN_X            = 1u <<  2,   // (0x02,2) → DS Square
    SC_BTN_Y            = 1u <<  3,   // (0x02,3) → DS Triangle
    SC_BTN_QUICK_ACCESS = 1u <<  4,   // (0x02,4) → DS Create
    SC_BTN_RSTICK_CLICK = 1u <<  5,   // (0x02,5) → DS RS
    SC_BTN_MENU         = 1u <<  6,   // (0x02,6) → DS Options
    SC_BTN_R4           = 1u <<  7,   // (0x02,7) → DS R4

    // Byte 0x03 (bits 8–15)
    SC_BTN_R5           = 1u <<  8,   // (0x03,0) → DS R5
    SC_BTN_R1           = 1u <<  9,   // (0x03,1) → DS R1
    SC_BTN_DPAD_DOWN    = 1u << 10,   // (0x03,2)
    SC_BTN_DPAD_RIGHT   = 1u << 11,   // (0x03,3)
    SC_BTN_DPAD_LEFT    = 1u << 12,   // (0x03,4)
    SC_BTN_DPAD_UP      = 1u << 13,   // (0x03,5)
    SC_BTN_VIEW         = 1u << 14,   // (0x03,6) → DS Select
    SC_BTN_LSTICK_CLICK = 1u << 15,   // (0x03,7) → DS LS

    // Byte 0x04 (bits 16–23)
    SC_BTN_STEAM        = 1u << 16,   // (0x04,0) → DS PS
    SC_BTN_L4           = 1u << 17,   // (0x04,1) → DS L4
    SC_BTN_L5           = 1u << 18,   // (0x04,2) → DS L5
    SC_BTN_L1           = 1u << 19,   // (0x04,3) → DS L1
    SC_BTN_RSTICK_CAP   = 1u << 20,   // (0x04,4) right joystick capacitive sensor
    SC_BTN_RPAD_TOUCH   = 1u << 21,   // (0x04,5)
    SC_BTN_RPAD_CLICK   = 1u << 22,   // (0x04,6) → DS Touchpad
    SC_BTN_R2_DIGITAL   = 1u << 23,   // (0x04,7) → DS R2

    // Byte 0x05 (bits 24–31)
    SC_BTN_LSTICK_CAP   = 1u << 24,   // (0x05,0) left joystick capacitive sensor
    SC_BTN_LPAD_TOUCH   = 1u << 25,   // (0x05,1)
    SC_BTN_LPAD_CLICK   = 1u << 26,   // (0x05,2) → DS Touchpad
    SC_BTN_L2_DIGITAL   = 1u << 27,   // (0x05,3) → DS L2
    SC_BTN_RGRIP_CAP    = 1u << 28,   // (0x05,4) right capacitive grip sensor
    SC_BTN_LGRIP_CAP    = 1u << 29,   // (0x05,5) left capacitive grip sensor
};

struct __attribute__((packed)) SCTouchReport {
    uint8_t  id;                //  0: 0x40
    uint8_t  flags;             //  1: touch flags / packet type
    int8_t   right_touch_x;     //  2: right touchpad X velocity (signed)
    int8_t   right_touch_y;     //  3: right touchpad Y velocity (signed)
    int8_t   left_touch_y;      //  4: left touchpad Y velocity (signed) — note Y before X
    int8_t   left_touch_x;      //  5: left touchpad X velocity (signed)
};
static_assert(sizeof(SCTouchReport) == SC_REPORT_TOUCH_SIZE, "SCTouchReport layout mismatch");

struct __attribute__((packed)) SCReport {
    uint8_t  id;             //  0: 0x42
    uint8_t  seq;            //  1: rolling counter
    uint8_t  btns[4];        //  2–5: button bits (sc_buttons() reads as LE uint32)
    uint16_t ltrigger;       //  6–7: left trigger  0–32767
    uint16_t rtrigger;       //  8–9: right trigger 0–32767
    int16_t  left_stick_x;   // 10–11
    int16_t  left_stick_y;   // 12–13
    int16_t  right_stick_x;  // 14–15
    int16_t  right_stick_y;  // 16–17
    int16_t  lpad_x;         // 18–19: left touchpad X position
    int16_t  lpad_y;         // 20–21: left touchpad Y position
    int16_t  lpad_force;     // 22–23: left touchpad press force
    int16_t  rpad_x;         // 24–25: right touchpad X position
    int16_t  rpad_y;         // 26–27: right touchpad Y position
    int16_t  rpad_force;     // 28–29: right touchpad press force
    uint8_t  heartbeat;      // 30: bits[7:6] = 2-bit keep-alive counter (~1 Hz), bits[5:0] = device status
    uint8_t  checksum;       // 31: suspected checksum / CRC (not yet confirmed)
    uint16_t counter;        // 32–33: free-running 16-bit counter
    int16_t  acc_x;          // 34–35
    int16_t  acc_y;          // 36–37
    int16_t  acc_z;          // 38–39
    int16_t  gyro_x;         // 40–41
    int16_t  gyro_y;         // 42–43
    int16_t  gyro_z;         // 44–45
    int16_t  quat_w;         // 46–47
    int16_t  quat_x;         // 48–49
    int16_t  quat_y;         // 50–51
    int16_t  quat_z;         // 52–53
};
static_assert(sizeof(SCReport) == 54, "SCReport layout mismatch");

static inline uint32_t sc_buttons(const SCReport& r) {
    return (uint32_t)r.btns[0]
         | ((uint32_t)r.btns[1] <<  8)
         | ((uint32_t)r.btns[2] << 16)
         | ((uint32_t)r.btns[3] << 24);
}
