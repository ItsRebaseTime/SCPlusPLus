#pragma once
// GPIO pin assignments for the ESP32-S31-Function-CoreBoard-1.
//
// Codec/amplifier pins below were extracted directly from the official
// schematic (https://dl.espressif.com/schematics/esp32-s31-function-coreboard-1-schematics.pdf,
// "Codec:" page) by tracing each net (I2S_MCLK, I2S_SCLK, I2S_ASDOUT,
// I2S_LRCK, I2S_DSDIN, ESP_I2C_SDA, ESP_I2C_SCL, PA_CTRL) from the ES8311
// IC (U7) back to its GPIO on the ESP32-S31-WROOM-3 module (U1) pinout
// table on the schematic's page 2. Not guessed — confirmed against the
// real hardware design.
//
// Not board-derived: RGB LED (GPIO60) and BOOT button (GPIO61) come from
// the published user guide, not the schematic crop above, but are
// consistent with it.

// ── ES8311 audio codec (I2C control + I2S data) ────────────────────────────
// Codec chip-enable (Codec_CE) is hardware-strapped via a resistor divider
// on the board — no GPIO needed to enable the codec itself.
static constexpr int CODEC_I2C_SDA_GPIO = 51;
static constexpr int CODEC_I2C_SCL_GPIO = 50;
static constexpr int CODEC_I2S_MCLK_GPIO   = 52;
static constexpr int CODEC_I2S_SCLK_GPIO   = 53;  // BCLK
static constexpr int CODEC_I2S_ASDOUT_GPIO = 54;  // codec ADC (mic) -> MCU
static constexpr int CODEC_I2S_LRCK_GPIO   = 55;  // word select
static constexpr int CODEC_I2S_DSDIN_GPIO  = 56;  // MCU -> codec DAC (speaker)

// ── NS4150B speaker amplifier ───────────────────────────────────────────────
// PA_CTRL enable/shutdown. Polarity not yet confirmed against the NS4150B
// datasheet from the schematic alone — assumed active-high (standard for
// this class of Class-D amp SD/CTRL pins) until verified on hardware.
static constexpr int AMP_CTRL_GPIO = 57;

// ── Misc board I/O (from the official user guide) ──────────────────────────
static constexpr int RGB_LED_GPIO   = 60;
static constexpr int BOOT_BTN_GPIO  = 61;
