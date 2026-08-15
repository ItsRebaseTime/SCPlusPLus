// ES8311 + NS4150B bring-up. Structure follows ESP-IDF's own
// examples/peripherals/i2s/i2s_codec/i2s_es8311 reference exactly (real,
// build-verified-upstream pattern) rather than inventing an API shape,
// since esp_codec_dev/es8311 weren't available to inspect directly until
// vendored by a build. Actual data path uses raw i2s_channel_read/write
// (same as the reference example) — esp_codec_dev is used only for the
// I2C control path (open/volume/gain), not the data path.
#include "le_audio_codec.h"
#include "config.h"

#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "esp_codec_dev_defaults.h"
#include "esp_codec_dev.h"
#include "esp_log.h"

static const char *TAG = "le_audio_codec";

static constexpr int I2S_NUM = 0;
static constexpr int I2C_NUM = 0;
static constexpr int        MCLK_MULTIPLE = 384;  // matches upstream example; covers up to 24-bit width

static i2s_chan_handle_t     s_tx_handle = nullptr;
static i2s_chan_handle_t     s_rx_handle = nullptr;
static esp_codec_dev_handle_t s_codec_handle = nullptr;
static uint32_t               s_sample_rate = 16000;

static bool i2s_driver_init(uint32_t sample_rate_hz) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    if (i2s_new_channel(&chan_cfg, &s_tx_handle, &s_rx_handle) != ESP_OK) {
        ESP_LOGE(TAG, "i2s_new_channel failed");
        return false;
    }

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = (gpio_num_t)CODEC_I2S_MCLK_GPIO,
            .bclk = (gpio_num_t)CODEC_I2S_SCLK_GPIO,
            .ws   = (gpio_num_t)CODEC_I2S_LRCK_GPIO,
            .dout = (gpio_num_t)CODEC_I2S_DSDIN_GPIO,
            .din  = (gpio_num_t)CODEC_I2S_ASDOUT_GPIO,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    std_cfg.clk_cfg.mclk_multiple = (i2s_mclk_multiple_t)MCLK_MULTIPLE;

    if (i2s_channel_init_std_mode(s_tx_handle, &std_cfg) != ESP_OK ||
        i2s_channel_init_std_mode(s_rx_handle, &std_cfg) != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_init_std_mode failed");
        return false;
    }
    if (i2s_channel_enable(s_tx_handle) != ESP_OK || i2s_channel_enable(s_rx_handle) != ESP_OK) {
        ESP_LOGE(TAG, "i2s_channel_enable failed");
        return false;
    }
    return true;
}

static bool es8311_init(uint32_t sample_rate_hz) {
    i2c_master_bus_handle_t i2c_bus = nullptr;
    i2c_master_bus_config_t i2c_cfg = {
        .i2c_port = I2C_NUM,
        .sda_io_num = (gpio_num_t)CODEC_I2C_SDA_GPIO,
        .scl_io_num = (gpio_num_t)CODEC_I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags = { .enable_internal_pullup = true },
    };
    if (i2c_new_master_bus(&i2c_cfg, &i2c_bus) != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed");
        return false;
    }

    audio_codec_i2c_cfg_t ctrl_cfg = {
        .port = I2C_NUM,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
        .bus_handle = i2c_bus,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&ctrl_cfg);
    if (!ctrl_if) { ESP_LOGE(TAG, "audio_codec_new_i2c_ctrl failed"); return false; }

    audio_codec_i2s_cfg_t i2s_data_cfg = {
        .port = I2S_NUM,
        .rx_handle = s_rx_handle,
        .tx_handle = s_tx_handle,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_data_cfg);
    if (!data_if) { ESP_LOGE(TAG, "audio_codec_new_i2s_data failed"); return false; }

    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    if (!gpio_if) { ESP_LOGE(TAG, "audio_codec_new_gpio failed"); return false; }

    // pa_pin lets the es8311 driver own the NS4150B enable line directly —
    // it's toggled automatically on codec open/close/mute, no separate GPIO
    // handling needed on our side. Polarity (pa_reverted) not yet confirmed
    // against the NS4150B datasheet — assumed active-high; flip if the
    // amp turns out inverted on real hardware.
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH,
        .pa_pin = AMP_CTRL_GPIO,
        .pa_reverted = false,
        .master_mode = false,
        .use_mclk = true,
        .hw_gain = { .pa_voltage = 5.0, .codec_dac_voltage = 3.3 },
        .mclk_div = MCLK_MULTIPLE,
    };
    const audio_codec_if_t *es8311_if = es8311_codec_new(&es8311_cfg);
    if (!es8311_if) { ESP_LOGE(TAG, "es8311_codec_new failed"); return false; }

    esp_codec_dev_cfg_t dev_cfg = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = es8311_if,
        .data_if = data_if,
    };
    s_codec_handle = esp_codec_dev_new(&dev_cfg);
    if (!s_codec_handle) { ESP_LOGE(TAG, "esp_codec_dev_new failed"); return false; }

    esp_codec_dev_sample_info_t sample_cfg = {
        .bits_per_sample = I2S_DATA_BIT_WIDTH_16BIT,
        .channel = 2,
        .channel_mask = 0x03,
        .sample_rate = (int)sample_rate_hz,
    };
    if (esp_codec_dev_open(s_codec_handle, &sample_cfg) != ESP_CODEC_DEV_OK) {
        ESP_LOGE(TAG, "esp_codec_dev_open failed");
        return false;
    }
    return true;
}

bool codec_init(uint32_t sample_rate_hz) {
    s_sample_rate = sample_rate_hz;
    if (!i2s_driver_init(sample_rate_hz)) return false;
    if (!es8311_init(sample_rate_hz)) return false;
    ESP_LOGI(TAG, "codec init OK at %u Hz", (unsigned)sample_rate_hz);
    return true;
}

bool codec_set_sample_rate(uint32_t sample_rate_hz) {
    if (sample_rate_hz == s_sample_rate) return true;
    i2s_std_clk_config_t clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate_hz);
    clk_cfg.mclk_multiple = (i2s_mclk_multiple_t)MCLK_MULTIPLE;
    if (i2s_channel_disable(s_tx_handle) != ESP_OK || i2s_channel_disable(s_rx_handle) != ESP_OK) return false;
    bool ok = (i2s_channel_reconfig_std_clock(s_tx_handle, &clk_cfg) == ESP_OK) &&
              (i2s_channel_reconfig_std_clock(s_rx_handle, &clk_cfg) == ESP_OK);
    i2s_channel_enable(s_tx_handle);
    i2s_channel_enable(s_rx_handle);
    if (!ok) { ESP_LOGE(TAG, "i2s_channel_reconfig_std_clock failed"); return false; }
    s_sample_rate = sample_rate_hz;
    ESP_LOGI(TAG, "codec sample rate -> %u Hz", (unsigned)sample_rate_hz);
    return true;
}

size_t codec_write(const int16_t *samples, size_t byte_len) {
    size_t written = 0;
    esp_err_t err = i2s_channel_write(s_tx_handle, samples, byte_len, &written, 1000);
    if (err != ESP_OK) ESP_LOGW(TAG, "i2s_channel_write err=%d", err);
    return written;
}

size_t codec_read(int16_t *samples, size_t byte_len) {
    size_t read_bytes = 0;
    esp_err_t err = i2s_channel_read(s_rx_handle, samples, byte_len, &read_bytes, 1000);
    if (err != ESP_OK) ESP_LOGW(TAG, "i2s_channel_read err=%d", err);
    return read_bytes;
}

void codec_set_speaker_volume(int volume_pct) {
    if (!s_codec_handle) return;
    esp_codec_dev_set_out_vol(s_codec_handle, volume_pct);
}

void codec_set_mic_gain(int gain_pct) {
    if (!s_codec_handle) return;
    esp_codec_dev_set_in_gain(s_codec_handle, (float)gain_pct);
}

void codec_loopback_task(void *) {
    constexpr size_t BUF_BYTES = 2400;  // matches upstream example's echo-mode buffer size
    auto *buf = (int16_t *)malloc(BUF_BYTES);
    if (!buf) { ESP_LOGE(TAG, "no memory for loopback buffer"); return; }

    ESP_LOGI(TAG, "loopback: mic -> speaker");
    for (;;) {
        const size_t got = codec_read(buf, BUF_BYTES);
        if (got == 0) continue;
        const size_t sent = codec_write(buf, got);
        if (sent != got) ESP_LOGW(TAG, "loopback: read %u bytes, wrote %u", (unsigned)got, (unsigned)sent);
    }
}
