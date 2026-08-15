// Bidirectional LE Audio (BAP Unicast Server) sharing one NimBLE GATT
// server/connection with the DualSense Edge HID service. See le_audio_bap.h
// for the architectural rationale (why this owns the whole BLE lifecycle
// instead of using espressif/esp_bt_audio's monolithic esp_bt_audio_init()).
//
// Compiled as C, not C++ (unlike the rest of this project's app code):
// esp_ble_audio's own public headers (esp_ble_audio_defs.h et al, via
// lib/include/audio.h) use bare `enum bt_bap_ascs_reason;` forward
// declarations — legal C, but not legal ISO C++ (no underlying-type opaque
// enum syntax used), and unlike the int<->enum conversion issues elsewhere
// in this project, -fpermissive does not relax this one. The rest of
// esp_ble_audio's own API implementation is itself plain C for the same
// reason. ds_edge_hid.h/le_audio_codec.h are intentionally NOT #included
// here (both use C++-only syntax — a reference parameter and a default
// argument, respectively) — the few functions this file calls from them are
// forward-declared locally instead.
//
// LC3 encode/decode: sink (speaker) direction decodes incoming ISO frames in
// on_stream_recv() and writes PCM to the codec; source (mic) direction is
// driven by mic_encode_task(), which reads PCM from the codec, LC3-encodes
// it, and sends it via esp_ble_audio_bap_stream_send() while the source ASE
// is in the streaming state. Codec I2S is stereo-interleaved (matches
// le_audio_codec.cpp's I2S_SLOT_MODE_STEREO config) while BAP negotiates
// mono (PACS location MONO_AUDIO) — mono_to_stereo()/stereo_to_mono() bridge
// the two.
#include "le_audio_bap.h"

#include "esp_bt.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/bas/ble_svc_bas.h"
#include "services/dis/ble_svc_dis.h"

#include "esp_ble_audio_defs.h"
#include "esp_ble_audio_pacs_api.h"
#include "esp_ble_audio_bap_api.h"
#include "esp_ble_audio_lc3_defs.h"
#include "esp_ble_audio_codec_api.h"

#include "esp_lc3_dec.h"
#include "esp_lc3_enc.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

// From ds_edge_hid.cpp (C++, exposed with C linkage via its own
// extern "C" — see ds_edge_hid.h). Declared locally rather than via
// #include "ds_edge_hid.h" because that header also declares ds_send(const
// SCReport&), a C++ reference parameter this C translation unit can't parse.
struct ble_gatt_svc_def;
extern const struct ble_gatt_svc_def* ds_edge_hid_get_services(void);
extern void ds_edge_hid_on_connect(uint16_t conn_handle);
extern void ds_edge_hid_on_disconnect(void);

// From le_audio_codec.cpp. Declared locally rather than via #include
// "le_audio_codec.h" because that header declares codec_init() with a C++
// default argument.
extern bool codec_set_sample_rate(uint32_t sample_rate_hz);
extern size_t codec_write(const int16_t *samples, size_t byte_len);
extern size_t codec_read(int16_t *samples, size_t byte_len);

static const char *TAG = "le_audio_bap";

static uint8_t s_own_addr_type = 0;

// ---------------------------------------------------------------------
// LC3 codec state. One decoder (sink/speaker) + one encoder (source/mic),
// (re)opened whenever ASCS negotiates a codec config for that direction.
// ---------------------------------------------------------------------
static void *s_lc3_dec_handle = NULL;
static void *s_lc3_enc_handle = NULL;
static volatile bool s_sink_streaming = false;
static volatile bool s_source_streaming = false;
static uint16_t s_source_seq_num = 0;
static TaskHandle_t s_mic_task_handle = NULL;

static void mono_to_stereo(const int16_t *mono, int16_t *stereo, size_t n_samples) {
    for (size_t i = 0; i < n_samples; i++) {
        stereo[2 * i] = mono[i];
        stereo[2 * i + 1] = mono[i];
    }
}

static void stereo_to_mono(const int16_t *stereo, int16_t *mono, size_t n_samples) {
    for (size_t i = 0; i < n_samples; i++) {
        mono[i] = stereo[2 * i];
    }
}

// ---------------------------------------------------------------------
// PACS: one LC3 capability per direction, 16kHz/10ms mono (matches
// le_audio_codec.cpp's codec_init(16000) default). Frame length range
// (20-40 octets) covers the standard 16_1/16_2 unicast presets.
// ---------------------------------------------------------------------
static uint8_t s_pacs_cap_data[] = ESP_BLE_AUDIO_CODEC_CAP_LC3_DATA(
    BT_AUDIO_CODEC_CAP_FREQ_16KHZ, BT_AUDIO_CODEC_CAP_DURATION_10,
    BT_AUDIO_CODEC_CAP_CHAN_COUNT_SUPPORT(1), 20, 40, 1);
static uint8_t s_pacs_cap_meta[] = ESP_BLE_AUDIO_CODEC_CAP_LC3_META(ESP_BLE_AUDIO_CONTEXT_TYPE_CONVERSATIONAL);

static esp_ble_audio_pacs_cap_t s_pacs_cap_snk = {
    .codec_cap = &ESP_BLE_AUDIO_CODEC_CAP_LC3(s_pacs_cap_data, s_pacs_cap_meta),
};
static esp_ble_audio_pacs_cap_t s_pacs_cap_src = {
    .codec_cap = &ESP_BLE_AUDIO_CODEC_CAP_LC3(s_pacs_cap_data, s_pacs_cap_meta),
};

// ---------------------------------------------------------------------
// ASE stream objects — one sink (speaker), one source (mic).
// ---------------------------------------------------------------------
static esp_ble_audio_bap_stream_t s_sink_stream;
static esp_ble_audio_bap_stream_t s_source_stream;
static bool s_sink_in_use = false;
static bool s_source_in_use = false;

// ---------------------------------------------------------------------
// ASCS unicast-server callbacks (ASE state machine)
// ---------------------------------------------------------------------
static int on_ase_config(struct bt_conn *conn, const esp_ble_audio_bap_ep_t *ep,
                          esp_ble_audio_dir_t dir, const esp_ble_audio_codec_cfg_t *codec_cfg,
                          esp_ble_audio_bap_stream_t **stream,
                          esp_ble_audio_bap_qos_cfg_pref_t *const pref,
                          esp_ble_audio_bap_ascs_rsp_t *rsp) {
    ESP_LOGI(TAG, "ASE config: dir=%d", (int)dir);

    uint32_t freq_hz = 16000;
    {
        esp_ble_audio_codec_cfg_freq_t freq;
        if (esp_ble_audio_codec_cfg_get_freq(codec_cfg, &freq) == ESP_OK)
            esp_ble_audio_codec_cfg_freq_to_freq_hz(freq, &freq_hz);
    }
    uint32_t frame_dur_us = 10000;
    {
        esp_ble_audio_codec_cfg_frame_dur_t dur;
        if (esp_ble_audio_codec_cfg_get_frame_dur(codec_cfg, &dur) == ESP_OK)
            esp_ble_audio_codec_cfg_frame_dur_to_frame_dur_us(dur, &frame_dur_us);
    }
    uint16_t octets_per_frame = 40;
    esp_ble_audio_codec_cfg_get_octets_per_frame(codec_cfg, &octets_per_frame);

    if (dir == ESP_BLE_AUDIO_DIR_SINK) {
        if (s_sink_in_use) { *rsp = ESP_BLE_AUDIO_BAP_ASCS_RSP(ESP_BLE_AUDIO_BAP_ASCS_RSP_CODE_NO_MEM, ESP_BLE_AUDIO_BAP_ASCS_REASON_NONE); return -1; }
        s_sink_in_use = true;
        *stream = &s_sink_stream;

        if (s_lc3_dec_handle) { esp_lc3_dec_close(s_lc3_dec_handle); s_lc3_dec_handle = NULL; }
        esp_lc3_dec_cfg_t dec_cfg = ESP_LC3_DEC_CONFIG_DEFAULT();
        dec_cfg.sample_rate = freq_hz;
        dec_cfg.channel = 1;
        dec_cfg.bits_per_sample = 16;
        dec_cfg.frame_dms = (uint8_t)(frame_dur_us / 100);
        dec_cfg.nbyte = octets_per_frame;
        if (esp_lc3_dec_open(&dec_cfg, sizeof(dec_cfg), &s_lc3_dec_handle) != ESP_AUDIO_ERR_OK) {
            ESP_LOGE(TAG, "esp_lc3_dec_open failed");
            *rsp = ESP_BLE_AUDIO_BAP_ASCS_RSP(ESP_BLE_AUDIO_BAP_ASCS_RSP_CODE_CONF_UNSUPPORTED, ESP_BLE_AUDIO_BAP_ASCS_REASON_NONE);
            return -1;
        }
    } else {
        if (s_source_in_use) { *rsp = ESP_BLE_AUDIO_BAP_ASCS_RSP(ESP_BLE_AUDIO_BAP_ASCS_RSP_CODE_NO_MEM, ESP_BLE_AUDIO_BAP_ASCS_REASON_NONE); return -1; }
        s_source_in_use = true;
        *stream = &s_source_stream;

        if (s_lc3_enc_handle) { esp_lc3_enc_close(s_lc3_enc_handle); s_lc3_enc_handle = NULL; }
        esp_lc3_enc_config_t enc_cfg = ESP_LC3_ENC_CONFIG_DEFAULT();
        enc_cfg.sample_rate = freq_hz;
        enc_cfg.channel = 1;
        enc_cfg.bits_per_sample = 16;
        enc_cfg.frame_dms = (uint8_t)(frame_dur_us / 100);
        enc_cfg.nbyte = octets_per_frame;
        if (esp_lc3_enc_open(&enc_cfg, sizeof(enc_cfg), &s_lc3_enc_handle) != ESP_AUDIO_ERR_OK) {
            ESP_LOGE(TAG, "esp_lc3_enc_open failed");
            *rsp = ESP_BLE_AUDIO_BAP_ASCS_RSP(ESP_BLE_AUDIO_BAP_ASCS_RSP_CODE_CONF_UNSUPPORTED, ESP_BLE_AUDIO_BAP_ASCS_REASON_NONE);
            return -1;
        }
        s_source_seq_num = 0;
    }

    // Default QoS preference: unframed, 1M PHY, matches the 16_2_1 unicast
    // preset (10ms interval, moderate latency/retransmission).
    // Plain C (unlike C++) requires a compound-literal cast to use a
    // brace-init-list macro expansion as an assignment RHS rather than a
    // declaration initializer.
    *pref = (esp_ble_audio_bap_qos_cfg_pref_t)ESP_BLE_AUDIO_BAP_QOS_CFG_PREF(true, ESP_BLE_AUDIO_BAP_QOS_CFG_1M, 2, 10, 0, 40000, 0, 40000);

    codec_set_sample_rate(freq_hz);
    ESP_LOGI(TAG, "ASE configured at %u Hz, %u us frames, %u octets/frame",
             (unsigned)freq_hz, (unsigned)frame_dur_us, (unsigned)octets_per_frame);
    return 0;
}

static int on_ase_reconfig(esp_ble_audio_bap_stream_t *stream, esp_ble_audio_dir_t dir,
                            const esp_ble_audio_codec_cfg_t *codec_cfg,
                            esp_ble_audio_bap_qos_cfg_pref_t *const pref,
                            esp_ble_audio_bap_ascs_rsp_t *rsp) {
    ESP_LOGI(TAG, "ASE reconfig: dir=%d", (int)dir);
    // Plain C (unlike C++) requires a compound-literal cast to use a
    // brace-init-list macro expansion as an assignment RHS rather than a
    // declaration initializer.
    *pref = (esp_ble_audio_bap_qos_cfg_pref_t)ESP_BLE_AUDIO_BAP_QOS_CFG_PREF(true, ESP_BLE_AUDIO_BAP_QOS_CFG_1M, 2, 10, 0, 40000, 0, 40000);
    return 0;
}

static int on_ase_qos(esp_ble_audio_bap_stream_t *stream, const esp_ble_audio_bap_qos_cfg_t *qos,
                       esp_ble_audio_bap_ascs_rsp_t *rsp) {
    ESP_LOGI(TAG, "ASE QoS configured");
    return 0;
}

static int on_ase_enable(esp_ble_audio_bap_stream_t *stream, const uint8_t meta[], size_t meta_len,
                          esp_ble_audio_bap_ascs_rsp_t *rsp) {
    ESP_LOGI(TAG, "ASE enable");
    return 0;
}

static int on_ase_start(esp_ble_audio_bap_stream_t *stream, esp_ble_audio_bap_ascs_rsp_t *rsp) {
    ESP_LOGI(TAG, "ASE start");
    return 0;
}

static int on_ase_metadata(esp_ble_audio_bap_stream_t *stream, const uint8_t meta[], size_t meta_len,
                            esp_ble_audio_bap_ascs_rsp_t *rsp) {
    return 0;
}

static int on_ase_disable(esp_ble_audio_bap_stream_t *stream, esp_ble_audio_bap_ascs_rsp_t *rsp) {
    ESP_LOGI(TAG, "ASE disable");
    return 0;
}

static int on_ase_stop(esp_ble_audio_bap_stream_t *stream, esp_ble_audio_bap_ascs_rsp_t *rsp) {
    ESP_LOGI(TAG, "ASE stop");
    return 0;
}

static int on_ase_release(esp_ble_audio_bap_stream_t *stream, esp_ble_audio_bap_ascs_rsp_t *rsp) {
    ESP_LOGI(TAG, "ASE release");
    if (stream == &s_sink_stream) {
        s_sink_in_use = false;
        s_sink_streaming = false;
        if (s_lc3_dec_handle) { esp_lc3_dec_close(s_lc3_dec_handle); s_lc3_dec_handle = NULL; }
    }
    if (stream == &s_source_stream) {
        s_source_in_use = false;
        s_source_streaming = false;
        if (s_lc3_enc_handle) { esp_lc3_enc_close(s_lc3_enc_handle); s_lc3_enc_handle = NULL; }
    }
    return 0;
}

static esp_ble_audio_bap_unicast_server_cb_t s_server_cb = {
    .config = on_ase_config,
    .reconfig = on_ase_reconfig,
    .qos = on_ase_qos,
    .enable = on_ase_enable,
    .start = on_ase_start,
    .metadata = on_ase_metadata,
    .disable = on_ase_disable,
    .stop = on_ase_stop,
    .release = on_ase_release,
};

// ---------------------------------------------------------------------
// Stream ops (data path).
// ---------------------------------------------------------------------
static void on_stream_started(esp_ble_audio_bap_stream_t *stream) {
    ESP_LOGI(TAG, "stream started");
    if (stream == &s_sink_stream) s_sink_streaming = true;
    if (stream == &s_source_stream) s_source_streaming = true;
}

static void on_stream_stopped(esp_ble_audio_bap_stream_t *stream, uint8_t reason) {
    ESP_LOGI(TAG, "stream stopped, reason=%u", reason);
    if (stream == &s_sink_stream) s_sink_streaming = false;
    if (stream == &s_source_stream) s_source_streaming = false;
}

// Decodes one incoming LC3 frame (speaker direction) and writes the result
// to the codec, duplicated across both I2S stereo slots.
static void on_stream_recv(esp_ble_audio_bap_stream_t *stream, const struct bt_iso_recv_info *info,
                            const uint8_t *data, uint16_t len) {
    if (!s_lc3_dec_handle) return;

    static int16_t s_mono_buf[480];   // headroom: up to 480 samples/frame
    static int16_t s_stereo_buf[960]; // 2x for L+R interleave

    esp_audio_dec_in_raw_t raw = {0};
    raw.buffer = (uint8_t*)data;
    raw.len = len;
    esp_audio_dec_out_frame_t out = {0};
    out.buffer = (uint8_t*)s_mono_buf;
    out.len = sizeof(s_mono_buf);
    esp_audio_dec_info_t dec_info;

    esp_audio_err_t ret = esp_lc3_dec_decode(s_lc3_dec_handle, &raw, &out, &dec_info);
    if (ret != ESP_AUDIO_ERR_OK) {
        ESP_LOGW(TAG, "LC3 decode failed: %d", (int)ret);
        return;
    }

    size_t n_samples = out.decoded_size / sizeof(int16_t);
    if (n_samples > sizeof(s_mono_buf) / sizeof(int16_t)) n_samples = sizeof(s_mono_buf) / sizeof(int16_t);
    mono_to_stereo(s_mono_buf, s_stereo_buf, n_samples);
    codec_write(s_stereo_buf, n_samples * 2 * sizeof(int16_t));
}

static void on_stream_sent(esp_ble_audio_bap_stream_t *stream, void *user_data) {
}

static void on_stream_connected(esp_ble_audio_bap_stream_t *stream) {
    ESP_LOGI(TAG, "stream ISO connected");
}

static void on_stream_disconnected(esp_ble_audio_bap_stream_t *stream, uint8_t reason) {
    ESP_LOGI(TAG, "stream ISO disconnected, reason=%u", reason);
    if (stream == &s_sink_stream) s_sink_streaming = false;
    if (stream == &s_source_stream) s_source_streaming = false;
}

static esp_ble_audio_bap_stream_ops_t s_stream_ops = {
    .started = on_stream_started,
    .stopped = on_stream_stopped,
    .recv = on_stream_recv,
    .sent = on_stream_sent,
    .connected = on_stream_connected,
    .disconnected = on_stream_disconnected,
};

// ---------------------------------------------------------------------
// GAP / advertising / host lifecycle (shared between HID and Audio)
// ---------------------------------------------------------------------
static int gap_event_handler(struct ble_gap_event *event, void *arg);

static void start_advertising(void) {
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    fields.tx_pwr_lvl_is_present = 1;
    fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    // Wearable Audio Device / Headset appearance — closer to the combo
    // device's actual behavior than a plain HID gamepad value, since this
    // one advertisement now represents both roles. Worth validating against
    // a real host: whether one appearance value serves both profiles
    // acceptably, or whether the two need to be distinguished some other
    // way, is unverified.
    fields.appearance = ESP_BLE_AUDIO_APPEARANCE_WEARABLE_AUDIO_DEVICE_HEADSET;
    fields.appearance_is_present = 1;

    // Advertise both the HID-over-GATT service UUID and PACS, so host OS
    // Bluetooth stacks can recognize this as a combo HID+LE-Audio device.
    static const ble_uuid16_t adv_uuids[] = {
        BLE_UUID16_INIT(0x1812),  // HID
        BLE_UUID16_INIT(BT_UUID_PACS_VAL),
    };
    fields.uuids16 = (ble_uuid16_t*)adv_uuids;
    fields.num_uuids16 = 2;
    fields.uuids16_is_complete = 1;

    static const char name[] = "DualSense Edge";
    fields.name = (const uint8_t*)name;
    fields.name_len = sizeof(name) - 1;
    fields.name_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields failed rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params advp = {0};
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &advp, gap_event_handler, NULL);
    if (rc != 0) ESP_LOGE(TAG, "adv_start failed rc=%d", rc);
}

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ds_edge_hid_on_connect(event->connect.conn_handle);
        } else {
            ESP_LOGW(TAG, "BLE connect failed, status=%d", event->connect.status);
            start_advertising();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ds_edge_hid_on_disconnect();
        start_advertising();
        return 0;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU update: %d", event->mtu.value);
        return 0;
    default:
        return 0;
    }
}

static void on_sync(void) {
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    start_advertising();
}

static void on_reset(int reason) {
    ESP_LOGW(TAG, "BLE host reset, reason=%d", reason);
}

static void nimble_host_task(void *arg) {
    nimble_port_run();
    nimble_port_freertos_deinit();
}

// Mic (source ASE) data path: reads PCM from the codec, LC3-encodes it, and
// sends it over the CIS while the source stream is active. Idles (checking
// once per 10ms) the rest of the time. Runs for the lifetime of the app —
// simpler than tearing the task down between ASE enable/release cycles, and
// the idle-check cost is negligible.
static void mic_encode_task(void *arg) {
    static int16_t stereo_buf[960];
    static int16_t mono_buf[480];
    static uint8_t enc_out_buf[400];

    for (;;) {
        if (!s_source_streaming || !s_lc3_enc_handle) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        int in_size = 0, out_size = 0;
        if (esp_lc3_enc_get_frame_size(s_lc3_enc_handle, &in_size, &out_size) != ESP_AUDIO_ERR_OK ||
            in_size <= 0 || out_size <= 0 ||
            (size_t)in_size > sizeof(mono_buf) || (size_t)out_size > sizeof(enc_out_buf)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        size_t n_samples = (size_t)in_size / sizeof(int16_t);
        size_t stereo_bytes = n_samples * 2 * sizeof(int16_t);
        if (stereo_bytes > sizeof(stereo_buf)) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        size_t got = codec_read(stereo_buf, stereo_bytes);
        if (got < stereo_bytes) {
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }
        stereo_to_mono(stereo_buf, mono_buf, n_samples);

        esp_audio_enc_in_frame_t in_frame = {0};
        in_frame.buffer = (uint8_t*)mono_buf;
        in_frame.len = (uint32_t)in_size;
        esp_audio_enc_out_frame_t out_frame = {0};
        out_frame.buffer = enc_out_buf;
        out_frame.len = (uint32_t)out_size;

        if (esp_lc3_enc_process(s_lc3_enc_handle, &in_frame, &out_frame) == ESP_AUDIO_ERR_OK) {
            esp_ble_audio_bap_stream_send(&s_source_stream, enc_out_buf,
                                          (uint16_t)out_frame.encoded_bytes, s_source_seq_num++);
        }
    }
}

// ---------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------
void le_audio_bap_begin(void) {
    // BT controller must be up before esp_nimble_init()-based bring-up
    // (matches the pattern in espressif/esp_bt_audio's own reference
    // example, which calls this itself before esp_bt_audio_init()).
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_cfg));
    ESP_ERROR_CHECK(esp_bt_controller_enable(ESP_BT_MODE_BLE));

    esp_err_t err = esp_nimble_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_nimble_init failed: %d", err);
        return;
    }

    ble_hs_cfg.sync_cb = on_sync;
    ble_hs_cfg.reset_cb = on_reset;
    ble_hs_cfg.sm_bonding = 0;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 0;
    ble_hs_cfg.sm_our_key_dist = 0;
    ble_hs_cfg.sm_their_key_dist = 0;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ble_svc_dis_init();
    ble_svc_dis_manufacturer_name_set("Sony Interactive Entertainment");
    ble_svc_dis_model_number_set("DualSense Edge Wireless Controller");
    {
        uint8_t pnp[7] = {
            0x02, 0x4C, 0x05, 0xF2, 0x0D, 0x08, 0x04,  // Sony VID 0x054C, DS Edge PID 0x0DF2, ver 0x0408
        };
        ble_svc_dis_pnp_id_set((const char*)pnp);
    }
    ble_svc_bas_init();

    // PACS: bidirectional (sink = speaker, source = mic).
    esp_ble_audio_pacs_register_param_t pacs_param = {0};
    pacs_param.snk_pac = true;
    pacs_param.snk_loc = true;
    pacs_param.src_pac = true;
    pacs_param.src_loc = true;
    ESP_ERROR_CHECK(esp_ble_audio_pacs_register(&pacs_param));
    ESP_ERROR_CHECK(esp_ble_audio_pacs_cap_register(ESP_BLE_AUDIO_DIR_SINK, &s_pacs_cap_snk));
    ESP_ERROR_CHECK(esp_ble_audio_pacs_cap_register(ESP_BLE_AUDIO_DIR_SOURCE, &s_pacs_cap_src));
    ESP_ERROR_CHECK(esp_ble_audio_pacs_set_location(ESP_BLE_AUDIO_DIR_SINK, ESP_BLE_AUDIO_LOCATION_MONO_AUDIO));
    ESP_ERROR_CHECK(esp_ble_audio_pacs_set_location(ESP_BLE_AUDIO_DIR_SOURCE, ESP_BLE_AUDIO_LOCATION_MONO_AUDIO));
    ESP_ERROR_CHECK(esp_ble_audio_pacs_set_available_contexts(ESP_BLE_AUDIO_DIR_SINK, ESP_BLE_AUDIO_CONTEXT_TYPE_CONVERSATIONAL));
    ESP_ERROR_CHECK(esp_ble_audio_pacs_set_available_contexts(ESP_BLE_AUDIO_DIR_SOURCE, ESP_BLE_AUDIO_CONTEXT_TYPE_CONVERSATIONAL));

    // ASCS: 1 sink ASE + 1 source ASE.
    esp_ble_audio_bap_unicast_server_register_param_t ascs_param = {0};
    ascs_param.snk_cnt = 1;
    ascs_param.src_cnt = 1;
    ESP_ERROR_CHECK(esp_ble_audio_bap_unicast_server_register(&ascs_param));
    ESP_ERROR_CHECK(esp_ble_audio_bap_unicast_server_register_cb(&s_server_cb));
    ESP_ERROR_CHECK(esp_ble_audio_bap_stream_cb_register(&s_sink_stream, &s_stream_ops));
    ESP_ERROR_CHECK(esp_ble_audio_bap_stream_cb_register(&s_source_stream, &s_stream_ops));

    // HID service (ds_edge_hid.cpp) onto the same GATT server.
    const struct ble_gatt_svc_def *hid_svcs = ds_edge_hid_get_services();
    int rc = ble_gatts_count_cfg(hid_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "gatts_count_cfg (HID) failed rc=%d", rc); return; }
    rc = ble_gatts_add_svcs(hid_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "gatts_add_svcs (HID) failed rc=%d", rc); return; }

    ble_svc_gap_device_name_set("DualSense Edge");

    rc = ble_gatts_start();
    if (rc != 0) { ESP_LOGE(TAG, "gatts_start failed rc=%d", rc); return; }

    xTaskCreate(mic_encode_task, "mic_enc", 4096, NULL, 5, &s_mic_task_handle);

    nimble_port_freertos_init(nimble_host_task);
}
