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
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/ble_gap.h"
#include "host/ble_gatt.h"
#include "host/ble_store.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "services/bas/ble_svc_bas.h"
#include "services/dis/ble_svc_dis.h"

#include "esp_ble_audio_defs.h"
#include "esp_ble_audio_common_api.h"
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
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"

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
extern void ds_edge_hid_on_subscribe(uint16_t attr_handle, bool notify_enabled);

// From le_audio_codec.cpp. Declared locally rather than via #include
// "le_audio_codec.h" because that header declares codec_init() with a C++
// default argument.
extern bool codec_set_sample_rate(uint32_t sample_rate_hz);
extern size_t codec_write(const int16_t *samples, size_t byte_len);
extern size_t codec_read(int16_t *samples, size_t byte_len);

static const char *TAG = "le_audio_bap";

// Real hardware bring-up (2026-08-15): PACS/ASCS registration temporarily
// disabled. LE Audio can't stream on this dev machine's kernel yet
// regardless (bluetoothd: "BAP requires ISO Socket which is not enabled" —
// a Linux kernel capability, not something this firmware or BlueZ config
// controls), while ASCS's ASE state notifications were confirmed (via a raw
// ATT probe, bypassing BlueZ) to sometimes interleave with BlueZ's
// multi-part Report Map read right after connecting, an interleaving
// BlueZ's own reader isn't robust to — this intermittently truncated what
// the kernel's HID parser received, breaking gamepad recognition on an
// unpredictable, timing-dependent basis. Since the audio side is already
// non-functional here for an unrelated reason, disabling it removes that
// interference and makes gamepad recognition reliable instead of racy.
// A `static const bool` (not #if) so the callback structs/functions below
// stay referenced and compiled either way — flipping this back to true is
// the only change needed to re-enable LE Audio once ISO socket support is
// confirmed available.
static const bool LE_AUDIO_ASCS_ENABLED = false;

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
    // Real hardware bring-up (2026-08-15): flags + appearance + tx_pwr_lvl +
    // 2 service UUIDs + the full device name together exceeded legacy
    // advertising's 31-byte payload limit (ble_gap_adv_set_fields() failed
    // with rc=4 / BLE_HS_EMSGSIZE — confirmed on real hardware, never
    // caught before since this is the first time this code actually ran on
    // a device). Split across the primary advertisement (compact identity:
    // flags + both service UUIDs, ~9 bytes) and the scan response
    // (appearance + tx power + name, ~23 bytes) — both comfortably under 31
    // bytes on their own, standard NimBLE pattern for this exact problem.
    struct ble_hs_adv_fields fields = {0};
    fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;

    // Advertise the HID-over-GATT service UUID. PACS is left out of both
    // the advertisement and (see LE_AUDIO_ASCS_ENABLED) actual registration
    // for now — advertising a service this device isn't currently serving
    // would be actively misleading, not just incomplete.
    static const ble_uuid16_t adv_uuids[] = {
        BLE_UUID16_INIT(0x1812),  // HID
    };
    fields.uuids16 = (ble_uuid16_t*)adv_uuids;
    fields.num_uuids16 = 1;
    fields.uuids16_is_complete = 1;

    int rc = ble_gap_adv_set_fields(&fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_set_fields failed rc=%d", rc);
        return;
    }

    struct ble_hs_adv_fields rsp_fields = {0};
    rsp_fields.tx_pwr_lvl_is_present = 1;
    rsp_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    // Real hardware bring-up (2026-08-15): the Wearable Audio Device/Headset
    // appearance this used to advertise (on the theory that it's "closer to
    // the combo device's actual behavior") tested badly against a real host
    // — bonded via BlueZ (Paired/Bonded/Trusted all yes), but no uhid input
    // device was ever created, even after a fresh reconnect while already
    // bonded+trusted. Switched to the HID/Gamepad category (0x03C4, Bluetooth
    // SIG assigned numbers) since that's what a HOGP-consuming host's input
    // plugin is actually looking for; this device's audio role is still
    // fully discoverable via the advertised PACS service UUID regardless of
    // the appearance category chosen for the icon/HID-recognition role.
    rsp_fields.appearance = 0x03C4;
    rsp_fields.appearance_is_present = 1;

    static const char name[] = "DualSense Edge";
    rsp_fields.name = (const uint8_t*)name;
    rsp_fields.name_len = sizeof(name) - 1;
    rsp_fields.name_is_complete = 1;

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0) {
        ESP_LOGE(TAG, "adv_rsp_set_fields failed rc=%d", rc);
        return;
    }

    struct ble_gap_adv_params advp = {0};
    advp.conn_mode = BLE_GAP_CONN_MODE_UND;
    advp.disc_mode = BLE_GAP_DISC_MODE_GEN;
    rc = ble_gap_adv_start(s_own_addr_type, NULL, BLE_HS_FOREVER, &advp, gap_event_handler, NULL);
    if (rc != 0) ESP_LOGE(TAG, "adv_start failed rc=%d", rc);
    else ESP_LOGI(TAG, "advertising started");
}

// Real hardware bring-up (2026-08-15): sm_bonding=1 alone only makes
// bonding *possible* — nothing actually triggers it, since none of the
// HID/PACS characteristics require encryption (so the central never sees
// an "insufficient authentication" ATT error to react to) and NimBLE
// peripherals don't initiate security on their own. Real BLE HID
// peripherals proactively send a Security Request right after connecting;
// do the same here so BlueZ's HOGP/BAP host-side plugins — which both
// require a bonded link before they'll create a uhid device or wire up
// audio — actually get one.
//
// Calling ble_gap_security_initiate() synchronously from inside the
// BLE_GAP_EVENT_CONNECT callback itself reliably failed with
// BLE_HS_EALREADY ("security procedure already in progress") on real
// hardware, even against a from-scratch central with no prior bond at
// all — confirmed by erasing the NVS bond store and retrying. The NimBLE
// reference examples (bleprph) never call this from inside the connect
// callback either. Deferring it to a short-lived task a moment after the
// callback returns avoids whatever internal state the host is still
// settling at that exact point.
static void security_initiate_task(void *arg) {
    uint16_t conn_handle = (uint16_t)(uintptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(200));
    ESP_LOGI(TAG, "free heap before security_initiate: %lu", (unsigned long)esp_get_free_heap_size());
    int sec_rc = ble_gap_security_initiate(conn_handle);
    if (sec_rc != 0) {
        ESP_LOGW(TAG, "security_initiate failed rc=%d", sec_rc);
    }
    vTaskDelete(NULL);
}

static int gap_event_handler(struct ble_gap_event *event, void *arg) {
    switch (event->type) {
    case BLE_GAP_EVENT_CONNECT:
        if (event->connect.status == 0) {
            ds_edge_hid_on_connect(event->connect.conn_handle);
            xTaskCreate(security_initiate_task, "sec_init", 3072,
                        (void*)(uintptr_t)event->connect.conn_handle, tskIDLE_PRIORITY + 1, NULL);
        } else {
            ESP_LOGW(TAG, "BLE connect failed, status=%d", event->connect.status);
            start_advertising();
        }
        return 0;
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGI(TAG, "disconnect reason=%d (0x%x)", event->disconnect.reason, event->disconnect.reason);
        // Real hardware bring-up (2026-08-15): reproduced by hand, repeatedly
        // — if the central "forgets" this device (re-pairing, or its own
        // bond store getting cleared) without this peripheral's NVS-
        // persisted bond also being cleared, the *link layer itself* tries
        // to resume encryption with the now one-sided stored LTK and the
        // controller tears the connection down immediately (~130ms, before
        // any GATT/SMP traffic) with HCI error 0x05 (BLE_ERR_AUTH_FAIL,
        // wrapped by NimBLE as BLE_HS_HCI_ERR(BLE_ERR_AUTH_FAIL) = 0x205).
        // This happens entirely below the SM host layer — neither
        // BLE_GAP_EVENT_ENC_CHANGE nor BLE_GAP_EVENT_REPEAT_PAIRING ever
        // fire for it (confirmed: both are otherwise-correct handlers for
        // the *host*-level version of this same problem, added earlier, but
        // neither one is reachable from this specific failure path) — so
        // without this check, every future pairing attempt repeats the
        // identical failure forever, until someone manually erases this
        // device's NVS partition. Deleting our half of the stale bond here
        // lets the very next attempt (start_advertising() below already
        // restarts advertising for it) start clean instead.
        if (event->disconnect.reason == BLE_HS_HCI_ERR(BLE_ERR_AUTH_FAIL)) {
            ble_store_util_delete_peer(&event->disconnect.conn.peer_id_addr);
        }
        ds_edge_hid_on_disconnect();
        start_advertising();
        return 0;
    case BLE_GAP_EVENT_MTU:
        ESP_LOGI(TAG, "MTU update: %d", event->mtu.value);
        return 0;
    case BLE_GAP_EVENT_SUBSCRIBE:
        ds_edge_hid_on_subscribe(event->subscribe.attr_handle, event->subscribe.cur_notify);
        return 0;
    case BLE_GAP_EVENT_ENC_CHANGE:
        ESP_LOGI(TAG, "encryption change: conn=%d status=%d free_heap=%lu",
                 event->enc_change.conn_handle, event->enc_change.status,
                 (unsigned long)esp_get_free_heap_size());
        // Real hardware bring-up (2026-08-15): reproduced by hand — if the
        // central "forgets"/removes this device on its side (e.g. the user
        // re-pairs, or the host's own bond store gets cleared) without this
        // peripheral's NVS-persisted bond also being cleared, NimBLE
        // recognizes the reconnecting peer and tries to resume encryption
        // with the now-one-sided stored LTK. The central rejects it and the
        // link drops almost immediately (~130ms, before any GATT traffic),
        // which without this fix repeats identically forever — every future
        // pairing attempt fails silently until someone manually erases this
        // device's NVS partition. Deleting our half of the stale bond on
        // failure lets the very next attempt start clean instead.
        if (event->enc_change.status != 0) {
            struct ble_gap_conn_desc desc;
            if (ble_gap_conn_find(event->enc_change.conn_handle, &desc) == 0) {
                ble_store_util_delete_peer(&desc.peer_id_addr);
            }
        }
        return 0;
    case BLE_GAP_EVENT_REPEAT_PAIRING: {
        // We already hold a bond for this peer, but it's attempting a fresh
        // pairing (e.g. the peer forgot/removed its side). Sacrifice the old
        // bond for convenience rather than reject the new pairing outright.
        ESP_LOGI(TAG, "repeat pairing: conn=%d", event->repeat_pairing.conn_handle);
        struct ble_gap_conn_desc desc;
        int rc = ble_gap_conn_find(event->repeat_pairing.conn_handle, &desc);
        if (rc == 0) {
            int64_t t0 = esp_timer_get_time();
            ble_store_util_delete_peer(&desc.peer_id_addr);
            ESP_LOGI(TAG, "repeat pairing: delete_peer took %lld us", esp_timer_get_time() - t0);
        }
        return BLE_GAP_REPEAT_PAIRING_RETRY;
    }
    default:
        return 0;
    }
}

// Real hardware bring-up (2026-08-15): esp_ble_audio_common_init() issues an
// HCI command internally and fails with BLE_HS_ENOTSYNCED — confirmed via
// the exact NimBLE log line at the crash site — if called before the
// NimBLE host has synced with the controller. That only happens once
// nimble_port_freertos_init()'s host task is actually running and
// processing events, i.e. strictly after le_audio_bap_begin() returns. So
// unlike the "register all GATT services, then start the host" order this
// file used against the older (pioarduino, April) IDF snapshot — which
// worked fine there — esp_ble_audio's PACS/ASCS registration (and
// everything that depends on it: the HID service sharing the same
// ble_gatts_start() call) now has to be deferred to run from inside
// on_sync() instead. Guarded by s_services_registered since sync_cb can
// fire again after a host reset/resync.
static bool s_services_registered = false;
static void mic_encode_task(void *arg);

static void register_services_and_start(void) {
    if (s_services_registered) return;
    s_services_registered = true;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    ESP_ERROR_CHECK(esp_ble_audio_common_init(NULL));

    ble_svc_dis_init();
    ble_svc_dis_manufacturer_name_set("Sony Interactive Entertainment");
    ble_svc_dis_model_number_set("DualSense Edge Wireless Controller");
    // Real hardware bring-up (2026-08-15): ble_svc_dis_pnp_id_set() only
    // stores the POINTER it's given (ble_svc_dis.c: `ble_svc_dis_data.pnp_id
    // = value;`), not a copy — it re-reads through that pointer later,
    // whenever a real central actually requests the PnP ID characteristic.
    // The array here used to be a local declared in a scope block that
    // closed on the very next line, so by the time any client read it, this
    // was a dangling pointer into long-since-reused stack memory — BlueZ
    // read back garbage (VID=0x0309/PID=0x7E2F, not Sony's real
    // 0x054C:0x0DF2) instead of a crash, since nothing else caught the
    // invalid access. `static` gives it program-lifetime storage instead.
    static const uint8_t s_pnp_id[7] = {
        0x02, 0x4C, 0x05, 0xF2, 0x0D, 0x08, 0x04,  // Sony VID 0x054C, DS Edge PID 0x0DF2, ver 0x0408
    };
    ble_svc_dis_pnp_id_set((const char*)s_pnp_id);
    ble_svc_bas_init();

    if (LE_AUDIO_ASCS_ENABLED) {
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
        // Real hardware bring-up (2026-08-15): "available" contexts must be a
        // subset of "supported" contexts — the blob's PACS validation rejected
        // set_available_contexts() with "PacsInvSuppMask" because supported
        // contexts defaults to empty and was never explicitly set.
        ESP_ERROR_CHECK(esp_ble_audio_pacs_set_supported_contexts(ESP_BLE_AUDIO_DIR_SINK, ESP_BLE_AUDIO_CONTEXT_TYPE_CONVERSATIONAL));
        ESP_ERROR_CHECK(esp_ble_audio_pacs_set_supported_contexts(ESP_BLE_AUDIO_DIR_SOURCE, ESP_BLE_AUDIO_CONTEXT_TYPE_CONVERSATIONAL));
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
    }

    // HID service (ds_edge_hid.cpp) onto the same GATT server.
    const struct ble_gatt_svc_def *hid_svcs = ds_edge_hid_get_services();
    int rc = ble_gatts_count_cfg(hid_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "gatts_count_cfg (HID) failed rc=%d", rc); return; }
    rc = ble_gatts_add_svcs(hid_svcs);
    if (rc != 0) { ESP_LOGE(TAG, "gatts_add_svcs (HID) failed rc=%d", rc); return; }

    ble_svc_gap_device_name_set("DualSense Edge");

    // Real hardware bring-up (2026-08-15): esp_ble_audio has a two-phase
    // init/start API — esp_ble_audio_common_init() (called earlier, in
    // le_audio_bap_begin()) only initializes the GAP/GATT app-event
    // plumbing; esp_ble_audio_common_start() is the actual "start BLE Audio
    // services" call, and its NimBLE adapter (host/adapter/nimble/init.c's
    // bt_le_nimble_audio_start(), confirmed by reading the adapter source
    // directly — esp_ble_audio itself is a closed-source .a blob, but this
    // adapter layer around it is not) is what actually calls NimBLE's
    // ble_gatts_start() internally. This code was calling our own
    // ble_gatts_start() directly instead and never calling
    // esp_ble_audio_common_start() at all — which happened to still report
    // rc=0 and still populate ble_gatts_show_local()'s view correctly, but
    // a real GATT client's "Discover All Primary Services" (ATT Read By
    // Group Type, UUID 0x2800) against the device returned ATT error 0x0A
    // (Attribute Not Found) across the *entire* handle range regardless —
    // confirmed with a raw ATT probe bypassing BlueZ/NimBLE's higher-level
    // caching entirely. This is *the* root cause of gamepad-tester sites
    // never recognizing the device and LE Audio never showing up as a
    // Linux audio device: nothing could ever discover our services at all.
    // Calling the documented esp_ble_audio_common_start() entry point
    // instead — after all our own ble_gatts_add_svcs() calls, matching its
    // own internal comment ("Register GMAS before ble_gatts_start") — lets
    // it drive ble_gatts_start() itself in the sequence/context it
    // actually expects.
    ESP_ERROR_CHECK(esp_ble_audio_common_start(NULL));

    if (LE_AUDIO_ASCS_ENABLED) {
        xTaskCreate(mic_encode_task, "mic_enc", 4096, NULL, 5, &s_mic_task_handle);
    }
}

static void on_sync(void) {
    ble_hs_id_infer_auto(0, &s_own_addr_type);
    register_services_and_start();
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
    // Real hardware bring-up (2026-08-15): the PHY calibration data the BT
    // controller loads on init lives in NVS. Without initializing it first,
    // esp_phy_load_cal_data_from_nvs() logs a warning and falls back to full
    // calibration, but the *following* step then hits an unguarded null
    // dereference (Guru Meditation / Load access fault) inside phy_init —
    // this crash is what "Real hardware bring-up" in BUILD.md previously
    // reported, before the actual cause was traced here. Standard ESP-IDF
    // boilerplate (nvs_flash_init(), with the erase-and-retry fallback for a
    // truncated/incompatible partition) fixes it.
    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(nvs_err);

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
    // Real hardware bring-up (2026-08-15): the previous sm_bonding=0 "simpler
    // first pass" (see ds_edge_hid.cpp's now-resolved KNOWN GAP comment)
    // turned out to be why nothing worked against a real host — BlueZ's HOGP
    // input plugin and its LE Audio/BAP plugin both require a bonded link
    // before they'll create a uhid device / wire up an audio device; an
    // unbonded GATT connection (which is all sm_bonding=0 ever produced) was
    // silently insufficient for either, despite the GAP connection itself
    // succeeding. This device has no display/keyboard, so Just Works
    // (sm_mitm=0) is the only pairing method available — LE Secure
    // Connections (sm_sc=1) is used since CONFIG_BT_NIMBLE_SM_SC=y already
    // compiles it in and it's the modern preferred method.
    ble_hs_cfg.sm_bonding = 1;
    ble_hs_cfg.sm_mitm = 0;
    ble_hs_cfg.sm_sc = 1;
    ble_hs_cfg.sm_our_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;
    ble_hs_cfg.sm_their_key_dist = BLE_SM_PAIR_KEY_DIST_ENC | BLE_SM_PAIR_KEY_DIST_ID;

    // Real hardware bring-up (2026-08-15): ble_svc_gap_init()/
    // ble_svc_gatt_init() used to run right here, before the host task
    // starts — and NimBLE's own host startup (ble_hs.c's ble_hs_start(),
    // called automatically once the host syncs with the controller) calls
    // ble_gatts_start() itself for whatever's registered by that point.
    // That's standard NimBLE behavior, but esp_ble_audio_common_init() and
    // everything downstream of it (PACS/ASCS/HID registration,
    // esp_ble_audio_common_start() -> ble_gatts_start()) has to be deferred
    // to register_services_and_start(), called from on_sync() — needs the
    // host already synced, confirmed via a real BLE_HS_ENOTSYNCED error
    // otherwise. In NimBLE's default "static" service mode, every
    // ble_gatts_start() call unconditionally frees and rebuilds the *entire*
    // live ATT attribute table from scratch (confirmed by instrumenting
    // ble_att_svr_start()/ble_att_svr_register() directly and capturing
    // real boot output — this is not documented anywhere obvious), so that
    // automatic first call (registering just GAP+GATT) was being silently
    // wiped out and orphaned by the second, real one moments later. This
    // was the actual root cause of gamepad-tester sites and Linux's
    // Bluetooth stack alike never being able to discover *any* of this
    // device's services — confirmed with a raw ATT "Discover All Primary
    // Services" probe returning ATT error 0x0A (Attribute Not Found) across
    // the entire handle range, despite every individual registration call
    // reporting success. Moving GAP/GATT registration into
    // register_services_and_start() too means nothing is registered before
    // the host's automatic first ble_gatts_start() call runs, so that call
    // has nothing meaningful to lose — everything (GAP+GATT+DIS+BAS+PACS+
    // ASCS+HID) ends up registered together in the one call that actually
    // matters.
    nimble_port_freertos_init(nimble_host_task);
}
