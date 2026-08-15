#include "sc_input.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/portmacro.h"
#include "usb/usb_host.h"
#include "usb/usb_types_ch9.h"
#include "esp_private/usb_phy.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <string.h>
#include "startup_audio.h"

static const char *TAG = "sc_input";

static constexpr uint8_t SC_HID_IFACE = 0;
static constexpr size_t  SC_BUF_SIZE  = 64;

static SemaphoreHandle_t        s_host_ready = nullptr;
static usb_host_client_handle_t s_client     = nullptr;
static usb_device_handle_t      s_dev        = nullptr;
static usb_transfer_t*          s_xfer       = nullptr;
static uint8_t                  s_ep_in      = 0;
static uint8_t                  s_ep_out     = 0;

// Shared between USB host task (writer) and app_main task (reader).
// portMUX spinlock gives cross-core mutual exclusion without a semaphore.
static portMUX_TYPE  s_report_mux    = portMUX_INITIALIZER_UNLOCKED;
static bool          s_report_ready  = false;
static bool          s_touch_ready   = false;
static SCReport      s_report{};
static SCTouchReport s_touch_report{};

// Rumble output — control transfer to SC, serialised by s_ctrl_mux.
static constexpr size_t SC_HAPTIC_PAYLOAD = 9;  // bytes after report ID for 0x81/0x83
static usb_transfer_t*  s_ctrl_xfer    = nullptr;
static portMUX_TYPE     s_ctrl_mux     = portMUX_INITIALIZER_UNLOCKED;
static bool             s_ctrl_busy    = false;
static uint8_t          s_rumble_left      = 0;
static uint8_t          s_rumble_right     = 0;
static uint8_t          s_sent_left        = 0xFF;   // 0xFF forces first-send after connect
static uint8_t          s_sent_right       = 0xFF;
static uint8_t          s_tone_left        = 0;
static uint8_t          s_tone_right       = 0;
static uint8_t          s_sent_tone_left   = 0;
static uint8_t          s_sent_tone_right  = 0;

// PCM streaming for startup audio via SC touchpad haptic actuators (CH_LPAD / CH_RPAD).
// s_pcm_xfer is allocated once on first connect and never freed to avoid disconnect races.
// s_pcm_playing suppresses rumble_pump() for the duration so the rumble motors don't
// compete with the touchpad haptics for the shared OUT endpoint.
static usb_transfer_t*   s_pcm_xfer     = nullptr;
static SemaphoreHandle_t s_pcm_sem      = nullptr;
static TaskHandle_t      s_pcm_task_hdl = nullptr;
static volatile bool     s_pcm_playing  = false;

// Haptic audio streaming from DualSense host → SC touchpad actuators.
// Packets are queued from the BLE callback (non-blocking) and drained by a
// dedicated FreeRTOS task that calls pcm_send_raw() sequentially.
struct SCPcmPacket { uint8_t buf[64]; };
static constexpr int    PCM_HAP_QUEUE_DEPTH = 8;
static QueueHandle_t    s_pcm_hap_queue     = nullptr;
static TaskHandle_t     s_pcm_drain_hdl     = nullptr;

static void transfer_cb(usb_transfer_t* xfer) {
    if (xfer->status != USB_TRANSFER_STATUS_COMPLETED) {
        ESP_LOGW(TAG, "xfer status=%d", xfer->status);
        usb_host_transfer_submit(xfer);
        return;
    }
    if (xfer->actual_num_bytes < 1) {
        usb_host_transfer_submit(xfer);
        return;
    }

    const uint8_t id     = xfer->data_buffer[0];
    const int     nbytes = xfer->actual_num_bytes;

    // 0x40: short capacitive-touch event (6 bytes)
    // 0x43: hardware-info / pre-game-mode status report (15 bytes) — known, no action needed
    if (id == SC_REPORT_ID_TOUCH) {
        if (nbytes >= (int)SC_REPORT_TOUCH_SIZE) {
            const SCTouchReport* touch = (const SCTouchReport*)xfer->data_buffer;
            taskENTER_CRITICAL(&s_report_mux);
            memcpy(&s_touch_report, touch, sizeof(SCTouchReport));
            s_touch_ready = true;
            taskEXIT_CRITICAL(&s_report_mux);
        } else {
            ESP_LOGW(TAG, "touch: short packet len=%d", nbytes);
        }
        usb_host_transfer_submit(xfer);
        return;
    }
    if (id == SC_REPORT_ID_HW_INFO || id == SC_REPORT_ID_BTN_SEQ) {
        usb_host_transfer_submit(xfer);
        return;
    }

    const bool matched = (id == SC_REPORT_ID && nbytes >= (int)sizeof(SCReport));

    if (matched) {
        taskENTER_CRITICAL(&s_report_mux);
        memcpy(&s_report, xfer->data_buffer, sizeof(SCReport));
        s_report_ready = true;
        taskEXIT_CRITICAL(&s_report_mux);
    }
    usb_host_transfer_submit(xfer);  // re-arm
}

static void rumble_pump();
static void sc_pcm_init_task(void*);

static void ctrl_cb(usb_transfer_t*) {
    taskENTER_CRITICAL(&s_ctrl_mux);
    s_ctrl_busy = false;
    taskEXIT_CRITICAL(&s_ctrl_mux);
    if (s_dev) rumble_pump();
}

static void rumble_pump() {
    taskENTER_CRITICAL(&s_ctrl_mux);
    if (s_ctrl_busy || !s_dev || !s_ctrl_xfer || s_pcm_playing) {
        taskEXIT_CRITICAL(&s_ctrl_mux);
        return;
    }
    uint8_t channel, amplitude;
    uint16_t freq;
    bool send = false;
    if (s_rumble_left != s_sent_left) {
        channel = SC_CH_LRUMBLE; amplitude = s_rumble_left;  freq = 100;
        s_sent_left = amplitude; send = true;
    } else if (s_rumble_right != s_sent_right) {
        channel = SC_CH_RRUMBLE; amplitude = s_rumble_right; freq = 300;
        s_sent_right = amplitude; send = true;
    } else if (s_tone_left != s_sent_tone_left) {
        channel = SC_CH_LPAD; amplitude = s_tone_left; freq = 200;
        s_sent_tone_left = amplitude; send = true;
    } else if (s_tone_right != s_sent_tone_right) {
        channel = SC_CH_RPAD; amplitude = s_tone_right; freq = 200;
        s_sent_tone_right = amplitude; send = true;
    }
    if (!send) { taskEXIT_CRITICAL(&s_ctrl_mux); return; }
    s_ctrl_busy = true;
    const uint8_t ep_out = s_ep_out;  // snapshot under lock
    taskEXIT_CRITICAL(&s_ctrl_mux);

    const uint8_t cmd = amplitude ? SC_REPORT_ID_HAPTIC_PLAY : SC_REPORT_ID_HAPTIC_STOP;
    esp_err_t err;

    if (ep_out) {
        // Interrupt OUT: report ID is first byte (matches HIDAPI hid_write format).
        // Stop command (0x81) must have zero payload after channel; play (0x83) carries params.
        uint8_t data[10] = { cmd, channel };
        if (amplitude) {
            data[2] = amplitude;
            data[3] = (uint8_t)(freq & 0xFF);
            data[4] = (uint8_t)(freq >> 8);
            data[5] = 0xFF;
            data[6] = 0xFF;
        }
        memcpy(s_ctrl_xfer->data_buffer, data, sizeof(data));
        s_ctrl_xfer->num_bytes        = sizeof(data);
        s_ctrl_xfer->device_handle    = s_dev;
        s_ctrl_xfer->bEndpointAddress = ep_out;
        s_ctrl_xfer->callback         = ctrl_cb;
        s_ctrl_xfer->context          = nullptr;
        err = usb_host_transfer_submit(s_ctrl_xfer);
    } else {
        // HID SET_REPORT fallback: report ID in wValue, payload without it
        uint8_t data[SC_HAPTIC_PAYLOAD] = { channel };
        if (amplitude) {
            data[1] = amplitude;
            data[2] = (uint8_t)(freq & 0xFF);
            data[3] = (uint8_t)(freq >> 8);
            data[4] = 0xFF;
            data[5] = 0xFF;
        }
        auto* setup          = (usb_setup_packet_t*)s_ctrl_xfer->data_buffer;
        setup->bmRequestType = 0x21;
        setup->bRequest      = 0x09;
        setup->wValue        = (uint16_t)((0x02u << 8) | cmd);
        setup->wIndex        = SC_HID_IFACE;
        setup->wLength       = SC_HAPTIC_PAYLOAD;
        memcpy(s_ctrl_xfer->data_buffer + sizeof(usb_setup_packet_t), data, sizeof(data));
        s_ctrl_xfer->num_bytes        = sizeof(usb_setup_packet_t) + sizeof(data);
        s_ctrl_xfer->device_handle    = s_dev;
        s_ctrl_xfer->bEndpointAddress = 0;
        s_ctrl_xfer->callback         = ctrl_cb;
        s_ctrl_xfer->context          = nullptr;
        err = usb_host_transfer_submit_control(s_client, s_ctrl_xfer);
    }

    if (err != ESP_OK) {
        taskENTER_CRITICAL(&s_ctrl_mux);
        s_ctrl_busy = false;
        taskEXIT_CRITICAL(&s_ctrl_mux);
    }
}

static void pcm_xfer_cb(usb_transfer_t*) {
    xSemaphoreGive(s_pcm_sem);
}

// Send buf over the interrupt OUT endpoint, serialised with the rumble channel via s_ctrl_busy.
// Blocks the calling task until the transfer completes or times out.
static bool pcm_send_raw(const uint8_t* buf, size_t len) {
    if (!s_pcm_xfer || !s_dev || !s_ep_out) return false;
    for (int retries = 50; retries > 0; retries--) {
        taskENTER_CRITICAL(&s_ctrl_mux);
        if (!s_ctrl_busy) { s_ctrl_busy = true; taskEXIT_CRITICAL(&s_ctrl_mux); break; }
        taskEXIT_CRITICAL(&s_ctrl_mux);
        if (retries == 1) return false;
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    xSemaphoreTake(s_pcm_sem, 0);
    memcpy(s_pcm_xfer->data_buffer, buf, len);
    s_pcm_xfer->num_bytes        = len;
    s_pcm_xfer->device_handle    = s_dev;
    s_pcm_xfer->bEndpointAddress = s_ep_out;
    s_pcm_xfer->callback         = pcm_xfer_cb;
    s_pcm_xfer->context          = nullptr;
    const esp_err_t err = usb_host_transfer_submit(s_pcm_xfer);
    if (err != ESP_OK) {
        taskENTER_CRITICAL(&s_ctrl_mux);
        s_ctrl_busy = false;
        taskEXIT_CRITICAL(&s_ctrl_mux);
        return false;
    }
    const bool ok = xSemaphoreTake(s_pcm_sem, pdMS_TO_TICKS(500)) == pdTRUE;
    taskENTER_CRITICAL(&s_ctrl_mux);
    s_ctrl_busy = false;
    taskEXIT_CRITICAL(&s_ctrl_mux);
    return ok;
}

// Put the SC haptic engine into PCM streaming mode for CH_LPAD and CH_RPAD.
//
// Protocol (reversed from SteamHapticsPlayer / hardware capture):
//   For each channel × param combo, send one 0x86 init packet then a burst of 0x88 test
//   packets. The SC uses the test burst to calibrate its internal PCM buffer; skipping it
//   or shortening it causes audible distortion on the first real audio packets.
//
// Timing (empirically tuned on this hardware):
//   75 ms after 0x86 — minimum settling time before the channel accepts 0x88 data cleanly.
//     50 ms produced distortion; 75 ms is the lowest value that sounds correct.
//   7 test packets × 4 ms — minimum burst length for reliable calibration.
//     5 packets was insufficient; 7 is the practical floor.
//   Total setup time: 2 ch × 9 params × (75 ms + 7×4 ms) ≈ 1.85 s, plus 200 ms settling.
//
// SteamHapticsPlayer reference uses 100 ms / 10 packets (confirmed clean but slower).
// All 9 param values must be swept even though only one will be active during playback —
// the SC firmware appears to require the full calibration sweep on every power-on.
static void pcm_setup() {
    static const uint8_t channels[] = {SC_CH_LPAD, SC_CH_RPAD};
    static const uint8_t params[]   = {0, 1, 2, 4, 8, 16, 32, 64, 128};

    // Alternating 0xFF/0x00 blocks — test pattern used by SteamHapticsPlayer.
    uint8_t test[64] = {};
    test[0] = SC_REPORT_ID_PCM_DATA;
    test[1] = 31;
    for (int i = 0; i < 31; i++) {
        uint8_t s = ((i / 4) & 1) ? 0xFF : 0x00;
        test[2 + i]  = s;
        test[33 + i] = s;
    }

    for (uint8_t ch : channels) {
        for (uint8_t p : params) {
            if (!s_dev) return;
            uint8_t enable[4] = {SC_REPORT_ID_PCM_INIT, 0x02, ch, p};
            pcm_send_raw(enable, sizeof(enable));
            vTaskDelay(pdMS_TO_TICKS(75));
            for (int r = 0; r < 7 && s_dev; r++) {
                pcm_send_raw(test, sizeof(test));
                vTaskDelay(pdMS_TO_TICKS(4));
            }
        }
    }
}

// Stream startup_audio[] to CH_LPAD / CH_RPAD as 8 kHz signed-8-bit stereo PCM.
//
// Packet format (0x88): [id=0x88, count=31, L0..L30, R0..R30] — 64 bytes total.
// startup_audio is interleaved stereo (L0, R0, L1, R1, …); we deinterleave on the fly.
//
// Pacing: each packet carries 31 samples at 8 kHz → 3875 µs per packet.
// We use a coarse vTaskDelay for most of the interval and a µs busy-wait tail
// (esp_timer_get_time) for the remainder, matching SteamHapticsPlayer's approach.
//
// The loop starts one chunk into the audio (off = SPP) to mirror SteamHapticsPlayer's
// initial read-and-discard, which absorbs any residual latency from pcm_setup().
// The 200 ms pause between pcm_setup() and the loop gives the SC an additional settling
// window; removing it caused a brief buzz at the start of playback.
static void pcm_stream_task(void*) {
    constexpr size_t  SPP       = 31;
    constexpr int64_t PERIOD_US = (SPP * 1000000LL) / 8000;  // 3875 µs

    s_pcm_playing = true;
    pcm_setup();
    vTaskDelay(pdMS_TO_TICKS(200));

    const size_t stereo_pairs = startup_audio_len / 2;
    uint8_t pkt[64];

    int64_t next_us = esp_timer_get_time();
    for (size_t off = SPP; off < stereo_pairs && s_dev; off += SPP) {
        const size_t n = (stereo_pairs - off < SPP) ? (stereo_pairs - off) : SPP;
        pkt[0] = SC_REPORT_ID_PCM_DATA;
        pkt[1] = (uint8_t)n;
        for (size_t i = 0; i < SPP; i++) {
            const size_t si = off + (i < n ? i : n - 1);
            pkt[2        + i] = (uint8_t)startup_audio[si * 2];
            pkt[2 + SPP + i] = (uint8_t)startup_audio[si * 2 + 1];
        }
        if (!pcm_send_raw(pkt, sizeof(pkt))) break;

        next_us += PERIOD_US;
        const int64_t slack_us = next_us - esp_timer_get_time();
        if (slack_us > 1000)
            vTaskDelay(pdMS_TO_TICKS((slack_us - 500) / 1000));
        while (esp_timer_get_time() < next_us) {}
    }

    s_pcm_playing = false;
    // Force rumble resync: any sc_rumble() calls that arrived during playback were dropped,
    // so reset the "last sent" state to guarantee the next rumble_pump() re-sends them.
    taskENTER_CRITICAL(&s_ctrl_mux);
    s_sent_left = s_sent_right = 0xFF;
    taskEXIT_CRITICAL(&s_ctrl_mux);
    if (s_dev) rumble_pump();

    s_pcm_task_hdl = nullptr;
    vTaskDelete(nullptr);
}

static void channel_test_task(void*) {
    for (uint8_t ch = 0; ch < 6 && s_dev; ch++) {
        ESP_LOGI(TAG, "channel test: playing ch=%u", ch);
        uint8_t play[10] = {SC_REPORT_ID_HAPTIC_PLAY, ch, 180, 200 & 0xFF, 200 >> 8, 0xFF, 0xFF};
        pcm_send_raw(play, sizeof(play));
        vTaskDelay(pdMS_TO_TICKS(2000));
        uint8_t stop[10] = {SC_REPORT_ID_HAPTIC_STOP, ch};
        pcm_send_raw(stop, sizeof(stop));
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    ESP_LOGI(TAG, "channel test done");
    s_pcm_task_hdl = nullptr;
    vTaskDelete(nullptr);
}

void sc_channel_test_start() {
    if (!s_ep_out || !s_pcm_xfer || s_pcm_task_hdl) return;
    if (!s_pcm_sem) s_pcm_sem = xSemaphoreCreateBinary();
    xTaskCreatePinnedToCore(channel_test_task, "ch_test", 4096, nullptr, 3, &s_pcm_task_hdl, 1);
}

static void sc_startup_play() {
    if (!s_ep_out || !s_pcm_xfer || s_pcm_task_hdl) return;
    xTaskCreatePinnedToCore(pcm_stream_task, "pcm_stream", 4096, nullptr, 3, &s_pcm_task_hdl, 1);
}

static bool find_ep(usb_device_handle_t dev, uint8_t iface_num, bool want_in, uint8_t* ep_addr) {
    const usb_config_desc_t* cfg;
    if (usb_host_get_active_config_descriptor(dev, &cfg) != ESP_OK) return false;
    const uint8_t* p   = (const uint8_t*)cfg;
    const uint8_t* end = p + cfg->wTotalLength;
    bool in_iface = false;
    while (p + 2 <= end) {
        const uint8_t bLen = p[0], bType = p[1];
        if (bLen < 2) break;
        if (bType == USB_B_DESCRIPTOR_TYPE_INTERFACE) {
            in_iface = (p[2] == iface_num);
        } else if (bType == USB_B_DESCRIPTOR_TYPE_ENDPOINT && in_iface) {
            const bool is_in   = (p[2] & 0x80) != 0;
            const bool is_intr = (p[3] & 0x03) == 0x03;
            if (is_intr && is_in == want_in) {
                *ep_addr = p[2];
                return true;
            }
        }
        p += bLen;
    }
    return false;
}
static bool find_ep_in (usb_device_handle_t dev, uint8_t iface, uint8_t* ep) { return find_ep(dev, iface, true,  ep); }
static bool find_ep_out(usb_device_handle_t dev, uint8_t iface, uint8_t* ep) { return find_ep(dev, iface, false, ep); }

static void client_event_cb(const usb_host_client_event_msg_t* msg, void*) {
    if (msg->event == USB_HOST_CLIENT_EVENT_NEW_DEV) {
        if (usb_host_device_open(s_client, msg->new_dev.address, &s_dev) != ESP_OK) {
            ESP_LOGE(TAG, "device open failed");
            return;
        }
        const usb_device_desc_t* desc;
        usb_host_get_device_descriptor(s_dev, &desc);
        ESP_LOGI(TAG, "device: VID=0x%04X PID=0x%04X (want VID=0x%04X PID=0x%04X)",
                 desc->idVendor, desc->idProduct, SC_VID, SC_PID);
        if (desc->idVendor != SC_VID || desc->idProduct != SC_PID) {
            ESP_LOGW(TAG, "VID/PID mismatch — ignoring device");
            usb_host_device_close(s_client, s_dev);
            s_dev = nullptr;
            return;
        }
        usb_host_interface_claim(s_client, s_dev, SC_HID_IFACE, 0);
        if (!find_ep_in(s_dev, SC_HID_IFACE, &s_ep_in)) {
            ESP_LOGE(TAG, "no interrupt IN endpoint on iface %u", SC_HID_IFACE);
            return;
        }
        if (find_ep_out(s_dev, SC_HID_IFACE, &s_ep_out))
            ESP_LOGI(TAG, "found OUT ep=0x%02X — using interrupt OUT for haptics", s_ep_out);
        else
            ESP_LOGI(TAG, "no interrupt OUT ep — haptics will use SET_REPORT");
        ESP_LOGI(TAG, "found IN ep=0x%02X — starting transfers", s_ep_in);
        usb_host_transfer_alloc(SC_BUF_SIZE, 0, &s_xfer);
        s_xfer->device_handle    = s_dev;
        s_xfer->bEndpointAddress = s_ep_in;
        s_xfer->callback         = transfer_cb;
        s_xfer->context          = nullptr;
        s_xfer->num_bytes        = SC_BUF_SIZE;
        usb_host_transfer_submit(s_xfer);

        usb_host_transfer_alloc(sizeof(usb_setup_packet_t) + SC_HAPTIC_PAYLOAD, 0, &s_ctrl_xfer);
        taskENTER_CRITICAL(&s_ctrl_mux);
        s_ctrl_busy  = false;
        s_sent_left  = s_sent_right = 0xFF;  // force re-send after reconnect
        taskEXIT_CRITICAL(&s_ctrl_mux);
        rumble_pump();  // send initial stop to both channels
        if (!s_pcm_xfer) {
            usb_host_transfer_alloc(SC_BUF_SIZE, 0, &s_pcm_xfer);
            s_pcm_sem = xSemaphoreCreateBinary();
        }
        xTaskCreatePinnedToCore(sc_pcm_init_task, "pcm_init", 4096, nullptr, 3, nullptr, 1);
    } else if (msg->event == USB_HOST_CLIENT_EVENT_DEV_GONE) {
        ESP_LOGI(TAG, "device disconnected");
        // All pending transfers complete (with error) before DEV_GONE fires, so freeing is safe.
        if (s_xfer)      { usb_host_transfer_free(s_xfer);      s_xfer      = nullptr; }
        if (s_ctrl_xfer) { usb_host_transfer_free(s_ctrl_xfer); s_ctrl_xfer = nullptr; }
        if (s_dev) {
            usb_host_interface_release(s_client, s_dev, SC_HID_IFACE);
            usb_host_device_close(s_client, s_dev);
            s_dev = nullptr;
        }
        s_ep_in = s_ep_out = 0;
    }
}

static void usb_lib_task(void*) {
    // Explicitly configure the USB OTG PHY into host mode before the stack starts.
    // Without this the PHY defaults to device mode and root port reset fails.
    usb_phy_handle_t phy_hdl;
    const usb_phy_config_t phy_cfg = {
        .controller = USB_PHY_CTRL_OTG,
        .target     = USB_PHY_TARGET_INT,
        .otg_mode   = USB_OTG_MODE_HOST,
        .otg_speed  = USB_PHY_SPEED_FULL,
    };
    usb_new_phy(&phy_cfg, &phy_hdl);

    // skip_phy_setup = true: we configured the PHY above; don't let the library reset it
    const usb_host_config_t host_cfg = {
        .skip_phy_setup = true,
        .intr_flags     = ESP_INTR_FLAG_LEVEL1,
    };
    usb_host_install(&host_cfg);
    xSemaphoreGive(s_host_ready);
    for (;;) {
        uint32_t flags;
        usb_host_lib_handle_events(portMAX_DELAY, &flags);
        if (flags & USB_HOST_LIB_EVENT_FLAGS_NO_CLIENTS) usb_host_device_free_all();
    }
}

static void usb_client_task(void*) {
    xSemaphoreTake(s_host_ready, portMAX_DELAY);
    xSemaphoreGive(s_host_ready);  // restore so sc_usb_host_wait() callers can proceed
    const usb_host_client_config_t client_cfg = {
        .is_synchronous    = false,
        .max_num_event_msg = 5,
        .async = { .client_event_callback = client_event_cb, .callback_arg = nullptr },
    };
    usb_host_client_register(&client_cfg, &s_client);
    for (;;) {
        usb_host_client_handle_events(s_client, portMAX_DELAY);
    }
}

static void sc_pcm_drain_task(void*) {
    SCPcmPacket pkt;
    for (;;) {
        if (xQueueReceive(s_pcm_hap_queue, &pkt, portMAX_DELAY) == pdTRUE) {
            pcm_send_raw(pkt.buf, sizeof(pkt.buf));
            // s_ctrl_busy was just released — give rumble a chance to send
            // before we take the bus again for the next PCM packet.
            rumble_pump();
        }
    }
}

static void sc_pcm_init_task(void*) {
    sc_pcm_begin();
    vTaskDelete(nullptr);
}

void sc_pcm_begin() {
    if (!s_pcm_hap_queue)
        s_pcm_hap_queue = xQueueCreate(PCM_HAP_QUEUE_DEPTH, sizeof(SCPcmPacket));
    pcm_setup();
    if (!s_pcm_drain_hdl)
        xTaskCreatePinnedToCore(sc_pcm_drain_task, "pcm_drain", 4096, nullptr, 3,
                                &s_pcm_drain_hdl, 1);
}

void sc_pcm_enqueue(const int8_t* samples, size_t sample_count) {
    if (!s_pcm_hap_queue || !samples || sample_count == 0) return;
    const uint8_t n = (uint8_t)(sample_count > 31 ? 31 : sample_count);
    SCPcmPacket pkt;
    pkt.buf[0] = SC_REPORT_ID_PCM_DATA;
    pkt.buf[1] = n;
    for (uint8_t i = 0; i < n; i++) {
        pkt.buf[2      + i] = (uint8_t)samples[i * 2];      // L
        pkt.buf[2 + 31 + i] = (uint8_t)samples[i * 2 + 1];  // R
    }
    for (uint8_t i = n; i < 31; i++)
        pkt.buf[2 + i] = pkt.buf[2 + 31 + i] = 0;
    xQueueSendToBack(s_pcm_hap_queue, &pkt, 0);
}

void sc_host_begin() {
    s_host_ready = xSemaphoreCreateBinary();
    // Pin USB tasks to CPU1 so their event loops cannot starve IDLE0 on CPU0.
    xTaskCreatePinnedToCore(usb_lib_task,    "usb_lib",    8192, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(usb_client_task, "usb_client", 8192, nullptr, 4, nullptr, 1);
}

bool sc_report_poll(SCReport* out) {
    taskENTER_CRITICAL(&s_report_mux);
    if (!s_report_ready) {
        taskEXIT_CRITICAL(&s_report_mux);
        return false;
    }
    s_report_ready = false;
    memcpy(out, &s_report, sizeof(SCReport));
    taskEXIT_CRITICAL(&s_report_mux);
    return true;
}

bool sc_touch_poll(SCTouchReport* out) {
    taskENTER_CRITICAL(&s_report_mux);
    if (!s_touch_ready) {
        taskEXIT_CRITICAL(&s_report_mux);
        return false;
    }
    s_touch_ready = false;
    memcpy(out, &s_touch_report, sizeof(SCTouchReport));
    taskEXIT_CRITICAL(&s_report_mux);
    return true;
}

void sc_usb_host_wait() {
    xSemaphoreTake(s_host_ready, portMAX_DELAY);
    xSemaphoreGive(s_host_ready);
}

void sc_rumble(uint8_t left, uint8_t right) {
    taskENTER_CRITICAL(&s_ctrl_mux);
    s_rumble_left  = left;
    s_rumble_right = right;
    taskEXIT_CRITICAL(&s_ctrl_mux);
    rumble_pump();
}

void sc_haptic_tone(uint8_t left, uint8_t right) {
    taskENTER_CRITICAL(&s_ctrl_mux);
    s_tone_left  = left;
    s_tone_right = right;
    taskEXIT_CRITICAL(&s_ctrl_mux);
    rumble_pump();
}
