#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "sc_input.h"
#include "sc_report.h"
#include "ds_edge_hid.h"
#include "le_audio_codec.h"
#include "le_audio_bap.h"

static const char *TAG = "main";

extern "C" void app_main(void) {
#if defined(AUDIO_LOOPBACK_TEST)
    // Phase 2 hardware bring-up: isolated mic-to-speaker loopback, no USB
    // host / BLE involved. Build with -DAUDIO_LOOPBACK_TEST to test the
    // ES8311/NS4150B wiring on its own before wiring it into LE Audio
    // (Phase 3). Revert to the normal build for everything else.
    ESP_LOGI(TAG, "SC++ S31 — Phase 2 audio loopback test");
    if (!codec_init(16000)) {
        ESP_LOGE(TAG, "codec_init failed");
        return;
    }
    codec_set_speaker_volume(70);
    codec_set_mic_gain(70);
    codec_loopback_task(nullptr);  // never returns
    return;
#else
    ESP_LOGI(TAG, "SC++ S31 — boot");

    ESP_LOGI(TAG, "Initializing USB host mode...");
    sc_host_begin();
    sc_usb_host_wait();
    ESP_LOGI(TAG, "USB host ready");

    le_audio_bap_begin();
    ESP_LOGI(TAG, "DualSense Edge BLE HID + LE Audio ready");

    static uint32_t s_last_btns = 0;
    static uint32_t s_poll_count = 0;
    for (;;) {
        SCReport report;
        if (sc_report_poll(&report)) {
            s_poll_count++;
            uint32_t btns = sc_buttons(report);
            if (btns != s_last_btns) {
                ESP_LOGI(TAG, "SC buttons changed: 0x%08lx (poll_count=%lu)", (unsigned long)btns, (unsigned long)s_poll_count);
                s_last_btns = btns;
            }
            ds_send(report);
        } else {
            ds_send_idle();
        }
        vTaskDelay(pdMS_TO_TICKS(4));
    }
#endif
}
