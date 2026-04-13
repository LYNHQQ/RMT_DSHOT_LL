#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"
#include "esp_log.h"
#include "bidir_dshot.h"

#define DEMO_PERIOD_MS 20U
#define DEMO_THROTTLE  0U

static const char *TAG = "bidir_dshot_demo";

static void fill_uniform_values(uint16_t value, uint16_t values[BIDIR_DSHOT_CHANNEL_COUNT])
{
    for (uint32_t i = 0; i < BIDIR_DSHOT_CHANNEL_COUNT; i++) {
        values[i] = value;
    }
}

static void log_results(const bidir_dshot_reply_t replies[BIDIR_DSHOT_CHANNEL_COUNT], const esp_err_t results[BIDIR_DSHOT_CHANNEL_COUNT])
{
    for (uint32_t i = 0; i < BIDIR_DSHOT_CHANNEL_COUNT; i++) {
        if (results[i] == ESP_OK && replies[i].valid) {
            if (replies[i].is_erpm) {
                uint32_t erpm = replies[i].erpm_period_us ? (60000000UL / replies[i].erpm_period_us) : 0;

                ESP_LOGI(
                    TAG,
                    "ch=%" PRIu32 " erpm raw=0x%03X erpm_period_us=%" PRIu32 " erpm=%" PRIu32,
                    i,
                    replies[i].raw,
                    replies[i].erpm_period_us,
                    erpm
                );
            } else {
                ESP_LOGI(
                    TAG,
                    "ch=%" PRIu32 " EDT raw=0x%03X type=0x%02X value=0x%02X",
                    i,
                    replies[i].raw,
                    replies[i].edt_type,
                    replies[i].edt_value
                );
            }
        } else {
            ESP_LOGW(TAG, "ch=%" PRIu32 " receive failed: %s", i, esp_err_to_name(results[i]));
        }
    }
}

void app_main(void)
{
    bidir_dshot_config_t config = BIDIR_DSHOT_DEFAULT_CONFIG();
    uint16_t throttle_values[BIDIR_DSHOT_CHANNEL_COUNT];
    bidir_dshot_reply_t replies[BIDIR_DSHOT_CHANNEL_COUNT];
    esp_err_t results[BIDIR_DSHOT_CHANNEL_COUNT];

    fill_uniform_values(DEMO_THROTTLE, throttle_values);

    ESP_ERROR_CHECK(bidir_dshot_init(&config));
    ESP_ERROR_CHECK(bidir_dshot_send(throttle_values));

    while (true) {
        vTaskDelay(pdMS_TO_TICKS(DEMO_PERIOD_MS));
        ESP_ERROR_CHECK(bidir_dshot_receive(replies, results));
        log_results(replies, results);
        ESP_ERROR_CHECK(bidir_dshot_send(throttle_values));
    }
}
