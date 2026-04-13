#pragma once

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BIDIR_DSHOT_CHANNEL_COUNT 4U

typedef struct {
    int gpio_num[BIDIR_DSHOT_CHANNEL_COUNT];
} bidir_dshot_config_t;

typedef struct {
    int raw;
    uint32_t erpm_period_us;
    uint8_t edt_type;
    uint8_t edt_value;
    bool valid;
    bool is_erpm;
} bidir_dshot_reply_t;

#define BIDIR_DSHOT_DEFAULT_CONFIG() { \
    .gpio_num = {18, 17, 16, 15}, \
}

// Initialize the dedicated 4-channel bidirectional DShot engine once.
esp_err_t bidir_dshot_init(const bidir_dshot_config_t *config);

// Start one telemetry-enabled DShot frame on all 4 channels.
esp_err_t bidir_dshot_send(const uint16_t values[BIDIR_DSHOT_CHANNEL_COUNT]);

// Advanced path: explicitly control the request_telemetry bit.
esp_err_t bidir_dshot_send_raw(
    const uint16_t values[BIDIR_DSHOT_CHANNEL_COUNT],
    bool request_telemetry
);

// Stop RX, read the previous frame from RMT memory, and decode the 4 replies.
esp_err_t bidir_dshot_receive(
    bidir_dshot_reply_t replies[BIDIR_DSHOT_CHANNEL_COUNT],
    esp_err_t results[BIDIR_DSHOT_CHANNEL_COUNT]
);

#ifdef __cplusplus
}
#endif
