#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_clk_tree.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_private/periph_ctrl.h"
#include "esp_rom_gpio.h"
#include "hal/rmt_ll.h"
#include "hal/rmt_types.h"
#include "soc/gpio_sig_map.h"
#include "soc/rmt_periph.h"
#include "soc/soc_caps.h"

#define BDSHOT_GPIO_IO                 18
#define BDSHOT_RMT_GROUP_ID            0
#define BDSHOT_RMT_TX_CHANNEL          0
#define BDSHOT_RMT_RX_CHANNEL          0
#define BDSHOT_RMT_GROUP_CLK_DIV       1
#define BDSHOT_RMT_CHANNEL_CLK_DIV     2
#define BDSHOT_BAUD_HZ                 600000U
#define BDSHOT_FRAME_BITS              16U
#define BDSHOT_REPLY_BITS              21U
#define BDSHOT_REPLY_BIT_NS            1333U
#define BDSHOT_RX_IDLE_US              35U
#define BDSHOT_WAIT_TIMEOUT_MS         10U
#define BDSHOT_TEST_THROTTLE           0U
#define BDSHOT_REQUEST_TELEMETRY       true
#define BDSHOT_TX_MEM_INDEX            BDSHOT_RMT_TX_CHANNEL
#define BDSHOT_RX_MEM_INDEX            (SOC_RMT_TX_CANDIDATES_PER_GROUP + BDSHOT_RMT_RX_CHANNEL)

typedef struct {
    struct {
        rmt_symbol_word_t symbols[SOC_RMT_MEM_WORDS_PER_CHANNEL];
    } channels[SOC_RMT_CHANNELS_PER_GROUP];
} rmt_block_mem_t;

typedef struct {
    uint32_t group_clk_hz;
    uint32_t resolution_hz;
    uint16_t tx_bit_ticks;
    uint16_t tx_zero_high_ticks;
    uint16_t tx_one_high_ticks;
    uint16_t reply_bit_ticks;
    uint16_t rx_idle_ticks;
} bshot_timing_t;

typedef struct {
    int raw;
    uint32_t erpm_period_us;
    uint8_t ext_page;
    uint8_t ext_value;
    bool valid;
    bool is_erpm;
} bshot_reply_t;

typedef struct {
    TaskHandle_t waiting_task;
    volatile bool tx_done;
    volatile bool rx_done;
    volatile uint32_t rx_symbol_count;
    rmt_symbol_word_t rx_symbols[SOC_RMT_MEM_WORDS_PER_CHANNEL];
} bshot_rmt_context_t;

extern rmt_block_mem_t RMTMEM;

static const char *TAG = "bidir_dshot_ll";
static intr_handle_t s_rmt_intr_handle;
static portMUX_TYPE s_rmt_spinlock = portMUX_INITIALIZER_UNLOCKED;
static bshot_timing_t s_bshot_timing;
static bshot_rmt_context_t s_bshot_ctx;

static inline rmt_symbol_word_t *bshot_tx_mem_base(void)
{
    return &RMTMEM.channels[BDSHOT_TX_MEM_INDEX].symbols[0];
}

static inline rmt_symbol_word_t *bshot_rx_mem_base(void)
{
    return &RMTMEM.channels[BDSHOT_RX_MEM_INDEX].symbols[0];
}

static uint8_t bshot_crc4(uint16_t payload_12bit)
{
    uint8_t crc = 0;

    for (int i = 0; i < 3; i++) {
        crc ^= payload_12bit & 0x0F;
        payload_12bit >>= 4;
    }

    return crc & 0x0F;
}

static uint16_t bshot_make_frame(uint16_t throttle_or_command, bool request_telemetry)
{
    uint16_t payload = ((throttle_or_command & 0x07FF) << 1) | (request_telemetry ? 1U : 0U);
    uint8_t crc = (~bshot_crc4(payload)) & 0x0F;

    return (payload << 4) | crc;
}

static void bshot_fill_tx_symbols(uint16_t frame)
{
    rmt_symbol_word_t *tx_mem = bshot_tx_mem_base();

    memset(tx_mem, 0, sizeof(rmt_symbol_word_t) * SOC_RMT_MEM_WORDS_PER_CHANNEL);
    for (uint32_t i = 0; i < BDSHOT_FRAME_BITS; i++) {
        bool bit = (frame & (1U << (BDSHOT_FRAME_BITS - 1U - i))) != 0;
        uint16_t high_ticks = bit ? s_bshot_timing.tx_one_high_ticks : s_bshot_timing.tx_zero_high_ticks;

        tx_mem[i].level0 = 0;
        tx_mem[i].duration0 = high_ticks;
        tx_mem[i].level1 = 1;
        tx_mem[i].duration1 = s_bshot_timing.tx_bit_ticks - high_ticks;
    }

    // Match common bDShot implementations: stop after the final active-low pulse,
    // then let the line return to idle high immediately for ESC turnaround.
    tx_mem[BDSHOT_FRAME_BITS - 1].duration1 = 0;
}

static uint8_t bshot_decode_gcr_nibble(uint16_t value)
{
    switch (value) {
    case 0x19: return 0x00;
    case 0x1B: return 0x01;
    case 0x12: return 0x02;
    case 0x13: return 0x03;
    case 0x1D: return 0x04;
    case 0x15: return 0x05;
    case 0x16: return 0x06;
    case 0x17: return 0x07;
    case 0x1A: return 0x08;
    case 0x09: return 0x09;
    case 0x0A: return 0x0A;
    case 0x0B: return 0x0B;
    case 0x1E: return 0x0C;
    case 0x0D: return 0x0D;
    case 0x0E: return 0x0E;
    case 0x0F: return 0x0F;
    default: return 0xFF;
    }
}

static int bshot_decode_gcr_telemetry(uint32_t encoded_value)
{
    uint16_t mapped = 0;
    uint8_t left_shift = 0;

    // The ESC reply is received as a 21-bit stream whose leading shift bit is
    // removed by this single xor fold before 5b/4b nibble reversal.
    encoded_value ^= (encoded_value >> 1);

    for (int i = 0; i < 20; i += 5) {
        uint8_t nibble = bshot_decode_gcr_nibble((encoded_value >> i) & 0x1F);

        if (nibble > 0x0F) {
            return -1;
        }
        mapped |= ((uint16_t)nibble) << left_shift;
        left_shift += 4;
    }

    if ((((~((mapped >> 4) ^ (mapped >> 8) ^ (mapped >> 12))) & 0x0F)) != (mapped & 0x0F)) {
        return -1;
    }

    return mapped >> 4;
}

static bool bshot_decode_reply(bshot_reply_t *reply)
{
    uint32_t encoded = 0;
    uint32_t bit_count = 0;

    memset(reply, 0, sizeof(*reply));
    if (s_bshot_ctx.rx_symbol_count == 0) {
        return false;
    }
    if (s_bshot_ctx.rx_symbols[0].duration0 == 0 || s_bshot_ctx.rx_symbols[0].level0 != 0) {
        return false;
    }

    for (uint32_t i = 0; i < s_bshot_ctx.rx_symbol_count && i < SOC_RMT_MEM_WORDS_PER_CHANNEL; i++) {
        const rmt_symbol_word_t *symbol = &s_bshot_ctx.rx_symbols[i];
        uint16_t durations[2] = {symbol->duration0, symbol->duration1};
        uint8_t levels[2] = {symbol->level0, symbol->level1};

        for (int phase = 0; phase < 2; phase++) {
            uint32_t bits;

            if (durations[phase] == 0) {
                goto finish;
            }

            bits = (durations[phase] + (s_bshot_timing.reply_bit_ticks / 2U)) / s_bshot_timing.reply_bit_ticks;
            if (bits == 0) {
                bits = 1;
            }
            while (bits-- && bit_count < BDSHOT_REPLY_BITS) {
                encoded = (encoded << 1) | levels[phase];
                bit_count++;
            }
            if (bit_count >= BDSHOT_REPLY_BITS) {
                goto finish;
            }
        }
    }

finish:
    while (bit_count < BDSHOT_REPLY_BITS) {
        encoded = (encoded << 1) | 1U;
        bit_count++;
    }

    reply->raw = bshot_decode_gcr_telemetry(encoded);
    if (reply->raw < 0) {
        return false;
    }

    reply->valid = true;
    if (reply->raw & 0x100) {
        reply->is_erpm = true;
        if (reply->raw == 0x0FFF) {
            reply->erpm_period_us = 0;
        } else {
            reply->erpm_period_us = ((uint32_t)reply->raw & 0x1FFU) << (reply->raw >> 9);
        }
    } else {
        reply->ext_page = (reply->raw >> 9) & 0x07;
        reply->ext_value = reply->raw & 0xFF;
    }

    return true;
}

static void bshot_compute_timing(void)
{
    ESP_ERROR_CHECK(esp_clk_tree_src_get_freq_hz(
        (soc_module_clk_t)RMT_CLK_SRC_APB,
        ESP_CLK_TREE_SRC_FREQ_PRECISION_CACHED,
        &s_bshot_timing.group_clk_hz
    ));

    s_bshot_timing.resolution_hz = s_bshot_timing.group_clk_hz / BDSHOT_RMT_CHANNEL_CLK_DIV;
    s_bshot_timing.tx_bit_ticks = (s_bshot_timing.resolution_hz + (BDSHOT_BAUD_HZ / 2U)) / BDSHOT_BAUD_HZ;
    s_bshot_timing.tx_zero_high_ticks = (s_bshot_timing.tx_bit_ticks * 3U + 4U) / 8U;
    s_bshot_timing.tx_one_high_ticks = (s_bshot_timing.tx_bit_ticks * 3U + 2U) / 4U;
    s_bshot_timing.reply_bit_ticks = (uint16_t)(((uint64_t)s_bshot_timing.resolution_hz * BDSHOT_REPLY_BIT_NS + 500000000ULL) / 1000000000ULL);
    s_bshot_timing.rx_idle_ticks = (uint16_t)(((uint64_t)s_bshot_timing.resolution_hz * BDSHOT_RX_IDLE_US + 500000ULL) / 1000000ULL);
}

static void bshot_connect_tx_gpio(void)
{
    ESP_ERROR_CHECK(gpio_set_direction(BDSHOT_GPIO_IO, GPIO_MODE_INPUT_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(BDSHOT_GPIO_IO, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_level(BDSHOT_GPIO_IO, 1));
    esp_rom_gpio_connect_out_signal(
        BDSHOT_GPIO_IO,
        rmt_periph_signals.groups[BDSHOT_RMT_GROUP_ID].channels[BDSHOT_RMT_TX_CHANNEL].tx_sig,
        false,
        false
    );
}

static void bshot_release_line_to_rx(void)
{
    esp_rom_gpio_connect_out_signal(BDSHOT_GPIO_IO, SIG_GPIO_OUT_IDX, false, false);
    ESP_ERROR_CHECK(gpio_set_direction(BDSHOT_GPIO_IO, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(BDSHOT_GPIO_IO, GPIO_PULLUP_ONLY));
}

static void IRAM_ATTR bshot_prepare_rx_from_isr(void)
{
    esp_rom_gpio_connect_out_signal(BDSHOT_GPIO_IO, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction(BDSHOT_GPIO_IO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BDSHOT_GPIO_IO, GPIO_PULLUP_ONLY);

    rmt_ll_rx_enable(&RMT, BDSHOT_RMT_RX_CHANNEL, false);
    rmt_ll_rx_reset_pointer(&RMT, BDSHOT_RMT_RX_CHANNEL);
    rmt_ll_rx_set_mem_owner(&RMT, BDSHOT_RMT_RX_CHANNEL, RMT_LL_MEM_OWNER_HW);
    rmt_ll_clear_interrupt_status(
        &RMT,
        RMT_LL_EVENT_RX_DONE(BDSHOT_RMT_RX_CHANNEL) |
        RMT_LL_EVENT_RX_THRES(BDSHOT_RMT_RX_CHANNEL) |
        RMT_LL_EVENT_RX_ERROR(BDSHOT_RMT_RX_CHANNEL)
    );
    rmt_ll_rx_enable(&RMT, BDSHOT_RMT_RX_CHANNEL, true);
}

static void IRAM_ATTR bshot_rmt_isr(void *arg)
{
    (void)arg;
    BaseType_t task_woken = pdFALSE;
    uint32_t tx_status = rmt_ll_tx_get_interrupt_status(&RMT, BDSHOT_RMT_TX_CHANNEL);
    uint32_t rx_status = rmt_ll_rx_get_interrupt_status(&RMT, BDSHOT_RMT_RX_CHANNEL);

    if (tx_status & RMT_LL_EVENT_TX_DONE(BDSHOT_RMT_TX_CHANNEL)) {
        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_DONE(BDSHOT_RMT_TX_CHANNEL));
        s_bshot_ctx.tx_done = true;

        portENTER_CRITICAL_ISR(&s_rmt_spinlock);
        bshot_prepare_rx_from_isr();
        portEXIT_CRITICAL_ISR(&s_rmt_spinlock);
    }

    if (rx_status & RMT_LL_EVENT_RX_DONE(BDSHOT_RMT_RX_CHANNEL)) {
        uint32_t symbol_count;

        rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_RX_DONE(BDSHOT_RMT_RX_CHANNEL));
        portENTER_CRITICAL_ISR(&s_rmt_spinlock);
        rmt_ll_rx_enable(&RMT, BDSHOT_RMT_RX_CHANNEL, false);
        symbol_count = rmt_ll_rx_get_memory_writer_offset(&RMT, BDSHOT_RMT_RX_CHANNEL);
        if (symbol_count > SOC_RMT_MEM_WORDS_PER_CHANNEL) {
            symbol_count = SOC_RMT_MEM_WORDS_PER_CHANNEL;
        }
        rmt_ll_rx_set_mem_owner(&RMT, BDSHOT_RMT_RX_CHANNEL, RMT_LL_MEM_OWNER_SW);
        memcpy(s_bshot_ctx.rx_symbols, bshot_rx_mem_base(), symbol_count * sizeof(rmt_symbol_word_t));
        rmt_ll_rx_set_mem_owner(&RMT, BDSHOT_RMT_RX_CHANNEL, RMT_LL_MEM_OWNER_HW);
        portEXIT_CRITICAL_ISR(&s_rmt_spinlock);

        s_bshot_ctx.rx_symbol_count = symbol_count;
        s_bshot_ctx.rx_done = true;
        if (s_bshot_ctx.waiting_task != NULL) {
            vTaskNotifyGiveFromISR(s_bshot_ctx.waiting_task, &task_woken);
        }
    }

    if (task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void bshot_rmt_gpio_init(void)
{
    ESP_ERROR_CHECK(gpio_reset_pin(BDSHOT_GPIO_IO));
    ESP_ERROR_CHECK(gpio_set_pull_mode(BDSHOT_GPIO_IO, GPIO_PULLUP_ONLY));
    esp_rom_gpio_connect_in_signal(
        BDSHOT_GPIO_IO,
        rmt_periph_signals.groups[BDSHOT_RMT_GROUP_ID].channels[BDSHOT_RX_MEM_INDEX].rx_sig,
        false
    );
    bshot_release_line_to_rx();
}

static void bshot_rmt_hw_init(void)
{
    const uint32_t intr_mask = RMT_LL_EVENT_TX_DONE(BDSHOT_RMT_TX_CHANNEL) | RMT_LL_EVENT_RX_DONE(BDSHOT_RMT_RX_CHANNEL);

    bshot_compute_timing();
    bshot_rmt_gpio_init();

    PERIPH_RCC_ATOMIC() {
        rmt_ll_enable_bus_clock(BDSHOT_RMT_GROUP_ID, true);
        rmt_ll_reset_register(BDSHOT_RMT_GROUP_ID);
    }

    rmt_ll_mem_power_by_pmu(&RMT);
    rmt_ll_enable_mem_access_nonfifo(&RMT, true);
    rmt_ll_enable_interrupt(&RMT, UINT32_MAX, false);
    rmt_ll_clear_interrupt_status(&RMT, UINT32_MAX);
#if SOC_RMT_SUPPORT_TX_SYNCHRO
    rmt_ll_tx_clear_sync_group(&RMT);
#endif

    rmt_ll_set_group_clock_src(
        &RMT,
        BDSHOT_RMT_TX_CHANNEL,
        RMT_CLK_SRC_APB,
        BDSHOT_RMT_GROUP_CLK_DIV,
        1,
        0
    );
    rmt_ll_enable_group_clock(&RMT, true);

    rmt_ll_tx_reset_channels_clock_div(&RMT, 1U << BDSHOT_RMT_TX_CHANNEL);
    rmt_ll_rx_reset_channels_clock_div(&RMT, 1U << BDSHOT_RMT_RX_CHANNEL);
    rmt_ll_tx_set_channel_clock_div(&RMT, BDSHOT_RMT_TX_CHANNEL, BDSHOT_RMT_CHANNEL_CLK_DIV);
    rmt_ll_rx_set_channel_clock_div(&RMT, BDSHOT_RMT_RX_CHANNEL, BDSHOT_RMT_CHANNEL_CLK_DIV);

    rmt_ll_tx_set_mem_blocks(&RMT, BDSHOT_RMT_TX_CHANNEL, 1);
    rmt_ll_tx_set_limit(&RMT, BDSHOT_RMT_TX_CHANNEL, SOC_RMT_MEM_WORDS_PER_CHANNEL / 2U);
    rmt_ll_tx_enable_wrap(&RMT, BDSHOT_RMT_TX_CHANNEL, true);
    rmt_ll_tx_enable_carrier_modulation(&RMT, BDSHOT_RMT_TX_CHANNEL, false);
    rmt_ll_tx_fix_idle_level(&RMT, BDSHOT_RMT_TX_CHANNEL, 1, true);

    rmt_ll_rx_set_mem_blocks(&RMT, BDSHOT_RMT_RX_CHANNEL, 1);
    rmt_ll_rx_set_mem_owner(&RMT, BDSHOT_RMT_RX_CHANNEL, RMT_LL_MEM_OWNER_HW);
    rmt_ll_rx_enable_filter(&RMT, BDSHOT_RMT_RX_CHANNEL, false);
    rmt_ll_rx_set_idle_thres(&RMT, BDSHOT_RMT_RX_CHANNEL, s_bshot_timing.rx_idle_ticks);
#if SOC_RMT_SUPPORT_RX_PINGPONG
    rmt_ll_rx_set_limit(&RMT, BDSHOT_RMT_RX_CHANNEL, SOC_RMT_MEM_WORDS_PER_CHANNEL / 2U);
    rmt_ll_rx_enable_wrap(&RMT, BDSHOT_RMT_RX_CHANNEL, true);
#endif
#if SOC_RMT_SUPPORT_RX_DEMODULATION
    rmt_ll_rx_enable_carrier_demodulation(&RMT, BDSHOT_RMT_RX_CHANNEL, false);
#endif

    ESP_ERROR_CHECK(esp_intr_alloc_intrstatus(
        rmt_periph_signals.groups[BDSHOT_RMT_GROUP_ID].irq,
        ESP_INTR_FLAG_SHARED | ESP_INTR_FLAG_IRAM,
        (uint32_t)(uintptr_t)rmt_ll_get_interrupt_status_reg(&RMT),
        intr_mask,
        bshot_rmt_isr,
        NULL,
        &s_rmt_intr_handle
    ));

    rmt_ll_enable_interrupt(&RMT, intr_mask, true);
}

static esp_err_t bshot_transceive(uint16_t throttle_or_command, bool request_telemetry, bshot_reply_t *reply)
{
    uint16_t frame = bshot_make_frame(throttle_or_command, request_telemetry);
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(BDSHOT_WAIT_TIMEOUT_MS);

    s_bshot_ctx.waiting_task = xTaskGetCurrentTaskHandle();
    s_bshot_ctx.tx_done = false;
    s_bshot_ctx.rx_done = false;
    s_bshot_ctx.rx_symbol_count = 0;
    memset(s_bshot_ctx.rx_symbols, 0, sizeof(s_bshot_ctx.rx_symbols));

    bshot_fill_tx_symbols(frame);
    bshot_connect_tx_gpio();

    portENTER_CRITICAL(&s_rmt_spinlock);
    rmt_ll_rx_enable(&RMT, BDSHOT_RMT_RX_CHANNEL, false);
    rmt_ll_tx_stop(&RMT, BDSHOT_RMT_TX_CHANNEL);
    rmt_ll_tx_reset_channels_clock_div(&RMT, 1U << BDSHOT_RMT_TX_CHANNEL);
    rmt_ll_tx_reset_pointer(&RMT, BDSHOT_RMT_TX_CHANNEL);
    rmt_ll_clear_interrupt_status(
        &RMT,
        RMT_LL_EVENT_TX_DONE(BDSHOT_RMT_TX_CHANNEL) |
        RMT_LL_EVENT_TX_THRES(BDSHOT_RMT_TX_CHANNEL) |
        RMT_LL_EVENT_TX_ERROR(BDSHOT_RMT_TX_CHANNEL) |
        RMT_LL_EVENT_RX_DONE(BDSHOT_RMT_RX_CHANNEL) |
        RMT_LL_EVENT_RX_THRES(BDSHOT_RMT_RX_CHANNEL) |
        RMT_LL_EVENT_RX_ERROR(BDSHOT_RMT_RX_CHANNEL)
    );
    rmt_ll_tx_start(&RMT, BDSHOT_RMT_TX_CHANNEL);
    portEXIT_CRITICAL(&s_rmt_spinlock);

    while (!s_bshot_ctx.rx_done) {
        TickType_t now = xTaskGetTickCount();

        if (now >= deadline) {
            s_bshot_ctx.waiting_task = NULL;
            portENTER_CRITICAL(&s_rmt_spinlock);
            rmt_ll_rx_enable(&RMT, BDSHOT_RMT_RX_CHANNEL, false);
            portEXIT_CRITICAL(&s_rmt_spinlock);
            bshot_release_line_to_rx();
            return ESP_ERR_TIMEOUT;
        }

        ulTaskNotifyTake(pdTRUE, deadline - now);
    }

    s_bshot_ctx.waiting_task = NULL;
    bshot_release_line_to_rx();
    return bshot_decode_reply(reply) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
}

void app_main(void)
{
    bshot_reply_t reply;

    bshot_rmt_hw_init();

    ESP_LOGI(
        TAG,
        "bDShot ready: gpio=%d res=%" PRIu32 "Hz tx_ticks=%u zero=%u/%u one=%u/%u reply_ticks=%u",
        BDSHOT_GPIO_IO,
        s_bshot_timing.resolution_hz,
        s_bshot_timing.tx_bit_ticks,
        s_bshot_timing.tx_zero_high_ticks,
        s_bshot_timing.tx_bit_ticks - s_bshot_timing.tx_zero_high_ticks,
        s_bshot_timing.tx_one_high_ticks,
        s_bshot_timing.tx_bit_ticks - s_bshot_timing.tx_one_high_ticks,
        s_bshot_timing.reply_bit_ticks
    );
    ESP_LOGI(
        TAG,
        "single-wire half duplex on GPIO %d, test throttle=%u, telemetry_request=%u",
        BDSHOT_GPIO_IO,
        BDSHOT_TEST_THROTTLE,
        BDSHOT_REQUEST_TELEMETRY
    );

    while (true) {
        esp_err_t err = bshot_transceive(BDSHOT_TEST_THROTTLE, BDSHOT_REQUEST_TELEMETRY, &reply);

        if (err == ESP_OK && reply.valid) {
            if (reply.is_erpm) {
                uint32_t erpm = reply.erpm_period_us ? (60000000UL / reply.erpm_period_us) : 0;

                ESP_LOGI(
                    TAG,
                    "telemetry raw=0x%03X erpm_period_us=%" PRIu32 " erpm=%" PRIu32 " symbols=%" PRIu32,
                    reply.raw,
                    reply.erpm_period_us,
                    erpm,
                    s_bshot_ctx.rx_symbol_count
                );
            } else {
                ESP_LOGI(
                    TAG,
                    "extended telemetry raw=0x%03X page=%u value=0x%02X symbols=%" PRIu32,
                    reply.raw,
                    reply.ext_page,
                    reply.ext_value,
                    s_bshot_ctx.rx_symbol_count
                );
            }
        } else {
            ESP_LOGW(TAG, "bDShot receive failed: %s", esp_err_to_name(err));
        }

        vTaskDelay(pdMS_TO_TICKS(20));
    }
}
