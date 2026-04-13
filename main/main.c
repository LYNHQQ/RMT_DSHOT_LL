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

#define BDSHOT_CHANNEL_COUNT               4U
#define BDSHOT_RMT_GROUP_ID               0
#define BDSHOT_RMT_GROUP_CLK_DIV          1
#define BDSHOT_RMT_CHANNEL_CLK_DIV        2
#define BDSHOT_BAUD_HZ                    600000U
#define BDSHOT_FRAME_BITS                 16U
#define BDSHOT_REPLY_BITS                 21U
#define BDSHOT_REPLY_BIT_NS               1333U
#define BDSHOT_RX_IDLE_US                 35U
#define BDSHOT_WAIT_TIMEOUT_MS            10U
#define BDSHOT_PERIOD_MS                  20U
#define BDSHOT_CMD_EXTENDED_TELEMETRY_ENABLE 13U
#define BDSHOT_EDT_ENABLE_REPEATS         6U
#define BDSHOT_REQUEST_TELEMETRY          true
#define BDSHOT_TEST_THROTTLE              0U
#define BDSHOT_WORKER_TASK_STACK          4096U
#define BDSHOT_WORKER_TASK_PRIORITY       10U
#define BDSHOT_RMT_INTR_FLAGS             (ESP_INTR_FLAG_LEVEL3 | ESP_INTR_FLAG_IRAM)

typedef struct {
    struct {
        rmt_symbol_word_t symbols[SOC_RMT_MEM_WORDS_PER_CHANNEL];
    } channels[SOC_RMT_CHANNELS_PER_GROUP];
} rmt_block_mem_t;

typedef struct {
    uint32_t group_clk_hz;
    uint32_t resolution_hz;
    uint16_t tx_bit_ticks;
    uint16_t tx_zero_low_ticks;
    uint16_t tx_one_low_ticks;
    uint16_t reply_bit_ticks;
    uint16_t rx_idle_ticks;
} bshot_timing_t;

typedef struct {
    int raw;
    uint32_t erpm_period_us;
    uint8_t edt_type;
    uint8_t edt_value;
    bool valid;
    bool is_erpm;
} bshot_reply_t;

typedef struct {
    uint32_t index;
    int gpio_num;
    uint32_t tx_channel;
    uint32_t rx_channel;
    uint32_t tx_mem_index;
    uint32_t rx_mem_index;
    volatile bool tx_done;
    volatile bool rx_done;
    volatile uint32_t rx_symbol_count;
    rmt_symbol_word_t rx_symbols[SOC_RMT_MEM_WORDS_PER_CHANNEL];
} bshot_rmt_channel_t;

typedef struct {
    TaskHandle_t worker_task;
    uint32_t tx_channel_mask;
    bshot_rmt_channel_t channels[BDSHOT_CHANNEL_COUNT];
} bshot_rmt_context_t;

extern rmt_block_mem_t RMTMEM;

static const char *TAG = "bidir_dshot_ll";
static const int s_bshot_gpio_list[BDSHOT_CHANNEL_COUNT] = {18, 17, 16, 15};
static intr_handle_t s_rmt_intr_handle;
static portMUX_TYPE s_rmt_spinlock = portMUX_INITIALIZER_UNLOCKED;
static bshot_timing_t s_bshot_timing;
static bshot_rmt_context_t s_bshot_ctx;

static inline rmt_symbol_word_t *bshot_tx_mem_base(const bshot_rmt_channel_t *channel)
{
    return &RMTMEM.channels[channel->tx_mem_index].symbols[0];
}

static inline rmt_symbol_word_t *bshot_rx_mem_base(const bshot_rmt_channel_t *channel)
{
    return &RMTMEM.channels[channel->rx_mem_index].symbols[0];
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
    uint16_t payload = ((throttle_or_command & 0x07FFU) << 1) | (request_telemetry ? 1U : 0U);
    uint8_t crc = (~bshot_crc4(payload)) & 0x0F;

    return (payload << 4) | crc;
}

static inline rmt_symbol_word_t bshot_make_tx_symbol(bool bit)
{
    uint16_t low_ticks = bit ? s_bshot_timing.tx_one_low_ticks : s_bshot_timing.tx_zero_low_ticks;

    return (rmt_symbol_word_t) {
        .level0 = 0,
        .duration0 = low_ticks,
        .level1 = 1,
        .duration1 = s_bshot_timing.tx_bit_ticks - low_ticks,
    };
}

static void bshot_fill_tx_symbols(const bshot_rmt_channel_t *channel, uint16_t frame)
{
    rmt_symbol_word_t *tx_mem = bshot_tx_mem_base(channel);

    memset(tx_mem, 0, sizeof(rmt_symbol_word_t) * SOC_RMT_MEM_WORDS_PER_CHANNEL);
    for (uint32_t i = 0; i < BDSHOT_FRAME_BITS; i++) {
        bool bit = (frame & (1U << (BDSHOT_FRAME_BITS - 1U - i))) != 0;

        tx_mem[i] = bshot_make_tx_symbol(bit);
    }

    // Add an explicit high tail so the line returns high before the ESC turnaround window.
    tx_mem[BDSHOT_FRAME_BITS].level0 = 1;
    tx_mem[BDSHOT_FRAME_BITS].duration0 = 1;
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

    encoded_value ^= (encoded_value >> 1);

    for (int i = 0; i < 20; i += 5) {
        uint8_t nibble = bshot_decode_gcr_nibble((encoded_value >> i) & 0x1FU);

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

static bool bshot_is_erpm_frame(uint16_t raw_12bit)
{
    uint8_t prefix = (raw_12bit >> 8) & 0x0F;

    return prefix == 0x00 || (prefix & 0x01U) != 0;
}

static bool bshot_decode_reply(const bshot_rmt_channel_t *channel, bshot_reply_t *reply)
{
    uint32_t encoded = 0;
    uint32_t bit_count = 0;

    memset(reply, 0, sizeof(*reply));
    if (channel->rx_symbol_count == 0) {
        return false;
    }
    if (channel->rx_symbols[0].duration0 == 0 || channel->rx_symbols[0].level0 != 0) {
        return false;
    }

    for (uint32_t i = 0; i < channel->rx_symbol_count && i < SOC_RMT_MEM_WORDS_PER_CHANNEL; i++) {
        const rmt_symbol_word_t *symbol = &channel->rx_symbols[i];
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
    if (bshot_is_erpm_frame((uint16_t)reply->raw)) {
        reply->is_erpm = true;
        if (reply->raw == 0x0FFF) {
            reply->erpm_period_us = 0;
        } else {
            reply->erpm_period_us = ((uint32_t)reply->raw & 0x1FFU) << (reply->raw >> 9);
        }
    } else {
        reply->edt_type = (reply->raw >> 8) & 0x0F;
        reply->edt_value = reply->raw & 0xFF;
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
    s_bshot_timing.tx_zero_low_ticks = (s_bshot_timing.tx_bit_ticks * 3U + 4U) / 8U;
    s_bshot_timing.tx_one_low_ticks = (s_bshot_timing.tx_bit_ticks * 3U + 2U) / 4U;
    s_bshot_timing.reply_bit_ticks = (uint16_t)(((uint64_t)s_bshot_timing.resolution_hz * BDSHOT_REPLY_BIT_NS + 500000000ULL) / 1000000000ULL);
    s_bshot_timing.rx_idle_ticks = (uint16_t)(((uint64_t)s_bshot_timing.resolution_hz * BDSHOT_RX_IDLE_US + 500000ULL) / 1000000ULL);
}

static void bshot_init_channels(void)
{
    memset(&s_bshot_ctx, 0, sizeof(s_bshot_ctx));

    for (uint32_t i = 0; i < BDSHOT_CHANNEL_COUNT; i++) {
        bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];

        channel->index = i;
        channel->gpio_num = s_bshot_gpio_list[i];
        channel->tx_channel = i;
        channel->rx_channel = i;
        channel->tx_mem_index = i;
        channel->rx_mem_index = SOC_RMT_TX_CANDIDATES_PER_GROUP + i;
        s_bshot_ctx.tx_channel_mask |= 1U << channel->tx_channel;
    }
}

static void bshot_fill_uniform_values(uint16_t value, uint16_t values[BDSHOT_CHANNEL_COUNT])
{
    for (uint32_t i = 0; i < BDSHOT_CHANNEL_COUNT; i++) {
        values[i] = value;
    }
}

static void bshot_connect_tx_gpio(const bshot_rmt_channel_t *channel)
{
    ESP_ERROR_CHECK(gpio_set_direction(channel->gpio_num, GPIO_MODE_INPUT_OUTPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(channel->gpio_num, GPIO_PULLUP_ONLY));
    ESP_ERROR_CHECK(gpio_set_level(channel->gpio_num, 1));
    esp_rom_gpio_connect_out_signal(
        channel->gpio_num,
        rmt_periph_signals.groups[BDSHOT_RMT_GROUP_ID].channels[channel->tx_channel].tx_sig,
        false,
        false
    );
}

static void bshot_release_line_to_rx(const bshot_rmt_channel_t *channel)
{
    esp_rom_gpio_connect_out_signal(channel->gpio_num, SIG_GPIO_OUT_IDX, false, false);
    ESP_ERROR_CHECK(gpio_set_direction(channel->gpio_num, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(channel->gpio_num, GPIO_PULLUP_ONLY));
}

static void IRAM_ATTR bshot_prepare_rx_from_isr(const bshot_rmt_channel_t *channel)
{
    esp_rom_gpio_connect_out_signal(channel->gpio_num, SIG_GPIO_OUT_IDX, false, false);
    gpio_set_direction(channel->gpio_num, GPIO_MODE_INPUT);
    gpio_set_pull_mode(channel->gpio_num, GPIO_PULLUP_ONLY);

    rmt_ll_rx_enable(&RMT, channel->rx_channel, false);
    rmt_ll_rx_reset_pointer(&RMT, channel->rx_channel);
    rmt_ll_rx_set_mem_owner(&RMT, channel->rx_channel, RMT_LL_MEM_OWNER_HW);
    rmt_ll_clear_interrupt_status(
        &RMT,
        RMT_LL_EVENT_RX_DONE(channel->rx_channel) |
        RMT_LL_EVENT_RX_THRES(channel->rx_channel) |
        RMT_LL_EVENT_RX_ERROR(channel->rx_channel)
    );
    rmt_ll_rx_enable(&RMT, channel->rx_channel, true);
}

static void IRAM_ATTR bshot_rmt_isr(void *arg)
{
    (void)arg;
    BaseType_t task_woken = pdFALSE;

    for (uint32_t i = 0; i < BDSHOT_CHANNEL_COUNT; i++) {
        bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];
        uint32_t tx_status = rmt_ll_tx_get_interrupt_status(&RMT, channel->tx_channel);
        uint32_t rx_status = rmt_ll_rx_get_interrupt_status(&RMT, channel->rx_channel);

        if (tx_status & RMT_LL_EVENT_TX_DONE(channel->tx_channel)) {
            rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_TX_DONE(channel->tx_channel));
            channel->tx_done = true;

            portENTER_CRITICAL_ISR(&s_rmt_spinlock);
            bshot_prepare_rx_from_isr(channel);
            portEXIT_CRITICAL_ISR(&s_rmt_spinlock);
        }

        if (rx_status & RMT_LL_EVENT_RX_DONE(channel->rx_channel)) {
            uint32_t symbol_count;

            rmt_ll_clear_interrupt_status(&RMT, RMT_LL_EVENT_RX_DONE(channel->rx_channel));
            portENTER_CRITICAL_ISR(&s_rmt_spinlock);
            rmt_ll_rx_enable(&RMT, channel->rx_channel, false);
            symbol_count = rmt_ll_rx_get_memory_writer_offset(&RMT, channel->rx_channel);
            if (symbol_count > SOC_RMT_MEM_WORDS_PER_CHANNEL) {
                symbol_count = SOC_RMT_MEM_WORDS_PER_CHANNEL;
            }
            rmt_ll_rx_set_mem_owner(&RMT, channel->rx_channel, RMT_LL_MEM_OWNER_SW);
            memcpy(channel->rx_symbols, bshot_rx_mem_base(channel), symbol_count * sizeof(rmt_symbol_word_t));
            rmt_ll_rx_set_mem_owner(&RMT, channel->rx_channel, RMT_LL_MEM_OWNER_HW);
            portEXIT_CRITICAL_ISR(&s_rmt_spinlock);

            channel->rx_symbol_count = symbol_count;
            channel->rx_done = true;
            if (s_bshot_ctx.worker_task != NULL) {
                vTaskNotifyGiveFromISR(s_bshot_ctx.worker_task, &task_woken);
            }
        }
    }

    if (task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

static void bshot_rmt_gpio_init(void)
{
    for (uint32_t i = 0; i < BDSHOT_CHANNEL_COUNT; i++) {
        const bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];

        ESP_ERROR_CHECK(gpio_reset_pin(channel->gpio_num));
        ESP_ERROR_CHECK(gpio_set_pull_mode(channel->gpio_num, GPIO_PULLUP_ONLY));
        esp_rom_gpio_connect_in_signal(
            channel->gpio_num,
            rmt_periph_signals.groups[BDSHOT_RMT_GROUP_ID].channels[channel->rx_mem_index].rx_sig,
            false
        );
        bshot_release_line_to_rx(channel);
    }
}

static uint32_t bshot_rmt_interrupt_mask(void)
{
    uint32_t mask = 0;

    for (uint32_t i = 0; i < BDSHOT_CHANNEL_COUNT; i++) {
        const bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];

        mask |= RMT_LL_EVENT_TX_DONE(channel->tx_channel);
        mask |= RMT_LL_EVENT_TX_THRES(channel->tx_channel);
        mask |= RMT_LL_EVENT_TX_ERROR(channel->tx_channel);
        mask |= RMT_LL_EVENT_RX_DONE(channel->rx_channel);
        mask |= RMT_LL_EVENT_RX_THRES(channel->rx_channel);
        mask |= RMT_LL_EVENT_RX_ERROR(channel->rx_channel);
    }

    return mask;
}

static void bshot_rmt_hw_init(void)
{
    uint32_t interrupt_mask;

    bshot_compute_timing();
    bshot_init_channels();
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
    rmt_ll_tx_enable_sync(&RMT, true);
    rmt_ll_tx_clear_sync_group(&RMT);
    rmt_ll_tx_sync_group_add_channels(&RMT, s_bshot_ctx.tx_channel_mask);
#endif

    rmt_ll_set_group_clock_src(
        &RMT,
        0,
        RMT_CLK_SRC_APB,
        BDSHOT_RMT_GROUP_CLK_DIV,
        1,
        0
    );
    rmt_ll_enable_group_clock(&RMT, true);

    rmt_ll_tx_reset_channels_clock_div(&RMT, s_bshot_ctx.tx_channel_mask);
    rmt_ll_rx_reset_channels_clock_div(&RMT, s_bshot_ctx.tx_channel_mask);

    for (uint32_t i = 0; i < BDSHOT_CHANNEL_COUNT; i++) {
        const bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];

        rmt_ll_tx_set_channel_clock_div(&RMT, channel->tx_channel, BDSHOT_RMT_CHANNEL_CLK_DIV);
        rmt_ll_rx_set_channel_clock_div(&RMT, channel->rx_channel, BDSHOT_RMT_CHANNEL_CLK_DIV);

        rmt_ll_tx_set_mem_blocks(&RMT, channel->tx_channel, 1);
        rmt_ll_tx_set_limit(&RMT, channel->tx_channel, SOC_RMT_MEM_WORDS_PER_CHANNEL / 2U);
        rmt_ll_tx_enable_wrap(&RMT, channel->tx_channel, true);
        rmt_ll_tx_enable_carrier_modulation(&RMT, channel->tx_channel, false);
        rmt_ll_tx_fix_idle_level(&RMT, channel->tx_channel, 1, true);

        rmt_ll_rx_set_mem_blocks(&RMT, channel->rx_channel, 1);
        rmt_ll_rx_set_mem_owner(&RMT, channel->rx_channel, RMT_LL_MEM_OWNER_HW);
        rmt_ll_rx_enable_filter(&RMT, channel->rx_channel, false);
        rmt_ll_rx_set_idle_thres(&RMT, channel->rx_channel, s_bshot_timing.rx_idle_ticks);
#if SOC_RMT_SUPPORT_RX_PINGPONG
        rmt_ll_rx_set_limit(&RMT, channel->rx_channel, SOC_RMT_MEM_WORDS_PER_CHANNEL / 2U);
        rmt_ll_rx_enable_wrap(&RMT, channel->rx_channel, true);
#endif
#if SOC_RMT_SUPPORT_RX_DEMODULATION
        rmt_ll_rx_enable_carrier_demodulation(&RMT, channel->rx_channel, false);
#endif
    }

    interrupt_mask = bshot_rmt_interrupt_mask();
    ESP_ERROR_CHECK(esp_intr_alloc_intrstatus(
        rmt_periph_signals.groups[BDSHOT_RMT_GROUP_ID].irq,
        BDSHOT_RMT_INTR_FLAGS,
        (uint32_t)(uintptr_t)rmt_ll_get_interrupt_status_reg(&RMT),
        interrupt_mask,
        bshot_rmt_isr,
        NULL,
        &s_rmt_intr_handle
    ));
    rmt_ll_enable_interrupt(&RMT, interrupt_mask, true);
}

static void bshot_start_transceive(const uint16_t values[BDSHOT_CHANNEL_COUNT], bool request_telemetry)
{
    uint32_t interrupt_mask = bshot_rmt_interrupt_mask();

    for (uint32_t i = 0; i < BDSHOT_CHANNEL_COUNT; i++) {
        bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];
        uint16_t frame = bshot_make_frame(values[i], request_telemetry);

        channel->tx_done = false;
        channel->rx_done = false;
        channel->rx_symbol_count = 0;
        memset(channel->rx_symbols, 0, sizeof(channel->rx_symbols));
        bshot_fill_tx_symbols(channel, frame);
        bshot_connect_tx_gpio(channel);
    }

    portENTER_CRITICAL(&s_rmt_spinlock);
    for (uint32_t i = 0; i < BDSHOT_CHANNEL_COUNT; i++) {
        const bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];

        rmt_ll_rx_enable(&RMT, channel->rx_channel, false);
        rmt_ll_tx_stop(&RMT, channel->tx_channel);
        rmt_ll_tx_reset_pointer(&RMT, channel->tx_channel);
    }
    rmt_ll_tx_reset_channels_clock_div(&RMT, s_bshot_ctx.tx_channel_mask);
    rmt_ll_clear_interrupt_status(&RMT, interrupt_mask);
    for (uint32_t i = 0; i < BDSHOT_CHANNEL_COUNT; i++) {
        const bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];

        rmt_ll_tx_start(&RMT, channel->tx_channel);
    }
    portEXIT_CRITICAL(&s_rmt_spinlock);
}

static void bshot_stop_rx(const bshot_rmt_channel_t *channel)
{
    portENTER_CRITICAL(&s_rmt_spinlock);
    rmt_ll_rx_enable(&RMT, channel->rx_channel, false);
    portEXIT_CRITICAL(&s_rmt_spinlock);
    bshot_release_line_to_rx(channel);
}

static void bshot_wait_for_replies(bshot_reply_t replies[BDSHOT_CHANNEL_COUNT], esp_err_t results[BDSHOT_CHANNEL_COUNT])
{
    TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(BDSHOT_WAIT_TIMEOUT_MS);
    uint32_t pending_mask = (1U << BDSHOT_CHANNEL_COUNT) - 1U;

    memset(replies, 0, sizeof(bshot_reply_t) * BDSHOT_CHANNEL_COUNT);
    for (uint32_t i = 0; i < BDSHOT_CHANNEL_COUNT; i++) {
        results[i] = ESP_ERR_TIMEOUT;
    }

    while (pending_mask != 0U) {
        TickType_t now = xTaskGetTickCount();

        if (now >= deadline) {
            break;
        }
        if (ulTaskNotifyTake(pdTRUE, deadline - now) == 0) {
            break;
        }

        for (uint32_t i = 0; i < BDSHOT_CHANNEL_COUNT; i++) {
            bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];
            uint32_t channel_bit = 1U << i;

            if ((pending_mask & channel_bit) == 0U || !channel->rx_done) {
                continue;
            }

            bshot_release_line_to_rx(channel);
            results[i] = bshot_decode_reply(channel, &replies[i]) ? ESP_OK : ESP_ERR_INVALID_RESPONSE;
            pending_mask &= ~channel_bit;
        }
    }

    for (uint32_t i = 0; i < BDSHOT_CHANNEL_COUNT; i++) {
        const bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];
        uint32_t channel_bit = 1U << i;

        if ((pending_mask & channel_bit) != 0U) {
            bshot_stop_rx(channel);
        }
    }
}

static void bshot_log_reply(const bshot_rmt_channel_t *channel, const bshot_reply_t *reply)
{
    if (reply->is_erpm) {
        uint32_t erpm = reply->erpm_period_us ? (60000000UL / reply->erpm_period_us) : 0;

        ESP_LOGI(
            TAG,
            "ch=%" PRIu32 " gpio=%d erpm raw=0x%03X erpm_period_us=%" PRIu32 " erpm=%" PRIu32 " symbols=%" PRIu32,
            channel->index,
            channel->gpio_num,
            reply->raw,
            reply->erpm_period_us,
            erpm,
            channel->rx_symbol_count
        );
    } else {
        ESP_LOGI(
            TAG,
            "ch=%" PRIu32 " gpio=%d EDT raw=0x%03X type=0x%02X value=0x%02X symbols=%" PRIu32,
            channel->index,
            channel->gpio_num,
            reply->raw,
            reply->edt_type,
            reply->edt_value,
            channel->rx_symbol_count
        );
    }
}

static void bshot_log_results(const bshot_reply_t replies[BDSHOT_CHANNEL_COUNT], const esp_err_t results[BDSHOT_CHANNEL_COUNT])
{
    for (uint32_t i = 0; i < BDSHOT_CHANNEL_COUNT; i++) {
        const bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];

        if (results[i] == ESP_OK && replies[i].valid) {
            bshot_log_reply(channel, &replies[i]);
        } else {
            ESP_LOGW(
                TAG,
                "ch=%" PRIu32 " gpio=%d bDShot receive failed: %s",
                channel->index,
                channel->gpio_num,
                esp_err_to_name(results[i])
            );
        }
    }
}

static void bshot_enable_edt_sequence(void)
{
    uint16_t commands[BDSHOT_CHANNEL_COUNT];

    bshot_fill_uniform_values(BDSHOT_CMD_EXTENDED_TELEMETRY_ENABLE, commands);
    ESP_LOGI(
        TAG,
        "sending EDT enable command=%u repeats=%u on %u channels",
        BDSHOT_CMD_EXTENDED_TELEMETRY_ENABLE,
        BDSHOT_EDT_ENABLE_REPEATS,
        BDSHOT_CHANNEL_COUNT
    );

    for (uint32_t step = 0; step < BDSHOT_EDT_ENABLE_REPEATS; step++) {
        bshot_reply_t replies[BDSHOT_CHANNEL_COUNT];
        esp_err_t results[BDSHOT_CHANNEL_COUNT];

        ulTaskNotifyTake(pdTRUE, 0);
        bshot_start_transceive(commands, false);
        bshot_wait_for_replies(replies, results);

        for (uint32_t i = 0; i < BDSHOT_CHANNEL_COUNT; i++) {
            const bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];

            if (results[i] == ESP_OK && replies[i].valid) {
                if (step == (BDSHOT_EDT_ENABLE_REPEATS - 1U) && !replies[i].is_erpm && replies[i].edt_type == 0x0EU) {
                    ESP_LOGI(
                        TAG,
                        "ch=%" PRIu32 " gpio=%d EDT enable acknowledged, version=0x%02X",
                        channel->index,
                        channel->gpio_num,
                        replies[i].edt_value
                    );
                } else {
                    bshot_log_reply(channel, &replies[i]);
                }
            } else {
                ESP_LOGW(
                    TAG,
                    "ch=%" PRIu32 " gpio=%d EDT enable step %" PRIu32 "/%" PRIu32 " failed: %s",
                    channel->index,
                    channel->gpio_num,
                    step + 1U,
                    (uint32_t)BDSHOT_EDT_ENABLE_REPEATS,
                    esp_err_to_name(results[i])
                );
            }
        }
    }
}

static void bshot_worker_task(void *arg)
{
    uint16_t throttle_values[BDSHOT_CHANNEL_COUNT];

    (void)arg;
    s_bshot_ctx.worker_task = xTaskGetCurrentTaskHandle();

    bshot_enable_edt_sequence();
    bshot_fill_uniform_values(BDSHOT_TEST_THROTTLE, throttle_values);

    while (true) {
        bshot_reply_t replies[BDSHOT_CHANNEL_COUNT];
        esp_err_t results[BDSHOT_CHANNEL_COUNT];

        ulTaskNotifyTake(pdTRUE, 0);
        bshot_start_transceive(throttle_values, BDSHOT_REQUEST_TELEMETRY);
        bshot_wait_for_replies(replies, results);
        bshot_log_results(replies, results);
        vTaskDelay(pdMS_TO_TICKS(BDSHOT_PERIOD_MS));
    }
}

void app_main(void)
{
    bshot_rmt_hw_init();

    ESP_LOGI(
        TAG,
        "4x bDShot ready: gpios=%d,%d,%d,%d res=%" PRIu32 "Hz tx_ticks=%u zero=%u/%u one=%u/%u reply_ticks=%u",
        s_bshot_ctx.channels[0].gpio_num,
        s_bshot_ctx.channels[1].gpio_num,
        s_bshot_ctx.channels[2].gpio_num,
        s_bshot_ctx.channels[3].gpio_num,
        s_bshot_timing.resolution_hz,
        s_bshot_timing.tx_bit_ticks,
        s_bshot_timing.tx_zero_low_ticks,
        s_bshot_timing.tx_bit_ticks - s_bshot_timing.tx_zero_low_ticks,
        s_bshot_timing.tx_one_low_ticks,
        s_bshot_timing.tx_bit_ticks - s_bshot_timing.tx_one_low_ticks,
        s_bshot_timing.reply_bit_ticks
    );
    ESP_LOGI(
        TAG,
        "RMT interrupt flags=LEVEL3|IRAM, 35us RX idle stop on all channels"
    );

    xTaskCreate(
        bshot_worker_task,
        "bshot_worker",
        BDSHOT_WORKER_TASK_STACK,
        NULL,
        BDSHOT_WORKER_TASK_PRIORITY,
        NULL
    );
}
