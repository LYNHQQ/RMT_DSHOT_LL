#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "driver/gpio.h"
#include "esp_attr.h"
#include "esp_clk_tree.h"
#include "esp_err.h"
#include "esp_intr_alloc.h"
#include "esp_log.h"
#include "esp_private/periph_ctrl.h"
#include "esp_rom_gpio.h"
#include "hal/gpio_ll.h"
#include "hal/rmt_ll.h"
#include "hal/rmt_types.h"
#include "soc/gpio_sig_map.h"
#include "soc/rmt_periph.h"
#include "soc/soc_caps.h"
#include "bidir_dshot.h"

#define BDSHOT_RMT_GROUP_ID               0
#define BDSHOT_RMT_GROUP_CLK_DIV          1
#define BDSHOT_RMT_CHANNEL_CLK_DIV        2
#define BDSHOT_SYNC_LEADER_CHANNEL        1U
#define BDSHOT_TX_DONE_ALL_MASK           (RMT_LL_EVENT_TX_DONE(0) | RMT_LL_EVENT_TX_DONE(1) | RMT_LL_EVENT_TX_DONE(2) | RMT_LL_EVENT_TX_DONE(3))
#define BDSHOT_BAUD_HZ                    600000U
#define BDSHOT_FRAME_BITS                 16U
#define BDSHOT_REPLY_BITS                 21U
#define BDSHOT_REPLY_BIT_NS               1333U
#define BDSHOT_RX_IDLE_US                 35U
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

typedef bidir_dshot_reply_t bshot_reply_t;

typedef struct {
    uint32_t index;
    int gpio_num;
    uint32_t tx_channel;
    uint32_t rx_channel;
    uint32_t tx_mem_index;
    uint32_t rx_mem_index;
    volatile uint32_t rx_symbol_count;
} bshot_rmt_channel_t;

typedef struct {
    uint32_t tx_channel_mask;
    bshot_rmt_channel_t channels[BIDIR_DSHOT_CHANNEL_COUNT];
} bshot_rmt_context_t;

extern rmt_block_mem_t RMTMEM;

static const char *TAG = "bidir_dshot";
static const bidir_dshot_config_t s_bshot_default_config = BIDIR_DSHOT_DEFAULT_CONFIG();
static intr_handle_t s_rmt_intr_handle;
static portMUX_TYPE s_rmt_spinlock = portMUX_INITIALIZER_UNLOCKED;
static bidir_dshot_config_t s_bshot_config;
static bshot_timing_t s_bshot_timing;
static bshot_rmt_context_t s_bshot_ctx;
static bool s_bshot_initialized;
static bool s_bshot_rx_pending;

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

static void IRAM_ATTR bshot_fill_tx_symbols(const bshot_rmt_channel_t *channel, uint16_t frame)
{
    rmt_symbol_word_t *tx_mem = bshot_tx_mem_base(channel);

    memset(tx_mem, 0, sizeof(rmt_symbol_word_t) * SOC_RMT_MEM_WORDS_PER_CHANNEL);
    for (uint32_t i = 0; i < BDSHOT_FRAME_BITS; i++) {
        bool bit = (frame & (1U << (BDSHOT_FRAME_BITS - 1U - i))) != 0;

        tx_mem[i] = bshot_make_tx_symbol(bit);
    }

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

static bool bshot_decode_reply_symbols(const rmt_symbol_word_t *symbols, uint32_t symbol_count, bshot_reply_t *reply)
{
    uint32_t encoded = 0;
    uint32_t bit_count = 0;

    memset(reply, 0, sizeof(*reply));
    if (symbol_count == 0) {
        return false;
    }
    if (symbols[0].duration0 == 0 || symbols[0].level0 != 0) {
        return false;
    }

    for (uint32_t i = 0; i < symbol_count && i < SOC_RMT_MEM_WORDS_PER_CHANNEL; i++) {
        const rmt_symbol_word_t *symbol = &symbols[i];
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

static bool IRAM_ATTR bshot_read_reply_from_hw(const bshot_rmt_channel_t *channel, bshot_reply_t *reply)
{
    bool ok;
    const rmt_symbol_word_t *rx_mem = bshot_rx_mem_base(channel);

    portENTER_CRITICAL(&s_rmt_spinlock);
    rmt_ll_rx_set_mem_owner(&RMT, channel->rx_channel, RMT_LL_MEM_OWNER_SW);
    portEXIT_CRITICAL(&s_rmt_spinlock);

    ok = bshot_decode_reply_symbols(rx_mem, channel->rx_symbol_count, reply);

    portENTER_CRITICAL(&s_rmt_spinlock);
    rmt_ll_rx_set_mem_owner(&RMT, channel->rx_channel, RMT_LL_MEM_OWNER_HW);
    portEXIT_CRITICAL(&s_rmt_spinlock);

    return ok;
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

    for (uint32_t i = 0; i < BIDIR_DSHOT_CHANNEL_COUNT; i++) {
        bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];

        channel->index = i;
        channel->gpio_num = s_bshot_config.gpio_num[i];
        channel->tx_channel = i;
        channel->rx_channel = i;
        channel->tx_mem_index = i;
        channel->rx_mem_index = SOC_RMT_TX_CANDIDATES_PER_GROUP + i;
        s_bshot_ctx.tx_channel_mask |= 1U << channel->tx_channel;
    }
}

static void IRAM_ATTR bshot_connect_tx_gpio(const bshot_rmt_channel_t *channel)
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

static void IRAM_ATTR bshot_release_line_to_rx(const bshot_rmt_channel_t *channel)
{
    esp_rom_gpio_connect_out_signal(channel->gpio_num, SIG_GPIO_OUT_IDX, false, false);
    ESP_ERROR_CHECK(gpio_set_direction(channel->gpio_num, GPIO_MODE_INPUT));
    ESP_ERROR_CHECK(gpio_set_pull_mode(channel->gpio_num, GPIO_PULLUP_ONLY));
}

static inline void IRAM_ATTR bshot_prepare_rx_from_isr(const bshot_rmt_channel_t *channel)
{
    gpio_ll_matrix_out_default(&GPIO, channel->gpio_num);
    gpio_ll_output_disable(&GPIO, channel->gpio_num);
    gpio_ll_input_enable(&GPIO, channel->gpio_num);
    gpio_ll_pullup_en(&GPIO, channel->gpio_num);
    gpio_ll_pulldown_dis(&GPIO, channel->gpio_num);

    rmt_ll_rx_enable(&RMT, channel->rx_channel, false);
    rmt_ll_rx_reset_pointer(&RMT, channel->rx_channel);
    rmt_ll_rx_set_mem_owner(&RMT, channel->rx_channel, RMT_LL_MEM_OWNER_HW);
    rmt_ll_clear_interrupt_status(
        &RMT,
        RMT_LL_EVENT_RX_DONE(channel->rx_channel) |
        RMT_LL_EVENT_RX_ERROR(channel->rx_channel)
    );
    rmt_ll_rx_enable(&RMT, channel->rx_channel, true);
}

static void IRAM_ATTR bshot_rmt_isr(void *arg)
{
    (void)arg;
    bshot_rmt_channel_t *ch0 = &s_bshot_ctx.channels[0];
    bshot_rmt_channel_t *ch1 = &s_bshot_ctx.channels[1];
    bshot_rmt_channel_t *ch2 = &s_bshot_ctx.channels[2];
    bshot_rmt_channel_t *ch3 = &s_bshot_ctx.channels[3];
#if SOC_RMT_SUPPORT_TX_SYNCHRO
    if ((RMT.int_st.val & RMT_LL_EVENT_TX_DONE(BDSHOT_SYNC_LEADER_CHANNEL)) == 0U) {
        return;
    }

    portENTER_CRITICAL_ISR(&s_rmt_spinlock);
    rmt_ll_clear_interrupt_status(&RMT, BDSHOT_TX_DONE_ALL_MASK);
    bshot_prepare_rx_from_isr(ch0);
    bshot_prepare_rx_from_isr(ch1);
    bshot_prepare_rx_from_isr(ch2);
    bshot_prepare_rx_from_isr(ch3);
    portEXIT_CRITICAL_ISR(&s_rmt_spinlock);
#else
    uint32_t int_st = RMT.int_st.val;
    uint32_t tx_done_clear_mask = 0;
    bool ch0_tx_done = (int_st & RMT_LL_EVENT_TX_DONE(0)) != 0;
    bool ch1_tx_done = (int_st & RMT_LL_EVENT_TX_DONE(1)) != 0;
    bool ch2_tx_done = (int_st & RMT_LL_EVENT_TX_DONE(2)) != 0;
    bool ch3_tx_done = (int_st & RMT_LL_EVENT_TX_DONE(3)) != 0;

    if (ch0_tx_done) {
        tx_done_clear_mask |= RMT_LL_EVENT_TX_DONE(0);
    }
    if (ch1_tx_done) {
        tx_done_clear_mask |= RMT_LL_EVENT_TX_DONE(1);
    }
    if (ch2_tx_done) {
        tx_done_clear_mask |= RMT_LL_EVENT_TX_DONE(2);
    }
    if (ch3_tx_done) {
        tx_done_clear_mask |= RMT_LL_EVENT_TX_DONE(3);
    }
    if (tx_done_clear_mask == 0U) {
        return;
    }

    portENTER_CRITICAL_ISR(&s_rmt_spinlock);
    rmt_ll_clear_interrupt_status(&RMT, tx_done_clear_mask);
    if (ch0_tx_done) {
        bshot_prepare_rx_from_isr(ch0);
    }
    if (ch1_tx_done) {
        bshot_prepare_rx_from_isr(ch1);
    }
    if (ch2_tx_done) {
        bshot_prepare_rx_from_isr(ch2);
    }
    if (ch3_tx_done) {
        bshot_prepare_rx_from_isr(ch3);
    }
    portEXIT_CRITICAL_ISR(&s_rmt_spinlock);
#endif
}

static void IRAM_ATTR bshot_rmt_gpio_init(void)
{
    for (uint32_t i = 0; i < BIDIR_DSHOT_CHANNEL_COUNT; i++) {
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

static uint32_t IRAM_ATTR bshot_rmt_interrupt_mask(void)
{
#if SOC_RMT_SUPPORT_TX_SYNCHRO
    return RMT_LL_EVENT_TX_DONE(BDSHOT_SYNC_LEADER_CHANNEL);
#else
    return BDSHOT_TX_DONE_ALL_MASK;
#endif
}

static void IRAM_ATTR bshot_rmt_hw_init(void)
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

    for (uint32_t i = 0; i < BIDIR_DSHOT_CHANNEL_COUNT; i++) {
        const bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];

        rmt_ll_tx_set_channel_clock_div(&RMT, channel->tx_channel, BDSHOT_RMT_CHANNEL_CLK_DIV);
        rmt_ll_rx_set_channel_clock_div(&RMT, channel->rx_channel, BDSHOT_RMT_CHANNEL_CLK_DIV);

        rmt_ll_tx_set_mem_blocks(&RMT, channel->tx_channel, 1);
        rmt_ll_tx_set_limit(&RMT, channel->tx_channel, SOC_RMT_MEM_WORDS_PER_CHANNEL);
        rmt_ll_tx_enable_wrap(&RMT, channel->tx_channel, false);
        rmt_ll_tx_enable_carrier_modulation(&RMT, channel->tx_channel, false);
        rmt_ll_tx_fix_idle_level(&RMT, channel->tx_channel, 1, true);

        rmt_ll_rx_set_mem_blocks(&RMT, channel->rx_channel, 1);
        rmt_ll_rx_set_mem_owner(&RMT, channel->rx_channel, RMT_LL_MEM_OWNER_HW);
        rmt_ll_rx_enable_filter(&RMT, channel->rx_channel, false);
        rmt_ll_rx_set_idle_thres(&RMT, channel->rx_channel, s_bshot_timing.rx_idle_ticks);
#if SOC_RMT_SUPPORT_RX_PINGPONG
        rmt_ll_rx_set_limit(&RMT, channel->rx_channel, SOC_RMT_MEM_WORDS_PER_CHANNEL);
        rmt_ll_rx_enable_wrap(&RMT, channel->rx_channel, false);
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

static void IRAM_ATTR bshot_start_transceive(const uint16_t values[BIDIR_DSHOT_CHANNEL_COUNT], bool request_telemetry)
{
    uint32_t interrupt_mask = bshot_rmt_interrupt_mask();

    for (uint32_t i = 0; i < BIDIR_DSHOT_CHANNEL_COUNT; i++) {
        bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];
        uint16_t frame = bshot_make_frame(values[i], request_telemetry);

        channel->rx_symbol_count = 0;
        bshot_fill_tx_symbols(channel, frame);
        bshot_connect_tx_gpio(channel);
    }

    portENTER_CRITICAL(&s_rmt_spinlock);
    for (uint32_t i = 0; i < BIDIR_DSHOT_CHANNEL_COUNT; i++) {
        const bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];

        rmt_ll_rx_enable(&RMT, channel->rx_channel, false);
        rmt_ll_rx_set_mem_owner(&RMT, channel->rx_channel, RMT_LL_MEM_OWNER_HW);
        rmt_ll_tx_stop(&RMT, channel->tx_channel);
        rmt_ll_tx_reset_pointer(&RMT, channel->tx_channel);
    }
    rmt_ll_tx_reset_channels_clock_div(&RMT, s_bshot_ctx.tx_channel_mask);
    rmt_ll_clear_interrupt_status(&RMT, interrupt_mask | BDSHOT_TX_DONE_ALL_MASK);
    for (uint32_t i = 0; i < BIDIR_DSHOT_CHANNEL_COUNT; i++) {
        const bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];

        rmt_ll_tx_start(&RMT, channel->tx_channel);
    }
    portEXIT_CRITICAL(&s_rmt_spinlock);
}

static void IRAM_ATTR bshot_capture_reply_symbols(esp_err_t results[BIDIR_DSHOT_CHANNEL_COUNT])
{
    bshot_rmt_channel_t *ch0 = &s_bshot_ctx.channels[0];
    bshot_rmt_channel_t *ch1 = &s_bshot_ctx.channels[1];
    bshot_rmt_channel_t *ch2 = &s_bshot_ctx.channels[2];
    bshot_rmt_channel_t *ch3 = &s_bshot_ctx.channels[3];
    uint32_t ch0_symbol_count;
    uint32_t ch1_symbol_count;
    uint32_t ch2_symbol_count;
    uint32_t ch3_symbol_count;

    portENTER_CRITICAL(&s_rmt_spinlock);
    rmt_ll_rx_enable(&RMT, ch0->rx_channel, false);
    rmt_ll_rx_enable(&RMT, ch1->rx_channel, false);
    rmt_ll_rx_enable(&RMT, ch2->rx_channel, false);
    rmt_ll_rx_enable(&RMT, ch3->rx_channel, false);
    ch0_symbol_count = rmt_ll_rx_get_memory_writer_offset(&RMT, ch0->rx_channel);
    ch1_symbol_count = rmt_ll_rx_get_memory_writer_offset(&RMT, ch1->rx_channel);
    ch2_symbol_count = rmt_ll_rx_get_memory_writer_offset(&RMT, ch2->rx_channel);
    ch3_symbol_count = rmt_ll_rx_get_memory_writer_offset(&RMT, ch3->rx_channel);
    portEXIT_CRITICAL(&s_rmt_spinlock);

    if (ch0_symbol_count > SOC_RMT_MEM_WORDS_PER_CHANNEL) {
        ch0_symbol_count = SOC_RMT_MEM_WORDS_PER_CHANNEL;
    }
    if (ch1_symbol_count > SOC_RMT_MEM_WORDS_PER_CHANNEL) {
        ch1_symbol_count = SOC_RMT_MEM_WORDS_PER_CHANNEL;
    }
    if (ch2_symbol_count > SOC_RMT_MEM_WORDS_PER_CHANNEL) {
        ch2_symbol_count = SOC_RMT_MEM_WORDS_PER_CHANNEL;
    }
    if (ch3_symbol_count > SOC_RMT_MEM_WORDS_PER_CHANNEL) {
        ch3_symbol_count = SOC_RMT_MEM_WORDS_PER_CHANNEL;
    }

    ch0->rx_symbol_count = ch0_symbol_count;
    ch1->rx_symbol_count = ch1_symbol_count;
    ch2->rx_symbol_count = ch2_symbol_count;
    ch3->rx_symbol_count = ch3_symbol_count;

    results[0] = ch0_symbol_count ? ESP_OK : ESP_ERR_TIMEOUT;
    results[1] = ch1_symbol_count ? ESP_OK : ESP_ERR_TIMEOUT;
    results[2] = ch2_symbol_count ? ESP_OK : ESP_ERR_TIMEOUT;
    results[3] = ch3_symbol_count ? ESP_OK : ESP_ERR_TIMEOUT;
}

static void bshot_capture_and_parse_replies(bshot_reply_t replies[BIDIR_DSHOT_CHANNEL_COUNT], esp_err_t results[BIDIR_DSHOT_CHANNEL_COUNT])
{
    memset(replies, 0, sizeof(bshot_reply_t) * BIDIR_DSHOT_CHANNEL_COUNT);
    bshot_capture_reply_symbols(results);

    for (uint32_t i = 0; i < BIDIR_DSHOT_CHANNEL_COUNT; i++) {
        const bshot_rmt_channel_t *channel = &s_bshot_ctx.channels[i];

        if (results[i] != ESP_OK) {
            continue;
        }
        if (!bshot_read_reply_from_hw(channel, &replies[i])) {
            results[i] = ESP_ERR_INVALID_RESPONSE;
        }
    }
}

static esp_err_t bshot_validate_config(const bidir_dshot_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    for (uint32_t i = 0; i < BIDIR_DSHOT_CHANNEL_COUNT; i++) {
        if (!GPIO_IS_VALID_GPIO(config->gpio_num[i])) {
            return ESP_ERR_INVALID_ARG;
        }
    }

    return ESP_OK;
}

esp_err_t bidir_dshot_init(const bidir_dshot_config_t *config)
{
    const bidir_dshot_config_t *effective_config = config ? config : &s_bshot_default_config;
    esp_err_t err = bshot_validate_config(effective_config);

    if (err != ESP_OK) {
        return err;
    }
    if (s_bshot_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    s_bshot_config = *effective_config;
    bshot_rmt_hw_init();
    s_bshot_initialized = true;
    s_bshot_rx_pending = false;

    ESP_LOGI(
        TAG,
        "4x bDShot component ready: gpios=%d,%d,%d,%d res=%" PRIu32 "Hz tx_ticks=%u zero=%u/%u one=%u/%u reply_ticks=%u",
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
        "RMT interrupt flags=LEVEL3|IRAM, TX_DONE ISR arms RX, previous reply captured before next TX, fixed 4-channel fast path"
    );

    return ESP_OK;
}

esp_err_t IRAM_ATTR bidir_dshot_send_raw(
    const uint16_t values[BIDIR_DSHOT_CHANNEL_COUNT],
    bool request_telemetry
)
{
    if (!s_bshot_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (values == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_bshot_rx_pending) {
        return ESP_ERR_INVALID_STATE;
    }

    bshot_start_transceive(values, request_telemetry);
    s_bshot_rx_pending = true;
    return ESP_OK;
}

esp_err_t IRAM_ATTR bidir_dshot_send(const uint16_t values[BIDIR_DSHOT_CHANNEL_COUNT])
{
    return bidir_dshot_send_raw(values, true);
}

esp_err_t IRAM_ATTR bidir_dshot_receive(
    bidir_dshot_reply_t replies[BIDIR_DSHOT_CHANNEL_COUNT],
    esp_err_t results[BIDIR_DSHOT_CHANNEL_COUNT]
)
{
    if (!s_bshot_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    if (replies == NULL || results == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (!s_bshot_rx_pending) {
        return ESP_ERR_INVALID_STATE;
    }

    bshot_capture_and_parse_replies(replies, results);
    s_bshot_rx_pending = false;
    return ESP_OK;
}
