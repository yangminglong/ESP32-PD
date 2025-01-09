#include <string.h>
#include <stdint.h>

#include "esp_log.h"
#include "esp_timer.h"

#include "driver/gpio.h"
#include "driver/rmt_rx.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pd_types.h"
#include "pd_config.h"
#include "pd_coding.h"
#include "pd_rx.h"
#include "pd_tx.h"
#include "pd.h"
#include "crc32.h"

#define TAG "PD-RX"

/* PD bit rate is ~600kHz, so use a multiple of that */
#define PD_RX_FREQ (10000000)
#define PD_RX_SYMS (64)
#define PD_RX_LONG_DURATION (33) /* 3.3us */
#define PD_RX_SHORT_DURATION (PD_RX_LONG_DURATION / 2)
#define PD_RX_HIGH_DURATION ((PD_RX_SHORT_DURATION * 3) / 2)

uint8_t pd_dummy_buffer[64 * 4];

extern QueueHandle_t pd_queue_rx_ack;
extern QueueHandle_t pd_queue_rx_data;
extern QueueHandle_t pd_queue_empty;

static rmt_channel_handle_t pd_rx_chan = NULL;
static volatile bool pd_rx_ongoing_flag = false;

static bool pd_acknowlegde[6] = {
    [PD_TARGET_SOP] = true,
#ifdef PD_TEST_EMARKER_CABLE
    [PD_TARGET_SOP_P] = true,
    [PD_TARGET_SOP_PP] = true,
#endif
};

bool IRAM_ATTR
pd_rx_ongoing()
{
    return pd_rx_ongoing_flag;
}

static IRAM_ATTR bool pd_rc_bmc_handle_pulse(uint32_t duration)
{
    static pd_rx_buf_t *ctx = NULL;

    /* we care ourselves for our buffers */
    if (!ctx)
    {
        if (xQueueReceiveFromISR(pd_queue_empty, &ctx, NULL) != pdTRUE)
        {
            return false;
        }
        ctx->state = PD_RX_INIT;
        ctx->start_time =  esp_timer_get_time();
    }

    /* non-pulses are ignored and signal end-of-reception */
    if (!duration)
    {
        ctx->state = PD_RX_INIT;
        return false;
    }

    /* first action - initialize working variables */
    if (ctx->state == PD_RX_INIT)
    {
        memset(ctx, 0x00, sizeof(pd_rx_buf_t));
        ctx->state = PD_RX_PREAMBLE;
    }

    /* grant some extra time if the first short one was very short */
    bool long_pulse = (duration > PD_RX_HIGH_DURATION + ctx->last_shortened);
    if (!long_pulse && duration > PD_RX_SHORT_DURATION)
    {
        ctx->last_shortened = PD_RX_SHORT_DURATION - duration;
    }
    else
    {
        ctx->last_shortened = 0;
    }

    /* depending on the last state, handle long and short pulses */
    if (ctx->short_pulse)
    {
        ctx->short_pulse = false;

        /* had a short pulse, now only another short pulse is valid */
        if (long_pulse)
        {
            /* nope, that was not expected */
            ctx->state = PD_RX_INIT;
        }
        else
        {
            ctx->bit_data >>= 1;
            ctx->bit_data |= 0x10;
            ctx->bit_count++;
        }
    }
    else
    {
        if (long_pulse)
        {
            ctx->bit_data >>= 1;
            ctx->bit_data |= 0x00;
            ctx->bit_count++;
        }
        else
        {
            /* expect another short pulse */
            ctx->short_pulse = true;
        }
    }

    /* wait for a successfully decoded SYNC_1 during preamble */
    if (ctx->state == PD_RX_PREAMBLE)
    {
        if (line_code_decode[ctx->bit_data] == SYNC_1 || line_code_decode[ctx->bit_data] == RST_1)
        {
            ctx->state = PD_RX_SOP;
            ctx->bit_count = 5;
        }
    }

    /* fall through, enqueueing the SYNC_1 as well if we just synchronized */
    if (ctx->state == PD_RX_SOP)
    {
        /* will also catch the fist SYNC symbol detected in preamble */
        if (ctx->bit_count == 5)
        {
            ctx->bit_count %= 5;
            uint8_t symbol = line_code_decode[ctx->bit_data];

            ctx->symbols[ctx->symbol_count++] = symbol;

            if (ctx->symbol_count >= 4)
            {
                switch (BUILD_LE_UINT32(ctx->symbols, 0))
                {
                case TARGET_SOP:
                    ctx->target = PD_TARGET_SOP;
                    ctx->state = PD_RX_PAYLOAD;
                    break;
                case TARGET_SOP_P:
                    ctx->target = PD_TARGET_SOP_P;
                    ctx->state = PD_RX_PAYLOAD;
                    break;
                case TARGET_SOP_PP:
                    ctx->target = PD_TARGET_SOP_PP;
                    ctx->state = PD_RX_PAYLOAD;
                    break;
                case TARGET_SOP_PD:
                    ctx->target = PD_TARGET_SOP_PD;
                    ctx->state = PD_RX_PAYLOAD;
                    break;
                case TARGET_SOP_PPD:
                    ctx->target = PD_TARGET_SOP_PPD;
                    ctx->state = PD_RX_PAYLOAD;
                    break;
                case TARGET_HARD_RESET:
                    ctx->target = PD_TARGET_HARD_RESET;
                    ctx->state = PD_RX_PAYLOAD;
                    break;
                case TARGET_CABLE_RESET:
                    ctx->target = PD_TARGET_CABLE_RESET;
                    ctx->state = PD_RX_PAYLOAD;
                    break;
                default:
                    ctx->state = PD_RX_INIT;
                    break;
                }
            }
        }
    }

    /* fall through, enqueueing the SYNC_1 as well if we just synchronized */
    if (ctx->state == PD_RX_PAYLOAD)
    {
        if (ctx->bit_count == 5)
        {
            ctx->bit_count %= 5;
            uint8_t symbol = line_code_decode[ctx->bit_data];

            if (ctx->symbol_count < sizeof(ctx->symbols))
            {
                ctx->symbols[ctx->symbol_count++] = symbol;
            }

            if (symbol == EOP)
            {
                ctx->state = PD_RX_FINISHED;
            }
        }
    }

    if (ctx->state == PD_RX_FINISHED)
    {
        pd_rx_ongoing_flag = false;
        /* do most of the parsing already here as it is time critical.
           unfortunately the RMT TX cannot be started from IRAM, so we need a bottom half. */

        /* in doubt hand over raw frame */
        ctx->type = PD_BUF_TYPE_SYMBOLS;

        /* check if that was a valid packet with 5 symbols (4 SOP plus EOP) */
        if (ctx->symbol_count >= 5)
        {
            ctx->length = MIN(sizeof(ctx->payload), (ctx->symbol_count - 5) / 2);

            for (int pos = 0; pos < ctx->length; pos++)
            {
                ctx->payload[pos] = ctx->symbols[4 + 2 * pos + 0] | (ctx->symbols[4 + 2 * pos + 1] << 4);
            }

            if (ctx->length > 4)
            {
                /* 1.2us for CRC calculation */
                uint32_t crc_calc = crc32buf(ctx->payload, ctx->length - 4);
                uint32_t crc_pkt = BUILD_LE_UINT32(ctx->payload, ctx->length - 4);
                if (crc_calc == crc_pkt)
                {
                    ctx->type = PD_BUF_TYPE_DATA;
                }
            }
        }

        bool yield = false;

        ctx->dir = pd_acknowlegde[ctx->target] ? PD_PACKET_RECEIVED_ACKNOWLEDGED : PD_PACKET_RECEIVED;

        if (ctx->type == PD_BUF_TYPE_DATA && ctx->dir == PD_PACKET_RECEIVED_ACKNOWLEDGED)
        {
            uint16_t header = ctx->payload[0] | (ctx->payload[1] << 8);

            /* Extract individual fields */
            uint8_t num_data_objects = (header >> 12) & 0x07;
            uint32_t message_id = (header >> 9) & 0x07;
            uint8_t data_role = (header >> 5) & 0x01;
            uint8_t message_type = header & 0x1F;
            bool is_data = num_data_objects > 0;

            pd_rx_ack_t ack = {.message_id = message_id, .target = ctx->target};

            /* do not respond to our packets or to GoodCRC */
            if (data_role == PD_DATA_ROLE_DFP && (is_data || message_type != PD_CONTROL_GOOD_CRC))
            {
                if (xQueueSendFromISR(pd_queue_rx_ack, &ack, NULL) != pdTRUE)
                {
                    ESP_LOGE(TAG, "Failed to enqueue tx ack");
                }
                yield = true;
            }
        }

        /* Send the buffer to the user space queue and get a new one */
        if (xQueueSendFromISR(pd_queue_rx_data, &ctx, NULL) != pdTRUE)
        {
            xQueueSendFromISR(pd_queue_empty, &ctx, NULL);
        }
        ctx = NULL;

        return yield;
    }

    return false;
}

void IRAM_ATTR pd_rx_start()
{
    rmt_receive_config_t receive_config = {
        .signal_range_min_ns = 800,
        .signal_range_max_ns = 5000,
        .flags.en_partial_rx = true};

    ESP_ERROR_CHECK(rmt_receive(pd_rx_chan, pd_dummy_buffer, sizeof(pd_dummy_buffer), &receive_config));
    gpio_set_level(GPIO_CC1_IN, 1);
}

static IRAM_ATTR bool pd_rx_done_cbr(rmt_channel_handle_t channel, const rmt_rx_done_event_data_t *edata, void *user_data)
{
    bool yield = false;

    /*
    if (pd_tx_ongoing())
    {
        if (edata->flags.is_last)
        {
            pd_rc_bmc_handle_pulse(0);
            pd_rx_start();
        }
        return false;
    }*/

    pd_rx_ongoing_flag = true;

    for (int pos = 0; pos < edata->num_symbols; pos++)
    {
        yield |= pd_rc_bmc_handle_pulse(edata->received_symbols[pos].duration0);
        yield |= pd_rc_bmc_handle_pulse(edata->received_symbols[pos].duration1);
    }

    if (edata->flags.is_last)
    {
        pd_rc_bmc_handle_pulse(0);
        pd_rx_ongoing_flag = false;
        pd_rx_start();
    }

    return yield;
}

void pd_rx_init()
{
    rmt_rx_channel_config_t rx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = PD_RX_FREQ,
        .mem_block_symbols = PD_RX_SYMS,
        .gpio_num = GPIO_CC1_IN,
        .flags.invert_in = false,
        .flags.with_dma = false,
        .flags.io_loop_back = true,
        .intr_priority = 3,
    };
    rmt_rx_event_callbacks_t cbs = {
        .on_recv_done = pd_rx_done_cbr,
    };

    ESP_ERROR_CHECK(rmt_new_rx_channel(&rx_chan_config, &pd_rx_chan));
    ESP_ERROR_CHECK(rmt_rx_register_event_callbacks(pd_rx_chan, &cbs, NULL));
    ESP_ERROR_CHECK(rmt_enable(pd_rx_chan));
    pd_rx_start();
}
