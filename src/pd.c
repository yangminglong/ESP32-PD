
#include <string.h>
#include <stdint.h>

#include "esp_system.h"
#include "esp_flash.h"
#include "esp_rom_gpio.h"
#include "esp_timer.h"
#include "esp_chip_info.h"
#include "esp_efuse.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_random.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pd_types.h"
#include "pd_config.h"
#include "pd_rx.h"
#include "pd_tx.h"
#include "pd.h"
#include "crc32.h"

QueueHandle_t pd_queue_rx_ack;
QueueHandle_t pd_queue_rx_data;
QueueHandle_t pd_queue_empty;

static pd_state_t state;

#define TAG "PD"

void pd_mode(pd_mode_t mode)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .pull_down_en = 0,
        .pull_up_en = 0};

    switch (mode)
    {
    case PD_MODE_IDLE:
    {
#ifdef GPIO_CC1
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << GPIO_CC1);
        gpio_config(&io_conf);
#endif

        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << GPIO_PD);
        gpio_config(&io_conf);

        break;
    }

    case PD_MODE_SINK:
    {
#ifdef GPIO_CC1
        io_conf.mode = GPIO_MODE_INPUT;
        io_conf.pin_bit_mask = (1ULL << GPIO_CC1);
        gpio_config(&io_conf);
#endif

        io_conf.mode = GPIO_MODE_OUTPUT;
        io_conf.pin_bit_mask = (1ULL << GPIO_PD);
        gpio_config(&io_conf);

        io_conf.mode = GPIO_MODE_INPUT_OUTPUT;
        io_conf.pull_down_en = true;
        io_conf.pin_bit_mask = (1ULL << GPIO_CC1_IN);
        gpio_config(&io_conf);

        gpio_set_drive_capability(GPIO_PD, GPIO_DRIVE_CAP_3);
        gpio_set_drive_capability(GPIO_CC1_IN, GPIO_DRIVE_CAP_0);

        gpio_set_level(GPIO_CC1_IN, 1);
        gpio_set_level(GPIO_PD, 0);

        break;
    }
    }
}

/* Function to initialize the queues and preallocate buffers */
static void pd_init_queues(void)
{
    /* Create queues */
    pd_queue_rx_ack = xQueueCreate(PD_BUFFER_COUNT, sizeof(uint32_t));
    if (pd_queue_rx_ack == NULL)
    {
        ESP_LOGE(TAG, "Failed to create buffer_from_isr_queue");
        return;
    }

    pd_queue_rx_data = xQueueCreate(PD_BUFFER_COUNT, sizeof(pd_rx_buf_t *));
    if (pd_queue_rx_data == NULL)
    {
        ESP_LOGE(TAG, "Failed to create buffer_from_isr_queue");
        return;
    }

    pd_queue_empty = xQueueCreate(PD_BUFFER_COUNT, sizeof(pd_rx_buf_t *));
    if (pd_queue_empty == NULL)
    {
        ESP_LOGE(TAG, "Failed to create pd_queue_empty");
        return;
    }

    /* Preallocate buffers and push them to the pd_queue_empty */
    for (int index = 0; index < PD_BUFFER_COUNT; index++)
    {
        pd_rx_buf_t *buffer = calloc(1, sizeof(pd_rx_buf_t));
        if (buffer == NULL)
        {
            ESP_LOGE(TAG, "Failed to allocate buffer %d", index);
            break;
        }

        if (xQueueSend(pd_queue_empty, &buffer, portMAX_DELAY) != pdTRUE)
        {
            ESP_LOGE(TAG, "Failed to add buffer %d to pd_queue_empty", index);
            free(buffer);
        }
    }
}

void pd_user_task(void *pvParameters)
{
    pd_rx_buf_t *rx_data;

    while (1)
    {
        if (xQueueReceive(pd_queue_rx_data, &rx_data, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        /* disable all logging messages - the delay is too long. the power supply causes a RST if the CAP message
        doesn't get answered immediately. */

        /* ToDo: split protocol handling and logging messages into separate tasks */
        if (0)
            ESP_LOGI(TAG, "");

        switch (rx_data->type)
        {
        case PD_BUF_TYPE_DATA:
        {
            switch (rx_data->target)
            {
            case TARGET_SOP:
                if (0)
                    ESP_LOGI(TAG, "Target: SOP");
                break;
            case TARGET_SOP_P:
                if (0)
                    ESP_LOGI(TAG, "Target: SOP'");
                break;
            case TARGET_SOP_PP:
                if (0)
                    ESP_LOGI(TAG, "Target: SOP''");
                break;
            case TARGET_SOP_PD:
                if (0)
                    ESP_LOGI(TAG, "Target: SOP' Debug");
                break;
            case TARGET_SOP_PPD:
                if (0)
                    ESP_LOGI(TAG, "Target: SOP'' Debug");
                break;
            default:
                if (0)
                    ESP_LOGI(TAG, "Target: Unknown");
                break;
            }

            uint16_t header = (rx_data->payload[1] << 8) | rx_data->payload[0];

            /* Extract individual fields */
            uint8_t extended = (header >> 15) & 0x01;
            uint8_t num_data_objects = (header >> 12) & 0x07;
            uint8_t message_id = (header >> 9) & 0x07;
            uint8_t power_role_or_cable_plug = (header >> 8) & 0x01;
            uint8_t spec_revision = (header >> 6) & 0x03;
            uint8_t data_role_or_reserved = (header >> 5) & 0x01;
            uint8_t message_type = header & 0x1F;

            uint32_t pdos[8];
            for (int i = 0; i < num_data_objects; i++)
            {
                pdos[i] = BUILD_LE_UINT32(rx_data->payload, 2 + i * 4);
            }

            /* Print the fields */
            if (0)
                ESP_LOGI(TAG, "  Header Fields%s", (extended) ? " (extended)" : "");
            if (0)
                ESP_LOGI(TAG, "    DO: %" PRIu8 ", ID: %" PRIu8 ", PPR/CP: %" PRIu8 ", Rev: %" PRIu8 ", PDR: %" PRIu8 ", Type: %" PRIu8 "",
                         num_data_objects, message_id, power_role_or_cable_plug, spec_revision, data_role_or_reserved, message_type);

            /* when no objects, then its a control message */
            if (num_data_objects == 0)
            {
                if (0)
                    ESP_LOGI(TAG, "  Control:");
                switch (message_type)
                {
                case PD_CONTROL_SOFT_RESET:
                    if (0)
                        ESP_LOGI(TAG, "    Soft Reset");
                    uint8_t buf[64] = {0};
                    int pos = 0;

                    uint16_t header_tx = pd_tx_header(0, 0, state.message_id++, 0, 2, PD_DATA_ROLE_UFP, PD_CONTROL_ACCEPT);
                    buf[pos++] = header_tx;
                    buf[pos++] = header_tx >> 8;

                    uint32_t crc = crc32buf(buf, pos);
                    buf[pos++] = crc;
                    buf[pos++] = crc >> 8;
                    buf[pos++] = crc >> 16;
                    buf[pos++] = crc >> 24;

                    pd_tx(buf, pos);
                    break;

                case PD_CONTROL_GOOD_CRC:
                    if (0)
                        ESP_LOGI(TAG, "    Good CRC");
                    break;

                case PD_CONTROL_REJECT:
                    if (0)
                        ESP_LOGI(TAG, "    Rejected");
                    if (state.requested_object != 0)
                    {
                        state.accepted_object = 0;
                        state.requested_object = 0;
                    }
                    break;

                case PD_CONTROL_ACCEPT:
                    if (0)
                        ESP_LOGI(TAG, "    Accepted");
                    if (state.requested_object != 0)
                    {
                        state.accepted_object = state.requested_object;
                        state.requested_object = 0;
                    }
                    break;

                case PD_CONTROL_PS_RDY:
                    if (0)
                        ESP_LOGI(TAG, "    Power supply ready");
                    break;
                }
            }
            else
            {
                if (0)
                    ESP_LOGI(TAG, "  Data:");
                switch (message_type)
                {
                case PD_VENDOR_MESSAGE:
                {
                    uint32_t vdm_header = pdos[0];
                    uint16_t svid = (vdm_header & SVID_MASK) >> SVID_SHIFT;
                    uint8_t vdm_type = (vdm_header & VDM_TYPE_MASK) >> VDM_TYPE_SHIFT;
                    uint8_t vdm_version_major = (vdm_header & VDM_VERSION_MASK) >> VDM_VERSION_SHIFT;
                    uint8_t vdm_version_minor = (vdm_header & VDM_MINOR_MASK) >> VDM_MINOR_SHIFT;
                    uint8_t object_position = (vdm_header & OBJ_POS_MASK) >> OBJ_POS_SHIFT;
                    uint8_t command_type = (vdm_header & CMD_TYPE_MASK) >> CMD_TYPE_SHIFT;
                    uint8_t command = (vdm_header & COMMAND_MASK) >> COMMAND_SHIFT;

                    if (0)
                        ESP_LOGI(TAG, "    Vendor Message:");
                    if (0)
                        ESP_LOGI(TAG, "    Vendor Message:");
                    if (0)
                        ESP_LOGI(TAG, "      VDM Header 0x%08" PRIx32, vdm_header);
                    if (0)
                        ESP_LOGI(TAG, "      SVID:            0x%04X", svid);
                    if (0)
                        ESP_LOGI(TAG, "      VDM Type:        %s", vdm_type ? "Structured" : "Unstructured");
                    if (vdm_type)
                    {
                        if (0)
                            ESP_LOGI(TAG, "      VDM Version:     %u.%u", vdm_version_major, vdm_version_minor);
                        if (0)
                            ESP_LOGI(TAG, "      Object Position: %u", object_position);
                        if (0)
                            ESP_LOGI(TAG, "      Command Type:    %u", command_type);
                        if (0)
                            ESP_LOGI(TAG, "      Command:         %u", command);
                    }

                    for (uint32_t index = 1; index < num_data_objects; index++)
                    {
                        if (0)
                            ESP_LOGI(TAG, "      Data #%" PRIu32 ": 0x%08" PRIx32, index, pdos[index]);
                    }
                    break;
                }
                break;

                case PD_DATA_SOURCE_CAPABILITIES:
                    state.requested_object = 0;
                    if (0)
                        ESP_LOGI(TAG, "    Source Capabilities:");
                    for (uint32_t index = 0; index < num_data_objects; index++)
                    {
                        uint32_t pdo_value = pdos[index];
                        uint32_t type = (pdo_value >> 30) & 0x03;

                        switch (type)
                        {
                        case 0:
                        {
                            if (0)
                                ESP_LOGI(TAG, "    #%" PRIu32 ": Fixed Supply PDO", index);
                            uint32_t voltage = (pdo_value >> 10) & 0x3FF; /* in 50mV units */
                            uint32_t current = pdo_value & 0x3FF;         /* in 10mA units */
                            uint32_t epr_capable = (pdo_value >> 23) & 0x01;
                            float watts = (voltage * 50 * current * 10) / 1000000.0;

                            if (0)
                                ESP_LOGI(TAG, "        %" PRIu32 " mV, %" PRIu32 " mA (%2.2f W%s)",
                                         voltage * 50, current * 10, watts,
                                         epr_capable ? ", EPR" : "");

                            if (voltage == state.request_voltage_mv / 50)
                            {
                                // ESP_LOGW(TAG, "        ##### Requesting this mode #####");
                                state.requested_object = index + 1;
                            }
                            break;
                        }
                        case 1:
                        {
                            if (0)
                                ESP_LOGI(TAG, "    #%" PRIu32 ": Battery Supply PDO", index);

                            uint32_t max_voltage = (pdo_value >> 20) & 0x1FF; /* in 50mV units */
                            uint32_t min_voltage = (pdo_value >> 10) & 0x1FF; /* in 50mV units */
                            uint32_t max_power = (pdo_value >> 0) & 0x1FF;    /* in 250mW units */

                            if (0)
                                ESP_LOGI(TAG, "        %" PRIu32 " mV - %" PRIu32 " mV, %" PRIu32 " mW",
                                         min_voltage * 50, max_voltage * 50, max_power * 250);
                            break;
                        }
                        case 2:
                        {
                            if (0)
                                ESP_LOGI(TAG, "    #%" PRIu32 ": Variable Supply (non-battery) PDO", index);

                            uint32_t max_voltage = (pdo_value >> 20) & 0x1FF; /* in 50mV units */
                            uint32_t min_voltage = (pdo_value >> 10) & 0x1FF; /* in 50mV units */
                            uint32_t max_power = (pdo_value >> 0) & 0x1FF;    /* in 250mW units */

                            if (0)
                                ESP_LOGI(TAG, "        %" PRIu32 " mV - %" PRIu32 " mV, %" PRIu32 " mW",
                                         min_voltage * 50, max_voltage * 50, max_power * 250);
                            break;
                        }
                        case 3:
                        {
                            uint32_t subtype = (pdo_value >> 28) & 0x03;

                            switch (subtype)
                            {
                            case 0:
                            {
                                if (0)
                                    ESP_LOGI(TAG, "    #%" PRIu32 ": Augmented PDO (SPR PPS)", index);
                                bool limited = (pdo_value >> 27) & 0x01;
                                uint32_t max_voltage = (pdo_value >> 17) & 0xFF; /* in 100mV units */
                                uint32_t min_voltage = (pdo_value >> 8) & 0xFF;  /* in 100mV units */
                                uint32_t max_current = (pdo_value >> 0) & 0x7F;  /* in 50mA units */

                                if (0)
                                    ESP_LOGI(TAG, "        %" PRIu32 " mV - %" PRIu32 " mV, %" PRIu32 " mA (%s)",
                                             min_voltage * 100, max_voltage * 100, max_current * 50, limited ? "Limited" : "Unlimited");
                                break;
                            }
                            case 1:
                            {
                                if (0)
                                    ESP_LOGI(TAG, "    #%" PRIu32 ": Augmented PDO (EPR AVS)", index);
                                uint32_t peak_current = (pdo_value >> 26) & 0x03;
                                uint32_t max_voltage = (pdo_value >> 17) & 0xFF; /* in 100mV units */
                                uint32_t min_voltage = (pdo_value >> 8) & 0xFF;  /* in 100mV units */
                                uint32_t pdp = (pdo_value >> 0) & 0x7F;          /* in 1W units */

                                if (0)
                                    ESP_LOGI(TAG, "        Peak Current: %" PRIu32 ", Voltage Range: %" PRIu32 " mV - %" PRIu32 " mV, PDP: %" PRIu32 " W",
                                             peak_current, min_voltage * 100, max_voltage * 100, pdp);
                                break;
                            }
                            case 2:
                            {
                                if (0)
                                    ESP_LOGI(TAG, "    #%" PRIu32 ": Augmented PDO (SPR AVS)", index);
                                uint32_t peak_current = (pdo_value >> 26) & 0x03;
                                uint32_t max_current_15 = (pdo_value >> 10) & 0x1FF; /* in 10mA units */
                                uint32_t max_current_20 = (pdo_value >> 0) & 0x1FF;  /* in 10mA units */

                                if (0)
                                    ESP_LOGI(TAG, "        Peak Current: %" PRIu32 ", %" PRIu32 " mA@15V, %" PRIu32 " mA@20V",
                                             peak_current, max_current_15 * 10, max_current_20 * 10);
                                break;
                            }
                            case 3:
                            {
                                if (0)
                                    ESP_LOGI(TAG, "    #%" PRIu32 ": Augmented PDO (unknown)", index);
                                break;
                            }

                            default:
                                break;
                            }
                            break;
                        }
                        }
                    }

                    break;
                }
            }

            // ESP_LOGI(TAG, "  Data bytes:");
            // ESP_LOG_BUFFER_HEX(TAG, rx_data->payload, rx_data->length);
            break;
        }
        case PD_BUF_TYPE_SYMBOLS:
        {
            if (rx_data->target == TARGET_HARD_RESET)
            {
                ESP_LOGE(TAG, "Reset:");
                ESP_LOGE(TAG, "  Hard Reset");
                pd_state_reset();
            }
            else if (rx_data->target == TARGET_CABLE_RESET)
            {
                ESP_LOGE(TAG, "Reset:");
                ESP_LOGE(TAG, "  Cable Reset");
                pd_state_reset();
            }
            else
            {
                ESP_LOGE(TAG, "Failed packet:");
                ESP_LOGE(TAG, "  Raw symbols:");
                ESP_LOG_BUFFER_HEX(TAG, rx_data->symbols, rx_data->symbol_count);
                ESP_LOGE(TAG, "  Parsed data:");
                ESP_LOG_BUFFER_HEX(TAG, rx_data->payload, rx_data->length);
            }
            break;
        }
        case PD_BUF_TYPE_TIMINGS:
        {
            ESP_LOGI(TAG, "  Timings:");
            ESP_LOG_BUFFER_HEX(TAG, rx_data->symbols, rx_data->symbol_count);
            break;
        }
        default:
        {
            ESP_LOGI(TAG, "  Unknown buffer type");
            break;
        }
        }

        if (state.requested_object)
        {
            static int64_t last_time = 0;
            int64_t cur_time = esp_timer_get_time();

            /* retry if we weren't able to send. diry hack at the wrong place */
            if (cur_time - last_time > 10000)
            {
                last_time = cur_time;
                uint8_t buf[16] = {0};
                int pos = 0;

                uint16_t header_tx = pd_tx_header(0, 1, state.message_id++, 0, 2, PD_DATA_ROLE_UFP, PD_DATA_REQUEST);

                buf[pos++] = header_tx;
                buf[pos++] = header_tx >> 8;

                uint32_t pdo = 0;
                pdo |= (state.requested_object) << 28;
                pdo |= (state.request_current_ma / 10) << 10;
                pdo |= (state.request_current_ma / 10) << 0;

                buf[pos++] = pdo;
                buf[pos++] = pdo >> 8;
                buf[pos++] = pdo >> 16;
                buf[pos++] = pdo >> 24;

                uint32_t crc = crc32buf(buf, pos);
                buf[pos++] = crc;
                buf[pos++] = crc >> 8;
                buf[pos++] = crc >> 16;
                buf[pos++] = crc >> 24;

                pd_tx(buf, pos);
                //ESP_LOGW(TAG, "        ##### Requested mode #%" PRIu8 " #####", state.requested_object);
            }
        }

        /* Return the buffer to the empty buffer queue */
        if (xQueueSend(pd_queue_empty, &rx_data, portMAX_DELAY) != pdTRUE)
        {
            ESP_LOGE(TAG, "Failed to return buffer to pd_queue_empty");
            free(rx_data);
        }
    }
}

/**
 * @brief Bottom half that handles RX packets from the ISR and triggers the confirmation.
 *
 * This function is very time-critical as it processes received packets and sends a confirmation
 * (GoodCRC) within less than a millisecond. After sending the confirmation, it hands over the
 * buffer to the slower user task.
 */

void IRAM_ATTR pd_rx_ack_task()
{
    uint32_t message_id;
    uint8_t buf[6] = {0};

    while (1)
    {
        if (xQueueReceive(pd_queue_rx_ack, &message_id, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        uint16_t header_tx = pd_tx_header(0, 0, message_id, 0, 2, PD_DATA_ROLE_UFP, PD_CONTROL_GOOD_CRC);

        buf[0] = header_tx;
        buf[1] = header_tx >> 8;

        uint32_t crc = crc32buf(buf, 2);
        buf[2] = crc;
        buf[3] = crc >> 8;
        buf[4] = crc >> 16;
        buf[5] = crc >> 24;

        pd_tx_start(buf, 6);
    }
}

void pd_state_reset()
{
    memset(&state, 0, sizeof(state));
    state.request_voltage_mv = 12000;
    state.request_current_ma = 1000;
}

void pd_init()
{
    ESP_LOGI(TAG, "  * Initialize PD");
    pd_init_queues();
    pd_state_reset();

    /* the rx_ack task needs highes priority as it has to respons with a RMT tx within a few hundred usec */
    xTaskCreate(pd_rx_ack_task, "pd_rx_ack_task", 4096, NULL, configMAX_PRIORITIES - 1, NULL);
    /* the user task turned out to also be a bit "time critical". every CAP message needs to be answered immediately */
    xTaskCreate(pd_user_task, "pd_user_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);

    ESP_LOGI(TAG, "  * Initialize RX");
    pd_rx_init();
    ESP_LOGI(TAG, "  * Initialize TX");
    pd_tx_init();
    ESP_LOGI(TAG, "  * Enter sink mode");
    pd_mode(PD_MODE_SINK);

    if (0)
    {
        uint8_t *buf = calloc(1, 100);
        while (1)
        {
            for (int pos = 0; pos < 100; pos++)
            {
                buf[pos] = (2 * pos) << 4 | (2 * pos + 1);
                vTaskDelay(75 / portTICK_PERIOD_MS);
                pd_tx_start(buf, 1 + pos);
            }
        }
    }

    if (0)
    {
        while (1)
        {
            for (int pos = 0; pos < 100; pos++)
            {
                vTaskDelay(10000 / portTICK_PERIOD_MS);

                uint8_t buf[64] = {0};
                int pos = 0;

                ESP_LOGI(TAG, "  Get status");
                uint16_t header_tx = pd_tx_header(0, 0, state.message_id++, 0, 2, PD_DATA_ROLE_UFP, PD_CONTROL_GET_STATUS);

                buf[pos++] = header_tx;
                buf[pos++] = header_tx >> 8;

                uint32_t crc = crc32buf(buf, pos);
                buf[pos++] = crc;
                buf[pos++] = crc >> 8;
                buf[pos++] = crc >> 16;
                buf[pos++] = crc >> 24;

                pd_tx(buf, pos);
            }
        }
    }
}
