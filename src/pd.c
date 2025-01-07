
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
QueueHandle_t pd_queue_rx_data_log;
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

    pd_queue_rx_data_log = xQueueCreate(PD_BUFFER_COUNT, sizeof(pd_rx_buf_t *));
    if (pd_queue_rx_data_log == NULL)
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

void pd_protocol_task(void *pvParameters)
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

        switch (rx_data->type)
        {
        case PD_BUF_TYPE_DATA:
        {
            switch (rx_data->target)
            {
            case TARGET_SOP:
                break;
            case TARGET_SOP_P:
            case TARGET_SOP_PP:
            case TARGET_SOP_PD:
            case TARGET_SOP_PPD:
            default:
                goto skip;
            }

            uint16_t header = (rx_data->payload[1] << 8) | rx_data->payload[0];

            /* Extract individual fields */
            uint8_t num_data_objects = (header >> 12) & 0x07;
            uint8_t message_type = header & 0x1F;

            uint32_t pdos[8];
            for (int i = 0; i < num_data_objects; i++)
            {
                pdos[i] = BUILD_LE_UINT32(rx_data->payload, 2 + i * 4);
            }

            /* when no objects, then its a control message */
            if (num_data_objects == 0)
            {
                switch (message_type)
                {
                case PD_CONTROL_SOFT_RESET:
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
                    break;

                case PD_CONTROL_REJECT:
                    if (state.requested_object != 0)
                    {
                        state.accepted_object = 0;
                        state.requested_object = 0;
                    }
                    break;

                case PD_CONTROL_ACCEPT:
                    if (state.requested_object != 0)
                    {
                        state.accepted_object = state.requested_object;
                        state.requested_object = 0;
                    }
                    break;

                case PD_CONTROL_PS_RDY:
                    break;
                }
            }
            else
            {
                switch (message_type)
                {
                case PD_VENDOR_MESSAGE:
                {
                    uint32_t vdm_header = pdos[0];
                    uint16_t svid = (vdm_header >> SVID_SHIFT) & SVID_MASK;
                    uint8_t vdm_type = (vdm_header >> VDM_TYPE_SHIFT) & VDM_TYPE_MASK;
                    uint8_t vdm_version_major = (vdm_header >> VDM_VERSION_SHIFT) & VDM_VERSION_MASK;
                    uint8_t vdm_version_minor = (vdm_header >> VDM_MINOR_SHIFT) & VDM_MINOR_MASK;
                    uint8_t object_position = (vdm_header >> OBJ_POS_SHIFT) & OBJ_POS_MASK;
                    uint8_t command_type = (vdm_header >> CMD_TYPE_SHIFT) & CMD_TYPE_MASK;
                    uint8_t command = (vdm_header >> COMMAND_SHIFT) & COMMAND_MASK;

                    /* C.1.2 Discover Identity Command response - Active Cable. */

                    break;
                }
                break;

                case PD_DATA_SOURCE_CAPABILITIES:
                    state.requested_object = 0;
                    for (uint32_t index = 0; index < num_data_objects; index++)
                    {
                        uint32_t pdo_value = pdos[index];
                        uint32_t type = (pdo_value >> 30) & 0x03;

                        switch (type)
                        {
                        case 0:
                        {
                            uint32_t voltage = (pdo_value >> 10) & 0x3FF; /* in 50mV units */
                            uint32_t current = pdo_value & 0x3FF;         /* in 10mA units */

                            if (voltage == state.request_voltage_mv / 50)
                            {
                                state.requested_object = index + 1;
                            }
                            break;
                        }
                        default:
                            break;
                        }
                    }

                    break;
                }
            }

            break;
        }
        case PD_BUF_TYPE_SYMBOLS:
        {
            if (rx_data->target == TARGET_HARD_RESET)
            {
                pd_state_reset();
            }
            else if (rx_data->target == TARGET_CABLE_RESET)
            {
                pd_state_reset();
            }
            break;
        }
        default:
        {
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
                // ESP_LOGW(TAG, "        ##### Requested mode #%" PRIu8 " #####", state.requested_object);
            }
        }

    skip:

        /* Return the buffer to the empty buffer queue */
        if (xQueueSend(pd_queue_rx_data_log, &rx_data, portMAX_DELAY) != pdTRUE)
        {
            ESP_LOGE(TAG, "Failed to return buffer to pd_queue_empty");
            free(rx_data);
        }
    }
}

void parse_vdm_fields(uint32_t pdos[7])
{
    uint32_t vdm_header = pdos[0];
    uint16_t svid = (vdm_header >> SVID_SHIFT) & SVID_MASK;
    uint8_t vdm_type = (vdm_header >> VDM_TYPE_SHIFT) & VDM_TYPE_MASK;
    uint8_t vdm_version_major = (vdm_header >> VDM_VERSION_SHIFT) & VDM_VERSION_MASK;
    uint8_t vdm_version_minor = (vdm_header >> VDM_MINOR_SHIFT) & VDM_MINOR_MASK;
    uint8_t object_position = (vdm_header >> OBJ_POS_SHIFT) & OBJ_POS_MASK;
    uint8_t command_type = (vdm_header >> CMD_TYPE_SHIFT) & CMD_TYPE_MASK;
    uint8_t command = (vdm_header >> COMMAND_SHIFT) & COMMAND_MASK;

    ESP_LOGI(TAG, "    Vendor Message:");
    ESP_LOGI(TAG, "      SVID:            0x%04X", svid);
    ESP_LOGI(TAG, "      VDM Type:        %s", vdm_type ? "Structured" : "Unstructured");

    if (vdm_type)
    {
        const char *type[] = {"REQ", "ACK", "NAK", "BUSY"};
        const char *cmd[] = {"Reserved", "Discover Identity", "Discover SVIDs", "Discover Modes", "Enter Mode", "Exit Mode", "Attention"};
        ESP_LOGI(TAG, "      VDM Version:     %u.%u", vdm_version_major, vdm_version_minor);
        ESP_LOGI(TAG, "      Object Position: %u", object_position);
        ESP_LOGI(TAG, "      Command Type:    %u (%s)", command_type, (command_type < 7) ? type[command_type] : (command_type < 16) ? "Reserved" : "SVID Specific");
        ESP_LOGI(TAG, "      Command:         %u (%s)", command, cmd[command]);
    }

    /* 0 REQ, 1 ACK  */
    if (command_type == 1)
    {
        /* ID Header VDO */
        uint32_t id_header = pdos[1];
        uint8_t usb_host = (id_header >> USB_HOST_SHIFT) & USB_HOST_MASK;
        uint8_t usb_device = (id_header >> USB_DEVICE_SHIFT) & USB_DEVICE_MASK;
        uint8_t sop_product_type = (id_header >> SOP_PRODUCT_TYPE_SHIFT) & SOP_PRODUCT_TYPE_MASK;
        uint8_t modal_operation = (id_header >> MODAL_OPERATION_SHIFT) & MODAL_OPERATION_MASK;
        uint16_t usb_vendor_id = (id_header >> USB_VENDOR_ID_SHIFT) & USB_VENDOR_ID_MASK;

        ESP_LOGI(TAG, "      ID Header VDO:");
        ESP_LOGI(TAG, "        USB Host Capable:          %" PRIu8, usb_host);
        ESP_LOGI(TAG, "        USB Device Capable:        %" PRIu8, usb_device);
        ESP_LOGI(TAG, "        SOP' Product Type:         %" PRIu8, sop_product_type);
        ESP_LOGI(TAG, "        Modal Operation Supported: %" PRIu8, modal_operation);
        ESP_LOGI(TAG, "        USB Vendor ID:             0x%04" PRIX16, usb_vendor_id);

        /* Cert Stat VDO */
        uint32_t cert_stat = pdos[2];
        ESP_LOGI(TAG, "      Cert Stat VDO:");
        ESP_LOGI(TAG, "        USB-IF XID:                0x%08" PRIX32, cert_stat);

        /* Product VDO */
        uint32_t product_vdo = pdos[3];
        uint16_t usb_product_id = (product_vdo >> USB_PRODUCT_ID_SHIFT) & USB_PRODUCT_ID_MASK;
        uint16_t bcd_device = (product_vdo >> BCD_DEVICE_SHIFT) & BCD_DEVICE_MASK;

        ESP_LOGI(TAG, "      Product VDO:");
        ESP_LOGI(TAG, "        USB Product ID:            0x%04" PRIX16, usb_product_id);
        ESP_LOGI(TAG, "        Device Version:            0x%04" PRIX16, bcd_device);

        if (sop_product_type == 3 || sop_product_type == 4)
        {
            /* Cable VDO1 */
            uint32_t cable_vdo1 = pdos[4];
            uint8_t hw_version = (cable_vdo1 >> HW_VERSION_SHIFT) & HW_VERSION_MASK;
            uint8_t fw_version = (cable_vdo1 >> FW_VERSION_SHIFT) & FW_VERSION_MASK;
            uint8_t vdo_version = (cable_vdo1 >> VDO_VERSION_SHIFT) & VDO_VERSION_MASK;
            uint8_t plug_type = (cable_vdo1 >> PLUG_TYPE_SHIFT) & PLUG_TYPE_MASK;
            uint8_t epr_capable = (cable_vdo1 >> EPR_CAPABLE_SHIFT) & EPR_CAPABLE_MASK;
            uint8_t cable_latency = (cable_vdo1 >> CABLE_LATENCY_SHIFT) & CABLE_LATENCY_MASK;
            uint8_t cable_termination = (cable_vdo1 >> CABLE_TERMINATION_SHIFT) & CABLE_TERMINATION_MASK;
            uint8_t max_vbus_voltage = (cable_vdo1 >> MAX_VBUS_VOLTAGE_SHIFT) & MAX_VBUS_VOLTAGE_MASK;
            uint8_t sbu_supported = (cable_vdo1 >> SBU_SUPPORTED_SHIFT) & SBU_SUPPORTED_MASK;
            uint8_t sbu_type = (cable_vdo1 >> SBU_TYPE_SHIFT) & SBU_TYPE_MASK;
            uint8_t vbus_current = (cable_vdo1 >> VBUS_CURRENT_SHIFT) & VBUS_CURRENT_MASK;
            uint8_t vbus_through = (cable_vdo1 >> VBUS_THROUGH_SHIFT) & VBUS_THROUGH_MASK;
            uint8_t sop_controller = (cable_vdo1 >> SOP_CONTROLLER_SHIFT) & SOP_CONTROLLER_MASK;

            ESP_LOGI(TAG, "      Cable VDO1:");
            ESP_LOGI(TAG, "        HW Version:                %" PRIu8, hw_version);
            ESP_LOGI(TAG, "        FW Version:                %" PRIu8, fw_version);
            ESP_LOGI(TAG, "        VDO Version:               %" PRIu8, vdo_version);
            ESP_LOGI(TAG, "        Plug Type:                 %" PRIu8, plug_type);
            ESP_LOGI(TAG, "        EPR Capable:               %" PRIu8, epr_capable);
            ESP_LOGI(TAG, "        Cable Latency:             %" PRIu8, cable_latency);
            ESP_LOGI(TAG, "        Cable Termination:         %" PRIu8, cable_termination);
            ESP_LOGI(TAG, "        Max VBUS Voltage:          %" PRIu8, max_vbus_voltage);
            ESP_LOGI(TAG, "        SBU Supported:             %" PRIu8, sbu_supported);
            ESP_LOGI(TAG, "        SBU Type:                  %" PRIu8, sbu_type);
            ESP_LOGI(TAG, "        VBUS Current Handling:     %" PRIu8, vbus_current);
            ESP_LOGI(TAG, "        VBUS Through Cable:        %" PRIu8, vbus_through);
            ESP_LOGI(TAG, "        SOP'' Controller Present:  %" PRIu8, sop_controller);

            /* Cable VDO2 */
            uint32_t cable_vdo2 = pdos[5];
            uint8_t max_operating_temp = (cable_vdo2 >> MAX_OPERATING_TEMP_SHIFT) & MAX_OPERATING_TEMP_MASK;
            uint8_t shutdown_temp = (cable_vdo2 >> SHUTDOWN_TEMP_SHIFT) & SHUTDOWN_TEMP_MASK;
            uint8_t u3_cld_power = (cable_vdo2 >> U3_CLD_POWER_SHIFT) & U3_CLD_POWER_MASK;
            uint8_t u3_to_u0_transition = (cable_vdo2 >> U3_TO_U0_TRANSITION_SHIFT) & U3_TO_U0_TRANSITION_MASK;
            uint8_t physical_connection = (cable_vdo2 >> PHYSICAL_CONNECTION_SHIFT) & PHYSICAL_CONNECTION_MASK;
            uint8_t active_element = (cable_vdo2 >> ACTIVE_ELEMENT_SHIFT) & ACTIVE_ELEMENT_MASK;
            uint8_t usb4_supported = (cable_vdo2 >> USB4_SUPPORTED_SHIFT) & USB4_SUPPORTED_MASK;
            uint8_t usb2_hub_hops = (cable_vdo2 >> USB2_HUB_HOPS_SHIFT) & USB2_HUB_HOPS_MASK;
            uint8_t usb2_supported = (cable_vdo2 >> USB2_SUPPORTED_SHIFT) & USB2_SUPPORTED_MASK;
            uint8_t usb3_2_supported = (cable_vdo2 >> USB3_2_SUPPORTED_SHIFT) & USB3_2_SUPPORTED_MASK;
            uint8_t usb_lanes_supported = (cable_vdo2 >> USB_LANES_SUPPORTED_SHIFT) & USB_LANES_SUPPORTED_MASK;
            uint8_t optically_isolated = (cable_vdo2 >> OPTICALLY_ISOLATED_SHIFT) & OPTICALLY_ISOLATED_MASK;
            uint8_t usb4_asymmetric = (cable_vdo2 >> USB4_ASYMMETRIC_SHIFT) & USB4_ASYMMETRIC_MASK;
            uint8_t usb_gen = (cable_vdo2 >> USB_GEN_SHIFT) & USB_GEN_MASK;

            ESP_LOGI(TAG, "      Cable VDO2:");
            ESP_LOGI(TAG, "        Max Operating Temperature: %d°C", max_operating_temp);
            ESP_LOGI(TAG, "        Shutdown Temperature:      %d°C", shutdown_temp);
            ESP_LOGI(TAG, "        U3/CLd Power:              %" PRIu8, u3_cld_power);
            ESP_LOGI(TAG, "        U3 to U0 Transition Mode:  %" PRIu8, u3_to_u0_transition);
            ESP_LOGI(TAG, "        Physical Connection:       %" PRIu8, physical_connection);
            ESP_LOGI(TAG, "        Active Element:            %" PRIu8, active_element);
            ESP_LOGI(TAG, "        USB4 Supported:            %" PRIu8, usb4_supported);
            ESP_LOGI(TAG, "        USB 2.0 Hub Hops Consumed: %" PRIu8, usb2_hub_hops);
            ESP_LOGI(TAG, "        USB 2.0 Supported:         %" PRIu8, usb2_supported);
            ESP_LOGI(TAG, "        USB 3.2 Supported:         %" PRIu8, usb3_2_supported);
            ESP_LOGI(TAG, "        USB Lanes Supported:       %" PRIu8, usb_lanes_supported);
            ESP_LOGI(TAG, "        Optically Isolated:        %" PRIu8, optically_isolated);
            ESP_LOGI(TAG, "        USB4 Asym. Mode Supported: %" PRIu8, usb4_asymmetric);
            ESP_LOGI(TAG, "        USB Gen 1b (Gen 2 plus):   %" PRIu8, usb_gen);
        }
    }
}

void pd_log_task(void *pvParameters)
{
    pd_rx_buf_t *rx_data;

    while (1)
    {
        if (xQueueReceive(pd_queue_rx_data_log, &rx_data, portMAX_DELAY) != pdTRUE)
        {
            continue;
        }

        /* disable all logging messages - the delay is too long. the power supply causes a RST if the CAP message
        doesn't get answered immediately. */

        /* ToDo: split protocol handling and logging messages into separate tasks */

        ESP_LOGI(TAG, "");

        switch (rx_data->type)
        {
        case PD_BUF_TYPE_DATA:
        {
            switch (rx_data->target)
            {
            case TARGET_SOP:
                ESP_LOGI(TAG, "Target: SOP");
                break;
            case TARGET_SOP_P:
                ESP_LOGI(TAG, "Target: SOP'");
                break;
            case TARGET_SOP_PP:
                ESP_LOGI(TAG, "Target: SOP''");
                break;
            case TARGET_SOP_PD:
                ESP_LOGI(TAG, "Target: SOP' Debug");
                break;
            case TARGET_SOP_PPD:
                ESP_LOGI(TAG, "Target: SOP'' Debug");
                break;
            default:
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

            ESP_LOGI(TAG, "  Header Fields%s", (extended) ? " (extended)" : "");

            ESP_LOGI(TAG, "    DO: %" PRIu8 ", ID: %" PRIu8 ", PPR/CP: %" PRIu8 ", Rev: %" PRIu8 ", PDR: %" PRIu8 ", Type: %" PRIu8 "",
                     num_data_objects, message_id, power_role_or_cable_plug, spec_revision, data_role_or_reserved, message_type);

            /* when no objects, then its a control message */
            if (num_data_objects == 0)
            {

                ESP_LOGI(TAG, "  Control:");
                switch (message_type)
                {
                case PD_CONTROL_SOFT_RESET:

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

                    ESP_LOGI(TAG, "    Good CRC");
                    break;

                case PD_CONTROL_REJECT:

                    ESP_LOGI(TAG, "    Rejected");
                    if (state.requested_object != 0)
                    {
                        state.accepted_object = 0;
                        state.requested_object = 0;
                    }
                    break;

                case PD_CONTROL_ACCEPT:

                    ESP_LOGI(TAG, "    Accepted");
                    if (state.requested_object != 0)
                    {
                        state.accepted_object = state.requested_object;
                        state.requested_object = 0;
                    }
                    break;

                case PD_CONTROL_PS_RDY:

                    ESP_LOGI(TAG, "    Power supply ready");
                    break;
                }
            }
            else
            {

                ESP_LOGI(TAG, "  Data:");
                switch (message_type)
                {
                case PD_VENDOR_MESSAGE:
                {
                    for (uint32_t index = 0; index < num_data_objects; index++)
                    {
                        ESP_LOGI(TAG, "      Data #%" PRIu32 ": 0x%08" PRIx32, index, pdos[index]);
                    }

                    parse_vdm_fields(pdos);
                    break;
                }
                break;

                case PD_DATA_SOURCE_CAPABILITIES:
                    ESP_LOGI(TAG, "    Source Capabilities:");
                    for (uint32_t index = 0; index < num_data_objects; index++)
                    {
                        uint32_t pdo_value = pdos[index];
                        uint32_t type = (pdo_value >> 30) & 0x03;

                        switch (type)
                        {
                        case 0:
                        {
                            ESP_LOGI(TAG, "    #%" PRIu32 ": Fixed Supply PDO", index);
                            uint32_t voltage = (pdo_value >> 10) & 0x3FF; /* in 50mV units */
                            uint32_t current = pdo_value & 0x3FF;         /* in 10mA units */
                            uint32_t epr_capable = (pdo_value >> 23) & 0x01;
                            float watts = (voltage * 50 * current * 10) / 1000000.0;

                            ESP_LOGI(TAG, "        %" PRIu32 " mV, %" PRIu32 " mA (%2.2f W%s)",
                                     voltage * 50, current * 10, watts,
                                     epr_capable ? ", EPR" : "");

                            break;
                        }
                        case 1:
                        {
                            ESP_LOGI(TAG, "    #%" PRIu32 ": Battery Supply PDO", index);

                            uint32_t max_voltage = (pdo_value >> 20) & 0x1FF; /* in 50mV units */
                            uint32_t min_voltage = (pdo_value >> 10) & 0x1FF; /* in 50mV units */
                            uint32_t max_power = (pdo_value >> 0) & 0x1FF;    /* in 250mW units */

                            ESP_LOGI(TAG, "        %" PRIu32 " mV - %" PRIu32 " mV, %" PRIu32 " mW",
                                     min_voltage * 50, max_voltage * 50, max_power * 250);
                            break;
                        }
                        case 2:
                        {
                            ESP_LOGI(TAG, "    #%" PRIu32 ": Variable Supply (non-battery) PDO", index);

                            uint32_t max_voltage = (pdo_value >> 20) & 0x1FF; /* in 50mV units */
                            uint32_t min_voltage = (pdo_value >> 10) & 0x1FF; /* in 50mV units */
                            uint32_t max_power = (pdo_value >> 0) & 0x1FF;    /* in 250mW units */

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
                                ESP_LOGI(TAG, "    #%" PRIu32 ": Augmented PDO (SPR PPS)", index);
                                bool limited = (pdo_value >> 27) & 0x01;
                                uint32_t max_voltage = (pdo_value >> 17) & 0xFF; /* in 100mV units */
                                uint32_t min_voltage = (pdo_value >> 8) & 0xFF;  /* in 100mV units */
                                uint32_t max_current = (pdo_value >> 0) & 0x7F;  /* in 50mA units */

                                ESP_LOGI(TAG, "        %" PRIu32 " mV - %" PRIu32 " mV, %" PRIu32 " mA (%s)",
                                         min_voltage * 100, max_voltage * 100, max_current * 50, limited ? "Limited" : "Unlimited");
                                break;
                            }
                            case 1:
                            {

                                ESP_LOGI(TAG, "    #%" PRIu32 ": Augmented PDO (EPR AVS)", index);
                                uint32_t peak_current = (pdo_value >> 26) & 0x03;
                                uint32_t max_voltage = (pdo_value >> 17) & 0xFF; /* in 100mV units */
                                uint32_t min_voltage = (pdo_value >> 8) & 0xFF;  /* in 100mV units */
                                uint32_t pdp = (pdo_value >> 0) & 0x7F;          /* in 1W units */

                                ESP_LOGI(TAG, "        Peak Current: %" PRIu32 ", Voltage Range: %" PRIu32 " mV - %" PRIu32 " mV, PDP: %" PRIu32 " W",
                                         peak_current, min_voltage * 100, max_voltage * 100, pdp);
                                break;
                            }
                            case 2:
                            {

                                ESP_LOGI(TAG, "    #%" PRIu32 ": Augmented PDO (SPR AVS)", index);
                                uint32_t peak_current = (pdo_value >> 26) & 0x03;
                                uint32_t max_current_15 = (pdo_value >> 10) & 0x1FF; /* in 10mA units */
                                uint32_t max_current_20 = (pdo_value >> 0) & 0x1FF;  /* in 10mA units */

                                ESP_LOGI(TAG, "        Peak Current: %" PRIu32 ", %" PRIu32 " mA@15V, %" PRIu32 " mA@20V",
                                         peak_current, max_current_15 * 10, max_current_20 * 10);
                                break;
                            }
                            case 3:
                            {

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

            break;
        }
        case PD_BUF_TYPE_SYMBOLS:
        {
            if (rx_data->target == TARGET_HARD_RESET)
            {
                ESP_LOGE(TAG, "Reset:");
                ESP_LOGE(TAG, "  Hard Reset");
            }
            else if (rx_data->target == TARGET_CABLE_RESET)
            {
                ESP_LOGE(TAG, "Reset:");
                ESP_LOGE(TAG, "  Cable Reset");
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
    xTaskCreate(pd_protocol_task, "pd_protocol_task", 4096, NULL, configMAX_PRIORITIES - 2, NULL);
    xTaskCreate(pd_log_task, "pd_log_task", 4096, NULL, tskIDLE_PRIORITY + 1, NULL);

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
