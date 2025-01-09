
#include <string.h>
#include <stdint.h>

#include "esp_log.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "pd_types.h"
#include "pd_proto.h"
#include "pd_config.h"
#include "pd.h"
#include "crc32.h"

#define TAG "PD"

void pd_dump_msg_header(pd_msg_header *hdr)
{
    ESP_LOGE(TAG, "  Header Fields%s", (hdr->extended) ? " (extended)" : "");
    ESP_LOGE(TAG, "    DO: %" PRIu8 ", ID: %" PRIu8 ", PPR/CP: %" PRIu8 ", Rev: %" PRIu8 ", PDR: %" PRIu8 ", Type: %" PRIu8 "",
             hdr->num_data_objects, hdr->message_id, hdr->power_role, hdr->spec_revision, hdr->data_role, hdr->message_type);
}

void pd_dump_vdm(pd_vdm_packet *pkt)
{
    ESP_LOGI(TAG, "    Vendor Message:");
    ESP_LOGI(TAG, "      SVID:            0x%04X", pkt->vdm_header.svid);
    ESP_LOGI(TAG, "      VDM Type:        %s", pkt->vdm_header.vdm_type ? "Structured" : "Unstructured");

    if (pkt->vdm_header.vdm_type)
    {
        const char *type[] = {"REQ", "ACK", "NAK", "BUSY"};
        const char *cmd[] = {"Reserved", "Discover Identity", "Discover SVIDs", "Discover Modes", "Enter Mode", "Exit Mode", "Attention"};
        ESP_LOGI(TAG, "      VDM Version:     %u.%u", pkt->vdm_header.vdm_version_major, pkt->vdm_header.vdm_version_minor);
        ESP_LOGI(TAG, "      Object Position: %u", pkt->vdm_header.object_position);
        ESP_LOGI(TAG, "      Command Type:    %u (%s)", pkt->vdm_header.command_type,
                 (pkt->vdm_header.command_type < 7) ? type[pkt->vdm_header.command_type] : (pkt->vdm_header.command_type < 16) ? "Reserved"
                                                                                                                               : "SVID Specific");
        ESP_LOGI(TAG, "      Command:         %u (%s)", pkt->vdm_header.command, cmd[pkt->vdm_header.command]);
    }

    /* 0 REQ, 1 ACK  */
    if (pkt->vdm_header.command_type == 1)
    {
        ESP_LOGI(TAG, "      ID Header VDO:");
        ESP_LOGI(TAG, "        USB Host Capable:          %" PRIu8, pkt->id_header.usb_host);
        ESP_LOGI(TAG, "        USB Device Capable:        %" PRIu8, pkt->id_header.usb_device);
        ESP_LOGI(TAG, "        SOP' Product Type:         %" PRIu8, pkt->id_header.sop_product_type);
        ESP_LOGI(TAG, "        Modal Operation Supported: %" PRIu8, pkt->id_header.modal_operation);
        ESP_LOGI(TAG, "        USB Vendor ID:             0x%04" PRIX16, pkt->id_header.usb_vendor_id);

        ESP_LOGI(TAG, "      Cert Stat VDO:");
        ESP_LOGI(TAG, "        USB-IF XID:                0x%08" PRIX32, pkt->crt_stat.usb_if_xid);

        ESP_LOGI(TAG, "      Product VDO:");
        ESP_LOGI(TAG, "        USB Product ID:            0x%04" PRIX16, pkt->product.usb_product_id);
        ESP_LOGI(TAG, "        Device Version:            0x%04" PRIX16, pkt->product.bcd_device);

        if (pkt->id_header.sop_product_type == 3 || pkt->id_header.sop_product_type == 4)
        {
            ESP_LOGI(TAG, "      Cable VDO1:");
            ESP_LOGI(TAG, "        HW Version:                %" PRIu8, pkt->cable_1.hw_version);
            ESP_LOGI(TAG, "        FW Version:                %" PRIu8, pkt->cable_1.fw_version);
            ESP_LOGI(TAG, "        VDO Version:               %" PRIu8, pkt->cable_1.vdo_version);
            ESP_LOGI(TAG, "        Plug Type:                 %" PRIu8, pkt->cable_1.plug_type);
            ESP_LOGI(TAG, "        EPR Capable:               %" PRIu8, pkt->cable_1.epr_capable);
            ESP_LOGI(TAG, "        Cable Latency:             %" PRIu8, pkt->cable_1.cable_latency);
            ESP_LOGI(TAG, "        Cable Termination:         %" PRIu8, pkt->cable_1.cable_termination);
            ESP_LOGI(TAG, "        Max VBUS Voltage:          %" PRIu8, pkt->cable_1.max_vbus_voltage);
            ESP_LOGI(TAG, "        SBU Supported:             %" PRIu8, pkt->cable_1.sbu_supported);
            ESP_LOGI(TAG, "        SBU Type:                  %" PRIu8, pkt->cable_1.sbu_type);
            ESP_LOGI(TAG, "        VBUS Current Handling:     %" PRIu8, pkt->cable_1.vbus_current);
            ESP_LOGI(TAG, "        VBUS Through Cable:        %" PRIu8, pkt->cable_1.vbus_through);
            ESP_LOGI(TAG, "        SOP'' Controller Present:  %" PRIu8, pkt->cable_1.sop_controller);

            if (pkt->id_header.sop_product_type == 4)
            {
                ESP_LOGI(TAG, "      Cable VDO2:");
                ESP_LOGI(TAG, "        Max Operating Temperature: %" PRIu8 "°C", pkt->cable_2.max_operating_temp);
                ESP_LOGI(TAG, "        Shutdown Temperature:      %" PRIu8 "°C", pkt->cable_2.shutdown_temp);
                ESP_LOGI(TAG, "        U3/CLd Power:              %" PRIu8, pkt->cable_2.u3_cld_power);
                ESP_LOGI(TAG, "        U3 to U0 Transition Mode:  %" PRIu8, pkt->cable_2.u3_to_u0_transition);
                ESP_LOGI(TAG, "        Physical Connection:       %" PRIu8, pkt->cable_2.physical_connection);
                ESP_LOGI(TAG, "        Active Element:            %" PRIu8, pkt->cable_2.active_element);
                ESP_LOGI(TAG, "        USB4 Supported:            %" PRIu8, pkt->cable_2.usb4_supported);
                ESP_LOGI(TAG, "        USB 2.0 Hub Hops Consumed: %" PRIu8, pkt->cable_2.usb2_hub_hops);
                ESP_LOGI(TAG, "        USB 2.0 Supported:         %" PRIu8, pkt->cable_2.usb2_supported);
                ESP_LOGI(TAG, "        USB 3.2 Supported:         %" PRIu8, pkt->cable_2.usb3_2_supported);
                ESP_LOGI(TAG, "        USB Lanes Supported:       %" PRIu8, pkt->cable_2.usb_lanes_supported);
                ESP_LOGI(TAG, "        Optically Isolated:        %" PRIu8, pkt->cable_2.optically_isolated);
                ESP_LOGI(TAG, "        USB4 Asym. Mode Supported: %" PRIu8, pkt->cable_2.usb4_asymmetric);
                ESP_LOGI(TAG, "        USB Gen 1b (Gen 2 plus):   %" PRIu8, pkt->cable_2.usb_gen);
            }
        }
    }
}

void pd_parse_vdm(pd_vdm_packet *pkt, uint32_t pdos[7])
{
    uint32_t vdm_header = pdos[0];

    pkt->vdm_header.svid = (vdm_header >> SVID_SHIFT) & SVID_MASK;
    pkt->vdm_header.vdm_type = (vdm_header >> VDM_TYPE_SHIFT) & VDM_TYPE_MASK;
    pkt->vdm_header.vdm_version_major = (vdm_header >> VDM_VERSION_SHIFT) & VDM_VERSION_MASK;
    pkt->vdm_header.vdm_version_minor = (vdm_header >> VDM_MINOR_SHIFT) & VDM_MINOR_MASK;
    pkt->vdm_header.object_position = (vdm_header >> OBJ_POS_SHIFT) & OBJ_POS_MASK;
    pkt->vdm_header.command_type = (vdm_header >> CMD_TYPE_SHIFT) & CMD_TYPE_MASK;
    pkt->vdm_header.command = (vdm_header >> COMMAND_SHIFT) & COMMAND_MASK;

    /* 0 REQ, 1 ACK  */
    if (pkt->vdm_header.command_type == 1)
    {
        /* ID Header VDO */
        uint32_t id_header = pdos[1];
        pkt->id_header.usb_host = (id_header >> USB_HOST_SHIFT) & USB_HOST_MASK;
        pkt->id_header.usb_device = (id_header >> USB_DEVICE_SHIFT) & USB_DEVICE_MASK;
        pkt->id_header.sop_product_type = (id_header >> SOP_PRODUCT_TYPE_SHIFT) & SOP_PRODUCT_TYPE_MASK;
        pkt->id_header.modal_operation = (id_header >> MODAL_OPERATION_SHIFT) & MODAL_OPERATION_MASK;
        pkt->id_header.usb_vendor_id = (id_header >> USB_VENDOR_ID_SHIFT) & USB_VENDOR_ID_MASK;
        /* Cert Stat VDO */
        pkt->crt_stat.usb_if_xid = pdos[2];
        /* Product VDO */
        uint32_t product_vdo = pdos[3];
        pkt->product.usb_product_id = (product_vdo >> USB_PRODUCT_ID_SHIFT) & USB_PRODUCT_ID_MASK;
        pkt->product.bcd_device = (product_vdo >> BCD_DEVICE_SHIFT) & BCD_DEVICE_MASK;

        if (pkt->id_header.sop_product_type == 3 || pkt->id_header.sop_product_type == 4)
        {
            /* Cable VDO1 */
            uint32_t cable_vdo1 = pdos[4];
            pkt->cable_1.hw_version = (cable_vdo1 >> HW_VERSION_SHIFT) & HW_VERSION_MASK;
            pkt->cable_1.fw_version = (cable_vdo1 >> FW_VERSION_SHIFT) & FW_VERSION_MASK;
            pkt->cable_1.vdo_version = (cable_vdo1 >> VDO_VERSION_SHIFT) & VDO_VERSION_MASK;
            pkt->cable_1.plug_type = (cable_vdo1 >> PLUG_TYPE_SHIFT) & PLUG_TYPE_MASK;
            pkt->cable_1.epr_capable = (cable_vdo1 >> EPR_CAPABLE_SHIFT) & EPR_CAPABLE_MASK;
            pkt->cable_1.cable_latency = (cable_vdo1 >> CABLE_LATENCY_SHIFT) & CABLE_LATENCY_MASK;
            pkt->cable_1.cable_termination = (cable_vdo1 >> CABLE_TERMINATION_SHIFT) & CABLE_TERMINATION_MASK;
            pkt->cable_1.max_vbus_voltage = (cable_vdo1 >> MAX_VBUS_VOLTAGE_SHIFT) & MAX_VBUS_VOLTAGE_MASK;
            pkt->cable_1.sbu_supported = (cable_vdo1 >> SBU_SUPPORTED_SHIFT) & SBU_SUPPORTED_MASK;
            pkt->cable_1.sbu_type = (cable_vdo1 >> SBU_TYPE_SHIFT) & SBU_TYPE_MASK;
            pkt->cable_1.vbus_current = (cable_vdo1 >> VBUS_CURRENT_SHIFT) & VBUS_CURRENT_MASK;
            pkt->cable_1.vbus_through = (cable_vdo1 >> VBUS_THROUGH_SHIFT) & VBUS_THROUGH_MASK;
            pkt->cable_1.sop_controller = (cable_vdo1 >> SOP_CONTROLLER_SHIFT) & SOP_CONTROLLER_MASK;

            if (pkt->id_header.sop_product_type == 4)
            {
                /* Cable VDO2 */
                uint32_t cable_vdo2 = pdos[5];
                pkt->cable_2.max_operating_temp = (cable_vdo2 >> MAX_OPERATING_TEMP_SHIFT) & MAX_OPERATING_TEMP_MASK;
                pkt->cable_2.shutdown_temp = (cable_vdo2 >> SHUTDOWN_TEMP_SHIFT) & SHUTDOWN_TEMP_MASK;
                pkt->cable_2.u3_cld_power = (cable_vdo2 >> U3_CLD_POWER_SHIFT) & U3_CLD_POWER_MASK;
                pkt->cable_2.u3_to_u0_transition = (cable_vdo2 >> U3_TO_U0_TRANSITION_SHIFT) & U3_TO_U0_TRANSITION_MASK;
                pkt->cable_2.physical_connection = (cable_vdo2 >> PHYSICAL_CONNECTION_SHIFT) & PHYSICAL_CONNECTION_MASK;
                pkt->cable_2.active_element = (cable_vdo2 >> ACTIVE_ELEMENT_SHIFT) & ACTIVE_ELEMENT_MASK;
                pkt->cable_2.usb4_supported = (cable_vdo2 >> USB4_SUPPORTED_SHIFT) & USB4_SUPPORTED_MASK;
                pkt->cable_2.usb2_hub_hops = (cable_vdo2 >> USB2_HUB_HOPS_SHIFT) & USB2_HUB_HOPS_MASK;
                pkt->cable_2.usb2_supported = (cable_vdo2 >> USB2_SUPPORTED_SHIFT) & USB2_SUPPORTED_MASK;
                pkt->cable_2.usb3_2_supported = (cable_vdo2 >> USB3_2_SUPPORTED_SHIFT) & USB3_2_SUPPORTED_MASK;
                pkt->cable_2.usb_lanes_supported = (cable_vdo2 >> USB_LANES_SUPPORTED_SHIFT) & USB_LANES_SUPPORTED_MASK;
                pkt->cable_2.optically_isolated = (cable_vdo2 >> OPTICALLY_ISOLATED_SHIFT) & OPTICALLY_ISOLATED_MASK;
                pkt->cable_2.usb4_asymmetric = (cable_vdo2 >> USB4_ASYMMETRIC_SHIFT) & USB4_ASYMMETRIC_MASK;
                pkt->cable_2.usb_gen = (cable_vdo2 >> USB_GEN_SHIFT) & USB_GEN_MASK;
            }
        }
    }
}

void pd_build_vdm(pd_vdm_packet *pkt, uint32_t pdos[7])
{
    uint32_t header = 0;
    header |= ((pkt->vdm_header.svid & SVID_MASK) << SVID_SHIFT);
    header |= ((pkt->vdm_header.vdm_type & VDM_TYPE_MASK) << VDM_TYPE_SHIFT);
    header |= ((pkt->vdm_header.vdm_version_major & VDM_VERSION_MASK) << VDM_VERSION_SHIFT);
    header |= ((pkt->vdm_header.vdm_version_minor & VDM_MINOR_MASK) << VDM_MINOR_SHIFT);
    header |= ((pkt->vdm_header.object_position & OBJ_POS_MASK) << OBJ_POS_SHIFT);
    header |= ((pkt->vdm_header.command_type & CMD_TYPE_MASK) << CMD_TYPE_SHIFT);
    header |= ((pkt->vdm_header.command & COMMAND_MASK) << COMMAND_SHIFT);
    pdos[0] = header;

    /* 0 REQ, 1 ACK  */
    if (pkt->vdm_header.command_type == 1)
    {
        /* ID Header VDO */
        pdos[1] = ((pkt->id_header.usb_host & USB_HOST_MASK) << USB_HOST_SHIFT) |
                  ((pkt->id_header.usb_device & USB_DEVICE_MASK) << USB_DEVICE_SHIFT) |
                  ((pkt->id_header.sop_product_type & SOP_PRODUCT_TYPE_MASK) << SOP_PRODUCT_TYPE_SHIFT) |
                  ((pkt->id_header.modal_operation & MODAL_OPERATION_MASK) << MODAL_OPERATION_SHIFT) |
                  ((pkt->id_header.usb_vendor_id & USB_VENDOR_ID_MASK) << USB_VENDOR_ID_SHIFT);

        /* Cert Stat VDO */
        pdos[2] = pkt->crt_stat.usb_if_xid;

        /* Product VDO */
        pdos[3] = ((pkt->product.usb_product_id & USB_PRODUCT_ID_MASK) << USB_PRODUCT_ID_SHIFT) |
                  ((pkt->product.bcd_device & BCD_DEVICE_MASK) << BCD_DEVICE_SHIFT);

        if (pkt->id_header.sop_product_type == 3 || pkt->id_header.sop_product_type == 4)
        {
            uint32_t cable_vdo1 = 0;
            cable_vdo1 |= (pkt->cable_1.hw_version & HW_VERSION_MASK) << HW_VERSION_SHIFT;
            cable_vdo1 |= (pkt->cable_1.fw_version & FW_VERSION_MASK) << FW_VERSION_SHIFT;
            cable_vdo1 |= (pkt->cable_1.vdo_version & VDO_VERSION_MASK) << VDO_VERSION_SHIFT;
            cable_vdo1 |= (pkt->cable_1.plug_type & PLUG_TYPE_MASK) << PLUG_TYPE_SHIFT;
            cable_vdo1 |= (pkt->cable_1.epr_capable & EPR_CAPABLE_MASK) << EPR_CAPABLE_SHIFT;
            cable_vdo1 |= (pkt->cable_1.cable_latency & CABLE_LATENCY_MASK) << CABLE_LATENCY_SHIFT;
            cable_vdo1 |= (pkt->cable_1.cable_termination & CABLE_TERMINATION_MASK) << CABLE_TERMINATION_SHIFT;
            cable_vdo1 |= (pkt->cable_1.max_vbus_voltage & MAX_VBUS_VOLTAGE_MASK) << MAX_VBUS_VOLTAGE_SHIFT;
            cable_vdo1 |= (pkt->cable_1.sbu_supported & SBU_SUPPORTED_MASK) << SBU_SUPPORTED_SHIFT;
            cable_vdo1 |= (pkt->cable_1.sbu_type & SBU_TYPE_MASK) << SBU_TYPE_SHIFT;
            cable_vdo1 |= (pkt->cable_1.vbus_current & VBUS_CURRENT_MASK) << VBUS_CURRENT_SHIFT;
            cable_vdo1 |= (pkt->cable_1.vbus_through & VBUS_THROUGH_MASK) << VBUS_THROUGH_SHIFT;
            cable_vdo1 |= (pkt->cable_1.sop_controller & SOP_CONTROLLER_MASK) << SOP_CONTROLLER_SHIFT;
            cable_vdo1 |= (pkt->cable_1.usb_speed & USB_SPEED_MASK) << USB_SPEED_SHIFT;
            pdos[4] = cable_vdo1;

            if (pkt->id_header.sop_product_type == 4)
            {
                /* Cable VDO2 */
                uint32_t cable_vdo2 = 0;
                cable_vdo2 |= (pkt->cable_2.max_operating_temp & MAX_OPERATING_TEMP_MASK) << MAX_OPERATING_TEMP_SHIFT;
                cable_vdo2 |= (pkt->cable_2.shutdown_temp & SHUTDOWN_TEMP_MASK) << SHUTDOWN_TEMP_SHIFT;
                cable_vdo2 |= (pkt->cable_2.u3_cld_power & U3_CLD_POWER_MASK) << U3_CLD_POWER_SHIFT;
                cable_vdo2 |= (pkt->cable_2.u3_to_u0_transition & U3_TO_U0_TRANSITION_MASK) << U3_TO_U0_TRANSITION_SHIFT;
                cable_vdo2 |= (pkt->cable_2.physical_connection & PHYSICAL_CONNECTION_MASK) << PHYSICAL_CONNECTION_SHIFT;
                cable_vdo2 |= (pkt->cable_2.active_element & ACTIVE_ELEMENT_MASK) << ACTIVE_ELEMENT_SHIFT;
                cable_vdo2 |= (pkt->cable_2.usb4_supported & USB4_SUPPORTED_MASK) << USB4_SUPPORTED_SHIFT;
                cable_vdo2 |= (pkt->cable_2.usb2_hub_hops & USB2_HUB_HOPS_MASK) << USB2_HUB_HOPS_SHIFT;
                cable_vdo2 |= (pkt->cable_2.usb2_supported & USB2_SUPPORTED_MASK) << USB2_SUPPORTED_SHIFT;
                cable_vdo2 |= (pkt->cable_2.usb3_2_supported & USB3_2_SUPPORTED_MASK) << USB3_2_SUPPORTED_SHIFT;
                cable_vdo2 |= (pkt->cable_2.usb_lanes_supported & USB_LANES_SUPPORTED_MASK) << USB_LANES_SUPPORTED_SHIFT;
                cable_vdo2 |= (pkt->cable_2.optically_isolated & OPTICALLY_ISOLATED_MASK) << OPTICALLY_ISOLATED_SHIFT;
                cable_vdo2 |= (pkt->cable_2.usb4_asymmetric & USB4_ASYMMETRIC_MASK) << USB4_ASYMMETRIC_SHIFT;
                cable_vdo2 |= (pkt->cable_2.usb_gen & USB_GEN_MASK) << USB_GEN_SHIFT;
                pdos[5] = cable_vdo2;
            }
        }
    }
}

void pd_parse_msg_header(pd_msg_header *hdr, uint8_t *data)
{
    uint16_t header = BUILD_LE_UINT16(data, 0);

    /* Extract individual fields */
    hdr->extended = (header >> EXTENDED_SHIFT) & EXTENDED_MASK;
    hdr->num_data_objects = (header >> NUM_DATA_OBJ_SHIFT) & NUM_DATA_OBJ_MASK;
    hdr->message_id = (header >> MESSAGE_ID_SHIFT) & MESSAGE_ID_MASK;
    hdr->power_role = (header >> POWER_ROLE_SHIFT) & POWER_ROLE_MASK;
    hdr->spec_revision = (header >> SPEC_REVISION_SHIFT) & SPEC_REVISION_MASK;
    hdr->data_role = (header >> DATA_ROLE_SHIFT) & DATA_ROLE_MASK;
    hdr->message_type = header & MESSAGE_TYPE_MASK;
}

void pd_build_msg_header(pd_msg_header *hdr, uint8_t *data)
{
    uint16_t header = 0;

    /* Build header from struct fields */
    header |= (hdr->extended & EXTENDED_MASK) << EXTENDED_SHIFT;
    header |= (hdr->num_data_objects & NUM_DATA_OBJ_MASK) << NUM_DATA_OBJ_SHIFT;
    header |= (hdr->message_id & MESSAGE_ID_MASK) << MESSAGE_ID_SHIFT;
    header |= (hdr->power_role & POWER_ROLE_MASK) << POWER_ROLE_SHIFT;
    header |= (hdr->spec_revision & SPEC_REVISION_MASK) << SPEC_REVISION_SHIFT;
    header |= (hdr->data_role & DATA_ROLE_MASK) << DATA_ROLE_SHIFT;
    header |= (hdr->message_type & MESSAGE_TYPE_MASK);

    SPLIT_LE_UINT16(header, data, 0);
}
