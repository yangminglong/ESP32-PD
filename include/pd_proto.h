
#pragma once

#define EXTENDED_SHIFT 15
#define EXTENDED_MASK 0x01
#define NUM_DATA_OBJ_SHIFT 12
#define NUM_DATA_OBJ_MASK 0x07
#define MESSAGE_ID_SHIFT 9
#define MESSAGE_ID_MASK 0x07
#define POWER_ROLE_SHIFT 8
#define POWER_ROLE_MASK 0x01
#define SPEC_REVISION_SHIFT 6
#define SPEC_REVISION_MASK 0x03
#define DATA_ROLE_SHIFT 5
#define DATA_ROLE_MASK 0x01
#define MESSAGE_TYPE_MASK 0x1F

/* Structured VDM Header */
#define SVID_SHIFT 16
#define VDM_TYPE_SHIFT 15
#define VDM_VERSION_SHIFT 13
#define VDM_MINOR_SHIFT 11
#define OBJ_POS_SHIFT 8
#define CMD_TYPE_SHIFT 6
#define COMMAND_SHIFT 0

#define SVID_MASK 0x0000FFFF
#define VDM_TYPE_MASK 0x00000001
#define VDM_VERSION_MASK 0x00000003
#define VDM_MINOR_MASK 0x00000003
#define OBJ_POS_MASK 0x00000007
#define CMD_TYPE_MASK 0x00000003
#define RESERVED_MASK 0x00000001
#define COMMAND_MASK 0x0000001F

/* ID Header VDO  */
#define USB_HOST_SHIFT 31
#define USB_DEVICE_SHIFT 30
#define SOP_PRODUCT_TYPE_SHIFT 27
#define MODAL_OPERATION_SHIFT 26
#define USB_VENDOR_ID_SHIFT 0

#define USB_HOST_MASK 0x00000001
#define USB_DEVICE_MASK 0x00000001
#define SOP_PRODUCT_TYPE_MASK 0x00000007
#define MODAL_OPERATION_MASK 0x00000001
#define USB_VENDOR_ID_MASK 0x0000FFFF

/* Product VDO */
#define USB_PRODUCT_ID_SHIFT 16
#define BCD_DEVICE_SHIFT 0

#define USB_PRODUCT_ID_MASK 0x0000FFFF
#define BCD_DEVICE_MASK 0x0000FFFF

/* Cable VDO1 */
#define HW_VERSION_SHIFT 28
#define FW_VERSION_SHIFT 24
#define VDO_VERSION_SHIFT 21
#define PLUG_TYPE_SHIFT 18
#define EPR_CAPABLE_SHIFT 17
#define CABLE_LATENCY_SHIFT 13
#define CABLE_TERMINATION_SHIFT 11
#define MAX_VBUS_VOLTAGE_SHIFT 9
#define SBU_SUPPORTED_SHIFT 8
#define SBU_TYPE_SHIFT 7
#define VBUS_CURRENT_SHIFT 5
#define VBUS_THROUGH_SHIFT 4
#define SOP_CONTROLLER_SHIFT 3
#define USB_SPEED_SHIFT 0

#define HW_VERSION_MASK 0x0000000F
#define FW_VERSION_MASK 0x0000000F
#define VDO_VERSION_MASK 0x00000007
#define PLUG_TYPE_MASK 0x00000003
#define EPR_CAPABLE_MASK 0x00000001
#define CABLE_LATENCY_MASK 0x0000000F
#define CABLE_TERMINATION_MASK 0x00000003
#define MAX_VBUS_VOLTAGE_MASK 0x00000003
#define SBU_SUPPORTED_MASK 0x00000001
#define SBU_TYPE_MASK 0x00000001
#define VBUS_CURRENT_MASK 0x00000003
#define VBUS_THROUGH_MASK 0x00000001
#define SOP_CONTROLLER_MASK 0x00000001
#define USB_SPEED_MASK 0x00000007

/* Cable VDO2 */
#define MAX_OPERATING_TEMP_SHIFT 24
#define SHUTDOWN_TEMP_SHIFT 16
#define U3_CLD_POWER_SHIFT 12
#define U3_TO_U0_TRANSITION_SHIFT 11
#define PHYSICAL_CONNECTION_SHIFT 10
#define ACTIVE_ELEMENT_SHIFT 9
#define USB4_SUPPORTED_SHIFT 8
#define USB2_HUB_HOPS_SHIFT 6
#define USB2_SUPPORTED_SHIFT 5
#define USB3_2_SUPPORTED_SHIFT 4
#define USB_LANES_SUPPORTED_SHIFT 3
#define OPTICALLY_ISOLATED_SHIFT 2
#define USB4_ASYMMETRIC_SHIFT 1
#define USB_GEN_SHIFT 0

#define MAX_OPERATING_TEMP_MASK 0x000000FF
#define SHUTDOWN_TEMP_MASK 0x000000FF
#define U3_CLD_POWER_MASK 0x00000007
#define U3_TO_U0_TRANSITION_MASK 0x00000001
#define PHYSICAL_CONNECTION_MASK 0x00000001
#define ACTIVE_ELEMENT_MASK 0x00000001
#define USB4_SUPPORTED_MASK 0x00000001
#define USB2_HUB_HOPS_MASK 0x00000003
#define USB2_SUPPORTED_MASK 0x00000001
#define USB3_2_SUPPORTED_MASK 0x00000001
#define USB_LANES_SUPPORTED_MASK 0x00000001
#define OPTICALLY_ISOLATED_MASK 0x00000001
#define USB4_ASYMMETRIC_MASK 0x00000001
#define USB_GEN_MASK 0x00000001

typedef enum
{
    PD_VDM_CMD_TYPE_REQ = 0,
    PD_VDM_CMD_TYPE_ACK,
    PD_VDM_CMD_TYPE_NAK,
    PD_VDM_CMD_TYPE_BUSY,
} pd_vdm_cmd_type;

typedef enum
{
    PD_VDM_CMD_DISCOVER_IDENTIY = 1,
    PD_VDM_CMD_DISCOVER_SVIDS,
    PD_VDM_CMD_DISCOVER_MODES,
    PD_VDM_CMD_ENTER_MODE,
    PD_VDM_CMD_EXIT_MODE,
    PD_VDM_CMD_ATTENTION
} pd_vdm_command;

typedef enum
{
    PD_DATA_ROLE_UFP = 0,
    PD_DATA_ROLE_DFP = 1,
    PD_DATA_ROLE_CABLE = 1,
} pd_data_role_t;

typedef struct
{
    uint8_t extended;
    uint8_t num_data_objects;
    uint8_t message_id;
    pd_data_role_t power_role;
    uint8_t spec_revision;
    pd_data_role_t data_role;
    uint8_t message_type;
} pd_msg_header;

typedef struct pd_msg_s pd_msg;

struct pd_msg_s
{
    pd_rx_target_t target;
    pd_msg_header header;
    uint32_t pdo[7];
    bool immediate;
    void (*cbr)(pd_msg *msg, bool success);
    void *cbr_private;
};

typedef struct
{
    uint16_t svid;
    uint8_t vdm_type;
    uint8_t vdm_version_major;
    uint8_t vdm_version_minor;
    uint8_t object_position;
    pd_vdm_cmd_type command_type;
    pd_vdm_command command;
} pd_vdm_header;

typedef struct
{
    uint8_t usb_host;
    uint8_t usb_device;
    uint8_t sop_product_type;
    uint8_t modal_operation;
    uint16_t usb_vendor_id;
} pd_vdm_id_header_vdo;

typedef struct
{
    uint32_t usb_if_xid;
} pd_vdm_crt_stat_vdo;

typedef struct
{
    uint16_t usb_product_id;
    uint16_t bcd_device;
} pd_vdm_product_vdo;

typedef struct
{
    uint8_t hw_version;
    uint8_t fw_version;
    uint8_t vdo_version;
    uint8_t plug_type;
    uint8_t epr_capable;
    uint8_t cable_latency;
    uint8_t cable_termination;
    uint8_t max_vbus_voltage;
    uint8_t sbu_supported;
    uint8_t sbu_type;
    uint8_t vbus_current;
    uint8_t vbus_through;
    uint8_t sop_controller;
    uint8_t usb_speed;
} pd_vdm_cable_vdo1;

typedef struct
{
    uint8_t max_operating_temp;
    uint8_t shutdown_temp;
    uint8_t u3_cld_power;
    uint8_t u3_to_u0_transition;
    uint8_t physical_connection;
    uint8_t active_element;
    uint8_t usb4_supported;
    uint8_t usb2_hub_hops;
    uint8_t usb2_supported;
    uint8_t usb3_2_supported;
    uint8_t usb_lanes_supported;
    uint8_t optically_isolated;
    uint8_t usb4_asymmetric;
    uint8_t usb_gen;
} pd_vdm_cable_vdo2;

typedef struct
{
    pd_vdm_header vdm_header;
    pd_vdm_id_header_vdo id_header;
    pd_vdm_crt_stat_vdo crt_stat;
    pd_vdm_product_vdo product;
    pd_vdm_cable_vdo1 cable_1;
    pd_vdm_cable_vdo2 cable_2;
} pd_vdm_packet;

void pd_parse_msg_header(pd_msg_header *hdr, uint8_t *data);
void pd_build_msg_header(pd_msg_header *hdr, uint8_t *data);
void pd_dump_msg_header(pd_msg_header *hdr);

void pd_parse_vdm(pd_vdm_packet *pkt, uint32_t pdos[7]);
void pd_build_vdm(pd_vdm_packet *pkt, uint32_t pdos[7]);
void pd_dump_vdm(pd_vdm_packet *pkt);
