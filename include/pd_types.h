#pragma once

#include <stdint.h>

#define BUILD_LE_UINT16(arr, idx) ((arr[(idx)]) | ((arr[(idx) + 1]) << 8))
#define BUILD_LE_UINT32(arr, idx) ((arr[(idx)]) | ((arr[(idx) + 1]) << 8) | ((arr[(idx) + 2]) << 16) | ((arr[(idx) + 3]) << 24))

/* Enum for special 4b5b symbols */
typedef enum
{
    HEX_0 = 0x0,
    HEX_1 = 0x1,
    HEX_2 = 0x2,
    HEX_3 = 0x3,
    HEX_4 = 0x4,
    HEX_5 = 0x5,
    HEX_6 = 0x6,
    HEX_7 = 0x7,
    HEX_8 = 0x8,
    HEX_9 = 0x9,
    HEX_A = 0xA,
    HEX_B = 0xB,
    HEX_C = 0xC,
    HEX_D = 0xD,
    HEX_E = 0xE,
    HEX_F = 0xF,
    SYNC_1 = 0x10,
    SYNC_2 = 0x11,
    RST_1 = 0x12,
    RST_2 = 0x13,
    EOP = 0x14,
    SYNC_3 = 0x15,
    ERROR = 0xFF
} line_code_symbol_t;

typedef enum
{
    PD_DATA_ROLE_UFP = 0,
    PD_DATA_ROLE_DFP,
} pd_data_role_t;

typedef enum
{
    PD_BUF_TYPE_INVALID = 0,
    PD_BUF_TYPE_TIMINGS,
    PD_BUF_TYPE_SYMBOLS,
    PD_BUF_TYPE_DATA,
} buf_type_t;

#define TARGET_SOP (SYNC_1 | (SYNC_1 << 8) | (SYNC_1 << 16) | (SYNC_2 << 24))
#define TARGET_SOP_P (SYNC_1 | (SYNC_1 << 8) | (SYNC_3 << 16) | (SYNC_3 << 24))
#define TARGET_SOP_PP (SYNC_1 | (SYNC_3 << 8) | (SYNC_1 << 16) | (SYNC_3 << 24))
#define TARGET_SOP_PD (SYNC_1 | (RST_2 << 8) | (RST_2 << 16) | (SYNC_3 << 24))
#define TARGET_SOP_PPD (SYNC_1 | (RST_2 << 8) | (SYNC_3 << 16) | (SYNC_2 << 24))
#define TARGET_HARD_RESET (RST_1 | (RST_1 << 8) | (RST_1 << 16) | (RST_2 << 24))
#define TARGET_CABLE_RESET (RST_1 | (SYNC_1 << 8) | (RST_1 << 16) | (SYNC_3 << 24))

typedef enum
{
    PD_RX_INIT = 0,
    PD_RX_PREAMBLE,
    PD_RX_SOP,
    PD_RX_PAYLOAD,
    PD_RX_FINISHED
} pd_rx_state_t;

typedef enum
{
    PD_TX_PATTERN,
    PD_TX_SYNC,
    PD_TX_DATA,
    PD_TX_EOP,
    PD_TX_DONE
} pd_tx_state_t;

typedef struct
{
    /* - PD working variables - */
    pd_rx_state_t state;

    /* - BMC working variables - */
    uint32_t bit_count;
    uint32_t bit_data;
    bool short_pulse;
    int last_shortened;
    /* we have 2 bytes for header, 7*4 for payload, 4 for checksum and one EOP symbol */
    int symbol_count;
    uint8_t symbols[/*2 * (2 + 7 * 4 + 4) + 1 */ 256];

    /* - parsed data for next layer - */
    uint32_t target;
    buf_type_t type;
    uint8_t length;
    uint8_t payload[/*2 + 7 * 4 + 4 */ 256];
} pd_rx_buf_t;

typedef struct
{
    bool level;
    uint8_t data_pos;
    uint8_t sync_bits;
    uint8_t sync_symbols;
    uint8_t eop_symbols;
    pd_tx_state_t state;
} pd_tx_ctx_t;

typedef struct
{
    bool connected;
    uint8_t message_id;
    uint8_t requested_object;
    uint8_t accepted_object;
    uint32_t request_voltage_mv;
    uint32_t request_current_ma;
} pd_state_t;

typedef enum
{
    PD_CONTROL_RESERVED = 0x00,
    PD_CONTROL_GOOD_CRC = 0x01,
    PD_CONTROL_GOTO_MIN = 0x02, /* Deprecated */
    PD_CONTROL_ACCEPT = 0x03,
    PD_CONTROL_REJECT = 0x04,
    PD_CONTROL_PING = 0x05, /* Deprecated */
    PD_CONTROL_PS_RDY = 0x06,
    PD_CONTROL_GET_SOURCE_CAP = 0x07,
    PD_CONTROL_GET_SINK_CAP = 0x08,
    PD_CONTROL_DR_SWAP = 0x09,
    PD_CONTROL_PR_SWAP = 0x0A,
    PD_CONTROL_VCONN_SWAP = 0x0B,
    PD_CONTROL_WAIT = 0x0C,
    PD_CONTROL_SOFT_RESET = 0x0D,
    PD_CONTROL_DATA_RESET = 0x0E,
    PD_CONTROL_DATA_RESET_COMPLETE = 0x0F,
    PD_CONTROL_NOT_SUPPORTED = 0x10,
    PD_CONTROL_GET_SOURCE_CAP_EXTENDED = 0x11,
    PD_CONTROL_GET_STATUS = 0x12,
    PD_CONTROL_FR_SWAP = 0x13,
    PD_CONTROL_GET_PPS_STATUS = 0x14,
    PD_CONTROL_GET_COUNTRY_CODES = 0x15,
    PD_CONTROL_GET_SINK_CAP_EXTENDED = 0x16,
    PD_CONTROL_GET_SOURCE_INFO = 0x17,
    PD_CONTROL_GET_REVISION = 0x18,
    PD_DATA_SOURCE_CAPABILITIES = 0x01,
    PD_DATA_REQUEST = 0x02,
    PD_DATA_BIST = 0x03,
    PD_DATA_SINK_CAPABILITIES = 0x04,
    PD_DATA_BATTERY_STATUS = 0x05,
    PD_DATA_ALERT = 0x06,
    PD_DATA_GET_COUNTRY_INFO = 0x07,
    PD_DATA_ENTER_USB = 0x08,
    PD_DATA_EPR_REQUEST = 0x09,
    PD_DATA_EPR_MODE = 0x0A,
    PD_DATA_SOURCE_INFO = 0x0B,
    PD_DATA_REVISION = 0x0C,
    PD_VENDOR_MESSAGE = 0x0F
} pd_message_type_t;

typedef enum
{
    PD_MODE_IDLE,
    PD_MODE_SINK
} pd_mode_t;

/* Structured VDM Header Bit Masks */
#define SVID_MASK        0x0000FFFF
#define VDM_TYPE_MASK    0x00000001
#define VDM_VERSION_MASK 0x00000003
#define VDM_MINOR_MASK   0x00000003
#define OBJ_POS_MASK     0x00000007
#define CMD_TYPE_MASK    0x00000003
#define RESERVED_MASK    0x00000001
#define COMMAND_MASK     0x0000001F

/* Bit Shift Values */
#define SVID_SHIFT        16
#define VDM_TYPE_SHIFT    15
#define VDM_VERSION_SHIFT 13
#define VDM_MINOR_SHIFT   11
#define OBJ_POS_SHIFT     8
#define CMD_TYPE_SHIFT    6
#define COMMAND_SHIFT     0

/* ID Header VDO Bit Masks */
#define USB_HOST_MASK         0x00000001
#define USB_DEVICE_MASK       0x00000001
#define SOP_PRODUCT_TYPE_MASK 0x00000003
#define MODAL_OPERATION_MASK  0x00000001
#define USB_VENDOR_ID_MASK    0x0000FFFF

/* ID Header VDO Bit Shifts */
#define USB_HOST_SHIFT         31
#define USB_DEVICE_SHIFT       30
#define SOP_PRODUCT_TYPE_SHIFT 27
#define MODAL_OPERATION_SHIFT  26
#define USB_VENDOR_ID_SHIFT    0


/* Product VDO */
#define USB_PRODUCT_ID_SHIFT   16
#define BCD_DEVICE_SHIFT       0
#define USB_PRODUCT_ID_MASK    0x0000FFFF
#define BCD_DEVICE_MASK        0x0000FFFF

/* Cable VDO1 */
#define HW_VERSION_SHIFT            28
#define FW_VERSION_SHIFT            24
#define VDO_VERSION_SHIFT           21
#define PLUG_TYPE_SHIFT             18
#define EPR_CAPABLE_SHIFT           17
#define CABLE_LATENCY_SHIFT         13
#define CABLE_TERMINATION_SHIFT     11
#define MAX_VBUS_VOLTAGE_SHIFT      9
#define SBU_SUPPORTED_SHIFT         8
#define SBU_TYPE_SHIFT              7
#define VBUS_CURRENT_SHIFT          5
#define VBUS_THROUGH_SHIFT          4
#define SOP_CONTROLLER_SHIFT        3
#define HW_VERSION_MASK             0x0000000F
#define FW_VERSION_MASK             0x0000000F
#define VDO_VERSION_MASK            0x00000007
#define PLUG_TYPE_MASK              0x00000003
#define EPR_CAPABLE_MASK            0x00000001
#define CABLE_LATENCY_MASK          0x0000000F
#define CABLE_TERMINATION_MASK      0x00000003
#define MAX_VBUS_VOLTAGE_MASK       0x00000003
#define SBU_SUPPORTED_MASK          0x00000001
#define SBU_TYPE_MASK               0x00000001
#define VBUS_CURRENT_MASK           0x00000003
#define VBUS_THROUGH_MASK           0x00000001
#define SOP_CONTROLLER_MASK         0x00000001

/* Cable VDO2 Bit Masks */
#define MAX_OPERATING_TEMP_MASK     0x000000FF
#define SHUTDOWN_TEMP_MASK          0x000000FF
#define RESERVED_CABLE3_MASK        0x00000001
#define U3_CLD_POWER_MASK           0x00000007
#define U3_TO_U0_TRANSITION_MASK    0x00000001
#define PHYSICAL_CONNECTION_MASK    0x00000001
#define ACTIVE_ELEMENT_MASK         0x00000001
#define USB4_SUPPORTED_MASK         0x00000001
#define USB2_HUB_HOPS_MASK          0x00000003
#define USB2_SUPPORTED_MASK         0x00000001
#define USB3_2_SUPPORTED_MASK       0x00000001
#define USB_LANES_SUPPORTED_MASK    0x00000001
#define OPTICALLY_ISOLATED_MASK     0x00000001
#define USB4_ASYMMETRIC_MASK        0x00000001
#define USB_GEN_MASK                0x00000001

/* Cable VDO2 Bit Shifts */
#define MAX_OPERATING_TEMP_SHIFT     24
#define SHUTDOWN_TEMP_SHIFT          16
#define U3_CLD_POWER_SHIFT           12
#define U3_TO_U0_TRANSITION_SHIFT    11
#define PHYSICAL_CONNECTION_SHIFT    10
#define ACTIVE_ELEMENT_SHIFT         9
#define USB4_SUPPORTED_SHIFT         8
#define USB2_HUB_HOPS_SHIFT          6
#define USB2_SUPPORTED_SHIFT         5
#define USB3_2_SUPPORTED_SHIFT       4
#define USB_LANES_SUPPORTED_SHIFT    3
#define OPTICALLY_ISOLATED_SHIFT     2
#define USB4_ASYMMETRIC_SHIFT        1
#define USB_GEN_SHIFT                0
