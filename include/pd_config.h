#pragma once

#include <stdint.h>
#include "pd_types.h"

// #define GPIO_CC1 3 // 通过与GPIO_TX连接来得到一个大约1.7V的分压器, ESP32内部上拉、下拉电阻为47K
#define GPIO_PD 4
#define GPIO_TX 3
#define GPIO_CC1_IN 0

#define PD_BUFFER_COUNT 64

#define PD_RX_ACK_TASK_PRIO (configMAX_PRIORITIES - 1)
#define PD_PROTOCOL_TASK_PRIO (configMAX_PRIORITIES - 2)
#define PD_TX_TASK_PRIO (configMAX_PRIORITIES - 2)
#define PD_LOG_TASK_PRIO (tskIDLE_PRIORITY + 1)


//#define PD_TEST_EMARKER_CABLE
#define PD_LOG_TX_PACKETS
