#pragma once

#include <stdint.h>
#include "pd_types.h"

#define GPIO_PD 4  //  状态指示？
// #define GPIO_CC1 3 // 通过与GPIO_TX连接来得到一个大约1.7V的分压器, ESP32内部上拉、下拉电阻为47K
#define GPIO_TX 3 // ^-^ 连接USB-CC. we now have configured the GPIO_CC1 pin and GPIO_TX pin to drive against each other. with these drive capabilities, we get a voltage divider to approx 1.7V, which is closer to the expected 1.1V on the PD lines
#define GPIO_CC1_IN 0 // ^-^ 连接MUN5233集电极. PD RX Path: Converts the 0-1.2V CC signal to 3.3V using a MUN5233 transistor and the IO in weak mode as a pull up.

#define PD_BUFFER_COUNT 64

#define PD_RX_ACK_TASK_PRIO (configMAX_PRIORITIES - 1)
#define PD_PROTOCOL_TASK_PRIO (configMAX_PRIORITIES - 2)
#define PD_TX_TASK_PRIO (configMAX_PRIORITIES - 2)
#define PD_LOG_TASK_PRIO (tskIDLE_PRIORITY + 1)


//#define PD_TEST_EMARKER_CABLE
#define PD_LOG_TX_PACKETS
