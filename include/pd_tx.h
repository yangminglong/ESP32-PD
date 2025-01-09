#pragma once

#include <stdint.h>
#include "pd_types.h"
#include "pd_proto.h"

void pd_tx_init();

/* check if a transmission is ongoing */
bool pd_tx_ongoing();

/* ToDo: Change the TX routines such that the checksum is automatically calculated 
   _OR_ create functions to build various packets and payloads which finally calc the CRC.
   Right now the caller needs to calculate checksums on its own.
   also: take care about retransmission when missing GoodCRC
*/

/* send a raw message on the CC line after checking it is idle */
void pd_tx(const uint8_t *data, size_t length);

/* enqueue an message for tx */
void pd_tx_enqueue(pd_msg *msg);

/* send a message on the CC line directly. assume it is free. used for rx confirmation */
void pd_tx_start(const uint8_t *data, size_t length);


void pd_tx_ack_received(uint32_t msg_id);

/* helper to build a header. need a proper packet factory. */
uint16_t pd_tx_header(
    uint8_t extended,
    uint8_t num_data_objects,
    uint8_t message_id,
    uint8_t power_role,
    uint8_t spec_revision,
    uint8_t data_role,
    uint8_t message_type);

