#pragma once

#include <stdint.h>
#include "pd_types.h"


/* Array mapping 5b symbols to corresponding 4b values or special symbols */
static IRAM_ATTR const line_code_symbol_t line_code_decode[32] = {
    ERROR,  /* 00000  Shall Not be used */
    ERROR,  /* 00001  Shall Not be used */
    ERROR,  /* 00010  Shall Not be used */
    ERROR,  /* 00011  Shall Not be used */
    ERROR,  /* 00100  Shall Not be used */
    ERROR,  /* 00101  Shall Not be used */
    SYNC_3, /* 00110  Startsynch #3     */
    RST_1,  /* 00111  Hard Reset #1     */
    ERROR,  /* 01000  Shall Not be used */
    HEX_1,  /* 01001  hex data 1        */
    HEX_4,  /* 01010  hex data 4        */
    HEX_5,  /* 01011  hex data 5        */
    ERROR,  /* 01100  Shall Not be used */
    EOP,    /* 01101  EOP End of Packet */
    HEX_6,  /* 01110  hex data 6        */
    HEX_7,  /* 01111  hex data 7        */
    ERROR,  /* 10000  Shall Not be used */
    SYNC_2, /* 10001  Startsynch #2     */
    HEX_8,  /* 10010  hex data 8        */
    HEX_9,  /* 10011  hex data 9        */
    HEX_2,  /* 10100  hex data 2        */
    HEX_3,  /* 10101  hex data 3        */
    HEX_A,  /* 10110  hex data A        */
    HEX_B,  /* 10111  hex data B        */
    SYNC_1, /* 11000  Startsynch #1     */
    RST_2,  /* 11001  Hard Reset #2     */
    HEX_C,  /* 11010  hex data C        */
    HEX_D,  /* 11011  hex data D        */
    HEX_E,  /* 11100  hex data E        */
    HEX_F,  /* 11101  hex data F        */
    HEX_0,  /* 11110  hex data 0        */
    ERROR,  /* 11111  Shall Not be used */
};

/* Array mapping 5b symbols to corresponding 4b values or special symbols */
static IRAM_ATTR const line_code_symbol_t line_code_encode[] = {
    0b11110,
    0b01001,
    0b10100,
    0b10101,
    0b01010,
    0b01011,
    0b01110,
    0b01111,
    0b10010,
    0b10011,
    0b10110,
    0b10111,
    0b11010,
    0b11011,
    0b11100,
    0b11101,
    0b11000, /* SYNC_1 */
    0b10001, /* SYNC_2 */
    0b00111, /* RST_1 */
    0b11001, /* RST_2 */
    0b01101, /* EOP */
    0b00110 /* SYNC_3 */
};
