///////////////////////////////////////////////////////////////////////////////
// The MIT License (MIT)
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
///////////////////////////////////////////////////////////////////////////////

#ifndef _SLCAN_H
#define _SLCAN_H

// Timestamp mode
enum slcan_timestamp_mode
{
    SLCAN_TIMESTAMP_OFF = 0,
    SLCAN_TIMESTAMP_MILLI,
    SLCAN_TIMESTAMP_MICRO,

    SLCAN_TIMESTAMP_INVALID
};

// Startup mode
enum slcan_auto_startup_mode
{
    SLCAN_AUTO_STARTUP_OFF = 0,
    SLCAN_AUTO_STARTUP_NORMAL,
    SLCAN_AUTO_STARTUP_LISTEN,

    SLCAN_AUTO_STARTUP_INVALID
};

// Status flags, value is bit position in the status flags
enum slcan_status_flag
{
    SLCAN_STS_CAN_RX_FIFO_FULL = 0, /* Message loss. Not mean the buffer is just full. */
    SLCAN_STS_CAN_TX_FIFO_FULL,     /* Message loss. Not mean the buffer is just full. */
    SLCAN_STS_ERROR_WARNING,
    SLCAN_STS_DATA_OVERRUN,
    SLCAN_STS_RESERVED,
    SLCAN_STS_ERROR_PASSIVE,
    SLCAN_STS_ARBITRATION_LOST,     /* Not supported */
    SLCAN_STS_BUS_ERROR
};

// Report flag, value is bit position in the register
enum slcan_report_flag
{
    SLCAN_REPORT_RX = 0,
    SLCAN_REPORT_TX,
    //SLCAN_REPORT_ERROR,
    //SLCAN_REPORT_OVRLOAD,
    SLCAN_REPORT_ESI = 4,
};

// Maximum rx buffer len
#define SLCAN_MTU           (1 + 138 + 8 + 1 + 1 + 16) 
                            /* z/Z for tx event 1 plus frame 138 plus timestamp 8 plus ESI 1 plus \r 1 plus some padding */
#define SLCAN_STD_ID_LEN    (3)
#define SLCAN_EXT_ID_LEN    (8)

// Public variables
extern uint8_t slcan_nibble_to_ascii[];
extern enum slcan_timestamp_mode slcan_timestamp_mode;
extern uint16_t slcan_report_reg;

// Prototypes
int32_t slcan_generate_rx_frame(uint8_t *buf, FDCAN_RxHeaderTypeDef *frame_header, uint8_t *frame_data);
int32_t slcan_generate_tx_event(uint8_t *buf, FDCAN_TxEventFifoTypeDef *tx_event, uint8_t *frame_data);
uint16_t slcan_get_timestamp_ms(void);
uint32_t slcan_get_timestamp_us_from_tim3(uint16_t tim3_us);
void slcan_set_timestamp_mode(enum slcan_timestamp_mode mode);
void slcan_set_report_mode(uint16_t reg);
enum slcan_timestamp_mode slcan_get_timestamp_mode(void);
uint16_t slcan_get_report_mode(void);

void slcan_parse_str(uint8_t *buf, uint8_t len);
void slcan_raise_error(enum slcan_status_flag err);
void slcan_clear_error(void);
uint8_t slcan_get_status_flags(void);

#endif // _SLCAN_H
