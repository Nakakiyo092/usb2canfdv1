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

#ifndef __BUFFER_H__
#define __BUFFER_H__

#include "can.h"
#include "usbd_cdc.h"

// CDC receive buffering
#define BUF_CDC_RX_NUM_BUFS 8
#define BUF_CDC_RX_BUF_SIZE CDC_DATA_FS_MAX_PACKET_SIZE // Size of RX buffer item

// CDC transmit buffering
#define BUF_CDC_TX_NUM_BUFS 3       // Should be >= 3 (triple or more buffering) to enable lock-free access.
#define BUF_CDC_TX_BUF_SIZE 4096    // Set to 64 * 64 for max single packet size

// CAN transmit buffering
#define BUF_CAN_TXQUEUE_LEN 64   // Number of buffers allocated

// Receive buffering: circular FIFO buffer
struct BufCdcRx
{
    uint8_t data[BUF_CDC_RX_NUM_BUFS][BUF_CDC_RX_BUF_SIZE];
    uint32_t msglen[BUF_CDC_RX_NUM_BUFS];
    uint8_t data_drop[BUF_CDC_RX_NUM_BUFS]; // Sets when data is dropped before each elament
    uint8_t head;
    uint8_t tail;
};

// Transmit buffering: lock-free (triple) buffer
struct BufCdcTx
{
    uint8_t data[BUF_CDC_TX_NUM_BUFS][BUF_CDC_TX_BUF_SIZE];
    uint32_t msglen[BUF_CDC_TX_NUM_BUFS];
    uint8_t head;
    uint8_t tail;
};

// Public variables
extern volatile struct BufCdcTx buf_cdc_tx;
extern volatile struct BufCdcRx buf_cdc_rx;

// Prototypes
void buf_init(void);
void buf_process(void);

void buf_enqueue_cdc(uint8_t* buf, uint16_t len);
uint8_t *buf_reserve_cdc_dest(uint16_t len);
void buf_commit_cdc_dest(uint16_t len);

FDCAN_TxHeaderTypeDef *buf_get_can_head_header(void);
FDCAN_TxHeaderTypeDef *buf_get_can_tail_header(void);
uint8_t *buf_get_can_head_data(void);
uint8_t *buf_get_can_tail_data(void);
HAL_StatusTypeDef buf_commit_can_head(void);
HAL_StatusTypeDef buf_release_can_tail(void);
void buf_clear_can_buffer(void);

#endif // _BUFFER_H
