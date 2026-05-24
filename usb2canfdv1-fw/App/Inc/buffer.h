///////////////////////////////////////////////////////////////////////////////
// GNU General Public License v3.0
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
//
// Full license text: https://www.gnu.org/licenses/gpl-3.0.txt
// See also: LICENSE.md in the root of this repository
///////////////////////////////////////////////////////////////////////////////

#ifndef USB2CANFDV1_BUFFER_H
#define USB2CANFDV1_BUFFER_H

#include <stdint.h>
#include "can.h"
#include "usbd_cdc.h"

// CDC receive buffering
#define BUF_CDC_RX_NUM_BUFS 8       // Should be >= 3 (triple buffering) to avoid dead lock
#define BUF_CDC_RX_BUF_SIZE CDC_DATA_FS_MAX_PACKET_SIZE // Size of RX buffer item

// CDC transmit buffering
#define BUF_CDC_TX_NUM_BUFS 3       // Should be >= 3 (triple buffering) to avoid dead lock
#define BUF_CDC_TX_BUF_SIZE 4096    // Set to 64 * 64 for max single packet size

// CAN transmit buffering
#define BUF_CAN_TXQUEUE_LEN 64   // Number of buffers allocated

// Receive buffering: circular FIFO buffer
struct BufCdcRx
{
    uint8_t data[BUF_CDC_RX_NUM_BUFS][BUF_CDC_RX_BUF_SIZE];
    uint32_t msglen[BUF_CDC_RX_NUM_BUFS];
    uint8_t data_drop[BUF_CDC_RX_NUM_BUFS]; // Sets when data is dropped before each element
    uint8_t head;
    uint8_t tail;
};

// Transmit buffering: circular FIFO buffer
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
FDCAN_TxHeaderTypeDef *buf_get_can_sent_header(uint8_t marker);
uint8_t *buf_get_can_head_data(void);
uint8_t *buf_get_can_sent_data(uint8_t marker);
HAL_StatusTypeDef buf_commit_can_head(void);
HAL_StatusTypeDef buf_release_can_until(uint8_t marker);
void buf_clear_can_buffer(void);

#endif // USB2CANFDV1_BUFFER_H
