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

// Manage cdc and can buffer


#include "usbd_cdc_if.h"
#include "buffer.h"
#include "can.h"
#include "led.h"
#include "slcan.h"

// Cirbuf structure for CAN TX frames
struct buf_can_tx
{
    FDCAN_TxHeaderTypeDef header[BUF_CAN_TXQUEUE_LEN];  // Header buffer
    uint8_t data[BUF_CAN_TXQUEUE_LEN][CAN_MAX_DATALEN]; // Data buffer
    uint16_t head;                              // Head pointer
    uint16_t send;                              // Send pointer
    uint16_t tail;                              // Tail pointer
    uint8_t full;                               // Set this when it is full, clear when the tail moves one.
};

// Public variables (shared with interrupts)
volatile struct buf_cdc_tx buf_cdc_tx = {0};
volatile struct buf_cdc_rx buf_cdc_rx = {0};

// Private variables
static struct buf_can_tx buf_can_tx = {0};
static uint8_t slcan_str[SLCAN_MTU];
static uint8_t slcan_str_index = 0;

// Private prototypes

// Initializes
void buf_init(void)
{
    buf_cdc_rx.head = 0;
    buf_cdc_rx.tail = 0;

    buf_cdc_tx.head = 1;
    buf_cdc_tx.msglen[buf_cdc_tx.head] = 0;
    buf_cdc_tx.tail = 0;
    buf_cdc_tx.msglen[buf_cdc_tx.tail] = 0;

    buf_can_tx.head = 0;
    buf_can_tx.send = 0;
    buf_can_tx.tail = 0;
    buf_can_tx.full = 0;
}

// Process
void buf_process(void)
{
    // Process cdc receive buffer
    __disable_irq();
    uint32_t tmp_head = buf_cdc_rx.head;
    __enable_irq();
    if (buf_cdc_rx.tail != tmp_head)
    {
        //  Process one whole buffer
        for (uint32_t i = 0; i < buf_cdc_rx.msglen[buf_cdc_rx.tail]; i++)
	    {
            if (buf_cdc_rx.data[buf_cdc_rx.tail][i] == '\r')
            {
                slcan_parse_str(slcan_str, slcan_str_index);
                slcan_str_index = 0;

                // Blink RX LED as slcan rx if bus closed
                if (can_get_bus_state() == BUS_CLOSED) led_blink_rxd();
            }
            else
            {
                // Check for buffer overflow
                if (slcan_str_index >= SLCAN_MTU)
                {
                    slcan_str_index = 0;
                }

                slcan_str[slcan_str_index++] = buf_cdc_rx.data[buf_cdc_rx.tail][i];
            }
        }

        // Move on to the next buffer
    	__disable_irq();
        buf_cdc_rx.tail = (buf_cdc_rx.tail + 1) % BUF_CDC_RX_NUM_BUFS;
    	__enable_irq();
    }

    // Process cdc transmit buffer
    uint32_t new_head = (buf_cdc_tx.head + 1UL) % BUF_CDC_TX_NUM_BUFS;
    if (new_head != buf_cdc_tx.tail)
    {
        if (0 < buf_cdc_tx.msglen[buf_cdc_tx.head])
        {
            buf_cdc_tx.head = new_head;
            buf_cdc_tx.msglen[new_head] = 0;
        }
    }
    __disable_irq();
    uint32_t new_tail = (buf_cdc_tx.tail + 1UL) % BUF_CDC_TX_NUM_BUFS;
    if (new_tail != buf_cdc_tx.head)
    {
        if (CDC_Transmit_FS((uint8_t *)buf_cdc_tx.data[new_tail], buf_cdc_tx.msglen[new_tail]) == USBD_OK)
        {
            buf_cdc_tx.tail = new_tail;
        }
    }
    __enable_irq();


    // Process can transmit buffer
    while ((buf_can_tx.send != buf_can_tx.head || buf_can_tx.full) && (HAL_FDCAN_GetTxFifoFreeLevel(can_get_handle()) > 0))
    {
        HAL_StatusTypeDef status;

        // Transmit can frame
        status = HAL_FDCAN_AddMessageToTxFifoQ(can_get_handle(), 
                                               &buf_can_tx.header[buf_can_tx.send], 
                                               buf_can_tx.data[buf_can_tx.send]);

        buf_can_tx.send = (buf_can_tx.send + 1) % BUF_CAN_TXQUEUE_LEN;

        if (status != HAL_OK)
        {
            slcan_raise_error(SLCAN_STS_DATA_OVERRUN);
        }
    }
}

// Enqueue data for transmission over USB CDC to host (copy and comit = slower)
void buf_enqueue_cdc(uint8_t* buf, uint16_t len)
{
    if (BUF_CDC_TX_BUF_SIZE < buf_cdc_tx.msglen[buf_cdc_tx.head] + len)
    {
        slcan_raise_error(SLCAN_STS_CAN_RX_FIFO_FULL);  // The data does not fit in the buffer
        return;
    }

    // Copy the data
    memcpy((uint8_t *)&buf_cdc_tx.data[buf_cdc_tx.head][buf_cdc_tx.msglen[buf_cdc_tx.head]], buf, len);
    buf_cdc_tx.msglen[buf_cdc_tx.head] += len;
}

// Get destination pointer of cdc buffer for len bytes data (Start position of write access)
// This function combined with buf_comit_cdc_dest will provide a faster access compared to buf_enqueue_cdc.
uint8_t *buf_get_cdc_dest(uint16_t len)
{
    if (BUF_CDC_TX_BUF_SIZE < buf_cdc_tx.msglen[buf_cdc_tx.head] + len)
    {
        slcan_raise_error(SLCAN_STS_CAN_RX_FIFO_FULL);  // The data will not fit in the buffer
        return NULL;
    }

    return (uint8_t *)&buf_cdc_tx.data[buf_cdc_tx.head][buf_cdc_tx.msglen[buf_cdc_tx.head]];
}

// Send the data bytes in destination area over USB CDC to host
void buf_comit_cdc_dest(uint16_t len)
{
    if (BUF_CDC_TX_BUF_SIZE < buf_cdc_tx.msglen[buf_cdc_tx.head] + len)
    {
        slcan_raise_error(SLCAN_STS_CAN_RX_FIFO_FULL);  // The data will not fit in the buffer
        return;
    }

    buf_cdc_tx.msglen[buf_cdc_tx.head] += len;
}

// Get destination pointer of can tx frame header
FDCAN_TxHeaderTypeDef *buf_get_can_dest_header(void)
{
    if (buf_can_tx.full)
    {
        slcan_raise_error(SLCAN_STS_CAN_TX_FIFO_FULL);;
        return NULL;
    }

    return &buf_can_tx.header[buf_can_tx.head];
}

// Get destination pointer of can tx frame data bytes
uint8_t *buf_get_can_dest_data(void)
{
    if (buf_can_tx.full)
    {
        slcan_raise_error(SLCAN_STS_CAN_TX_FIFO_FULL);;
        return NULL;
    }

    return buf_can_tx.data[buf_can_tx.head];
}

// Send the message in destination slot on the CAN bus.
HAL_StatusTypeDef buf_comit_can_dest(void)
{
    if (can_is_tx_enabled() == ENABLE)
    {
        // If the queue is full
        if (buf_can_tx.full)
        {
            slcan_raise_error(SLCAN_STS_CAN_TX_FIFO_FULL);;
            return HAL_ERROR;
        }

        // Increment the head pointer
        buf_can_tx.head = (buf_can_tx.head + 1) % BUF_CAN_TXQUEUE_LEN;
        if (buf_can_tx.head == buf_can_tx.tail) buf_can_tx.full = 1;
    }
    else
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Dequeue data bytes from the can tx buffer (Delete one frame)
uint8_t *buf_dequeue_can_tx_data(void)
{
    uint32_t tmp_tail = buf_can_tx.tail;

    buf_can_tx.tail = (buf_can_tx.tail + 1) % BUF_CAN_TXQUEUE_LEN;
    buf_can_tx.full = 0;

    return buf_can_tx.data[tmp_tail];
}

// Clear can tx buffer
void buf_clear_can_buffer(void)
{
    buf_can_tx.tail = buf_can_tx.head;
    buf_can_tx.send = buf_can_tx.head;
    buf_can_tx.full = 0;
}
