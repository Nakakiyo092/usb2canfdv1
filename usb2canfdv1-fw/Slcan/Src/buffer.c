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

// Maximum number of frames stored in HAL waiting for being sent
#define BUF_MAX_NBR_SEND_FRAMES         (3 + 3 + 2)         // SRAMCAN_TFQ_NBR 3 + SRAMCAN_TEF_NBR 3 + Margin

// Cirbuf structure for CAN TX frames
struct BufCanTx
{
    FDCAN_TxHeaderTypeDef header[BUF_CAN_TXQUEUE_LEN];  // Header buffer
    uint8_t data[BUF_CAN_TXQUEUE_LEN][CAN_MAX_DATALEN]; // Data buffer
    uint16_t head;                              // Head pointer
    uint16_t send;                              // Send pointer
    uint16_t tail;                              // Tail pointer
    uint8_t full;                               // Set this when it is full, clear when the tail moves one.
};

// Public variables (shared with interrupts)
volatile struct BufCdcTx buf_cdc_tx = {0};
volatile struct BufCdcRx buf_cdc_rx = {0};

// Private variables
static struct BufCanTx buf_can_tx = {0};
static uint8_t slcan_str[SLCAN_MTU];
static uint8_t slcan_str_index = 0;

// Private prototypes
static void buf_disable_irq();
static void buf_enable_irq();

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
    buf_disable_irq();
    uint8_t data_ready = (buf_cdc_rx.tail != buf_cdc_rx.head);
    buf_enable_irq();
    if (data_ready)
    {
        //  Process one whole buffer
        for (uint32_t i = 0; i < buf_cdc_rx.msglen[buf_cdc_rx.tail]; i++)
	    {
            if (buf_cdc_rx.data[buf_cdc_rx.tail][i] == '\r')    // \r = [CR] = delimiter
            {
                slcan_parse_str(slcan_str, slcan_str_index);
                slcan_str_index = 0;

                // Blink RX LED as slcan rx if bus closed
                if (can_get_bus_state() == BUS_CLOSED) led_blink_rxd();
            }
            else
            {
                slcan_str[slcan_str_index++] = buf_cdc_rx.data[buf_cdc_rx.tail][i];

                // Check for command length
                if (slcan_str_index == SLCAN_MTU)
                {
                    // Any incoming command longer than MTU (including a [CR]) is invalid.
                    // Ensure a [BELL] will be returned when receiving a [CR].
                    slcan_str_index = 0;                    // Clear the command and
                    slcan_str[slcan_str_index++] = '\a';    // ... mark as invalid (\a = [BELL])
                }
            }
        }

        // Move on to the next buffer
    	buf_disable_irq();
        buf_cdc_rx.tail = (buf_cdc_rx.tail + 1) % BUF_CDC_RX_NUM_BUFS;
    	buf_enable_irq();
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
    buf_disable_irq();
    uint32_t new_tail = (buf_cdc_tx.tail + 1UL) % BUF_CDC_TX_NUM_BUFS;
    if (new_tail != buf_cdc_tx.head)
    {
        if (CDC_Transmit_FS((uint8_t *)buf_cdc_tx.data[new_tail], buf_cdc_tx.msglen[new_tail]) == USBD_OK)
        {
            buf_cdc_tx.tail = new_tail;
        }
    }
    buf_enable_irq();


    // Process can transmit buffer
    while ((buf_can_tx.send != buf_can_tx.head) && (HAL_FDCAN_GetTxFifoFreeLevel(can_get_handle()) > 0))
    {
        HAL_StatusTypeDef status;

        // Transmit can frame
        status = HAL_FDCAN_AddMessageToTxFifoQ(can_get_handle(), 
                                               &buf_can_tx.header[buf_can_tx.send], 
                                               buf_can_tx.data[buf_can_tx.send]);

        buf_can_tx.send = (buf_can_tx.send + 1) % BUF_CAN_TXQUEUE_LEN;

        uint16_t nbr_send_frames;   // Number of frames in HAL waiting for being sent
        nbr_send_frames = (BUF_CAN_TXQUEUE_LEN + buf_can_tx.send - buf_can_tx.tail) % BUF_CAN_TXQUEUE_LEN;
        if (BUF_MAX_NBR_SEND_FRAMES < nbr_send_frames)
        {
            buf_delete_can_tail();  // Assume the frame is deleted in HAL
        }

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

// Get head pointer of can tx frame header
FDCAN_TxHeaderTypeDef *buf_get_can_head_header(void)
{
    if (buf_can_tx.full)
    {
        slcan_raise_error(SLCAN_STS_CAN_TX_FIFO_FULL);;
        return NULL;
    }

    return &buf_can_tx.header[buf_can_tx.head];
}

// Get tail pointer of can tx frame header
FDCAN_TxHeaderTypeDef *buf_get_can_tail_header(void)
{
    if ((buf_can_tx.head == buf_can_tx.tail) && !buf_can_tx.full)
    {
        slcan_raise_error(SLCAN_STS_DATA_OVERRUN);;
        return NULL;
    }

    return &buf_can_tx.header[buf_can_tx.tail];
}

// Get head pointer of can tx frame data bytes
uint8_t *buf_get_can_head_data(void)
{
    if (buf_can_tx.full)
    {
        slcan_raise_error(SLCAN_STS_CAN_TX_FIFO_FULL);;
        return NULL;
    }

    return buf_can_tx.data[buf_can_tx.head];
}

// Get tail pointer of can tx frame data bytes
uint8_t *buf_get_can_tail_data(void)
{
    if ((buf_can_tx.head == buf_can_tx.tail) && !buf_can_tx.full)
    {
        slcan_raise_error(SLCAN_STS_DATA_OVERRUN);;
        return NULL;
    }

    return buf_can_tx.data[buf_can_tx.tail];
}

// Send the message in head slot on the CAN bus.
HAL_StatusTypeDef buf_comit_can_head(void)
{
    if (can_is_tx_enabled() == ENABLE)
    {
        // If the queue is full
        if (buf_can_tx.full)
        {
            slcan_raise_error(SLCAN_STS_CAN_TX_FIFO_FULL);
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

// Delete one frame from the can tx buffer
HAL_StatusTypeDef buf_delete_can_tail(void)
{
    while ((buf_can_tx.head == buf_can_tx.tail) && !buf_can_tx.full)
    {
        return HAL_ERROR;
    }

    buf_can_tx.tail = (buf_can_tx.tail + 1) % BUF_CAN_TXQUEUE_LEN;
    buf_can_tx.full = 0;

    return HAL_OK;
}

// Clear can tx buffer
void buf_clear_can_buffer(void)
{
    buf_can_tx.tail = buf_can_tx.head;
    buf_can_tx.send = buf_can_tx.head;
    buf_can_tx.full = 0;
}

// Disable/Enable IRQ with memory barrier
void buf_disable_irq()
{
    __disable_irq();
    __DSB(); // Data Synchronization Barrier
    __ISB(); // Instruction Synchronization Barrier
}
void buf_enable_irq()
{
    __enable_irq();
    __ISB(); // Instruction Synchronization Barrier
}
