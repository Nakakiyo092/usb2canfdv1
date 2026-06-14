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

// Manage cdc (rx and tx) and can (tx) buffer (including error handling related to buffer full)

#include "usbd_cdc_if.h"
#include "buffer.h"
#include "can.h"
#include "led.h"
#include "parser.h"

// APP FIFO release pacing — bound on (send − tail) before forced release.
//
// Worst case: a burst of successes followed by all-failure pushes while
// the HAL TEF drains.
//   loop 1: push 3 frames, all succeed → 3 events fill HAL TEF
//   loops 2-4: push 3 frames each, all fail; process 1 event per loop
//   After loop 4: send=12, tail=3, nbr_sent_frames = TFQ * TEF = 9
//
// Other shapes stay below this peak:
//   - All-failure runs: TEF stays empty, growth is capped by buf_release_can_tail().
//   - Mixed runs: each released success drags older failed entries along, so
//     (send − tail) shrinks instead of drifting up.
//
// (3 * 3 * 2) = TFQ * TEF * Margin = 18 — 2× the worst case in case the
// pattern repeats, and well below BUF_CAN_TXQUEUE_LEN (64) so genuine
// overflow remains observable.
// See also: https://github.com/Nakakiyo092/usb2canfdv1/issues/49
#define BUF_MAX_NBR_SENT_FRAMES         (3 * 3 * 2)

// Cirbuf structure for CAN TX frames
struct BufCanTx
{
    FDCAN_TxHeaderTypeDef header[BUF_CAN_TXQUEUE_LEN];  // Header buffer
    uint8_t data[BUF_CAN_TXQUEUE_LEN][CAN_MAX_DATALEN]; // Data buffer
    uint16_t head;                              // Head index
    uint16_t send;                              // Send index
    uint16_t tail;                              // Tail index
    uint8_t full;                               // Set this when it is full, clear when the tail moves one.
};

// Public variables (shared with interrupts)
volatile struct BufCdcTx buf_cdc_tx = {0};
volatile struct BufCdcRx buf_cdc_rx = {0};

// Private variables
static struct BufCanTx buf_can_tx = {0};
static uint8_t cmd_line_buf[SLCAN_MTU];          // Command line buffer
static uint8_t cmd_line_buf_idx = 0;

// Private prototypes
static HAL_StatusTypeDef buf_release_can_tail(void);
static void buf_disable_irq(void);
static void buf_enable_irq(void);

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

    cmd_line_buf_idx = 0;
}

// Process
void buf_process(void)
{
    uint32_t cpy_head, new_head;
    uint32_t cpy_tail, new_tail;

	// This code for CDC buffer may include some redundant interrupt protection,
	// but it will remain in place for safety unless removing it yields a significant performance benefit.
	
    // Process cdc receive buffer
    // buf_cdc_rx.head is modified in interrupt, buf_cdc_rx.tail is referenced from interrupt.
    // buf_cdc_rx.head is referenced from main loop, buf_cdc_rx.tail is modified in main loop.
    // The head and tail are both 8-bit variable and atomic.
    // No need for interrupt disabling but "memory" clobber and memory barrier would be safe against compiler optimizations and CPU reordering.
    buf_disable_irq();
    cpy_head = buf_cdc_rx.head;
    buf_enable_irq();
    if (buf_cdc_rx.tail != cpy_head)
    {
        uint32_t idx_start = 0; // Start index of the data which is not corrupted

        // Check if the data in this buffer is corrupted due to overflow
        // If producer overflowed this slot, skip the torn prefix up to the first '\r'.
        uint8_t is_dropped = buf_cdc_rx.data_drop[buf_cdc_rx.tail];
        if (is_dropped)
        {
            // CDC Rx buffer overflow is intentionally reported through the
            // SLCAN status flag named SLCAN_STS_CAN_TX_FIFO_FULL (F bit 1).
            // The bit-to-source mapping (CAN and CDC share the same bits) is
            // defined in doc/2.-Command-List.md as the protocol contract.
            gen_raise_error(SLCAN_STS_CAN_TX_FIFO_FULL);
            cmd_line_buf_idx = 0;
            for (idx_start = 0; idx_start < buf_cdc_rx.msglen[buf_cdc_rx.tail]; idx_start++)
            {
                if (buf_cdc_rx.data[buf_cdc_rx.tail][idx_start] == '\r')    // \r = [CR] = delimiter
                {
                    break;
                }
            }
            idx_start++;
        }

        // Process one whole buffer
        for (uint32_t i = idx_start; i < buf_cdc_rx.msglen[buf_cdc_rx.tail]; i++)
	    {
            if (buf_cdc_rx.data[buf_cdc_rx.tail][i] == '\r')    // \r = [CR] = delimiter
            {
                psr_parse_str(cmd_line_buf, cmd_line_buf_idx);
                cmd_line_buf_idx = 0;

                // Blink RX LED as slcan rx if bus closed
                if (can_get_bus_state() == BUS_CLOSED) led_blink_rxd();
            }
            else
            {
                // Accumulate chars to reassemble them into a line of command (terminated by a [CR])
                cmd_line_buf[cmd_line_buf_idx++] = buf_cdc_rx.data[buf_cdc_rx.tail][i];

                // Check for command length
                if (cmd_line_buf_idx == SLCAN_MTU)
                {
                    // Any incoming command longer than MTU (including a [CR]) is invalid.
                    // Ensure a [BELL] will be returned when receiving a [CR].
                    cmd_line_buf_idx = 0;                    // Clear the command and
                    cmd_line_buf[cmd_line_buf_idx++] = '\a';    // ... mark as invalid (\a = [BELL])
                }
            }
        }

        // Move on to the next buffer
        new_tail = (buf_cdc_rx.tail + 1) % BUF_CDC_RX_NUM_BUFS;
        buf_disable_irq();
        buf_cdc_rx.tail = new_tail;
        buf_enable_irq();
    }

    // Process cdc transmit buffer
    // buf_cdc_tx.head is referenced from interrupt, buf_cdc_tx.tail is modified in interrupt.
    // buf_cdc_tx.head is modified in main loop, buf_cdc_tx.tail is modified in main loop.
    // The head and tail are both 8-bit variable and atomic.
    // No need for interrupt disabling for head but "memory" clobber and memory barrier would be safe against compiler optimizations and CPU reordering.
    buf_disable_irq();
    cpy_tail = buf_cdc_tx.tail;
    buf_enable_irq();
    new_head = (buf_cdc_tx.head + 1) % BUF_CDC_TX_NUM_BUFS;
    if (new_head != cpy_tail)
    {
        if (0 < buf_cdc_tx.msglen[buf_cdc_tx.head])
        {
            buf_disable_irq();
            buf_cdc_tx.head = new_head;
            buf_enable_irq();
            buf_cdc_tx.msglen[buf_cdc_tx.head] = 0;
        }
    }
    // Critical section against "CDC_TransmitCplt_FS"
	buf_disable_irq();
    new_tail = (buf_cdc_tx.tail + 1) % BUF_CDC_TX_NUM_BUFS;
    if (new_tail != buf_cdc_tx.head)
    {
        if (CDC_Transmit_FS((uint8_t *)buf_cdc_tx.data[new_tail], buf_cdc_tx.msglen[new_tail]) == USBD_OK)
        {
            buf_cdc_tx.tail = new_tail;
        }
    }
    buf_enable_irq();


    // Process can transmit buffer.
    // Guarded by BUS_OPENED: without this gate, frames still queued in
    // buf_can_tx after a C (channel close) are pushed at the HAL with no
    // controller available, fail, and route through gen_raise_error(
    // SLCAN_STS_DATA_OVERRUN). The next O clears the flag so there is no
    // host-visible misbehaviour today, but the gate makes the intent
    // explicit and stops the spurious DATA_OVERRUN flag from being raised
    // in the first place.
    if (can_get_bus_state() == BUS_OPENED)
    {
        while ((buf_can_tx.send != buf_can_tx.head) && (HAL_FDCAN_GetTxFifoFreeLevel(can_get_handle()) > 0))
        {
            HAL_StatusTypeDef status;

            // Transmit can frame
            status = HAL_FDCAN_AddMessageToTxFifoQ(can_get_handle(),
                                                   &buf_can_tx.header[buf_can_tx.send],
                                                   buf_can_tx.data[buf_can_tx.send]);

            // send is advanced unconditionally (drop-on-fail): advancing only on success risks
            // an infinite loop if the frame is permanently invalid (e.g., bad DLC). Frame loss
            // is detected as a marker mismatch and surfaced to the host via the F command.
            buf_can_tx.send = (buf_can_tx.send + 1) % BUF_CAN_TXQUEUE_LEN;

            uint16_t nbr_sent_frames;   // Number of frames in HAL waiting for being sent
            nbr_sent_frames = (BUF_CAN_TXQUEUE_LEN + buf_can_tx.send - buf_can_tx.tail) % BUF_CAN_TXQUEUE_LEN;
            if (BUF_MAX_NBR_SENT_FRAMES < nbr_sent_frames)
            {
                buf_release_can_tail();  // Assume the frame is deleted in HAL (Disabled retransmission or overflow)
                // Do not raise error here because it shold not be for disabled retransmission.
                // Overflow can be catched by checking the error flags, which is done in can.c.
            }

            if (status != HAL_OK)
            {
                gen_raise_error(SLCAN_STS_DATA_OVERRUN);
                // TODO Would it be better to try again later than dropping the frame?
            }
        }
    }
}

// Enqueue data for transmission over USB CDC to host (copy and commit = slower)
//
// Note on the overflow flag: CDC Tx buffer overflow is reported through
// SLCAN_STS_CAN_RX_FIFO_FULL (F bit 0). The flag name refers to CAN Rx but the
// same bit is shared by the CDC Tx side, as documented in doc/2.-Command-List.md.
void buf_enqueue_cdc(uint8_t* buf, uint16_t len)
{
    if (BUF_CDC_TX_BUF_SIZE < buf_cdc_tx.msglen[buf_cdc_tx.head] + len)
    {
        gen_raise_error(SLCAN_STS_CAN_RX_FIFO_FULL);  // CDC Tx overflow -> F bit 0; see doc/2
        return;
    }

    // Copy the data
    memcpy((uint8_t *)&buf_cdc_tx.data[buf_cdc_tx.head][buf_cdc_tx.msglen[buf_cdc_tx.head]], buf, len);
    buf_cdc_tx.msglen[buf_cdc_tx.head] += len;
}

// Get destination pointer of cdc buffer for len bytes data (Start position of write access)
// This function combined with buf_commit_cdc_dest will provide a faster access compared to buf_enqueue_cdc.
// Return NULL if the data does not fit in the buffer.
// See buf_enqueue_cdc above for the CDC Tx overflow / F bit 0 mapping rationale.
uint8_t *buf_reserve_cdc_dest(uint16_t len)
{
    if (BUF_CDC_TX_BUF_SIZE < buf_cdc_tx.msglen[buf_cdc_tx.head] + len)
    {
        // Raise error since the caller will not call commit after they fail to reserve buffer.
		gen_raise_error(SLCAN_STS_CAN_RX_FIFO_FULL);  // CDC Tx overflow -> F bit 0; see doc/2
        return NULL;
    }

    return (uint8_t *)&buf_cdc_tx.data[buf_cdc_tx.head][buf_cdc_tx.msglen[buf_cdc_tx.head]];
}

// Send the data bytes in destination area over USB CDC to host
// See buf_enqueue_cdc above for the CDC Tx overflow / F bit 0 mapping rationale.
void buf_commit_cdc_dest(uint16_t len)
{
    if (BUF_CDC_TX_BUF_SIZE < buf_cdc_tx.msglen[buf_cdc_tx.head] + len)
    {
        // The data will not fit in the buffer.
		gen_raise_error(SLCAN_STS_CAN_RX_FIFO_FULL);  // CDC Tx overflow -> F bit 0; see doc/2
        return;
    }

    buf_cdc_tx.msglen[buf_cdc_tx.head] += len;
}

// Get pointer to the frame header of the head can frame
// Return NULL if the buffer is full.
FDCAN_TxHeaderTypeDef *buf_get_can_head_header(void)
{
    if (buf_can_tx.full)
    {
        gen_raise_error(SLCAN_STS_CAN_TX_FIFO_FULL);
        return NULL;
    }

    return &buf_can_tx.header[buf_can_tx.head];
}

// Get pointer to the frame header of the sent can frame with the given marker
// Return NULL if the buffer is empty or the frame is not found.
FDCAN_TxHeaderTypeDef *buf_get_can_sent_header(uint8_t marker)
{
    if ((buf_can_tx.head == buf_can_tx.tail) && !buf_can_tx.full)
    {
        gen_raise_error(SLCAN_STS_DATA_OVERRUN);  // TODO Is this necessary?
        return NULL;
    }

    // TODO Deduplicate marker-search logic shared with buf_get_can_sent_data (e.g., static buf_find_can_marker helper)
    uint8_t idx = buf_can_tx.tail;
    while (idx != buf_can_tx.send)
    {
        if (buf_can_tx.header[idx].MessageMarker == marker)
        {
            return &buf_can_tx.header[idx];
        }
        idx = (idx + 1) % BUF_CAN_TXQUEUE_LEN;
    }

    return NULL;
}

// Get pointer to the frame data of the head can frame
// Return NULL if the buffer is full.
uint8_t *buf_get_can_head_data(void)
{
    if (buf_can_tx.full)
    {
        gen_raise_error(SLCAN_STS_CAN_TX_FIFO_FULL);
        return NULL;
    }

    return buf_can_tx.data[buf_can_tx.head];
}

// Get pointer to the frame data of the sent can frame with the given marker
// Return NULL if the buffer is empty or the frame is not found.
uint8_t *buf_get_can_sent_data(uint8_t marker)
{
    if ((buf_can_tx.head == buf_can_tx.tail) && !buf_can_tx.full)
    {
        gen_raise_error(SLCAN_STS_DATA_OVERRUN);  // TODO Is this necessary?
        return NULL;
    }

    // TODO Deduplicate marker-search logic shared with buf_get_can_sent_header (e.g., static buf_find_can_marker helper)
    uint8_t idx = buf_can_tx.tail;
    while (idx != buf_can_tx.send)
    {
        if (buf_can_tx.header[idx].MessageMarker == marker)
        {
            return buf_can_tx.data[idx];
        }
        idx = (idx + 1) % BUF_CAN_TXQUEUE_LEN;
    }

    return NULL;
}

// Send the message in head slot on the CAN bus.
HAL_StatusTypeDef buf_commit_can_head(void)
{
    if (can_is_tx_enabled() == ENABLE)
    {
        // If the queue is full
        if (buf_can_tx.full)
        {
            gen_raise_error(SLCAN_STS_CAN_TX_FIFO_FULL);
            return HAL_ERROR;
        }

        // Increment the head index
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
HAL_StatusTypeDef buf_release_can_tail(void)
{
    if ((buf_can_tx.head == buf_can_tx.tail) && !buf_can_tx.full)
    {
        return HAL_ERROR;
    }

    buf_can_tx.tail = (buf_can_tx.tail + 1) % BUF_CAN_TXQUEUE_LEN;
    buf_can_tx.full = 0;

    return HAL_OK;
}

// Delete frames in the can tx buffer until the frame with the given marker (including the frame).
HAL_StatusTypeDef buf_release_can_until(uint8_t marker)
{
    // If the buffer is empty or the frame with the marker is not found
    if (buf_get_can_sent_data(marker) == NULL)
    {
        return HAL_ERROR;
    }

    while (buf_can_tx.tail != buf_can_tx.send)
    {
        if (buf_can_tx.header[buf_can_tx.tail].MessageMarker == marker)
        {
            buf_can_tx.tail = (buf_can_tx.tail + 1) % BUF_CAN_TXQUEUE_LEN;
            break;
        }
        buf_can_tx.tail = (buf_can_tx.tail + 1) % BUF_CAN_TXQUEUE_LEN;
    }
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
static void buf_disable_irq(void)
{
    __disable_irq();
    __DSB(); // Data Synchronization Barrier
    __ISB(); // Instruction Synchronization Barrier
}
static void buf_enable_irq(void)
{
    __enable_irq();
    __DSB();
    __ISB();
}
