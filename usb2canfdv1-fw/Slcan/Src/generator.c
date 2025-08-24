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

// Generate outgoing slcan messages.

#include "stm32g0xx_hal.h"
#include "can.h"
#include "slcan.h"

// Public variables
uint8_t slcan_nibble_to_ascii[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};
enum slcan_timestamp_mode slcan_timestamp_mode = 0;
uint16_t slcan_report_reg = 1;   // Default: no timestamp, no ESI, no Tx, but with Rx

// Private methods
static int32_t slcan_generate_frame(uint8_t *buf, FDCAN_RxHeaderTypeDef *frame_header, uint8_t *frame_data);

// Generate a slcan message from a CAN frame
// Returns number of bytes written into buf
//  MIN: 1 (r) + SLCAN_STD_ID_LEN + 2 (DLC & [CR])
//  MAX: SLCAN_MTU - 1 (z/Z) - 16 (padding)
int32_t slcan_generate_frame(uint8_t *buf, FDCAN_RxHeaderTypeDef *frame_header, uint8_t *frame_data)
{
    // Start building the slcan message string at idx 0 in buf
    uint8_t msg_idx = 0;

    // Handle remote frames
    if (frame_header->RxFrameType == FDCAN_REMOTE_FRAME)
    {
        // Add character for frame type
        buf[msg_idx] = 'r';
    }
    // Handle classic CAN frames
    else if (frame_header->FDFormat == FDCAN_CLASSIC_CAN)
    {
        buf[msg_idx] = 't';
    }
    // Handle FD CAN frames
    else
    {
        // Frame with BRS enabled
        if (frame_header->BitRateSwitch == FDCAN_BRS_ON)
        {
            buf[msg_idx] = 'b';
        }
        // Frame with BRS disabled
        else
        {
            buf[msg_idx] = 'd';
        }
    }

    // Check id type
    if (frame_header->IdType == FDCAN_STANDARD_ID)
    {
        msg_idx = 1 + SLCAN_STD_ID_LEN;     // Type & ID
    }
    else
    {
        // Convert first char to upper case for extended frame
        buf[msg_idx] -= 32;     // 'a' - 'A'
        msg_idx = 1 + SLCAN_EXT_ID_LEN;     // Type & ID
    }

    // Add identifier to the buffer
    uint32_t tmp = frame_header->Identifier;
    for (uint8_t j = msg_idx - 1; j >= 1; j--)
    {
        // Add nibble to the buffer
        buf[j] = slcan_nibble_to_ascii[tmp & 0xF];
        tmp = tmp >> 4;
    }

    // Add DLC to the buffer
    buf[msg_idx++] = slcan_nibble_to_ascii[CAN_HAL_DLC_TO_STD_DLC(frame_header->DataLength)];
    int8_t bytes = can_dlc_to_bytes[CAN_HAL_DLC_TO_STD_DLC(frame_header->DataLength)];
    
    // Add data bytes
    // Data frame only. No data bytes for a remote frame.
    if (frame_header->RxFrameType != FDCAN_REMOTE_FRAME)
    {
        for (uint8_t j = 0; j < bytes; j++)
        {
            buf[msg_idx++] = slcan_nibble_to_ascii[frame_data[j] >> 4];
            buf[msg_idx++] = slcan_nibble_to_ascii[frame_data[j] & 0xF];
        }
    }

    // Add time stamp
    if (slcan_timestamp_mode == SLCAN_TIMESTAMP_MILLI)
    {
        uint16_t timestamp_ms = slcan_get_timestamp_ms();

        buf[msg_idx++] = slcan_nibble_to_ascii[(timestamp_ms >> 12) & 0xF];
        buf[msg_idx++] = slcan_nibble_to_ascii[(timestamp_ms >> 8) & 0xF];
        buf[msg_idx++] = slcan_nibble_to_ascii[(timestamp_ms >> 4) & 0xF];
        buf[msg_idx++] = slcan_nibble_to_ascii[timestamp_ms & 0xF];
    }
    else if (slcan_timestamp_mode == SLCAN_TIMESTAMP_MICRO)
    {
        uint32_t timestamp_us = slcan_get_timestamp_us_from_tim3(frame_header->RxTimestamp);

        buf[msg_idx++] = slcan_nibble_to_ascii[(timestamp_us >> 28) & 0xF];
        buf[msg_idx++] = slcan_nibble_to_ascii[(timestamp_us >> 24) & 0xF];
        buf[msg_idx++] = slcan_nibble_to_ascii[(timestamp_us >> 20) & 0xF];
        buf[msg_idx++] = slcan_nibble_to_ascii[(timestamp_us >> 16) & 0xF];
        buf[msg_idx++] = slcan_nibble_to_ascii[(timestamp_us >> 12) & 0xF];
        buf[msg_idx++] = slcan_nibble_to_ascii[(timestamp_us >> 8) & 0xF];
        buf[msg_idx++] = slcan_nibble_to_ascii[(timestamp_us >> 4) & 0xF];
        buf[msg_idx++] = slcan_nibble_to_ascii[timestamp_us & 0xF];
    }
    
    // Add error state indicator
    // FD frame only. No ESI for a classical frame.
    if ((slcan_report_reg >> SLCAN_REPORT_ESI) & 1)
    {
        if (frame_header->FDFormat == FDCAN_FD_CAN)
        {
            if (frame_header->ErrorStateIndicator == FDCAN_ESI_ACTIVE)
                buf[msg_idx++] = slcan_nibble_to_ascii[0];
            else
                buf[msg_idx++] = slcan_nibble_to_ascii[1];
        }
    }

    // Add CR for slcan EOL
    buf[msg_idx++] = '\r';

    // Return string length
    return msg_idx;
}

// Parse an incoming CAN frame into an outgoing slcan message
// Returns number of bytes written into buf
//  MIN: 1 (r) + SLCAN_STD_ID_LEN + 2 (DLC & [CR])
//  MAX: SLCAN_MTU - 1 (z/Z) - 16 (padding)
int32_t slcan_generate_rx_frame(uint8_t *buf, FDCAN_RxHeaderTypeDef *frame_header, uint8_t *frame_data)
{
    // Check if Rx reporting is required
    if (((slcan_report_reg >> SLCAN_REPORT_RX) & 1) == 0)
        return 0;

    if (buf == NULL)
        return 0;

    int32_t len = slcan_generate_frame(buf, frame_header, frame_data);

    // Return string length
    return len;
}

// Parse an incoming Tx event into an outgoing slcan message
// Returns number of bytes written into buf
//  MIN: 1 (r) + SLCAN_STD_ID_LEN + 2 (DLC & [CR])
//  MAX: SLCAN_MTU - 16 (padding)
int32_t slcan_generate_tx_event(uint8_t *buf, FDCAN_TxEventFifoTypeDef *tx_event, uint8_t *frame_data)
{
    // Check if Tx reporting is required
    if (((slcan_report_reg >> SLCAN_REPORT_TX) & 1) == 0)
        return 0;

    if (buf == NULL)
        return 0;

    if (tx_event->IdType == FDCAN_STANDARD_ID)
        buf[0] = 'z';
    else
        buf[0] = 'Z';

    FDCAN_RxHeaderTypeDef frame_header;
    frame_header.Identifier = tx_event->Identifier;
    frame_header.IdType = tx_event->IdType;
    frame_header.RxFrameType = tx_event->TxFrameType;
    frame_header.DataLength = tx_event->DataLength;
    frame_header.ErrorStateIndicator = tx_event->ErrorStateIndicator;
    frame_header.BitRateSwitch = tx_event->BitRateSwitch;
    frame_header.FDFormat = tx_event->FDFormat;
    frame_header.RxTimestamp = tx_event->TxTimestamp;
    int32_t len = slcan_generate_frame(&buf[1], &frame_header, frame_data);

    // Return string length
    return len + 1;
}


// Gets milli second timestamp for the current time (2bytes, Resets at 60,000ms)
uint16_t slcan_get_timestamp_ms(void)
{
    static uint16_t slcan_last_timestamp_ms = 0;
    static uint32_t slcan_last_time_ms = 0;

    uint32_t current_time_ms = HAL_GetTick();
    uint32_t time_diff_ms;

    time_diff_ms = (uint32_t)(current_time_ms - slcan_last_time_ms);

    slcan_last_timestamp_ms = (uint16_t)(((uint32_t)slcan_last_timestamp_ms + time_diff_ms % 60000) % 60000);
    slcan_last_time_ms = current_time_ms;

    return slcan_last_timestamp_ms;
}

// Gets micro second timestamp for the tim3 clock (4bytes, Resets at 3600,000,000us)
// The tim3_us does not have to be the current value but supposed to be close to it (like ~1ms).
// The difference between the current tim3 value and tim3_us should never be more than UINT16_MAX / 2 ~ 30ms.
uint32_t slcan_get_timestamp_us_from_tim3(uint16_t tim3_us)
{
    static uint32_t slcan_last_timestamp_us = 0;
    static uint32_t slcan_last_time_ms = 0;
    static uint16_t slcan_last_time_us = 0;

    uint32_t current_time_ms = HAL_GetTick();
    uint16_t current_time_us = tim3_us; // MAX 0xFFFF
    uint32_t time_diff_ms;
    uint64_t time_diff_us;
    uint64_t n_comp;

    time_diff_ms = (uint32_t)(current_time_ms - slcan_last_time_ms);
    time_diff_us = (uint64_t)((uint16_t)(current_time_us - slcan_last_time_us));

    if (time_diff_ms <= 1 && time_diff_us > UINT16_MAX / 2)
    {
        // Assume tim3 was sampled before the last timestamp
        // This can happen when a CAN frame is retrieved after answering a 'Z[CR]'
        // The amount of reversal should be close to the main loop (~100us)
        time_diff_us = (uint64_t)3600000000 - (uint16_t)(slcan_last_time_us - current_time_us);
    }
    else
    {
        // Compensate overflow of micro second counter using milli second counter
        n_comp = ((uint64_t)UINT16_MAX / 2 + time_diff_ms * 1000 - time_diff_us);   // MAX 0x10000, 0xFFFFFFFF * 1000, 0xFFFF
        n_comp = n_comp / ((uint64_t)UINT16_MAX + 1);                               // Number of overflows  MAX 0xFFFF * 1000 + ?
        time_diff_us = time_diff_us + n_comp * ((uint64_t)UINT16_MAX + 1);          // MAX 0xFFFF * 1000 * 0x10000
    }

    slcan_last_timestamp_us = (uint32_t)(((uint64_t)slcan_last_timestamp_us + time_diff_us) % 3600000000);
    slcan_last_time_ms = current_time_ms;
    slcan_last_time_us = current_time_us;

    return slcan_last_timestamp_us;
}

// Set the timestamp mode
void slcan_set_timestamp_mode(enum slcan_timestamp_mode mode)
{
    if (mode < SLCAN_TIMESTAMP_INVALID)
        slcan_timestamp_mode = mode;
    return;
}

// Set the report setting register
void slcan_set_report_mode(uint16_t reg)
{
    slcan_report_reg = reg;
    return;
}

// Report the current timestamp mode
enum slcan_timestamp_mode slcan_get_timestamp_mode(void)
{
    return slcan_timestamp_mode;
}

// Report the current report setting register value
uint16_t slcan_get_report_mode(void)
{
    return slcan_report_reg;
}
