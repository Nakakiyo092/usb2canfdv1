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
const uint8_t slcan_nibble_to_ascii[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

// Private variables
static enum SlcanFilterMode slcan_filter_mode = SLCAN_FILTER_DUAL_MODE;
static uint32_t slcan_filter_code = 0x00000000;
static uint32_t slcan_filter_mask = 0xFFFFFFFF;
static enum SlcanTimestampMode slcan_timestamp_mode = 0;
static uint16_t slcan_report_reg = 1;   // Default: no timestamp, no ESI, no Tx, but with Rx
static uint8_t slcan_status_flags = 0;

// Private methods
static uint16_t slcan_generate_frame(uint8_t *buf, FDCAN_RxHeaderTypeDef *frame_header, uint8_t *frame_data);
static HAL_StatusTypeDef slcan_configure_filter(void);

// Generate a slcan message from a CAN frame
// Returns number of bytes written into buf
//  MIN: 1 (r) + SLCAN_STD_ID_LEN + 2 (DLC & [CR])
//  MAX: SLCAN_MTU - 1 (z/Z) - 16 (padding)
uint16_t slcan_generate_frame(uint8_t *buf, FDCAN_RxHeaderTypeDef *frame_header, uint8_t *frame_data)
{
    // Start building the slcan message string at idx 0 in buf
    uint16_t msg_idx = 0;

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
    for (int8_t j = (int8_t)msg_idx - 1; j >= 1; j--)
    {
        // Add nibble to the buffer
        buf[j] = slcan_nibble_to_ascii[tmp & 0xF];
        tmp = tmp >> 4;
    }

    // Add DLC to the buffer
    buf[msg_idx++] = slcan_nibble_to_ascii[CAN_HAL_DLC_TO_STD_DLC(frame_header->DataLength)];
    uint8_t bytes = can_dlc_to_bytes[CAN_HAL_DLC_TO_STD_DLC(frame_header->DataLength)];
    
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
        // Use current time instead of frame timestamp
        // By this way the complex compensation for TIM3 overflow is not needed
        // and the main loop delya at most ~300us will not greatly affect the timestamp correctness.
        uint16_t timestamp_ms = slcan_get_timestamp_ms();

        buf[msg_idx++] = slcan_nibble_to_ascii[(timestamp_ms >> 12) & 0xF];
        buf[msg_idx++] = slcan_nibble_to_ascii[(timestamp_ms >> 8) & 0xF];
        buf[msg_idx++] = slcan_nibble_to_ascii[(timestamp_ms >> 4) & 0xF];
        buf[msg_idx++] = slcan_nibble_to_ascii[timestamp_ms & 0xF];
    }
    else if (slcan_timestamp_mode == SLCAN_TIMESTAMP_MICRO)
    {
        // If a CAN frame is re-transmitted, the reported timestamp corresponds to the final, successful transmission.
        // See the link for details.
        // https://github.com/Nakakiyo092/usb2canfdv1/issues/48
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
uint16_t slcan_generate_rx_frame(uint8_t *buf, FDCAN_RxHeaderTypeDef *frame_header, uint8_t *frame_data)
{
    // Check if Rx reporting is required
    if (((slcan_report_reg >> SLCAN_REPORT_RX) & 1) == 0)
        return 0;

    if (buf == NULL)
        return 0;

    uint16_t len = slcan_generate_frame(buf, frame_header, frame_data);

    // Return string length
    return len;
}

// Parse an incoming Tx event into an outgoing slcan message
// Returns number of bytes written into buf
//  MIN: 1 (r) + SLCAN_STD_ID_LEN + 2 (DLC & [CR])
//  MAX: SLCAN_MTU - 16 (padding)
uint16_t slcan_generate_tx_event(uint8_t *buf, FDCAN_TxEventFifoTypeDef *tx_event, uint8_t *frame_data)
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
    uint16_t len = slcan_generate_frame(&buf[1], &frame_header, frame_data);

    // Return string length
    return len + 1;
}


// Gets milli second timestamp for the current time (2bytes, Resets at 60,000ms)
// This implementation will breake if the timesatamp is not calculated for more than HAL_GetTick overflow (~49 days, or twice?).
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

// Gets micro second timestamp for the time tim3_us was taken (4bytes, Resets at 3600,000,000us)
// This implementation will breake if the timesatamp is not calculated for more than HAL_GetTick overflow (~49 days, or twice?).
// The calculation is based on the tim3 clock and the ms tick.
// The tim3_us does not have to be the current value but supposed to be close to it (like ~1ms).
// The difference between the current tim3 value and tim3_us should never be more than UINT16_MAX / 2 ~ 30ms.
// This is supported by the fact the observed maximum loop cycle time is about 300us.
// TODO: Implement check for the main loop and raise error if it is too large?
uint32_t slcan_get_timestamp_us_from_tim3(uint16_t tim3_us)
{
    static uint32_t slcan_last_timestamp_us = 0;
    static uint32_t slcan_last_time_ms = 0;
    static uint16_t slcan_last_time_us = 0;

    uint32_t current_time_ms = HAL_GetTick();    // TODO: Check if this tick syncs to TIM3
    uint16_t current_time_us = tim3_us; // MAX 0xFFFF
    uint32_t time_diff_ms;
    uint64_t time_diff_us;
    uint64_t n_comp;

    time_diff_ms = (uint32_t)(current_time_ms - slcan_last_time_ms);
    time_diff_us = (uint64_t)((uint16_t)(current_time_us - slcan_last_time_us));

    if (time_diff_ms <= 3 && time_diff_us > UINT16_MAX / 2)
    {
        // tim3_us was sampled before slcan_last_time_us (i.e. the frame arrived
        // slightly before the previous call).  This can happen when a CAN frame
        // is retrieved after processing a command such as 'Z[CR]'.
        // The reversal is small (bounded by the main-loop cycle, MAX ~300us).
        //
        // Let d = slcan_last_time_us - current_time_us  (positive, small).
        // Actual elapsed time = period - d  where period = 3 600 000 000 us.
        // So: new_timestamp = last_timestamp + (period - d)
        //                   = last_timestamp - d  (mod period)
        // which is computed as: 3600000000 - d
        // and then added to slcan_last_timestamp_us modulo 3600000000 below.
        time_diff_us = (uint64_t)3600000000 - (uint16_t)(slcan_last_time_us - current_time_us);
    }
    else
    {
        // Compensate overflow of micro second counter using milli second counter
        n_comp = ((uint64_t)UINT16_MAX / 2 + time_diff_ms * 1000 - time_diff_us);   // MAX 0x10000, 0xFFFFFFFF * 1000, 0xFFFF
        n_comp = n_comp / ((uint64_t)UINT16_MAX + 1);                               // Number of overflows  MAX 0x10000 * 1000
        time_diff_us = time_diff_us + n_comp * ((uint64_t)UINT16_MAX + 1);          // MAX 0x10000 * 1000 * 0x10000
    }

    slcan_last_timestamp_us = (uint32_t)(((uint64_t)slcan_last_timestamp_us + time_diff_us) % 3600000000);
    slcan_last_time_ms = current_time_ms;
    slcan_last_time_us = current_time_us;

    return slcan_last_timestamp_us;
}

// Setter and getter for the filter settings
HAL_StatusTypeDef slcan_set_filter_mode(enum SlcanFilterMode mode)
{
    if (mode < SLCAN_FILTER_INVALID)
        slcan_filter_mode = mode;
    else
        return HAL_ERROR;

    if (slcan_configure_filter() != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}
HAL_StatusTypeDef slcan_set_filter_code(uint32_t code)
{
    slcan_filter_code = code;

    if (slcan_configure_filter() != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}
HAL_StatusTypeDef slcan_set_filter_mask(uint32_t mask)
{
    slcan_filter_mask = mask;

    if (slcan_configure_filter() != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}
enum SlcanFilterMode slcan_get_filter_mode(void)
{
    return slcan_filter_mode;
}
uint32_t slcan_get_filter_code(void)
{
    return slcan_filter_code;
}
uint32_t slcan_get_filter_mask(void)
{
    return slcan_filter_mask;
}

// Configure filter settings
HAL_StatusTypeDef slcan_configure_filter(void)
{
    FunctionalState state_std = ENABLE;
    FunctionalState state_ext = ENABLE;

    if (slcan_filter_mode != SLCAN_FILTER_SIMPLE_MODE)
    {
        // TODO: Dual filter mode is not implemented yet. Pass all messages.

        // Mask definition, SLCAN: 0 -> Enable, STM32: 1 -> Enable
        if (can_set_filter_std(state_std, 0x000, 0x000) != HAL_OK)
        {
            return HAL_ERROR;
        }
        if (can_set_filter_ext(state_ext, 0x00000000, 0x00000000) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    else
    {
        // Frame type selection by AC0 bit 7 and AM0 bit 7. See the link for details.
        // https://github.com/Nakakiyo092/canable2-fw/issues/66
        if (!(slcan_filter_code >> 31) && !(slcan_filter_mask >> 31))
        {
            state_std = DISABLE;
        }
        else if ((slcan_filter_code >> 31) && !(slcan_filter_mask >> 31))
        {
            state_ext = DISABLE;
        }

        // Mask definition, SLCAN: 0 -> Enable, STM32: 1 -> Enable
        if (can_set_filter_std(state_std, slcan_filter_code & 0x7FF, (~slcan_filter_mask) & 0x7FF) != HAL_OK)
        {
            return HAL_ERROR;
        }
        if (can_set_filter_ext(state_ext, slcan_filter_code & 0x1FFFFFFF, (~slcan_filter_mask) & 0x1FFFFFFF) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }

    return HAL_OK;
}

// Setter and getter for the report mode
void slcan_set_report_mode(uint16_t reg)
{
    slcan_report_reg = reg;
    return;
}
void slcan_set_timestamp_mode(enum SlcanTimestampMode mode)
{
    if (mode < SLCAN_TIMESTAMP_INVALID)
        slcan_timestamp_mode = mode;
    return;
}
enum SlcanTimestampMode slcan_get_timestamp_mode(void)
{
    return slcan_timestamp_mode;
}
uint16_t slcan_get_report_mode(void)
{
    return slcan_report_reg;
}

// Setter and getter for the status flags
void slcan_raise_error(enum SlcanStatusFlag err)
{
    slcan_status_flags |= (uint8_t)(1 << err);
}

void slcan_clear_error(void)
{
    slcan_status_flags = 0;
}

uint8_t slcan_get_status_flags(void)
{
    return slcan_status_flags;
}
