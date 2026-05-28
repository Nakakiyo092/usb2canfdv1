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

// Generate outgoing slcan messages.

#include "stm32g0xx_hal.h"
#include "can.h"
#include "generator.h"

// Public variables
const uint8_t gen_nibble_to_ascii[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'A', 'B', 'C', 'D', 'E', 'F'};

// Private variables
static enum SlcanFilterMode gen_filter_mode = SLCAN_FILTER_DUAL_MODE;
static uint32_t gen_filter_code = 0x00000000;
static uint32_t gen_filter_mask = 0xFFFFFFFF;
static enum SlcanTimestampMode gen_timestamp_mode = 0;
static uint16_t gen_report_reg = 1;   // Default: no timestamp, no ESI, no Tx, but with Rx
static uint8_t gen_status_flags = 0;  // Owned by main loop only; MUST NOT be modified from ISR context.

// Private methods
static uint16_t gen_generate_frame(uint8_t *buf, FDCAN_RxHeaderTypeDef *frame_header, const uint8_t *frame_data);
static HAL_StatusTypeDef gen_configure_filter(void);

// Generate a slcan message from a CAN frame
// Returns number of bytes written into buf
//  MIN: 1 (r) + SLCAN_STD_ID_LEN + 2 (DLC & [CR])
//  MAX: SLCAN_MTU - 1 (z/Z) - 16 (padding)
uint16_t gen_generate_frame(uint8_t *buf, FDCAN_RxHeaderTypeDef *frame_header, const uint8_t *frame_data)
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
        buf[j] = gen_nibble_to_ascii[tmp & 0xF];
        tmp = tmp >> 4;
    }

    // Add DLC to the buffer
    buf[msg_idx++] = gen_nibble_to_ascii[CAN_HAL_DLC_TO_STD_DLC(frame_header->DataLength)];
    uint8_t bytes = can_dlc_to_bytes[CAN_HAL_DLC_TO_STD_DLC(frame_header->DataLength)];

    // Add data bytes
    // Data frame only. No data bytes for a remote frame.
    if (frame_header->RxFrameType != FDCAN_REMOTE_FRAME)
    {
        for (uint8_t j = 0; j < bytes; j++)
        {
            buf[msg_idx++] = gen_nibble_to_ascii[frame_data[j] >> 4];
            buf[msg_idx++] = gen_nibble_to_ascii[frame_data[j] & 0xF];
        }
    }

    // Add time stamp
    if (gen_timestamp_mode == SLCAN_TIMESTAMP_MILLI)
    {
        // Use current time instead of frame timestamp
        // By this way the complex compensation for TIM3 overflow is not needed
        // and the main loop delay at most ~300us will not greatly affect the timestamp correctness.
        uint16_t timestamp_ms = gen_get_timestamp_ms();

        buf[msg_idx++] = gen_nibble_to_ascii[(timestamp_ms >> 12) & 0xF];
        buf[msg_idx++] = gen_nibble_to_ascii[(timestamp_ms >> 8) & 0xF];
        buf[msg_idx++] = gen_nibble_to_ascii[(timestamp_ms >> 4) & 0xF];
        buf[msg_idx++] = gen_nibble_to_ascii[timestamp_ms & 0xF];
    }
    else if (gen_timestamp_mode == SLCAN_TIMESTAMP_MICRO)
    {
        // If a CAN frame is re-transmitted, the reported timestamp corresponds to the final, successful transmission.
        // See the link for details.
        // https://github.com/Nakakiyo092/usb2canfdv1/issues/48
        uint32_t timestamp_us = gen_get_timestamp_us_from_tim3(frame_header->RxTimestamp);

        buf[msg_idx++] = gen_nibble_to_ascii[(timestamp_us >> 28) & 0xF];
        buf[msg_idx++] = gen_nibble_to_ascii[(timestamp_us >> 24) & 0xF];
        buf[msg_idx++] = gen_nibble_to_ascii[(timestamp_us >> 20) & 0xF];
        buf[msg_idx++] = gen_nibble_to_ascii[(timestamp_us >> 16) & 0xF];
        buf[msg_idx++] = gen_nibble_to_ascii[(timestamp_us >> 12) & 0xF];
        buf[msg_idx++] = gen_nibble_to_ascii[(timestamp_us >> 8) & 0xF];
        buf[msg_idx++] = gen_nibble_to_ascii[(timestamp_us >> 4) & 0xF];
        buf[msg_idx++] = gen_nibble_to_ascii[timestamp_us & 0xF];
    }

    // Add error state indicator
    // FD frame only. No ESI for a classical frame.
    if ((gen_report_reg >> SLCAN_REPORT_ESI) & 1)
    {
        if (frame_header->FDFormat == FDCAN_FD_CAN)
        {
            if (frame_header->ErrorStateIndicator == FDCAN_ESI_ACTIVE)
                buf[msg_idx++] = gen_nibble_to_ascii[0];
            else
                buf[msg_idx++] = gen_nibble_to_ascii[1];
        }
    }

    // Add CR for slcan EOL
    buf[msg_idx++] = '\r';

    // Return string length
    return msg_idx;
}

// Generate an outgoing slcan message from an incoming CAN frame
// Returns number of bytes written into buf
//  MIN: 1 (r) + SLCAN_STD_ID_LEN + 2 (DLC & [CR])
//  MAX: SLCAN_MTU - 1 (z/Z) - 16 (padding)
uint16_t gen_generate_rx_frame(uint8_t *buf, FDCAN_RxHeaderTypeDef *frame_header, const uint8_t *frame_data)
{
    // Check if Rx reporting is required
    if (((gen_report_reg >> SLCAN_REPORT_RX) & 1) == 0)
        return 0;

    if (buf == NULL)
        return 0;

    uint16_t len = gen_generate_frame(buf, frame_header, frame_data);

    // Return string length
    return len;
}

// Generate an outgoing slcan message from an incoming Tx event
// Returns number of bytes written into buf
//  MIN: 1 (r) + SLCAN_STD_ID_LEN + 2 (DLC & [CR])
//  MAX: SLCAN_MTU - 16 (padding)
uint16_t gen_generate_tx_event(uint8_t *buf, FDCAN_TxEventFifoTypeDef *tx_event, const uint8_t *frame_data)
{
    // Check if Tx reporting is required
    if (((gen_report_reg >> SLCAN_REPORT_TX) & 1) == 0)
        return 0;

    if (buf == NULL)
        return 0;

    if (tx_event->IdType == FDCAN_STANDARD_ID)
        buf[0] = 'z';
    else
        buf[0] = 'Z';

    // Deliberately reuse gen_generate_frame by mapping Tx event fields into
    // an FDCAN_RxHeaderTypeDef. The two HAL structs share the same field names
    // for the data we need (Identifier, IdType, DataLength, etc.), so the
    // mapping is 1-to-1.  If a future HAL update renames or reorders these
    // fields the compiler will catch the mismatch.
    FDCAN_RxHeaderTypeDef frame_header;
    frame_header.Identifier = tx_event->Identifier;
    frame_header.IdType = tx_event->IdType;
    frame_header.RxFrameType = tx_event->TxFrameType;
    frame_header.DataLength = tx_event->DataLength;
    frame_header.ErrorStateIndicator = tx_event->ErrorStateIndicator;
    frame_header.BitRateSwitch = tx_event->BitRateSwitch;
    frame_header.FDFormat = tx_event->FDFormat;
    frame_header.RxTimestamp = tx_event->TxTimestamp;
    uint16_t len = gen_generate_frame(&buf[1], &frame_header, frame_data);

    // Return string length
    return len + 1;
}


// Gets milli second timestamp for the current time (2bytes, Resets at 60,000ms)
// This implementation will break if the timestamp is not calculated for more than HAL_GetTick overflow (~49.7 days).
uint16_t gen_get_timestamp_ms(void)
{
    static uint16_t gen_last_timestamp_ms = 0;
    static uint32_t gen_last_time_ms = 0;

    uint32_t current_time_ms = HAL_GetTick();
    uint32_t time_diff_ms;

    time_diff_ms = (uint32_t)(current_time_ms - gen_last_time_ms);

    gen_last_timestamp_ms = (uint16_t)(((uint32_t)gen_last_timestamp_ms + time_diff_ms % 60000) % 60000);
    gen_last_time_ms = current_time_ms;

    return gen_last_timestamp_ms;
}

// Gets micro second timestamp for the time tim3_us was taken (4bytes, Resets at 3600,000,000us)
// This implementation will break if the timestamp is not calculated for more than HAL_GetTick overflow (~49.7 days).
// The calculation is based on the combination of tim3 clock and the ms tick.
// The tim3_us does not have to be the current value but supposed to be close to it (like ~1ms).
// The difference between the current tim3 value and tim3_us should never be more than UINT16_MAX us / 2 ~ 30ms.
// This is supported by the fact the observed maximum loop cycle time is about 300us.
// TODO: Implement check for the main loop and raise error if it is too large?
// TODO: The logic in the function uses expensive 64bits calculation. Rewrite this using 32bits tim2.
uint32_t gen_get_timestamp_us_from_tim3(uint16_t tim3_us)
{
    static uint32_t gen_last_timestamp_us = 0;
    static uint32_t gen_last_time_ms = 0;
    static uint16_t gen_last_time_us = 0;

    // Note: HAL_GetTick() and TIM3 share the same clock source
    // but the moment of reading is not aligned. Small sample-time mismatch
    // (bounded by main-loop cycle, ~300us) is handled by the counter
    // mismatch branch below.
    uint32_t current_time_ms = HAL_GetTick();
    uint16_t current_time_us = tim3_us; // MAX 0xFFFF
    uint32_t time_diff_ms;
    uint64_t time_diff_us;
    uint64_t n_comp;

    time_diff_ms = (uint32_t)(current_time_ms - gen_last_time_ms);
    time_diff_us = (uint64_t)((uint16_t)(current_time_us - gen_last_time_us));

    // Counter mismatch (time_diff_ms <= 3 ms and time_diff_us > ~30 ms, this can happen)
    if (time_diff_ms <= 3 && time_diff_us > UINT16_MAX / 2)     // 3 ms >> main-loop cycle * CAN frame buffer size
    {
        // current_time_us was sampled before gen_last_time_us (i.e. the frame arrived
        // slightly before the previous call).  This can happen when a CAN frame
        // is retrieved after processing a command such as 'Z[CR]'.
        // The reversal is small (bounded by the main-loop cycle, MAX ~300us).
        //
        // Apparent elapsed time is negative (-d), where
        //   d = gen_last_time_us - current_time_us  (small positive).
        // Since the running timestamp wraps at period (3600,000,000 us),
        // adding (period - d) is equivalent to subtracting d modulo period.
        // We use that equivalence to keep all arithmetic in unsigned space.
        time_diff_us = (uint64_t)3600000000 - (uint16_t)(gen_last_time_us - current_time_us);
    }
    else
    {
        // Compensate overflow of micro second counter using milli second counter
        n_comp = (uint64_t)time_diff_ms * 1000 + UINT16_MAX / 2 - time_diff_us;     // MAX 0xFFFFFFFF * 1000, 0x10000, 0xFFFF
        n_comp = n_comp / ((uint64_t)UINT16_MAX + 1);                               // Number of overflows  MAX 0x10000 * 1000
        time_diff_us = time_diff_us + n_comp * ((uint64_t)UINT16_MAX + 1);          // MAX 0x10000 * 1000 * 0x10000
    }

    gen_last_timestamp_us = (uint32_t)(((uint64_t)gen_last_timestamp_us + time_diff_us) % 3600000000);
    gen_last_time_ms = current_time_ms;
    gen_last_time_us = current_time_us;

    return gen_last_timestamp_us;
}

// Setter and getter for the filter settings
HAL_StatusTypeDef gen_set_filter_mode(enum SlcanFilterMode mode)
{
    if (mode < SLCAN_FILTER_INVALID)
        gen_filter_mode = mode;
    else
        return HAL_ERROR;

    if (gen_configure_filter() != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}
HAL_StatusTypeDef gen_set_filter_code(uint32_t code)
{
    gen_filter_code = code;

    if (gen_configure_filter() != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}
HAL_StatusTypeDef gen_set_filter_mask(uint32_t mask)
{
    gen_filter_mask = mask;

    if (gen_configure_filter() != HAL_OK)
        return HAL_ERROR;

    return HAL_OK;
}
enum SlcanFilterMode gen_get_filter_mode(void)
{
    return gen_filter_mode;
}
uint32_t gen_get_filter_code(void)
{
    return gen_filter_code;
}
uint32_t gen_get_filter_mask(void)
{
    return gen_filter_mask;
}

// Configure filter settings
HAL_StatusTypeDef gen_configure_filter(void)
{
    FunctionalState state_std = ENABLE;
    FunctionalState state_ext = ENABLE;

    if (gen_filter_mode == SLCAN_FILTER_DUAL_MODE)
    {
        // Extract the four byte fields from the 4-byte Code and Mask
        // (SJA1000 / LAWICEL dual-filter register layout: AC0..AC3 / AM0..AM3)
        uint32_t ac0 = (gen_filter_code >> 24) & 0xFF;
        uint32_t am0 = (gen_filter_mask >> 24) & 0xFF;
        uint32_t ac1 = (gen_filter_code >> 16) & 0xFF;
        uint32_t am1 = (gen_filter_mask >> 16) & 0xFF;
        uint32_t ac2 = (gen_filter_code >>  8) & 0xFF;
        uint32_t am2 = (gen_filter_mask >>  8) & 0xFF;
        uint32_t ac3 = (gen_filter_code >>  0) & 0xFF;
        uint32_t am3 = (gen_filter_mask >>  0) & 0xFF;

        // --- Base (standard) CAN ID filter ---
        // Bit mapping per documentation:
        //   AC0[7:0] -> ID[10:3],  AC1[7:5] -> ID[2:0],  AC1[4:0] = don't-care ('.')
        // Filter 1 uses AC0/AC1; Filter 2 uses AC2/AC3 with the same mapping.
        uint32_t code_std_f1 = (ac0 << 3) | (ac1 >> 5);
        uint32_t mask_std_f1 = (am0 << 3) | (am1 >> 5);   // SLCAN convention: 1 = don't-care

        uint32_t code_std_f2 = (ac2 << 3) | (ac3 >> 5);
        uint32_t mask_std_f2 = (am2 << 3) | (am3 >> 5);   // SLCAN convention: 1 = don't-care

        // --- Extended CAN ID filter ---
        // Bit mapping per documentation:
        //   AC0[7:0] -> ID[28:21],  AC1[7:0] -> ID[20:13],  ID[12:0] = don't-care ('.')
        // Filter 1 uses AC0/AC1; Filter 2 uses AC2/AC3 with the same mapping.
        uint32_t code_ext_f1 = (ac0 << 21) | (ac1 << 13);
        uint32_t mask_ext_f1 = (am0 << 21) | (am1 << 13) | 0x1FFF;   // ID[12:0] always don't-care

        uint32_t code_ext_f2 = (ac2 << 21) | (ac3 << 13);
        uint32_t mask_ext_f2 = (am2 << 21) | (am3 << 13) | 0x1FFF;   // ID[12:0] always don't-care

        // Convert SLCAN mask (1=don't-care) to STM32 mask (1=must-match) and apply.
        // Filter 1 uses FilterIndex=0; Filter 2 uses FilterIndex=1 (routed to FIFO0).
        // A frame is accepted if either filter matches (logical OR).
        if (can_set_filter_std(state_std, code_std_f1, (~mask_std_f1) & 0x7FF) != HAL_OK)
            return HAL_ERROR;
        if (can_set_filter2_std(ENABLE, code_std_f2, (~mask_std_f2) & 0x7FF) != HAL_OK)
            return HAL_ERROR;
        if (can_set_filter_ext(state_ext, code_ext_f1, (~mask_ext_f1) & 0x1FFFFFFF) != HAL_OK)
            return HAL_ERROR;
        if (can_set_filter2_ext(ENABLE, code_ext_f2, (~mask_ext_f2) & 0x1FFFFFFF) != HAL_OK)
            return HAL_ERROR;
    }
    else if (gen_filter_mode == SLCAN_FILTER_SIMPLE_MODE)
    {
        // Reset the second filter slot to pass-all drain mode (FIFO1) when not in dual mode
        if (can_set_filter2_std(DISABLE, 0, 0) != HAL_OK)
            return HAL_ERROR;
        if (can_set_filter2_ext(DISABLE, 0, 0) != HAL_OK)
            return HAL_ERROR;

        // Frame type selection by AC0 bit 7 and AM0 bit 7. See the link for details.
        // https://github.com/Nakakiyo092/canable2-fw/issues/66
        if (!(gen_filter_code >> 31) && !(gen_filter_mask >> 31))
        {
            state_std = DISABLE;
        }
        else if ((gen_filter_code >> 31) && !(gen_filter_mask >> 31))
        {
            state_ext = DISABLE;
        }

        // Mask definition, SLCAN: 0 -> Enable, STM32: 1 -> Enable
        if (can_set_filter_std(state_std, gen_filter_code & 0x7FF, (~gen_filter_mask) & 0x7FF) != HAL_OK)
        {
            return HAL_ERROR;
        }
        if (can_set_filter_ext(state_ext, gen_filter_code & 0x1FFFFFFF, (~gen_filter_mask) & 0x1FFFFFFF) != HAL_OK)
        {
            return HAL_ERROR;
        }
    }
    else
    {
        // Single mode and any other unsupported filter modes
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Setter and getter for the report mode
void gen_set_report_mode(uint16_t reg)
{
    gen_report_reg = reg;
    return;
}
HAL_StatusTypeDef gen_set_timestamp_mode(enum SlcanTimestampMode mode)
{
    if (mode < SLCAN_TIMESTAMP_INVALID)
        gen_timestamp_mode = mode;
    else
        return HAL_ERROR;

    return HAL_OK;
}
enum SlcanTimestampMode gen_get_timestamp_mode(void)
{
    return gen_timestamp_mode;
}
uint16_t gen_get_report_mode(void)
{
    return gen_report_reg;
}

// Setter and getter for the status flags
void gen_raise_error(enum SlcanStatusFlag err)
{
    gen_status_flags |= (uint8_t)(1 << err);
}

void gen_clear_error(void)
{
    gen_status_flags = 0;
}

uint8_t gen_get_status_flags(void)
{
    return gen_status_flags;
}
