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

// Parse incoming slcan commands.

#include <stdio.h>
#include <string.h>
#include "stm32g0xx_hal.h"
#include "buffer.h"
#include "can.h"
#include "led.h"
#include "nvm.h"
#include "parser.h"
#ifdef DEBUG
#include "bootloader.h"
#endif

#define SLCAN_VERSION       "VW1K4"
#define SLCAN_SW_VERSION    "2.1.0"
#define SLCAN_RET_OK    ((uint8_t*)"\r")
#define SLCAN_RET_ERR   ((uint8_t*)"\a")
#define SLCAN_RET_LEN   1U

// Private variables
#ifndef DEBUG
static char *hw_sw_ver = SLCAN_VERSION "\r";
#else
static char *hw_sw_ver = SLCAN_VERSION "-DEBUG\r";
#endif
static char *hw_sw_ver_detail = "v: hardware=\"USB2CANFDV1\", software=\"" SLCAN_SW_VERSION "\", url=\"" "github.com/Nakakiyo092/usb2canfdv1" "\"\r";
static char *can_info = "I3050\r";
static char *can_info_detail = "i: protocol=\"ISO-CANFD\", clock_mhz=80, controller=\"STM32G0B1CB\"\r";

// Private methods
static HAL_StatusTypeDef psr_convert_str_to_number(uint8_t *buf, uint8_t len);
static void psr_parse_str_open(uint8_t *buf, uint8_t len);
static void psr_parse_str_close(uint8_t *buf, uint8_t len);
static void psr_parse_str_set_bitrate(uint8_t *buf, uint8_t len);
static void psr_parse_str_report_mode(uint8_t *buf, uint8_t len);
static void psr_parse_str_filter_mode(uint8_t *buf, uint8_t len);
static void psr_parse_str_filter_code(uint8_t *buf, uint8_t len);
static void psr_parse_str_filter_mask(uint8_t *buf, uint8_t len);
static void psr_parse_str_version(uint8_t *buf, uint8_t len);
static void psr_parse_str_can_info(uint8_t *buf, uint8_t len);
static void psr_parse_str_number(uint8_t *buf, uint8_t len);
static void psr_parse_str_status(uint8_t *buf, uint8_t len);
static void psr_parse_str_auto_startup(uint8_t *buf, uint8_t len);
#ifdef DEBUG
static void psr_parse_str_open_test_mode(uint8_t *buf, uint8_t len);
static void psr_parse_str_extended(uint8_t *buf, uint8_t len);
static void psr_parse_str_debug(uint8_t *buf, uint8_t len);
static void psr_parse_str_stall(uint8_t *buf, uint8_t len);
#endif

// Parse an incoming slcan command from the USB CDC port
void psr_parse_str(uint8_t *buf, uint8_t len)
{
    // msg_marker is intentionally not reset on Close/Open cycles: buf_release_can_until()
    // matches by value, so any non-overlapping starting point is valid.
    static uint8_t msg_marker = 0;

    // Reply OK to a blank command
    if (len == 0)
    {
        buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
        return;
    }

    // Convert an incoming slcan command from ASCII to number (2nd character to end)
    if (psr_convert_str_to_number(buf, len) != HAL_OK)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Handle each incoming command
    switch (buf[0])
    {
    // Open channel
    case 'O':
    case 'L':
        psr_parse_str_open(buf, len);
        return;
    // Close channel
    case 'C':
        psr_parse_str_close(buf, len);
        return;
    // Set bitrate
    case 'S':
    case 's':
    case 'Y':
    case 'y':
        psr_parse_str_set_bitrate(buf, len);
        return;
    // Get version number in standard + detailed style
    case 'V':
    case 'v':
        psr_parse_str_version(buf, len);
        return;
    // Get CAN controller information
    case 'I':
    case 'i':
        psr_parse_str_can_info(buf, len);
        return;
    // Get serial number
    case 'N':
        psr_parse_str_number(buf, len);
        return;
    // Read status flags
    case 'F':
    case 'f':
        psr_parse_str_status(buf, len);
        return;
    // Set report mode
    case 'Z':
    case 'z':
        psr_parse_str_report_mode(buf, len);
        return;
    // Set filter mode
    case 'W':
        psr_parse_str_filter_mode(buf, len);
        return;
    // Set filter code
    case 'M':
        psr_parse_str_filter_code(buf, len);
        return;
    // Set filter mask
    case 'm':
        psr_parse_str_filter_mask(buf, len);
        return;
    // Set auto startup mode
    case 'Q':
        psr_parse_str_auto_startup(buf, len);
        return;
#ifdef DEBUG
    // Open channel in test mode
    case '=':
    case '+':
    case '-':
        psr_parse_str_open_test_mode(buf, len);
        return;
    // Parse extended command
    case '!':
        psr_parse_str_extended(buf, len);
        return;
    // Parse debug command
    case '?':
        psr_parse_str_debug(buf, len);
        return;
    // Parse stall command (stall main loop for N ms; test aid for timing-sensitive tests)
    case '~':
        psr_parse_str_stall(buf, len);
        return;
#endif
    default:
        break;
    }

    // Set default header. All values overridden below as needed.
    FDCAN_TxHeaderTypeDef *frame_header = buf_get_can_head_header();
    uint8_t *frame_data = buf_get_can_head_data();

    if (frame_header == NULL || frame_data == NULL)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    frame_header->TxFrameType = FDCAN_DATA_FRAME;                // default to data frame
    frame_header->FDFormat = FDCAN_CLASSIC_CAN;                  // default to classic frame
    frame_header->IdType = FDCAN_STANDARD_ID;                    // default to standard ID
    frame_header->BitRateSwitch = FDCAN_BRS_OFF;                 // no bitrate switch
    frame_header->ErrorStateIndicator = FDCAN_ESI_ACTIVE;        // error active
    frame_header->TxEventFifoControl = FDCAN_STORE_TX_EVENTS;    // record tx events
    frame_header->MessageMarker = (uint32_t)msg_marker;          // increment only when the frame is queued (not here)

    // Handle each incoming command (transmit)
    switch (buf[0])
    {
    // Transmit remote frame command
    case 'r':
        frame_header->TxFrameType = FDCAN_REMOTE_FRAME;
        break;
    case 'R':
        frame_header->IdType = FDCAN_EXTENDED_ID;
        frame_header->TxFrameType = FDCAN_REMOTE_FRAME;
        break;

    // Transmit data frame command
    case 't':
        break;
    case 'T':
        frame_header->IdType = FDCAN_EXTENDED_ID;
        break;

    // CANFD transmit - no BRS
    case 'd':
        frame_header->FDFormat = FDCAN_FD_CAN;
        break;
    case 'D':
        frame_header->FDFormat = FDCAN_FD_CAN;
        frame_header->IdType = FDCAN_EXTENDED_ID;
        break;

    // CANFD transmit - with BRS
    case 'b':
        frame_header->FDFormat = FDCAN_FD_CAN;
        frame_header->BitRateSwitch = FDCAN_BRS_ON;
        break;
    case 'B':
        frame_header->FDFormat = FDCAN_FD_CAN;
        frame_header->BitRateSwitch = FDCAN_BRS_ON;
        frame_header->IdType = FDCAN_EXTENDED_ID;
        break;

    // Invalid command
    default:
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Start parsing at second byte (skip command byte)
    uint8_t parse_loc = 1;

    // Zero out identifier
    frame_header->Identifier = 0;

    // Default to standard ID
    uint8_t id_len = SLCAN_STD_ID_LEN;

    // Update length if message is extended ID
    if (frame_header->IdType == FDCAN_EXTENDED_ID)
        id_len = SLCAN_EXT_ID_LEN;

    // Iterate through ID bytes
    while (parse_loc <= id_len)
    {
        frame_header->Identifier = frame_header->Identifier << 4;
        frame_header->Identifier += buf[parse_loc++];
    }

    // If CAN ID is too large
    if (frame_header->IdType == FDCAN_STANDARD_ID && 0x7FF < frame_header->Identifier)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }
    else if (frame_header->IdType == FDCAN_EXTENDED_ID && 0x1FFFFFFF < frame_header->Identifier)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Attempt to parse DLC and check sanity
    uint8_t dlc_code_raw = buf[parse_loc++];

    // If dlc is too long
    // DO NOT RESTRICT THE DLC TO 8 !
    // https://github.com/Nakakiyo092/usb2canfdv1/issues/27
    // https://github.com/Nakakiyo092/annus-mirabilis
    if  (0xF < dlc_code_raw)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Set TX frame DLC according to HAL
    frame_header->DataLength = CAN_STD_DLC_TO_HAL_DLC(dlc_code_raw);

    // Calculate number of bytes we expect in the message
    uint8_t bytes_in_msg = can_dlc_to_bytes[CAN_HAL_DLC_TO_STD_DLC(frame_header->DataLength)];

    if (frame_header->TxFrameType == FDCAN_REMOTE_FRAME)
    {
        // Apply maximum data bytes for a remote frame
        bytes_in_msg = 0x0;
    }
    else if (frame_header->FDFormat == FDCAN_CLASSIC_CAN)
    {
        // Apply maximum data bytes for a classical data frame
        if (0x8 < bytes_in_msg) bytes_in_msg = 0x8;
    }

    // Check command length
    // parse_loc is always updated after a byte is parsed
    if (len != parse_loc + 2 * bytes_in_msg)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Parse data
    for (uint8_t i = 0; i < bytes_in_msg; i++)
    {
        frame_data[i] = (buf[parse_loc] << 4) + buf[parse_loc + 1];
        parse_loc += 2;
    }

    // Transmit the message
    if (buf_commit_can_head() != HAL_OK)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // The frame is queued, so increment message marker for the next message.
    // Don't do this before committing the frame to ensure that the marker in the buffer increments one by one.
    msg_marker++;

    // Send ACK
    if (((gen_get_report_mode() >> SLCAN_REPORT_TX) & 1) == 0)
    {
        if (frame_header->IdType == FDCAN_EXTENDED_ID)
            buf_enqueue_cdc((uint8_t *)"Z\r", 2);
        else
            buf_enqueue_cdc((uint8_t *)"z\r", 2);
    }
    else
    {
        buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
    }

    return;
}

// Convert from ASCII to number (2nd character to end)
HAL_StatusTypeDef psr_convert_str_to_number(uint8_t *buf, uint8_t len)
{
    // Convert from ASCII (2nd character to end)
    for (uint8_t i = 1; i < len; i++)
    {
        // Numbers
        if ('0' <= buf[i] && buf[i] <= '9')
            buf[i] = buf[i] - '0';
        // Uppercase letters
        else if ('A' <= buf[i] && buf[i] <= 'F')
            buf[i] = buf[i] - 'A' + 10;
        // Lowercase letters
        else if ('a' <= buf[i] && buf[i] <= 'f')
            buf[i] = buf[i] - 'a' + 10;
        // Invalid character
        else
            return HAL_ERROR;
    }
    return HAL_OK;
}

// Open channel
void psr_parse_str_open(uint8_t *buf, uint8_t len)
{
    // Check command length
    if (len != 1)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Check bus status
    if (can_get_bus_state() != BUS_CLOSED)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Reset variables
    gen_clear_error();

    // Set mode
    if (buf[0] == 'O')
        can_set_mode(FDCAN_MODE_NORMAL);
    else if (buf[0] == 'L')
        can_set_mode(FDCAN_MODE_BUS_MONITORING);

    can_set_auto_retransmit(ENABLE);

    // Open CAN port
    if (can_enable() != HAL_OK)
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
    else
        buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);

    return;
}

// Close channel
void psr_parse_str_close(uint8_t *buf, uint8_t len)
{
    // Check command length
    if (len != 1)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Check bus status
    if (can_get_bus_state() != BUS_OPENED)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Close CAN port
    if (can_disable() == HAL_OK)
        buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
    else
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);

    return;
}

// Set nominal bitrate
void psr_parse_str_set_bitrate(uint8_t *buf, uint8_t len)
{
    if (buf[0] == 'S' || buf[0] == 'Y')
    {
        // Check for valid length
        if (len != 2)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }
        HAL_StatusTypeDef ret;
        if (buf[0] == 'S')
            ret = can_set_nominal_bitrate(buf[1]);
        else
            ret = can_set_data_bitrate(buf[1]);

        if (ret == HAL_OK)
            buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
        else
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
    }
    else if (buf[0] == 's' && len == 5)
    {
        // sxxyy[CR]: LAWICEL-compatible BTR0/BTR1 register mapping
        // xx = BTR0 (buf[1..2]), yy = BTR1 (buf[3..4])
        uint8_t xx = ((uint8_t)buf[1] << 4) + buf[2];
        uint8_t yy = ((uint8_t)buf[3] << 4) + buf[4];

        struct CanBitrateCfg bitrate_cfg;
        bitrate_cfg.prescaler = (uint16_t)(2 * ((xx & 0x3F) + 1));
        bitrate_cfg.time_seg1 = (uint8_t)(5 * (yy & 0x0F) + 9);
        bitrate_cfg.time_seg2 = (uint8_t)(5 * ((yy >> 4) & 0x07) + 5);
        bitrate_cfg.sjw       = (uint8_t)(5 * ((xx >> 6) & 0x03) + 5);

        if (can_set_nominal_bitrate_cfg(bitrate_cfg) == HAL_OK)
            buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
        else
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
    }
    else if (buf[0] == 's' || buf[0] == 'y')
    {
        // Check for valid length
        if (len != 9)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }

        struct CanBitrateCfg bitrate_cfg;
        bitrate_cfg.prescaler = ((uint16_t)buf[1] << 4) + buf[2];
        bitrate_cfg.time_seg1 = ((uint16_t)buf[3] << 4) + buf[4];
        bitrate_cfg.time_seg2 = ((uint16_t)buf[5] << 4) + buf[6];
        bitrate_cfg.sjw = ((uint16_t)buf[7] << 4) + buf[8];

        HAL_StatusTypeDef ret;
        if (buf[0] == 's')
            ret = can_set_nominal_bitrate_cfg(bitrate_cfg);
        else
            ret = can_set_data_bitrate_cfg(bitrate_cfg);

        if (ret == HAL_OK)
            buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
        else
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
    }
    return;
}

// Set report mode
void psr_parse_str_report_mode(uint8_t *buf, uint8_t len)
{
    // Get timestamp
    if (buf[0] == 'Z' && len == 1)
    {
        // Check timestamp mode
        if (gen_get_timestamp_mode() == SLCAN_TIMESTAMP_MILLI)
        {
        	uint8_t* tmsstr = buf_reserve_cdc_dest(SLCAN_MTU);
            if (tmsstr == NULL) return;
        	uint16_t timestamp_ms = gen_get_timestamp_ms_from_tim3(TIM3->CNT);

        	tmsstr[0] = 'Z';
        	tmsstr[1] = gen_nibble_to_ascii[SLCAN_TIMESTAMP_MILLI];
        	tmsstr[2] = gen_nibble_to_ascii[(timestamp_ms >> 12) & 0xF];
        	tmsstr[3] = gen_nibble_to_ascii[(timestamp_ms >> 8) & 0xF];
        	tmsstr[4] = gen_nibble_to_ascii[(timestamp_ms >> 4) & 0xF];
        	tmsstr[5] = gen_nibble_to_ascii[timestamp_ms & 0xF];
        	tmsstr[6] = '\r';
            buf_commit_cdc_dest(7);
        }
        else if (gen_get_timestamp_mode() == SLCAN_TIMESTAMP_MICRO)
        {
        	uint8_t* tmsstr = buf_reserve_cdc_dest(SLCAN_MTU);
            if (tmsstr == NULL) return;
        	uint32_t timestamp_us = gen_get_timestamp_us_from_tim3(TIM3->CNT);

        	tmsstr[0] = 'Z';
        	tmsstr[1] = gen_nibble_to_ascii[SLCAN_TIMESTAMP_MICRO];
        	tmsstr[2] = gen_nibble_to_ascii[(timestamp_us >> 28) & 0xF];
        	tmsstr[3] = gen_nibble_to_ascii[(timestamp_us >> 24) & 0xF];
        	tmsstr[4] = gen_nibble_to_ascii[(timestamp_us >> 20) & 0xF];
        	tmsstr[5] = gen_nibble_to_ascii[(timestamp_us >> 16) & 0xF];
        	tmsstr[6] = gen_nibble_to_ascii[(timestamp_us >> 12) & 0xF];
        	tmsstr[7] = gen_nibble_to_ascii[(timestamp_us >> 8) & 0xF];
        	tmsstr[8] = gen_nibble_to_ascii[(timestamp_us >> 4) & 0xF];
        	tmsstr[9] = gen_nibble_to_ascii[timestamp_us & 0xF];
        	tmsstr[10] = '\r';
            buf_commit_cdc_dest(11);
        }
        else
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        }
        return;
    }

    // Get detailed time
    if (buf[0] == 'z' && len == 1)
    {
        // "z: time_ms=0x0000, time_us=0x00000000, cycle_time_us_ave_max=[0x000, 0x000]\r";

        // Reserve worst-case footprint up front. SLCAN_MTU is well above
        // the ~76 byte actual response, so this is a safe upper bound.
        // On failure NOTHING has been queued yet — the host therefore
        // never sees a truncated fragment of the response. F bit 0 is
        // raised inside buf_reserve_cdc_dest itself when the reservation
        // fails.
        //
        // The entire response is built by writing into the reserved
        // region directly — buf_enqueue_cdc is intentionally NOT used in
        // this handler. Interleaving the two APIs would rely on the
        // (undocumented) guarantee that both target the same head Tx slot
        // even when one of them advances msglen between the other's
        // pointer capture and write; by going reserve-only here we avoid
        // depending on that internal coupling.
        uint8_t* p = buf_reserve_cdc_dest(SLCAN_MTU);
        if (p == NULL) return;

        // Latch TIM3 once so the ms and us fields reflect the same
        // instant. Reading TIM3->CNT twice (the previous behaviour) made
        // the two fields disagree by a handful of timer ticks, which is
        // small in practice but inconsistent with the doc that describes
        // both fields as snapshots of the same "now".
        uint16_t tim3_now = (uint16_t)TIM3->CNT;
        uint16_t timestamp_ms = gen_get_timestamp_ms_from_tim3(tim3_now);
        uint32_t timestamp_us = gen_get_timestamp_us_from_tim3(tim3_now);

        // Read and clear cycle time. The max value accumulates from device boot
        // (or since the last z[CR] query), spanning open and closed periods.
        uint16_t cycle_ave = (uint16_t)(can_get_cycle_ave_time_ns() >= 4095000 ? 4095 : can_get_cycle_ave_time_ns() / 1000);
        uint16_t cycle_max = (uint16_t)(can_get_cycle_max_time_ns() >= 4095000 ? 4095 : can_get_cycle_max_time_ns() / 1000);
        can_clear_cycle_time();

        // Layout (matches the template comment above):
        //   "z: time_ms=0x"               13 B
        //   ms hex                         4 B
        //   ", time_us=0x"                12 B
        //   us hex                         8 B
        //   ", cycle_time_us_ave_max=[0x" 27 B
        //   cycle_ave hex                  3 B
        //   ", 0x"                         4 B
        //   cycle_max hex                  3 B
        //   "]\r"                          2 B
        //   -------------------------------------
        //   total                         76 B
        memcpy(p, "z: time_ms=0x", 13);
        p += 13;
        p[0] = gen_nibble_to_ascii[(timestamp_ms >> 12) & 0xF];
        p[1] = gen_nibble_to_ascii[(timestamp_ms >> 8) & 0xF];
        p[2] = gen_nibble_to_ascii[(timestamp_ms >> 4) & 0xF];
        p[3] = gen_nibble_to_ascii[timestamp_ms & 0xF];
        p += 4;

        memcpy(p, ", time_us=0x", 12);
        p += 12;
        p[0] = gen_nibble_to_ascii[(timestamp_us >> 28) & 0xF];
        p[1] = gen_nibble_to_ascii[(timestamp_us >> 24) & 0xF];
        p[2] = gen_nibble_to_ascii[(timestamp_us >> 20) & 0xF];
        p[3] = gen_nibble_to_ascii[(timestamp_us >> 16) & 0xF];
        p[4] = gen_nibble_to_ascii[(timestamp_us >> 12) & 0xF];
        p[5] = gen_nibble_to_ascii[(timestamp_us >> 8) & 0xF];
        p[6] = gen_nibble_to_ascii[(timestamp_us >> 4) & 0xF];
        p[7] = gen_nibble_to_ascii[timestamp_us & 0xF];
        p += 8;

        memcpy(p, ", cycle_time_us_ave_max=[0x", 27);
        p += 27;
        p[0] = gen_nibble_to_ascii[(cycle_ave >> 8) & 0xF];
        p[1] = gen_nibble_to_ascii[(cycle_ave >> 4) & 0xF];
        p[2] = gen_nibble_to_ascii[cycle_ave & 0xF];
        p += 3;

        memcpy(p, ", 0x", 4);
        p += 4;
        p[0] = gen_nibble_to_ascii[(cycle_max >> 8) & 0xF];
        p[1] = gen_nibble_to_ascii[(cycle_max >> 4) & 0xF];
        p[2] = gen_nibble_to_ascii[cycle_max & 0xF];
        p += 3;

        memcpy(p, "]\r", 2);

        buf_commit_cdc_dest(76);
        return;
    }

    // Set report mode
    if (can_get_bus_state() == BUS_CLOSED)
    {
        if (buf[0] == 'Z')
        {
            // Check for valid command
            if (len != 2 || SLCAN_TIMESTAMP_INVALID <= buf[1])
            {
                buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
                return;
            }

            if (gen_set_timestamp_mode(buf[1]) != HAL_OK)
            {
                buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
                return;
            }

            // 'Z' intentionally resets the full report register to the default value (Rx only,
            // no timestamp, no ESI, no Tx). Use 'z' to set individual report options.
            gen_set_report_mode(1);   // Default: no timestamp, no ESI, no Tx, but with Rx
            buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
            return;
        }
        else if (buf[0] == 'z')
        {
            // Check for valid command
            if (len != 5 || SLCAN_TIMESTAMP_INVALID <= buf[1])
            {
                buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
                return;
            }

            if (gen_set_timestamp_mode(buf[1]) != HAL_OK)
            {
                buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
                return;
            }
            gen_set_report_mode((buf[3] << 4) + buf[4]);
            buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
            return;
        }
    }
    // This command is only active if the CAN channel is closed.
    else
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }
}

// Set filter mode
void psr_parse_str_filter_mode(uint8_t *buf, uint8_t len)
{
    // Set filter mode
    if (can_get_bus_state() == BUS_CLOSED)
    {
        // Check for valid command
        if (len != 2 || SLCAN_FILTER_INVALID <= buf[1])
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }

        // Check if the filter mode is supported
        if (buf[1] != SLCAN_FILTER_DUAL_MODE && buf[1] != SLCAN_FILTER_SIMPLE_MODE)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }

        // Apply filter mode
        if (gen_set_filter_mode(buf[1]) != HAL_OK)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }
        buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
        return;
    }
    // Command can only be sent if the device is initiated but not open.
    else
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }
}

// Set filter code
void psr_parse_str_filter_code(uint8_t *buf, uint8_t len)
{
    // Set filter code
    if (can_get_bus_state() == BUS_CLOSED)
    {
        // Check for valid command
        if (len != 9)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }

        // Apply filter code
        uint32_t code = 0;
        for (uint8_t i = 0; i < 8; i++)
        {
            code = (code << 4) + buf[1 + i];
        }

        if (gen_set_filter_code(code) != HAL_OK)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }
        buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
        return;
    }
    // This command is only active if the CAN channel is initiated and not opened.
    else
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }
}

// Set filter mask
void psr_parse_str_filter_mask(uint8_t *buf, uint8_t len)
{
    // Set filter mask
    if (can_get_bus_state() == BUS_CLOSED)
    {
        // Check for valid command
        if (len != 9)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }

        // Apply filter mask
        uint32_t mask = 0;
        for (uint8_t i = 0; i < 8; i++)
        {
            mask = (mask << 4) + buf[1 + i];
        }

        if (gen_set_filter_mask(mask) != HAL_OK)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }
        buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
        return;
    }
    // This command is only active if the CAN channel is initiated and not opened.
    else
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }
}

// Get version number in standard + detailed style
void psr_parse_str_version(uint8_t *buf, uint8_t len)
{
    // Check command length
    if (len != 1)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    if (buf[0] == 'V')
        buf_enqueue_cdc((uint8_t *)hw_sw_ver, strlen(hw_sw_ver));
    else if (buf[0] == 'v')
        buf_enqueue_cdc((uint8_t *)hw_sw_ver_detail, strlen(hw_sw_ver_detail));

    return;
}

// Get can controller information
void psr_parse_str_can_info(uint8_t *buf, uint8_t len)
{
    // Check command length
    if (len != 1)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    if (buf[0] == 'I')
        buf_enqueue_cdc((uint8_t *)can_info, strlen(can_info));
    else if (buf[0] == 'i')
        buf_enqueue_cdc((uint8_t *)can_info_detail, strlen(can_info_detail));

    return;
}

// Get serial number
void psr_parse_str_number(uint8_t *buf, uint8_t len)
{
    if (len == 1)
    {
        // Report serial number
        uint16_t serial;
        uint8_t* numstr = buf_reserve_cdc_dest(SLCAN_MTU);
        if (numstr == NULL) return;
        if (nvm_get_serial_number(&serial) == HAL_OK)
        {
            snprintf((char*)numstr, SLCAN_MTU - 1, "N%04X\r", serial);
            buf_commit_cdc_dest(6);
        }
        else
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        }
        return;
    }
    else if (len == 5)
    {
        // Set serial number
        uint16_t serial = ((uint16_t)buf[1] << 12) + ((uint16_t)buf[2] << 8) + ((uint16_t)buf[3] << 4) + buf[4];
        if (nvm_update_serial_number(serial) == HAL_OK)
            buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
        else
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }
    else
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }
}

// Read status flags
void psr_parse_str_status(uint8_t *buf, uint8_t len)
{
    // Check command length
    if (len != 1)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Return the status flags
    if (can_get_bus_state() == BUS_OPENED)
    {
        if (buf[0] == 'F')
        {
            uint8_t* stsstr = buf_reserve_cdc_dest(SLCAN_MTU);
            if (stsstr == NULL) return;
            stsstr[0] = 'F';
            stsstr[1] = gen_nibble_to_ascii[gen_get_status_flags() >> 4];
            stsstr[2] = gen_nibble_to_ascii[gen_get_status_flags() & 0xF];
            stsstr[3] = '\r';
            buf_commit_cdc_dest(4);

            // This command also clear the RED Error LED.
            gen_clear_error();
        }
        else if (buf[0] == 'f')
        {
            // "f: node_sts=XXXXXXX, last_err_code=XXXX, err_cnt_tx_rx=[0x00, 0x00], th_bus_load_percent=00\r"

            uint8_t* stsstr = buf_reserve_cdc_dest(SLCAN_MTU);
            if (stsstr == NULL) return;

            struct CanErrorState err = can_get_error_state();

            uint16_t written = (uint16_t)snprintf((char*)stsstr, SLCAN_MTU - 1, "f: node_sts=%s, last_err_code=%s, err_cnt_tx_rx=[0x%02X, 0x%02X], th_bus_load_percent=%02d\r",
                                        (err.bus_off ? "BUS_OFF" : (err.err_pssv ? "ER_PSSV" : "ER_ACTV")),
                                        (err.last_err_code == FDCAN_PROTOCOL_ERROR_NONE ? "NONE" :
                                        (err.last_err_code == FDCAN_PROTOCOL_ERROR_STUFF ? "STUF" :
                                        (err.last_err_code == FDCAN_PROTOCOL_ERROR_FORM ? "FORM" :
                                        (err.last_err_code == FDCAN_PROTOCOL_ERROR_ACK ? "_ACK" :
                                        (err.last_err_code == FDCAN_PROTOCOL_ERROR_BIT1 ? "BIT1" :
                                        (err.last_err_code == FDCAN_PROTOCOL_ERROR_BIT0 ? "BIT0" :
                                        (err.last_err_code == FDCAN_PROTOCOL_ERROR_CRC ? "_CRC" : "SAME"))))))),
                                        (uint8_t)(err.tx_err_cnt),
                                        (uint8_t)(err.rx_err_cnt),
                                        (uint8_t)(can_get_bus_load_ppm() >= 990000 ? 99 : can_get_bus_load_ppm() / 10000));

            buf_commit_cdc_dest(written);
        }
    }
    // This command is only active if the CAN channel is open.
    else
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
    }
    return;
}

// Set auto startup mode
void psr_parse_str_auto_startup(uint8_t *buf, uint8_t len)
{
    // Set auto startup mode
    if (can_get_bus_state() == BUS_OPENED)
    {
        // Check for valid command
        if (len != 2 || SLCAN_AUTO_STARTUP_INVALID <= buf[1])
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }

        if (nvm_update_startup_cfg(buf[1]) != HAL_OK)
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        else
            buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);

        return;
    }
    // Command works only when CAN channel is open.
    else
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }
}

#ifdef DEBUG
// Open channel in test mode
void psr_parse_str_open_test_mode(uint8_t *buf, uint8_t len)
{
    // Check command length
    if (len != 1)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Check bus status
    if (can_get_bus_state() != BUS_CLOSED)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Reset variables
    gen_clear_error();

    // Set mode
    if (buf[0] == '=')
        can_set_mode(FDCAN_MODE_INTERNAL_LOOPBACK);
    else if (buf[0] == '+')
        can_set_mode(FDCAN_MODE_EXTERNAL_LOOPBACK);
    else
        can_set_mode(FDCAN_MODE_NORMAL);

    if (buf[0] == '-')  // No retransmit mode
        can_set_auto_retransmit(DISABLE);
    else
        can_set_auto_retransmit(ENABLE);

    // Open CAN port
    if (can_enable() != HAL_OK)
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
    else
        buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);

    return;
}
#endif

#ifdef DEBUG
// Parse extended command (upgrade mode)
void psr_parse_str_extended(uint8_t *buf, uint8_t len)
{
    if (can_get_bus_state() == BUS_CLOSED)
    {
        // Check for valid command
        if (len != 5)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }

        if (buf[1] == 0xB && buf[2] == 0x0 && buf[3] == 0x0 && buf[4] == 0x7)
        {
            buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
        	bootloader_enter_update_mode();
        }
        else
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);

        return;
    }
    else
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }
}
#endif

#ifdef DEBUG
// Parse debug command
void psr_parse_str_debug(uint8_t *buf, uint8_t len)
{
    // Check for valid command
    if (len != 1)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Debug output - no info
    uint8_t dbgstr[3];
    dbgstr[0] = '?';
    dbgstr[1] = '\r';
    dbgstr[2] = '\0';
    buf_enqueue_cdc(dbgstr, strlen((char *)dbgstr));

    return;
}
#endif

#ifdef DEBUG
// Parse stall command: blocks the main loop for the given number of milliseconds.
// Format: ~<HHHH>[CR]
//   HHHH: 4-digit hex (0..65535 ms)
// Response: [CR] on success, [BELL] on length error.
//
// Behaviour during the stall:
//   - ISRs (CAN, USB) continue to run; only the main loop is blocked.
//   - Data already handed to the USB HAL for transmission is sent out.
//   - Data merely enqueued in the APP-level CDC Tx buffer is NOT pumped to
//     the HAL, because that pump runs in the main loop. Such data, including
//     the ACK for this command itself, becomes visible to the host only
//     after HAL_Delay returns.
//   - Incoming bytes on USB CDC keep filling the APP-level CDC Rx buffer
//     via ISR. If the host sends more than that buffer holds during the
//     stall, the overflow path in buffer.c is exercised.
//   - The ACK is enqueued before HAL_Delay so that any Rx reports produced
//     after the stall are guaranteed to be ordered behind it.
//
// Used by tests that need to reproduce timing-sensitive scenarios:
//   - the us timestamp sentinel that fires when the SOF-to-report delay
//     exceeds the Note 3 design window (~20 ms);
//   - CDC Rx buffer overflow under host-driven flooding.
void psr_parse_str_stall(uint8_t *buf, uint8_t len)
{
    // 1 prefix char + 4 hex digits
    if (len != 5)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // buf[1..4] have already been converted from ASCII hex to nibbles
    // (0..15) by psr_convert_str_to_number; invalid characters were
    // rejected there.
    uint16_t ms = (uint16_t)(
        (buf[1] << 12) |
        (buf[2] <<  8) |
        (buf[3] <<  4) |
         buf[4]
    );

    buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);

    HAL_Delay(ms);

    return;
}
#endif
