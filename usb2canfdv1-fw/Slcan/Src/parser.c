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

// Parse incoming slcan commands.

#include <string.h>
#include "stm32g0xx_hal.h"
#include "bootloader.h"
#include "buffer.h"
#include "can.h"
#include "led.h"
#include "nvm.h"
#include "slcan.h"

// Filter mode
enum slcan_filter_mode
{
    // SLCAN_FILTER_DUAL_MODE = 0,     // Not supported
    // SLCAN_FILTER_SINGLE_MODE,       // Not supported
    SLCAN_FILTER_SIMPLE_ID_MODE = 2,

    SLCAN_FILTER_INVALID
};

#define SLCAN_RET_OK    ((uint8_t*)"\r")
#define SLCAN_RET_ERR   ((uint8_t*)"\a")
#define SLCAN_RET_LEN   (1)

// Private variables
static char *hw_sw_ver = "VW1K2\r";
static char *hw_sw_ver_detail = "v: hardware=\"USB2CANFDV1\", software=\"" "1.2.0" "\", url=\"" "github.com/Nakakiyo092/usb2canfdv1" "\"\r";
static char *can_info = "I3050\r";
static char *can_info_detail = "i: protocol=\"ISO-CANFD\", clock_mhz=60, controller=\"STM32G0B1CB\"\r";
static uint32_t slcan_filter_code = 0x00000000;
static uint32_t slcan_filter_mask = 0xFFFFFFFF;
static uint8_t slcan_status_flags = 0;

// Private methods
static HAL_StatusTypeDef slcan_convert_str_to_number(uint8_t *buf, uint8_t len);
static void slcan_parse_str_open(uint8_t *buf, uint8_t len);
static void slcan_parse_str_loop(uint8_t *buf, uint8_t len);
static void slcan_parse_str_close(uint8_t *buf, uint8_t len);
static void slcan_parse_str_set_bitrate(uint8_t *buf, uint8_t len);
static void slcan_parse_str_report_mode(uint8_t *buf, uint8_t len);
static void slcan_parse_str_filter_mode(uint8_t *buf, uint8_t len);
static void slcan_parse_str_filter_code(uint8_t *buf, uint8_t len);
static void slcan_parse_str_filter_mask(uint8_t *buf, uint8_t len);
static void slcan_parse_str_set_auto_retransmit(uint8_t *buf, uint8_t len);
static void slcan_parse_str_version(uint8_t *buf, uint8_t len);
static void slcan_parse_str_can_info(uint8_t *buf, uint8_t len);
static void slcan_parse_str_number(uint8_t *buf, uint8_t len);
static void slcan_parse_str_status(uint8_t *buf, uint8_t len);
static void slcan_parse_str_auto_startup(uint8_t *buf, uint8_t len);

// Parse an incoming slcan command from the USB CDC port
void slcan_parse_str(uint8_t *buf, uint8_t len)
{
    // Reply OK to a blank command
    if (len == 0)
    {
        buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
        return;
    }

    // Convert an incoming slcan command from ASCII to number (2nd character to end)
    if (slcan_convert_str_to_number(buf, len) != HAL_OK)
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
        slcan_parse_str_open(buf, len);
        return;
    // Open channel (loopback mode)
    case '=':
    case '+':
        slcan_parse_str_loop(buf, len);
        return;
    // Close channel
    case 'C':
        slcan_parse_str_close(buf, len);
        return;
    // Set bitrate
    case 'S':
    case 's':
    case 'Y':
    case 'y':
        slcan_parse_str_set_bitrate(buf, len);
        return;
    // Get version number in standard + detailed style
    case 'V':
    case 'v':
        slcan_parse_str_version(buf, len);
        return;
    // Get CAN controller information
    case 'I':
    case 'i':
        slcan_parse_str_can_info(buf, len);
        return;
    // Get serial number
    case 'N':
        slcan_parse_str_number(buf, len);
        return;
    // Read status flags
    case 'F':
    case 'f':
        slcan_parse_str_status(buf, len);
        return;
    // Set report mode
    case 'Z':
    case 'z':
        slcan_parse_str_report_mode(buf, len);
        return;
    // Set filter mode
    case 'W':
        slcan_parse_str_filter_mode(buf, len);
        return;
    // Set filter code
    case 'M':
        slcan_parse_str_filter_code(buf, len);
        return;
    // Set filter mask
    case 'm':
        slcan_parse_str_filter_mask(buf, len);
        return;
    // Set auto retransmit
    case '-':
        slcan_parse_str_set_auto_retransmit(buf, len);
        return;
    // Set auto startup mode
    case 'Q':
        slcan_parse_str_auto_startup(buf, len);
        return;
    // Enter firmware upgrade mode
    case 'X':
    	bootloader_enter_update_mode();
        break;
    // Debug function
    case '?':
    {
        uint8_t cycle_ave = (uint8_t)(can_get_cycle_ave_time_ns() >= 255000 ? 255 : can_get_cycle_ave_time_ns() / 1000);
        uint8_t cycle_max = (uint8_t)(can_get_cycle_max_time_ns() >= 255000 ? 255 : can_get_cycle_max_time_ns() / 1000);
        // "?XX-XX\r"
        uint8_t dbgstr[7];
        dbgstr[0] = '?';
        dbgstr[1] = slcan_nibble_to_ascii[cycle_ave >> 4];
        dbgstr[2] = slcan_nibble_to_ascii[cycle_ave & 0xF];
        dbgstr[3] = '-';
        dbgstr[4] = slcan_nibble_to_ascii[cycle_max >> 4];
        dbgstr[5] = slcan_nibble_to_ascii[cycle_max & 0xF];
        dbgstr[6] = '\r';
        buf_enqueue_cdc(dbgstr, strlen((char *)dbgstr));
        can_clear_cycle_time();
        return;
    }
    default:
        break;
    }

    // Set default header. All values overridden below as needed.
    FDCAN_TxHeaderTypeDef *frame_header = buf_get_can_dest_header();
    uint8_t *frame_data = buf_get_can_dest_data();

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
    frame_header->MessageMarker = 0;                             // not used

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

    // If dlc is too long for a remote frame
    if  (frame_header->TxFrameType == FDCAN_REMOTE_FRAME)
    {
        if  (0xF < dlc_code_raw)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }
    }
    // If dlc is too long for a classical frame
    else if (frame_header->FDFormat == FDCAN_CLASSIC_CAN)
    {
        if (0x8 < dlc_code_raw)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }
    }
    // If dlc is too long for a FD frame
    else
    {
        if (0xF < dlc_code_raw)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }
    }

    // Set TX frame DLC according to HAL
    frame_header->DataLength = CAN_STD_DLC_TO_HAL_DLC(dlc_code_raw);

    // Calculate number of bytes we expect in the message
    int8_t bytes_in_msg = can_dlc_to_bytes[CAN_HAL_DLC_TO_STD_DLC(frame_header->DataLength)];

    if (bytes_in_msg < 0)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Parse data
    // Data frame only. No data bytes for a remote frame.
    if (frame_header->TxFrameType != FDCAN_REMOTE_FRAME)
    {
        // TODO: Guard against walking off the end of the string!
        for (uint8_t i = 0; i < bytes_in_msg; i++)
        {
            frame_data[i] = (buf[parse_loc] << 4) + buf[parse_loc + 1];
            parse_loc += 2;
        }
    }

    // Check command length
    // parse_loc is always updated after a byte is parsed
    if (len != parse_loc)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Transmit the message
    if (buf_comit_can_dest() != HAL_OK)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    // Send ACK
    if (((slcan_report_reg >> SLCAN_REPORT_TX) & 1) == 0)
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
HAL_StatusTypeDef slcan_convert_str_to_number(uint8_t *buf, uint8_t len)
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
void slcan_parse_str_open(uint8_t *buf, uint8_t len)
{
    // Check command length
    if (len != 1)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    slcan_status_flags = 0;
    can_clear_cycle_time();

    if (buf[0] == 'O')
    {
        // Mode default
        if (can_set_mode(FDCAN_MODE_NORMAL) != HAL_OK)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }
    }
    else if (buf[0] == 'L')
    {
        // Mode silent
        if (can_set_mode(FDCAN_MODE_BUS_MONITORING) != HAL_OK)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }
    }
    // Open CAN port
    if (can_enable() != HAL_OK)
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
    else
        buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);

    return;
}

// Open channel (loopback mode)
void slcan_parse_str_loop(uint8_t *buf, uint8_t len)
{
    // Check command length
    if (len != 1)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }

    slcan_status_flags = 0;
    can_clear_cycle_time();

    // Mode loopback
    if (buf[0] == '+')
    {
        if (can_set_mode(FDCAN_MODE_EXTERNAL_LOOPBACK) != HAL_OK)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }
    }
    else
    {
        if (can_set_mode(FDCAN_MODE_INTERNAL_LOOPBACK) != HAL_OK)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }
    }
    // Open CAN port
    if (can_enable() != HAL_OK)
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
    else
        buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);

    return;
}

// Close channel
void slcan_parse_str_close(uint8_t *buf, uint8_t len)
{
    // Check command length
    if (len != 1)
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }
    // Close CAN port
    if (can_disable() == HAL_OK)
        buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
    else
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);

    slcan_status_flags = 0;
    can_clear_cycle_time();

    return;
}

// Set nominal bitrate
void slcan_parse_str_set_bitrate(uint8_t *buf, uint8_t len)
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
    else if (buf[0] == 's' || buf[0] == 'y')
    {
        // Check for valid length
        if (len != 9)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }

        struct can_bitrate_cfg bitrate_cfg;
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
void slcan_parse_str_report_mode(uint8_t *buf, uint8_t len)
{
    // Get timestamp
    if (buf[0] == 'Z' && len == 1)
    {
        // Check timestamp mode
        if (slcan_timestamp_mode == SLCAN_TIMESTAMP_MILLI)
        {
        	uint8_t* tmsstr = buf_get_cdc_dest(SLCAN_MTU);
        	uint16_t timestamp_ms = slcan_get_timestamp_ms();

        	tmsstr[0] = 'Z';
        	tmsstr[1] = slcan_nibble_to_ascii[(timestamp_ms >> 12) & 0xF];
        	tmsstr[2] = slcan_nibble_to_ascii[(timestamp_ms >> 8) & 0xF];
        	tmsstr[3] = slcan_nibble_to_ascii[(timestamp_ms >> 4) & 0xF];
        	tmsstr[4] = slcan_nibble_to_ascii[timestamp_ms & 0xF];
        	tmsstr[5] = '\r';
            buf_comit_cdc_dest(6);
        }
        else if (slcan_timestamp_mode == SLCAN_TIMESTAMP_MICRO)
        {
        	uint8_t* tmsstr = buf_get_cdc_dest(SLCAN_MTU);
        	uint32_t timestamp_us = slcan_get_timestamp_us_from_tim3(TIM3->CNT);

        	tmsstr[0] = 'Z';
        	tmsstr[1] = slcan_nibble_to_ascii[(timestamp_us >> 28) & 0xF];
        	tmsstr[2] = slcan_nibble_to_ascii[(timestamp_us >> 24) & 0xF];
        	tmsstr[3] = slcan_nibble_to_ascii[(timestamp_us >> 20) & 0xF];
        	tmsstr[4] = slcan_nibble_to_ascii[(timestamp_us >> 16) & 0xF];
        	tmsstr[5] = slcan_nibble_to_ascii[(timestamp_us >> 12) & 0xF];
        	tmsstr[6] = slcan_nibble_to_ascii[(timestamp_us >> 8) & 0xF];
        	tmsstr[7] = slcan_nibble_to_ascii[(timestamp_us >> 4) & 0xF];
        	tmsstr[8] = slcan_nibble_to_ascii[timestamp_us & 0xF];
        	tmsstr[9] = '\r';
            buf_comit_cdc_dest(10);
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
        // "z: time_ms=0x0000, time_us=0x00000000, cycle_time_us_ave_max=[0x00, 0x00]\r";

        uint8_t* timstr;

        buf_enqueue_cdc((uint8_t *)"z: time_ms=0x", 13);

        uint16_t timestamp_ms = slcan_get_timestamp_ms();
        uint32_t timestamp_us = slcan_get_timestamp_us_from_tim3(TIM3->CNT);

        timstr = buf_get_cdc_dest(SLCAN_MTU);
        timstr[0] = slcan_nibble_to_ascii[(timestamp_ms >> 12) & 0xF];
        timstr[1] = slcan_nibble_to_ascii[(timestamp_ms >> 8) & 0xF];
        timstr[2] = slcan_nibble_to_ascii[(timestamp_ms >> 4) & 0xF];
        timstr[3] = slcan_nibble_to_ascii[timestamp_ms & 0xF];
        buf_comit_cdc_dest(4);

        buf_enqueue_cdc((uint8_t *)", time_us=0x", 12);

        timstr = buf_get_cdc_dest(SLCAN_MTU);
        timstr[0] = slcan_nibble_to_ascii[(timestamp_us >> 28) & 0xF];
        timstr[1] = slcan_nibble_to_ascii[(timestamp_us >> 24) & 0xF];
        timstr[2] = slcan_nibble_to_ascii[(timestamp_us >> 20) & 0xF];
        timstr[3] = slcan_nibble_to_ascii[(timestamp_us >> 16) & 0xF];
        timstr[4] = slcan_nibble_to_ascii[(timestamp_us >> 12) & 0xF];
        timstr[5] = slcan_nibble_to_ascii[(timestamp_us >> 8) & 0xF];
        timstr[6] = slcan_nibble_to_ascii[(timestamp_us >> 4) & 0xF];
        timstr[7] = slcan_nibble_to_ascii[timestamp_us & 0xF];
        buf_comit_cdc_dest(8);

        buf_enqueue_cdc((uint8_t *)", cycle_time_us_ave_max=[0x", 27);

        if (can_get_bus_state() == BUS_CLOSED)
        {
        	// Cycle time calculation is disabled
            buf_enqueue_cdc((uint8_t *)"**, 0x**", 8);
        }
        else
        {
			uint8_t cycle_ave = (uint8_t)(can_get_cycle_ave_time_ns() >= 255000 ? 255 : can_get_cycle_ave_time_ns() / 1000);
			uint8_t cycle_max = (uint8_t)(can_get_cycle_max_time_ns() >= 255000 ? 255 : can_get_cycle_max_time_ns() / 1000);
			can_clear_cycle_time();

			timstr = buf_get_cdc_dest(SLCAN_MTU);
	        timstr[0] = slcan_nibble_to_ascii[cycle_ave >> 4];
	        timstr[1] = slcan_nibble_to_ascii[cycle_ave & 0xF];
	        buf_comit_cdc_dest(2);

	        buf_enqueue_cdc((uint8_t *)", 0x", 4);

	        timstr = buf_get_cdc_dest(SLCAN_MTU);
	        timstr[0] = slcan_nibble_to_ascii[cycle_max >> 4];
	        timstr[1] = slcan_nibble_to_ascii[cycle_max & 0xF];
	        buf_comit_cdc_dest(2);
        }

        buf_enqueue_cdc((uint8_t *)"]\r", 2);

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

            slcan_timestamp_mode = buf[1];
            slcan_report_reg = 1;   // Default: no timestamp, no ESI, no Tx, but with Rx
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

            slcan_timestamp_mode = buf[1];
            slcan_report_reg = (buf[3] << 4) + buf[4];
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
void slcan_parse_str_filter_mode(uint8_t *buf, uint8_t len)
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
        if (buf[1] != SLCAN_FILTER_SIMPLE_ID_MODE)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }

        buf_enqueue_cdc(SLCAN_RET_OK, SLCAN_RET_LEN);
        return;
    }
    // Command can only be sent if CAN232 is initiated but not open.
    else
    {
        buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
        return;
    }
}


// Set filter code
void slcan_parse_str_filter_code(uint8_t *buf, uint8_t len)
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

        slcan_filter_code = 0;
        for (uint8_t i = 0; i < 8; i++)
        {
            slcan_filter_code = (slcan_filter_code << 4) + buf[1 + i];
        }
        
        FunctionalState state_std = ENABLE;
        FunctionalState state_ext = ENABLE;
        if ((slcan_filter_code >> 31) && !(slcan_filter_mask >> 31))
        {
            state_ext = DISABLE;
        }
        else if (!(slcan_filter_code >> 31) && !(slcan_filter_mask >> 31))
        {
            state_std = DISABLE;
        }

        // Mask definition, SLCAN: 0 -> Enable, STM32: 1 -> Enable
        if (can_set_filter_std(state_std, slcan_filter_code & 0x7FF, (~slcan_filter_mask) & 0x7FF) != HAL_OK)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }
        if (can_set_filter_ext(state_ext, slcan_filter_code & 0x1FFFFFFF, (~slcan_filter_mask) & 0x1FFFFFFF) != HAL_OK)
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
void slcan_parse_str_filter_mask(uint8_t *buf, uint8_t len)
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

        slcan_filter_mask = 0;
        for (uint8_t i = 0; i < 8; i++)
        {
            slcan_filter_mask = (slcan_filter_mask << 4) + buf[1 + i];
        }

        FunctionalState state_std = ENABLE;
        FunctionalState state_ext = ENABLE;
        if ((slcan_filter_code >> 31) && !(slcan_filter_mask >> 31))
        {
            state_ext = DISABLE;
        }
        else if (!(slcan_filter_code >> 31) && !(slcan_filter_mask >> 31))
        {
            state_std = DISABLE;
        }

        // Mask definition, SLCAN: 0 -> Enable, STM32: 1 -> Enable
        if (can_set_filter_std(state_std, slcan_filter_code & 0x7FF, (~slcan_filter_mask) & 0x7FF) != HAL_OK)
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }
        if (can_set_filter_ext(state_ext, slcan_filter_code & 0x1FFFFFFF, (~slcan_filter_mask) & 0x1FFFFFFF) != HAL_OK)
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

// Set auto retransmit
static void slcan_parse_str_set_auto_retransmit(uint8_t *buf, uint8_t len)
{
    // Set auto retransmit
    if (can_get_bus_state() == BUS_CLOSED)
    {
        // Check for valid command
        if (len != 2 || 2 <= buf[1])
        {
            buf_enqueue_cdc(SLCAN_RET_ERR, SLCAN_RET_LEN);
            return;
        }

        // Apply the state
        if (can_set_auto_retransmit((buf[1] == 0) ? DISABLE : ENABLE) != HAL_OK)
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

// Get version number in standard + detailed style
void slcan_parse_str_version(uint8_t *buf, uint8_t len)
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
void slcan_parse_str_can_info(uint8_t *buf, uint8_t len)
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
void slcan_parse_str_number(uint8_t *buf, uint8_t len)
{
    if (len == 1)
    {
        // Report serial number
        uint16_t serial;
        char* numstr = (char*)buf_get_cdc_dest(SLCAN_MTU);
        if (nvm_get_serial_number(&serial) == HAL_OK)
        {
            snprintf(numstr, SLCAN_MTU - 1, "N%04X\r", serial);
            buf_comit_cdc_dest(6);
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
void slcan_parse_str_status(uint8_t *buf, uint8_t len)
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
            uint8_t* stsstr = buf_get_cdc_dest(SLCAN_MTU);
            stsstr[0] = 'F';
            stsstr[1] = slcan_nibble_to_ascii[slcan_status_flags >> 4];
            stsstr[2] = slcan_nibble_to_ascii[slcan_status_flags & 0xF];
            stsstr[3] = '\r';
            buf_comit_cdc_dest(4);

            // This command also clear the RED Error LED.
            slcan_status_flags = 0;
        }
        else if (buf[0] == 'f')
        {
            char* stsstr = (char*)buf_get_cdc_dest(SLCAN_MTU);

            struct can_error_state err = can_get_error_state();

            snprintf(stsstr, SLCAN_MTU - 1, "f: node_sts=%s, last_err_code=%s, err_cnt_tx_rx=[0x%02X, 0x%02X], est_bus_load_percent=%02d\r",
                                        (err.bus_off ? "BUS_OFF" : (err.err_pssv ? "ER_PSSV" : "ER_ACTV")),
                                        (err.last_err_code == FDCAN_PROTOCOL_ERROR_NONE ? "NONE" : 
                                        (err.last_err_code == FDCAN_PROTOCOL_ERROR_STUFF ? "STUF" : 
                                        (err.last_err_code == FDCAN_PROTOCOL_ERROR_FORM ? "FORM" : 
                                        (err.last_err_code == FDCAN_PROTOCOL_ERROR_ACK ? "_ACK" : 
                                        (err.last_err_code == FDCAN_PROTOCOL_ERROR_BIT1 ? "BIT1" : 
                                        (err.last_err_code == FDCAN_PROTOCOL_ERROR_BIT0 ? "BIT0" : 
                                        (err.last_err_code == FDCAN_PROTOCOL_ERROR_CRC ? "_CRC" : "SAME"))))))),
                                        (uint8_t)(err.tec),
                                        (uint8_t)(err.rec),
                                        (uint8_t)(can_get_bus_load_ppm() >= 990000 ? 99 : (can_get_bus_load_ppm() / 50000) * 5));

            buf_comit_cdc_dest(93);
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
void slcan_parse_str_auto_startup(uint8_t *buf, uint8_t len)
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

void slcan_raise_error(enum slcan_status_flag err)
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
