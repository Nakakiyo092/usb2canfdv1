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

#ifndef USB2CANFDV1_GENERATOR_H
#define USB2CANFDV1_GENERATOR_H

#include "stm32g0xx_hal.h"

// Filter mode
enum SlcanFilterMode
{
    SLCAN_FILTER_DUAL_MODE = 0,
    // SLCAN_FILTER_SINGLE_MODE = 1, // Not supported
    SLCAN_FILTER_SIMPLE_MODE = 2,

    SLCAN_FILTER_INVALID
};

// Timestamp mode
enum SlcanTimestampMode
{
    SLCAN_TIMESTAMP_OFF = 0,
    SLCAN_TIMESTAMP_MILLI,
    SLCAN_TIMESTAMP_MICRO,

    SLCAN_TIMESTAMP_INVALID
};

// Startup mode
enum SlcanAutoStartupMode
{
    SLCAN_AUTO_STARTUP_OFF = 0,
    SLCAN_AUTO_STARTUP_NORMAL,
    SLCAN_AUTO_STARTUP_LISTEN,

    SLCAN_AUTO_STARTUP_INVALID
};

// Status flags, value is bit position in the status flags
enum SlcanStatusFlag
{
    SLCAN_STS_CAN_RX_FIFO_FULL = 0, /* Message loss. Not mean the buffer is just full. */
    SLCAN_STS_CAN_TX_FIFO_FULL,     /* Message loss. Not mean the buffer is just full. */
    SLCAN_STS_ERROR_WARNING,
    SLCAN_STS_DATA_OVERRUN,
    SLCAN_STS_BUS_OFF,
    SLCAN_STS_ERROR_PASSIVE,
    SLCAN_STS_ARBITRATION_LOST,     /* Not supported */
    SLCAN_STS_BUS_ERROR
};

// Report flag, value is bit position in the register
enum SlcanReportFlag
{
    SLCAN_REPORT_RX = 0,
    SLCAN_REPORT_TX = 1,
    //SLCAN_REPORT_ERROR,
    //SLCAN_REPORT_OVRLOAD,
    SLCAN_REPORT_ESI = 4
};

// Maximum slcan message length
#define SLCAN_MTU           (1 + 138 + 8 + 1 + 1 + 16)
                            /* z/Z for tx event 1 plus frame 138 plus timestamp 8 plus ESI 1 plus \r 1 plus some padding */
#define SLCAN_STD_ID_LEN    (3)
#define SLCAN_EXT_ID_LEN    (8)

// Public variables
extern const uint8_t gen_nibble_to_ascii[];

// Prototypes
uint16_t gen_generate_rx_frame(uint8_t *buf, FDCAN_RxHeaderTypeDef *frame_header, const uint8_t *frame_data);
uint16_t gen_generate_tx_event(uint8_t *buf, FDCAN_TxEventFifoTypeDef *tx_event, const uint8_t *frame_data);
uint16_t gen_get_timestamp_ms(void);
uint32_t gen_get_timestamp_us_from_tim3(uint16_t tim3_us);

HAL_StatusTypeDef gen_set_filter_mode(enum SlcanFilterMode mode);
HAL_StatusTypeDef gen_set_filter_code(uint32_t code);
HAL_StatusTypeDef gen_set_filter_mask(uint32_t mask);
enum SlcanFilterMode gen_get_filter_mode(void);
uint32_t gen_get_filter_code(void);
uint32_t gen_get_filter_mask(void);

HAL_StatusTypeDef gen_set_timestamp_mode(enum SlcanTimestampMode mode);
void gen_set_report_mode(uint16_t reg);
enum SlcanTimestampMode gen_get_timestamp_mode(void);
uint16_t gen_get_report_mode(void);

void gen_raise_error(enum SlcanStatusFlag err);
void gen_clear_error(void);
uint8_t gen_get_status_flags(void);

#endif // USB2CANFDV1_GENERATOR_H
