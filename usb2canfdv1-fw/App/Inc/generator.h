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
enum GenFilterMode
{
    GEN_FILTER_DUAL_MODE = 0,
    // GEN_FILTER_SINGLE_MODE = 1, // Not supported
    GEN_FILTER_SIMPLE_MODE = 2,

    GEN_FILTER_INVALID
};

// Timestamp mode
enum GenTimestampMode
{
    GEN_TIMESTAMP_OFF = 0,
    GEN_TIMESTAMP_MILLI,
    GEN_TIMESTAMP_MICRO,

    GEN_TIMESTAMP_INVALID
};

// Startup mode
enum GenAutoStartupMode
{
    GEN_AUTO_STARTUP_OFF = 0,
    GEN_AUTO_STARTUP_NORMAL,
    GEN_AUTO_STARTUP_LISTEN,

    GEN_AUTO_STARTUP_INVALID
};

// Status flags, value is bit position in the status flags
enum GenStatusFlag
{
    GEN_STS_CAN_RX_FIFO_FULL = 0, /* Message loss. Not mean the buffer is just full. */
    GEN_STS_CAN_TX_FIFO_FULL,     /* Message loss. Not mean the buffer is just full. */
    GEN_STS_ERROR_WARNING,
    GEN_STS_DATA_OVERRUN,
    GEN_STS_BUS_OFF,
    GEN_STS_ERROR_PASSIVE,
    GEN_STS_ARBITRATION_LOST,     /* Not supported */
    GEN_STS_BUS_ERROR
};

// Report flag, value is bit position in the register
enum GenReportFlag
{
    GEN_REPORT_RX = 0,
    GEN_REPORT_TX = 1,
    //GEN_REPORT_ERROR,
    //GEN_REPORT_OVRLOAD,
    GEN_REPORT_ESI = 4
};

// Maximum slcan message length
#define GEN_MTU           (1 + 138 + 8 + 1 + 1 + 16)
                            /* z/Z for tx event 1 plus frame 138 plus timestamp 8 plus ESI 1 plus \r 1 plus some padding */
#define GEN_STD_ID_LEN    (3)
#define GEN_EXT_ID_LEN    (8)

// Public variables
extern const uint8_t gen_nibble_to_ascii[];

// Prototypes
uint16_t gen_generate_rx_frame(uint8_t *buf, FDCAN_RxHeaderTypeDef *frame_header, const uint8_t *frame_data);
uint16_t gen_generate_tx_event(uint8_t *buf, FDCAN_TxEventFifoTypeDef *tx_event, const uint8_t *frame_data);
uint16_t gen_get_timestamp_ms(void);
uint32_t gen_get_timestamp_us_from_tim3(uint16_t tim3_us);

HAL_StatusTypeDef gen_set_filter_mode(enum GenFilterMode mode);
HAL_StatusTypeDef gen_set_filter_code(uint32_t code);
HAL_StatusTypeDef gen_set_filter_mask(uint32_t mask);
enum GenFilterMode gen_get_filter_mode(void);
uint32_t gen_get_filter_code(void);
uint32_t gen_get_filter_mask(void);

HAL_StatusTypeDef gen_set_timestamp_mode(enum GenTimestampMode mode);
void gen_set_report_mode(uint16_t reg);
enum GenTimestampMode gen_get_timestamp_mode(void);
uint16_t gen_get_report_mode(void);

void gen_raise_error(enum GenStatusFlag err);
void gen_clear_error(void);
uint8_t gen_get_status_flags(void);

#endif // USB2CANFDV1_GENERATOR_H
