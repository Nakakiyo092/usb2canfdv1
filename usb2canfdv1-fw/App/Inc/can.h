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

#ifndef USB2CANFDV1_CAN_H
#define USB2CANFDV1_CAN_H

#include "stm32g0xx_hal.h"

// Classic CAN / CANFD nominal bitrates
enum CanBitrateNominal
{
    CAN_BITRATE_10K = 0,
    CAN_BITRATE_20K,
    CAN_BITRATE_50K,
    CAN_BITRATE_100K,
    CAN_BITRATE_125K,
    CAN_BITRATE_250K,
    CAN_BITRATE_500K,
    CAN_BITRATE_800K,
    CAN_BITRATE_1000K,

    CAN_BITRATE_INVALID,
};

// CANFD data bitrates
enum CanBitrateData
{
    CAN_DATA_BITRATE_500K = 0,
    CAN_DATA_BITRATE_1M = 1,
    CAN_DATA_BITRATE_2M = 2,
    // value 3 (3 Mbps) is not supported: exactly 3 Mbps cannot be achieved with this clock setup
    CAN_DATA_BITRATE_4M = 4,
    CAN_DATA_BITRATE_5M = 5,

    CAN_DATA_BITRATE_INVALID,
};

// Bus state
enum CanBusState
{
    BUS_CLOSED,
    BUS_OPENED
};

// Structure for CAN protocol status and error counters
struct CanErrorState
{
    uint8_t bus_off;        // Copy of BusOff in FDCAN_ProtocolStatus
    uint8_t err_pssv;       // Copy of ErrorPassive in FDCAN_ProtocolStatus
    uint8_t tx_err_cnt;     // Copy of TxErrorCnt in FDCAN_ErrorCounters
    uint8_t rx_err_cnt;     // Copy of RxErrorCnt in FDCAN_ErrorCounters (rx err active) / 128 (rx err passive)
    uint32_t last_err_code; // Copy of LastErrorCode or DataLastErrorCode in FDCAN_ProtocolStatus
};

// Structure for CAN/FD bitrate configuration
struct CanBitrateCfg
{
    uint16_t prescaler;
    uint8_t time_seg1;
    uint8_t time_seg2;
    uint8_t sjw;
};

#define CAN_STD_DLC_TO_HAL_DLC(val)   ((uint32_t)(val) * FDCAN_DLC_BYTES_1)
#define CAN_HAL_DLC_TO_STD_DLC(val)   ((uint8_t)(((val) / FDCAN_DLC_BYTES_1) & 0xF))

// CANFD parameter
#define CAN_MAX_DATALEN                 64  // CAN maximum data length. Must be 64 for canfd.

// Public variable
#define CAN_DLC_TO_BYTES_SIZE           16  // Number of entries in can_dlc_to_bytes (DLC 0x0..0xF)
extern uint8_t can_dlc_to_bytes[];

// Prototypes
void can_init(void);
HAL_StatusTypeDef can_enable(void);
HAL_StatusTypeDef can_disable(void);
void can_process(void);

// Bit rate functions
HAL_StatusTypeDef can_set_nominal_bitrate(enum CanBitrateNominal bitrate);
HAL_StatusTypeDef can_set_data_bitrate(enum CanBitrateData bitrate);
HAL_StatusTypeDef can_set_nominal_bitrate_cfg(struct CanBitrateCfg bitrate_cfg);
HAL_StatusTypeDef can_set_data_bitrate_cfg(struct CanBitrateCfg bitrate_cfg);
struct CanBitrateCfg can_get_nominal_bitrate_cfg(void);
struct CanBitrateCfg can_get_data_bitrate_cfg(void);

// Filter functions
HAL_StatusTypeDef can_set_filter_std(FunctionalState state, uint32_t code, uint32_t mask);
HAL_StatusTypeDef can_set_filter_ext(FunctionalState state, uint32_t code, uint32_t mask);
FunctionalState can_is_filter_std_enabled(void);
FunctionalState can_is_filter_ext_enabled(void);
uint32_t can_get_filter_std_code(void);
uint32_t can_get_filter_std_mask(void);
uint32_t can_get_filter_ext_code(void);
uint32_t can_get_filter_ext_mask(void);

// Second filter slot functions (FilterIndex=1)
// state=ENABLE: acceptance filter routed to FIFO0; state=DISABLE: resets to pass-all drain (FIFO1)
HAL_StatusTypeDef can_set_filter2_std(FunctionalState state, uint32_t code, uint32_t mask);
HAL_StatusTypeDef can_set_filter2_ext(FunctionalState state, uint32_t code, uint32_t mask);

// CAN mode and status
HAL_StatusTypeDef can_set_mode(uint32_t mode);
HAL_StatusTypeDef can_set_auto_retransmit(FunctionalState state);
enum CanBusState can_get_bus_state(void);
struct CanErrorState can_get_error_state(void);
FunctionalState can_is_tx_enabled(void);
uint32_t can_get_bus_load_ppm(void);

// Cycle time functions
void can_clear_cycle_time(void);
uint32_t can_get_cycle_ave_time_ns(void);
uint32_t can_get_cycle_max_time_ns(void);

FDCAN_HandleTypeDef *can_get_handle(void);

#endif // USB2CANFDV1_CAN_H
