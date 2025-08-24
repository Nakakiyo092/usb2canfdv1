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

#ifndef _CAN_H
#define _CAN_H

// Classic CAN / CANFD nominal bitrates
enum can_bitrate_nominal
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
enum can_bitrate_data
{
    CAN_DATA_BITRATE_500K = 0,
    CAN_DATA_BITRATE_1M = 1,
    CAN_DATA_BITRATE_2M = 2,
    CAN_DATA_BITRATE_4M = 4,
    CAN_DATA_BITRATE_5M = 5,

    CAN_DATA_BITRATE_INVALID,
};

// Bus state
enum can_bus_state
{
    BUS_CLOSED,
    BUS_OPENED
};

// Structure for CAN protocol status and error counters
struct can_error_state
{
    uint8_t bus_off;    // Copy of BusOff in FDCAN_ProtocolStatus
    uint8_t err_pssv;   // Copy of ErrorPassive in FDCAN_ProtocolStatus
    uint8_t tec;        // Copy of TxErrorCnt in FDCAN_ErrorCounters
    uint8_t rec;        // Copy of RxErrorCnt in FDCAN_ErrorCounters (rx error active) / 128 (rx error passive)
    uint32_t last_err_code; // Copy of LastErrorCode or DataLastErrorCode in FDCAN_ProtocolStatus
};

// Structure for CAN/FD bitrate configuration
struct can_bitrate_cfg
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
extern uint8_t can_dlc_to_bytes[];

// Prototypes
void can_init(void);
HAL_StatusTypeDef can_enable(void);
HAL_StatusTypeDef can_disable(void);
void can_process(void);

// Bit rate functions
HAL_StatusTypeDef can_set_nominal_bitrate(enum can_bitrate_nominal bitrate);
HAL_StatusTypeDef can_set_data_bitrate(enum can_bitrate_data bitrate);
HAL_StatusTypeDef can_set_nominal_bitrate_cfg(struct can_bitrate_cfg bitrate_cfg);
HAL_StatusTypeDef can_set_data_bitrate_cfg(struct can_bitrate_cfg bitrate_cfg);
struct can_bitrate_cfg can_get_bitrate_cfg(void);
struct can_bitrate_cfg can_get_data_bitrate_cfg(void);

// Filter functions
HAL_StatusTypeDef can_set_filter_std(FunctionalState state, uint32_t code, uint32_t mask);
HAL_StatusTypeDef can_set_filter_ext(FunctionalState state, uint32_t code, uint32_t mask);
FunctionalState can_is_filter_std_enabled(void);
FunctionalState can_is_filter_ext_enabled(void);
uint32_t can_get_filter_std_code(void);
uint32_t can_get_filter_std_mask(void);
uint32_t can_get_filter_ext_code(void);
uint32_t can_get_filter_ext_mask(void);

// CAN mode and status
HAL_StatusTypeDef can_set_mode(uint32_t mode);
HAL_StatusTypeDef can_set_auto_retransmit(FunctionalState state);
enum can_bus_state can_get_bus_state(void);
struct can_error_state can_get_error_state(void);
FunctionalState can_is_tx_enabled(void);
uint32_t can_get_bus_load_ppm(void);

// Cycle time functions
void can_clear_cycle_time(void);
uint32_t can_get_cycle_ave_time_ns(void);
uint32_t can_get_cycle_max_time_ns(void);

FDCAN_HandleTypeDef *can_get_handle(void);

#endif // _CAN_H
