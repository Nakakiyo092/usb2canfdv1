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

// Initializes and provides methods to interact with the FDCAN peripheral

#include "stm32g0xx_hal.h"
#include "usbd_cdc_if.h"
#include "fdcan.h"
#include "buffer.h"
#include "can.h"
#include "led.h"
#include "slcan.h"

// Bit number for each frame type with zero data length
#define CAN_BIT_NBR_WOD_CBFF            47
#define CAN_BIT_NBR_WOD_CEFF            67
#define CAN_BIT_NBR_WOD_FBFF_ARBIT      30
#define CAN_BIT_NBR_WOD_FEFF_ARBIT      49
#define CAN_BIT_NBR_WOD_FXFF_DATA_S     26
#define CAN_BIT_NBR_WOD_FXFF_DATA_L     30

// Parameter to calculate bus load
#define CAN_TIME_CNT_MAX_REWIND         360         /* Max cycle ~120ms X 3 times margin. should be < MIN_BIT_NBR * 9 */
#define CAN_BUS_LOAD_BUILDUP_PPM        1125000     /* Compensate stuff bits and round down in laod calc */

// Public variable
uint8_t can_dlc_to_bytes[] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

// Private variables
static FDCAN_FilterTypeDef can_std_filter;
static FDCAN_FilterTypeDef can_ext_filter;
static FDCAN_FilterTypeDef can_std_pass_all;
static FDCAN_FilterTypeDef can_ext_pass_all;
static enum can_bus_state can_bus_state;
static struct can_error_state can_error_state = {0};
static uint32_t can_mode = FDCAN_MODE_NORMAL;
static FunctionalState can_auto_retransmit = ENABLE;
static struct can_bitrate_cfg can_bit_cfg_nominal, can_bit_cfg_data = {0};

static uint32_t can_cycle_max_time_ns = 0;
static uint32_t can_cycle_ave_time_ns = 0;
static uint32_t can_bit_time_ns = 0;
static uint32_t can_bus_load_ppm = 0;

// Private methods
static void can_update_bit_time_ns(void);
static uint16_t can_get_bit_number_in_rx_frame(FDCAN_RxHeaderTypeDef *pRxHeader);
static uint16_t can_get_bit_number_in_tx_event(FDCAN_TxEventFifoTypeDef *pRxHeader);

// Initialize CAN peripheral settings, but don't actually start the peripheral
void can_init(void)
{
    // Initialize default CAN filter configuration
    can_std_filter.IdType = FDCAN_STANDARD_ID;
    can_std_filter.FilterIndex = 0;
    can_std_filter.FilterType = FDCAN_FILTER_MASK;
    can_std_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    can_std_filter.FilterID1 = 0x7FF;
    can_std_filter.FilterID2 = 0x000;

    can_ext_filter.IdType = FDCAN_EXTENDED_ID;
    can_ext_filter.FilterIndex = 0;
    can_ext_filter.FilterType = FDCAN_FILTER_MASK;
    can_ext_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    can_ext_filter.FilterID1 = 0x1FFFFFFF;
    can_ext_filter.FilterID2 = 0x00000000;

    can_std_pass_all.IdType = FDCAN_STANDARD_ID;
    can_std_pass_all.FilterIndex = 1;
    can_std_pass_all.FilterType = FDCAN_FILTER_MASK;
    can_std_pass_all.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    can_std_pass_all.FilterID1 = 0x7FF;
    can_std_pass_all.FilterID2 = 0x000;

    can_ext_pass_all.IdType = FDCAN_EXTENDED_ID;
    can_ext_pass_all.FilterIndex = 1;
    can_ext_pass_all.FilterType = FDCAN_FILTER_MASK;
    can_ext_pass_all.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
    can_ext_pass_all.FilterID1 = 0x1FFFFFFF;
    can_ext_pass_all.FilterID2 = 0x00000000;

    // default to 125 kbit/s & 2Mbit/s
    can_set_nominal_bitrate(CAN_BITRATE_125K);
    can_set_data_bitrate(CAN_DATA_BITRATE_2M);
    hfdcan1.Instance = FDCAN1;
    can_bus_state = BUS_CLOSED;
}

// Start the CAN peripheral and open the channel
HAL_StatusTypeDef can_enable(void)
{
    if (can_bus_state == BUS_CLOSED)
    {
        // Reset error counter etc.
        __HAL_RCC_FDCAN_FORCE_RESET();
        __HAL_RCC_FDCAN_RELEASE_RESET();

        hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
        hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;

        hfdcan1.Init.Mode = can_mode;
        hfdcan1.Init.AutoRetransmission = can_auto_retransmit;
        hfdcan1.Init.TransmitPause = DISABLE;
        hfdcan1.Init.ProtocolException = ENABLE;

        hfdcan1.Init.NominalPrescaler = can_bit_cfg_nominal.prescaler;
        hfdcan1.Init.NominalSyncJumpWidth = can_bit_cfg_nominal.sjw;
        hfdcan1.Init.NominalTimeSeg1 = can_bit_cfg_nominal.time_seg1;
        hfdcan1.Init.NominalTimeSeg2 = can_bit_cfg_nominal.time_seg2;

        // FD only
        hfdcan1.Init.DataPrescaler = can_bit_cfg_data.prescaler;
        hfdcan1.Init.DataSyncJumpWidth = can_bit_cfg_data.sjw;
        hfdcan1.Init.DataTimeSeg1 = can_bit_cfg_data.time_seg1;
        hfdcan1.Init.DataTimeSeg2 = can_bit_cfg_data.time_seg2;

        hfdcan1.Init.StdFiltersNbr = 2;
        hfdcan1.Init.ExtFiltersNbr = 2;
        hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;

        if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK) return HAL_ERROR;

        // Setup Tx delay compensation
        uint32_t offset = can_bit_cfg_data.prescaler * can_bit_cfg_data.time_seg1;
        if (offset <= 0x1E)
        {
            if (HAL_FDCAN_ConfigTxDelayCompensation(&hfdcan1, offset, 0) != HAL_OK) return HAL_ERROR;
            if (HAL_FDCAN_EnableTxDelayCompensation(&hfdcan1) != HAL_OK) return HAL_ERROR;
        }
        else
        {
            // The offset value 0x1E corresponds to bitrate 1Mbps @ 50% sampling point or 2Mbps @ 100% sampling point.
            // Turn off at 1Mbps and Turn on at 2Mbps
            HAL_FDCAN_DisableTxDelayCompensation(&hfdcan1);
        }

        if (HAL_FDCAN_ConfigFilter(&hfdcan1, &can_std_filter) != HAL_OK) return HAL_ERROR;
        if (HAL_FDCAN_ConfigFilter(&hfdcan1, &can_ext_filter) != HAL_OK) return HAL_ERROR;
        if (HAL_FDCAN_ConfigFilter(&hfdcan1, &can_std_pass_all) != HAL_OK) return HAL_ERROR;
        if (HAL_FDCAN_ConfigFilter(&hfdcan1, &can_ext_pass_all) != HAL_OK) return HAL_ERROR;
        HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);

        HAL_FDCAN_ConfigTimestampCounter(&hfdcan1, FDCAN_TIMESTAMP_PRESC_1);
        // Internal does not work to get time. External use TIM3 as source. See RM0444.
        HAL_FDCAN_EnableTimestampCounter(&hfdcan1, FDCAN_TIMESTAMP_EXTERNAL);

        if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) return HAL_ERROR;

        buf_clear_can_buffer();

        can_update_bit_time_ns();
        can_clear_cycle_time();
        can_bus_load_ppm = 0;
        can_error_state.last_err_code = FDCAN_PROTOCOL_ERROR_NONE;

        led_turn_txd(LED_OFF);

        can_bus_state = BUS_OPENED;

        return HAL_OK;
    }
    return HAL_ERROR;
}

// Disable the CAN peripheral and close the channel
HAL_StatusTypeDef can_disable(void)
{
    if (can_bus_state == BUS_OPENED)
    {
        HAL_FDCAN_Stop(&hfdcan1);
        HAL_FDCAN_DeInit(&hfdcan1);

        // Reset error counter etc.
        __HAL_RCC_FDCAN_FORCE_RESET();
        __HAL_RCC_FDCAN_RELEASE_RESET();

        buf_clear_can_buffer();

        led_turn_txd(LED_ON);

        can_bus_state = BUS_CLOSED;

        return HAL_OK;
    }
    return HAL_ERROR;
}

// Process data from CAN tx/rx circular buffers
void can_process(void)
{
    static uint16_t last_frame_time_cnt = 0;
    static uint32_t bit_cnt_message = 0;
    FDCAN_TxEventFifoTypeDef tx_event;
    FDCAN_RxHeaderTypeDef rx_msg_header;
    uint8_t rx_msg_data[64] = {0};

    // If message transmitted on bus, parse the frame
    if (HAL_FDCAN_GetTxEvent(&hfdcan1, &tx_event) == HAL_OK)
    {
        int32_t len = slcan_generate_tx_event(buf_get_cdc_dest(SLCAN_MTU), &tx_event, buf_dequeue_can_tx_data());
        buf_comit_cdc_dest(len);

        if (tx_event.TxTimestamp != last_frame_time_cnt)    // Don't count same frame.
        {
            bit_cnt_message += can_get_bit_number_in_tx_event(&tx_event);
            last_frame_time_cnt = tx_event.TxTimestamp;
        }

        led_blink_txd();
    }

    // Message has been accepted, pull it from the buffer
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO0, &rx_msg_header, rx_msg_data) == HAL_OK)
    {
        int32_t len = slcan_generate_rx_frame(buf_get_cdc_dest(SLCAN_MTU), &rx_msg_header, rx_msg_data);
        buf_comit_cdc_dest(len);

        if (rx_msg_header.RxTimestamp != last_frame_time_cnt)   // Don't count same frame.
        {
            bit_cnt_message += can_get_bit_number_in_rx_frame(&rx_msg_header);
            last_frame_time_cnt = rx_msg_header.RxTimestamp;
        }

        led_blink_rxd();
    }

    // Message has been received but not been accepted, pull it from the buffer
    if (HAL_FDCAN_GetRxMessage(&hfdcan1, FDCAN_RX_FIFO1, &rx_msg_header, rx_msg_data) == HAL_OK)
    {
        if (rx_msg_header.RxTimestamp != last_frame_time_cnt)   // Don't count same frame.
        {
            bit_cnt_message += can_get_bit_number_in_rx_frame(&rx_msg_header);
            last_frame_time_cnt = rx_msg_header.RxTimestamp;
        }

        led_blink_rxd();
    }

    // Update bus load
    static uint32_t tick_last = 0;
    uint32_t tick_now = HAL_GetTick();
    if (100 <= (uint32_t)(tick_now - tick_last))    // Update in every 100ms interval
    {
        uint32_t rate_us_per_ms = (uint32_t)bit_cnt_message * can_bit_time_ns / 1000 / 100;   // MAX: 1000 @ 1Mbps
        can_bus_load_ppm = (can_bus_load_ppm * 7 + (uint32_t)CAN_BUS_LOAD_BUILDUP_PPM * rate_us_per_ms / 1000) >> 3;
        bit_cnt_message = 0;
        tick_last = tick_now;
    }

    // Check for message loss
    if (__HAL_FDCAN_GET_FLAG(&hfdcan1, FDCAN_FLAG_TX_EVT_FIFO_ELT_LOST))
    {
        slcan_raise_error(SLCAN_STS_DATA_OVERRUN);
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, FDCAN_FLAG_TX_EVT_FIFO_ELT_LOST);
    }

    if (__HAL_FDCAN_GET_FLAG(&hfdcan1, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST))
    {
        slcan_raise_error(SLCAN_STS_DATA_OVERRUN);
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, FDCAN_FLAG_RX_FIFO0_MESSAGE_LOST);
    }

    if (__HAL_FDCAN_GET_FLAG(&hfdcan1, FDCAN_FLAG_RX_FIFO1_MESSAGE_LOST))
    {
        slcan_raise_error(SLCAN_STS_DATA_OVERRUN);
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, FDCAN_FLAG_RX_FIFO1_MESSAGE_LOST);
    }

    // Check for bus state and error counter
    FDCAN_ProtocolStatusTypeDef sts;
    FDCAN_ErrorCountersTypeDef cnt;

    HAL_FDCAN_GetProtocolStatus(&hfdcan1, &sts);
    HAL_FDCAN_GetErrorCounters(&hfdcan1, &cnt);

    uint8_t rx_err_cnt = (uint8_t)(cnt.RxErrorPassive ? 128 : cnt.RxErrorCnt);
    if (rx_err_cnt > can_error_state.rec || cnt.TxErrorCnt > can_error_state.tec)
        slcan_raise_error(SLCAN_STS_BUS_ERROR);
    if (sts.BusOff && !can_error_state.bus_off)
    	slcan_raise_error(SLCAN_STS_BUS_ERROR);  // Capture counter increase that caused bus off

    can_error_state.bus_off = (uint8_t)sts.BusOff;
    can_error_state.err_pssv = (uint8_t)sts.ErrorPassive;
    can_error_state.tec = (uint8_t)cnt.TxErrorCnt;
    can_error_state.rec = (uint8_t)rx_err_cnt;
    if (sts.DataLastErrorCode != FDCAN_PROTOCOL_ERROR_NONE && sts.DataLastErrorCode != FDCAN_PROTOCOL_ERROR_NO_CHANGE)
        can_error_state.last_err_code = sts.DataLastErrorCode;
    if (sts.LastErrorCode != FDCAN_PROTOCOL_ERROR_NONE && sts.LastErrorCode != FDCAN_PROTOCOL_ERROR_NO_CHANGE)
        can_error_state.last_err_code = sts.LastErrorCode;

    // Check for bus error flags
    if (__HAL_FDCAN_GET_FLAG(&hfdcan1, FDCAN_FLAG_ERROR_WARNING))
    {
        slcan_raise_error(SLCAN_STS_ERROR_WARNING);
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, FDCAN_FLAG_ERROR_WARNING);
    }

    if (__HAL_FDCAN_GET_FLAG(&hfdcan1, FDCAN_FLAG_ERROR_PASSIVE))
    {
        slcan_raise_error(SLCAN_STS_ERROR_PASSIVE);
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, FDCAN_FLAG_ERROR_PASSIVE);
    }

    if (__HAL_FDCAN_GET_FLAG(&hfdcan1, FDCAN_FLAG_BUS_OFF))
    {
        // No status flag for bus off
        __HAL_FDCAN_CLEAR_FLAG(&hfdcan1, FDCAN_FLAG_BUS_OFF);
    }

    // Update cycle time
    static uint32_t last_time_stamp_cnt = 0;
    uint16_t curr_time_stamp_cnt = HAL_FDCAN_GetTimestampCounter(&hfdcan1);
    uint32_t cycle_time_ns;
    if (last_time_stamp_cnt <= curr_time_stamp_cnt)
        cycle_time_ns = ((uint32_t)curr_time_stamp_cnt - last_time_stamp_cnt) * 1000;
    else
        cycle_time_ns = ((uint32_t)UINT16_MAX - last_time_stamp_cnt + 1 + curr_time_stamp_cnt) * 1000;

    if (can_cycle_max_time_ns < cycle_time_ns)
        can_cycle_max_time_ns = cycle_time_ns;
        
    can_cycle_ave_time_ns = ((uint32_t)can_cycle_ave_time_ns * 15 + cycle_time_ns) >> 4;
    
    last_time_stamp_cnt = curr_time_stamp_cnt;

    // Green LED on during bus closed
    if (can_bus_state == BUS_CLOSED)
        led_turn_txd(LED_ON);

}

// Set the nominal bitrate of the CAN peripheral
HAL_StatusTypeDef can_set_nominal_bitrate(enum can_bitrate_nominal bitrate)
{
    if (can_bus_state == BUS_OPENED)
    {
        // cannot set bitrate while on bus
        return HAL_ERROR;
    }

    // Set default bitrate 125k
    can_bit_cfg_nominal.prescaler = 8;
    can_bit_cfg_nominal.sjw = 8;
    can_bit_cfg_nominal.time_seg1 = 70;
    can_bit_cfg_nominal.time_seg2 = 9;

    switch (bitrate)
    {
    case CAN_BITRATE_10K:
        can_bit_cfg_nominal.prescaler = 100;
        break;
    case CAN_BITRATE_20K:
        can_bit_cfg_nominal.prescaler = 50;
        break;
    case CAN_BITRATE_50K:
        can_bit_cfg_nominal.prescaler = 20;
        break;
    case CAN_BITRATE_100K:
        can_bit_cfg_nominal.prescaler = 10;
        break;
    case CAN_BITRATE_125K:
        break;
    case CAN_BITRATE_250K:
        can_bit_cfg_nominal.prescaler = 4;
        break;
    case CAN_BITRATE_500K:
        can_bit_cfg_nominal.prescaler = 2;
        break;
    case CAN_BITRATE_800K:
        can_bit_cfg_nominal.prescaler = 1;
        can_bit_cfg_nominal.sjw = 10;
        can_bit_cfg_nominal.time_seg1 = 88;
        can_bit_cfg_nominal.time_seg2 = 11;
        break;
    case CAN_BITRATE_1000K:
        can_bit_cfg_nominal.prescaler = 1;
        break;
    default:
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Set the data bitrate of the CAN peripheral
HAL_StatusTypeDef can_set_data_bitrate(enum can_bitrate_data bitrate)
{
    if (can_bus_state == BUS_OPENED)
    {
        // cannot set bitrate while on bus
        return HAL_ERROR;
    }

    // Set default bitrate 2M
    can_bit_cfg_data.prescaler = 1;
    can_bit_cfg_data.sjw = 8;
    can_bit_cfg_data.time_seg1 = 30;
    can_bit_cfg_data.time_seg2 = 9;

    switch (bitrate)
    {
    case CAN_DATA_BITRATE_500K:
        can_bit_cfg_data.prescaler = 4;
        break;
    case CAN_DATA_BITRATE_1M:
        can_bit_cfg_data.prescaler = 2;
        break;
    case CAN_DATA_BITRATE_2M:
        break;
    case CAN_DATA_BITRATE_4M:
        can_bit_cfg_data.prescaler = 1;
        can_bit_cfg_data.sjw = 4;
        can_bit_cfg_data.time_seg1 = 14;
        can_bit_cfg_data.time_seg2 = 5;
        break;
    case CAN_DATA_BITRATE_5M:
        can_bit_cfg_data.prescaler = 1;
        can_bit_cfg_data.sjw = 3;
        can_bit_cfg_data.time_seg1 = 11;
        can_bit_cfg_data.time_seg2 = 4;
        break;
    default:
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Set the nominal bitrate configuration of the CAN peripheral
HAL_StatusTypeDef can_set_nominal_bitrate_cfg(struct can_bitrate_cfg bitrate_cfg)
{
    if (can_bus_state == BUS_OPENED)
    {
        // cannot set bitrate while on bus
        return HAL_ERROR;
    }

    if (!IS_FDCAN_NOMINAL_PRESCALER(bitrate_cfg.prescaler)) return HAL_ERROR;
    if (!IS_FDCAN_NOMINAL_TSEG1(bitrate_cfg.time_seg1)) return HAL_ERROR;
    if (!IS_FDCAN_NOMINAL_TSEG2(bitrate_cfg.time_seg2)) return HAL_ERROR;
    if (!IS_FDCAN_NOMINAL_SJW(bitrate_cfg.sjw)) return HAL_ERROR;

    can_bit_cfg_nominal = bitrate_cfg;

    return HAL_OK;
}

// Set the data bitrate configuration of the CAN peripheral
HAL_StatusTypeDef can_set_data_bitrate_cfg(struct can_bitrate_cfg bitrate_cfg)
{
    if (can_bus_state == BUS_OPENED)
    {
        // cannot set bitrate while on bus
        return HAL_ERROR;
    }

    if (!IS_FDCAN_DATA_PRESCALER(bitrate_cfg.prescaler)) return HAL_ERROR;
    if (!IS_FDCAN_DATA_TSEG1(bitrate_cfg.time_seg1)) return HAL_ERROR;
    if (!IS_FDCAN_DATA_TSEG2(bitrate_cfg.time_seg2)) return HAL_ERROR;
    if (!IS_FDCAN_DATA_SJW(bitrate_cfg.sjw)) return HAL_ERROR;

    can_bit_cfg_data = bitrate_cfg;

    return HAL_OK;
}

// Get the data bitrate configuration of the CAN peripheral
struct can_bitrate_cfg can_get_data_bitrate_cfg(void)
{
    return can_bit_cfg_data;
}

// Get the nominal bitrate configuration of the CAN peripheral
struct can_bitrate_cfg can_get_bitrate_cfg(void)
{
    return can_bit_cfg_nominal;
}

// Set filter for standard CAN ID
HAL_StatusTypeDef can_set_filter_std(FunctionalState state, uint32_t code, uint32_t mask)
{
    HAL_StatusTypeDef ret = HAL_OK;
    
    if (can_bus_state == BUS_OPENED) return HAL_ERROR;
    if (state == ENABLE)
        can_std_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    else if (state == DISABLE)
        can_std_filter.FilterConfig = FDCAN_FILTER_DISABLE;
    else
        ret = HAL_ERROR;

    if (code > 0x7FF)
        ret = HAL_ERROR;
    else
        can_std_filter.FilterID1 = code;

    if (mask > 0x7FF)
        ret = HAL_ERROR;
    else
        can_std_filter.FilterID2 = mask;
    
    return ret;
}

// Set filter for extended CAN ID
HAL_StatusTypeDef can_set_filter_ext(FunctionalState state, uint32_t code, uint32_t mask)
{
    HAL_StatusTypeDef ret = HAL_OK;
    
    if (can_bus_state == BUS_OPENED) return HAL_ERROR;
    if (state == ENABLE)
        can_ext_filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    else if (state == DISABLE)
        can_ext_filter.FilterConfig = FDCAN_FILTER_DISABLE;
    else
        ret = HAL_ERROR;

    if (code > 0x1FFFFFFF)
        ret = HAL_ERROR;
    else
        can_ext_filter.FilterID1 = code;

    if (mask > 0x1FFFFFFF)
        ret = HAL_ERROR;
    else
        can_ext_filter.FilterID2 = mask;
    
    return ret;
}

// Get filter state for standard CAN ID
FunctionalState can_is_filter_std_enabled(void)
{
    if (can_std_filter.FilterConfig == FDCAN_FILTER_DISABLE)
        return DISABLE;
    else
        return ENABLE;
}

// Get filter state for extended CAN ID
FunctionalState can_is_filter_ext_enabled(void)
{
    if (can_ext_filter.FilterConfig == FDCAN_FILTER_DISABLE)
        return DISABLE;
    else
        return ENABLE;
}

// Get filter for standard CAN ID
uint32_t can_get_filter_std_code(void)
{
    return can_std_filter.FilterID1 & 0x7FF;
}

// Get filter for standard CAN ID
uint32_t can_get_filter_std_mask(void)
{
    return can_std_filter.FilterID2 & 0x7FF;
}

// Get filter for extended CAN ID
uint32_t can_get_filter_ext_code(void)
{
    return can_ext_filter.FilterID1 & 0x1FFFFFFF;
}

// Get filter for extended CAN ID
uint32_t can_get_filter_ext_mask(void)
{
    return can_ext_filter.FilterID2 & 0x1FFFFFFF;
}

// Set CAN peripheral to the specific mode
// normal: FDCAN_MODE_NORMAL
// silent: FDCAN_MODE_BUS_MONITORING
// loopback: FDCAN_MODE_INTERNAL_LOOPBACK
// external: FDCAN_MODE_EXTERNAL_LOOPBACK
HAL_StatusTypeDef can_set_mode(uint32_t mode)
{
    if (can_bus_state == BUS_OPENED)
    {
        // cannot set silent mode while on bus
        return HAL_ERROR;
    }
    can_mode = mode;

    return HAL_OK;
}

// Set auto retransmit function
HAL_StatusTypeDef can_set_auto_retransmit(FunctionalState state)
{
    if (can_bus_state == BUS_OPENED)
    {
        // cannot set state while on bus
        return HAL_ERROR;
    }
    can_auto_retransmit = state;

    return HAL_OK;
}

// Return bus status
enum can_bus_state can_get_bus_state(void)
{
    return can_bus_state;
}

struct can_error_state can_get_error_state(void)
{
    return can_error_state;
}

FunctionalState can_is_tx_enabled(void)
{
    if (can_bus_state == BUS_CLOSED)
        return DISABLE;
    else if (hfdcan1.Init.Mode == FDCAN_MODE_BUS_MONITORING)
        return DISABLE;
    else if (can_error_state.bus_off)
        return DISABLE;
    else
        return ENABLE;
}

// Return CAN bus load in ppm
uint32_t can_get_bus_load_ppm(void)
{
    return can_bus_load_ppm;
}

// Clear the maximum and average cycle time
void can_clear_cycle_time(void)
{
    can_cycle_max_time_ns = 0;
    can_cycle_ave_time_ns = 0;
}

// Return the maximum cycle time in nano seconds
uint32_t can_get_cycle_max_time_ns(void)
{
    return can_cycle_max_time_ns;
}

// Return the average cycle time in nano seconds
uint32_t can_get_cycle_ave_time_ns(void)
{
    return can_cycle_ave_time_ns;
}

// Return reference to CAN handle
FDCAN_HandleTypeDef *can_get_handle(void)
{
    return &hfdcan1;
}

// Get the nominal one bit time in nanoseconds
void can_update_bit_time_ns(void)
{
    can_bit_time_ns = ((uint32_t)1 + can_bit_cfg_nominal.time_seg1 + can_bit_cfg_nominal.time_seg2);
    can_bit_time_ns = can_bit_time_ns * can_bit_cfg_nominal.prescaler;    // Tq in one bit
    can_bit_time_ns = can_bit_time_ns * 1000;                             // MAX: (1 + 256 + 128) * 1000
    can_bit_time_ns = can_bit_time_ns / 60;                              // Clock: 60MHz = (60 / 1000) GHz

    return;
}

// Return the duration of the rx frame in the nominal bit number
uint16_t can_get_bit_number_in_rx_frame(FDCAN_RxHeaderTypeDef *pRxHeader)
{
    uint16_t time_msg, time_data;
    uint8_t data_bytes = can_dlc_to_bytes[CAN_HAL_DLC_TO_STD_DLC(pRxHeader->DataLength)];

    if (pRxHeader->RxFrameType == FDCAN_REMOTE_FRAME && pRxHeader->IdType == FDCAN_STANDARD_ID)
    {
        time_msg = CAN_BIT_NBR_WOD_CBFF;
    }
    else if (pRxHeader->RxFrameType == FDCAN_REMOTE_FRAME && pRxHeader->IdType == FDCAN_EXTENDED_ID)
    {
        time_msg = CAN_BIT_NBR_WOD_CEFF;
    }
    else if (pRxHeader->FDFormat == FDCAN_CLASSIC_CAN && pRxHeader->IdType == FDCAN_STANDARD_ID)
    {
        time_msg = CAN_BIT_NBR_WOD_CBFF + (uint16_t)data_bytes * 8;
    }
    else if (pRxHeader->FDFormat == FDCAN_CLASSIC_CAN && pRxHeader->IdType == FDCAN_EXTENDED_ID)
    {
        time_msg = CAN_BIT_NBR_WOD_CEFF + (uint16_t)data_bytes * 8;
    }
    else    // For FD frames
    {
        if (pRxHeader->IdType == FDCAN_STANDARD_ID) time_msg = CAN_BIT_NBR_WOD_FBFF_ARBIT;
        else                                        time_msg = CAN_BIT_NBR_WOD_FEFF_ARBIT;

        if (data_bytes <= 16)
            time_data = CAN_BIT_NBR_WOD_FXFF_DATA_S;    // Short CRC
        else
            time_data = CAN_BIT_NBR_WOD_FXFF_DATA_L;    // Long CRC

        time_data = time_data + (uint16_t)data_bytes * 8;

        if (pRxHeader->BitRateSwitch == FDCAN_BRS_ON)
        {
            if (can_bit_cfg_nominal.prescaler == 0) return 0;   // Uninitialized bitrate (avoid zero-div)

            uint32_t rate_ppm;  // Nominal bit time vs data bit time
            rate_ppm = ((uint32_t)1 + can_bit_cfg_data.time_seg1 + can_bit_cfg_data.time_seg2);
            rate_ppm = rate_ppm * can_bit_cfg_data.prescaler;     // Tq in one bit (data)
            rate_ppm = rate_ppm * 1000000;  // MAX: 32 * (32 + 16) * 1000000 
            rate_ppm = rate_ppm / ((uint32_t)1 + can_bit_cfg_nominal.time_seg1 + can_bit_cfg_nominal.time_seg2);
            rate_ppm = rate_ppm / can_bit_cfg_nominal.prescaler;

            time_msg = time_msg + ((uint32_t)time_data * rate_ppm) / 1000000;
        }
        else
        {
            time_msg = time_msg + time_data;
        }
    }
    return time_msg;
}

// Return the duration of the tx event in the nominal bit number
uint16_t can_get_bit_number_in_tx_event(FDCAN_TxEventFifoTypeDef *pTxEvent)
{
    FDCAN_RxHeaderTypeDef frame_header;
    //frame_header.Identifier = pTxEvent->Identifier;
    frame_header.IdType = pTxEvent->IdType;
    frame_header.RxFrameType = pTxEvent->TxFrameType;
    frame_header.DataLength = pTxEvent->DataLength;
    //frame_header.ErrorStateIndicator = pTxEvent->ErrorStateIndicator;
    frame_header.BitRateSwitch = pTxEvent->BitRateSwitch;
    frame_header.FDFormat = pTxEvent->FDFormat;
    //frame_header.RxTimestamp = pTxEvent->TxTimestamp;
    return can_get_bit_number_in_rx_frame(&frame_header);
}
