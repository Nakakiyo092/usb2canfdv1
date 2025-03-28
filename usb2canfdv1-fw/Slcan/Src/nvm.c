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

// Read and write to non-volatile memory.

#include "stm32g0xx_hal.h"
#include "can.h"
#include "led.h"
#include "nvm.h"
#include "slcan.h"

// Memory status
enum nvm_memory_status
{
    NVM_MEMORY_WRITTEN = 0xA,
    NVM_MEMORY_CLEARED = 0xF    /* Flash memory store 0xFF when cleared */
};

#define NVM_PAGE_NUMBER_DATA      (62)                          /* Page number of data area (see RM0444-3.3.1) */
#define NVM_ERASE_OK              (0xFFFFFFFF)
#define NVM_ADDR_ORIGIN           (0x0801F000)                  /* Start address of data area in flash */
#define NVM_ADDR_SERIAL_NUMBER    (NVM_ADDR_ORIGIN + 0x000UL)
#define NVM_ADDR_STP_CONFIG       (NVM_ADDR_ORIGIN + 0x008UL)   /* Auto startup configuration */
#define NVM_ADDR_STP_NOM_BITRATE  (NVM_ADDR_ORIGIN + 0x010UL)
#define NVM_ADDR_STP_DATA_BITRATE (NVM_ADDR_ORIGIN + 0x018UL)
#define NVM_ADDR_STP_FILTER_STD   (NVM_ADDR_ORIGIN + 0x020UL)
#define NVM_ADDR_STP_FILTER_EXT   (NVM_ADDR_ORIGIN + 0x028UL)

#define NVM_EXTRACT_MEM_STS(val)  ((uint8_t)(((val) >> 60) & 0x0F))
#define NVM_IS_WRITTEN(val)       (NVM_EXTRACT_MEM_STS(val) == NVM_MEMORY_WRITTEN)
#define NVM_WRITE_MEM_STS(val)    ((((uint64_t)val) & 0x0FFFFFFFFFFFFFFF) | (((uint64_t)NVM_MEMORY_WRITTEN) << 60))

// Private variables
static uint64_t nvm_serial_number_raw;
static uint64_t nvm_stp_config_raw;
static uint64_t nvm_stp_nom_bitrate_raw;
static uint64_t nvm_stp_data_bitrate_raw;
static uint64_t nvm_stp_filter_std_raw;
static uint64_t nvm_stp_filter_ext_raw;

// Private methods
static HAL_StatusTypeDef nvm_write_to_flash(void);

// Read data from non-volatile memory and store it in RAM
void nvm_init(void)
{
    // Read data form flash and store it in private variable (RAM)
    nvm_serial_number_raw =     *(uint64_t *)NVM_ADDR_SERIAL_NUMBER;
    nvm_stp_config_raw =        *(uint64_t *)NVM_ADDR_STP_CONFIG;
    nvm_stp_nom_bitrate_raw =   *(uint64_t *)NVM_ADDR_STP_NOM_BITRATE;
    nvm_stp_data_bitrate_raw =  *(uint64_t *)NVM_ADDR_STP_DATA_BITRATE;
    nvm_stp_filter_std_raw =    *(uint64_t *)NVM_ADDR_STP_FILTER_STD;
    nvm_stp_filter_ext_raw =    *(uint64_t *)NVM_ADDR_STP_FILTER_EXT;

    return;
}

// Get serial number
HAL_StatusTypeDef nvm_get_serial_number(uint16_t *num)
{
    if (NVM_IS_WRITTEN(nvm_serial_number_raw))
    {
        *num = (uint16_t)(nvm_serial_number_raw & 0xFFFF);
        return HAL_OK;
    }
    return HAL_ERROR;
}

// Update serial number
HAL_StatusTypeDef nvm_update_serial_number(uint16_t num)
{
    // Check if the serial number is the same
    if (NVM_WRITE_MEM_STS(num) == nvm_serial_number_raw)
    {
        return HAL_OK;
    }

    // Write to the flash
    nvm_serial_number_raw = NVM_WRITE_MEM_STS(num);
    if (nvm_write_to_flash() != HAL_OK)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Apply auto startup configuration
HAL_StatusTypeDef nvm_apply_startup_cfg(void)
{
    // Check if the memory is written
    if (!NVM_IS_WRITTEN(nvm_stp_config_raw)) return HAL_ERROR;
    if (!NVM_IS_WRITTEN(nvm_stp_nom_bitrate_raw)) return HAL_ERROR;
    if (!NVM_IS_WRITTEN(nvm_stp_data_bitrate_raw)) return HAL_ERROR;
    if (!NVM_IS_WRITTEN(nvm_stp_filter_std_raw)) return HAL_ERROR;
    if (!NVM_IS_WRITTEN(nvm_stp_filter_ext_raw)) return HAL_ERROR;

    // Read and apply the main configuration
    uint8_t startup_mode = (uint8_t)(nvm_stp_config_raw & 0xFF);

    if (startup_mode == SLCAN_AUTO_STARTUP_OFF)
        return HAL_OK;

    if (SLCAN_AUTO_STARTUP_INVALID <= startup_mode)
        return HAL_ERROR;

    uint8_t timestamp_mode = (uint8_t)((nvm_stp_config_raw >> 8) & 0xFF);

    if (SLCAN_TIMESTAMP_INVALID <= timestamp_mode)
        return HAL_ERROR;

    slcan_set_timestamp_mode(timestamp_mode);

    uint16_t report_reg = (uint16_t)((nvm_stp_config_raw >> 16) & 0xFFFF);
    slcan_set_report_mode(report_reg);

    // Read and apply bitrate
    struct can_bitrate_cfg bitrate;
    bitrate.prescaler = (uint16_t)((nvm_stp_nom_bitrate_raw) & 0xFF);
    bitrate.time_seg1 = (uint8_t)((nvm_stp_nom_bitrate_raw >> 8) & 0xFF);
    bitrate.time_seg2 = (uint8_t)((nvm_stp_nom_bitrate_raw >> 16) & 0xFF);
    bitrate.sjw = (uint8_t)((nvm_stp_nom_bitrate_raw >> 24) & 0xFF);
    can_set_nominal_bitrate_cfg(bitrate);

    bitrate.prescaler = (uint16_t)((nvm_stp_data_bitrate_raw) & 0xFF);
    bitrate.time_seg1 = (uint8_t)((nvm_stp_data_bitrate_raw >> 8) & 0xFF);
    bitrate.time_seg2 = (uint8_t)((nvm_stp_data_bitrate_raw >> 16) & 0xFF);
    bitrate.sjw = (uint8_t)((nvm_stp_data_bitrate_raw >> 24) & 0xFF);
    can_set_data_bitrate_cfg(bitrate);

    // Read and apply filter
    FunctionalState state;
    uint32_t code, mask;
    state = ((nvm_stp_filter_std_raw >> 22) & 0x1) ? ENABLE : DISABLE;
    code = (nvm_stp_filter_std_raw & 0x7FF);
    mask = ((nvm_stp_filter_std_raw >> 11) & 0x7FF);
    can_set_filter_std(state, code, mask);

    state = ((nvm_stp_filter_ext_raw >> 58) & 0x1) ? ENABLE : DISABLE;
    code = (nvm_stp_filter_ext_raw & 0x1FFFFFFF);
    mask = ((nvm_stp_filter_ext_raw >> 29) & 0x1FFFFFFF);
    can_set_filter_ext(state, code, mask);

    // Start the CAN peripheral
    if (startup_mode == SLCAN_AUTO_STARTUP_NORMAL)
    {
        slcan_clear_error();

        // Default to normal mode
        if (can_set_mode(FDCAN_MODE_NORMAL) != HAL_OK)
            return HAL_ERROR;

        // Open CAN port
        if (can_enable() != HAL_OK)
            return HAL_ERROR;

        return HAL_OK;
    }
    else if (startup_mode == SLCAN_AUTO_STARTUP_LISTEN)
    {
        slcan_clear_error();

        // Mode silent
        if (can_set_mode(FDCAN_MODE_BUS_MONITORING) != HAL_OK)
            return HAL_ERROR;

        // Open CAN port
        if (can_enable() != HAL_OK)
            return HAL_ERROR;

        return HAL_OK;
    }       
    return HAL_ERROR;
}

// Update auto startup configuration
HAL_StatusTypeDef nvm_update_startup_cfg(uint8_t mode)
{
    // Make raw data for startup configuration
    uint64_t startup_cfg = 0;

    startup_cfg = (startup_cfg | (uint64_t)mode);

    if (0xFF < slcan_get_timestamp_mode()) return HAL_ERROR;
    startup_cfg = (startup_cfg | ((uint64_t)slcan_get_timestamp_mode() << 8));
    startup_cfg = (startup_cfg | ((uint64_t)slcan_get_report_mode() << 16));
    startup_cfg = NVM_WRITE_MEM_STS(startup_cfg);

    // Make raw data for nominal bitrate
    uint64_t nom_bitrate = 0;

    if (0xFF < can_get_bitrate_cfg().prescaler) return HAL_ERROR;
    nom_bitrate = (nom_bitrate | (uint64_t)can_get_bitrate_cfg().prescaler);
    nom_bitrate = (nom_bitrate | ((uint64_t)can_get_bitrate_cfg().time_seg1 << 8));
    nom_bitrate = (nom_bitrate | ((uint64_t)can_get_bitrate_cfg().time_seg2 << 16));
    nom_bitrate = (nom_bitrate | ((uint64_t)can_get_bitrate_cfg().sjw << 24));
    nom_bitrate = NVM_WRITE_MEM_STS(nom_bitrate);

    // Make raw data for data bitrate
    uint64_t data_bitrate = 0;

    if (0xFF < can_get_data_bitrate_cfg().prescaler) return HAL_ERROR;
    data_bitrate = (data_bitrate | (uint64_t)can_get_data_bitrate_cfg().prescaler);
    data_bitrate = (data_bitrate | ((uint64_t)can_get_data_bitrate_cfg().time_seg1 << 8));
    data_bitrate = (data_bitrate | ((uint64_t)can_get_data_bitrate_cfg().time_seg2 << 16));
    data_bitrate = (data_bitrate | ((uint64_t)can_get_data_bitrate_cfg().sjw << 24));
    data_bitrate = NVM_WRITE_MEM_STS(data_bitrate);

    // Make raw data for standard filter
    uint64_t filter_std = 0;
    filter_std = (filter_std | ((uint64_t)can_get_filter_std_code() & 0x7FF));
    filter_std = (filter_std | (((uint64_t)can_get_filter_std_mask() & 0x7FF) << 11));
    filter_std = (filter_std | ((uint64_t)(can_is_filter_std_enabled() == ENABLE) << 22));
    filter_std = NVM_WRITE_MEM_STS(filter_std);

    // Make raw data for extended filter
    uint64_t filter_ext = 0;
    filter_ext = (filter_ext | ((uint64_t)can_get_filter_ext_code() & 0x1FFFFFFF));
    filter_ext = (filter_ext | (((uint64_t)can_get_filter_ext_mask() & 0x1FFFFFFF) << 29));
    filter_ext = (filter_ext | ((uint64_t)(can_is_filter_ext_enabled() == ENABLE) << 58));
    filter_ext = NVM_WRITE_MEM_STS(filter_ext);

    // Check if the configuration is the same
    if (startup_cfg == nvm_stp_config_raw)
        if (nom_bitrate == nvm_stp_nom_bitrate_raw && data_bitrate == nvm_stp_data_bitrate_raw)
            if (filter_std == nvm_stp_filter_std_raw && filter_ext == nvm_stp_filter_ext_raw)
                return HAL_OK;

    // Update the RAM data
    nvm_stp_config_raw = startup_cfg;
    nvm_stp_nom_bitrate_raw = nom_bitrate;
    nvm_stp_data_bitrate_raw = data_bitrate;
    nvm_stp_filter_std_raw = filter_std;
    nvm_stp_filter_ext_raw = filter_ext;

    // Write to the flash
    if (nvm_write_to_flash() != HAL_OK)
    {
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

// Write the RAM data to the data area in the flash memory
HAL_StatusTypeDef nvm_write_to_flash(void)
{
    // Unlock the flash
    HAL_FLASH_Unlock();
    
    // Erase the page
    FLASH_EraseInitTypeDef erase;
    erase.TypeErase = FLASH_TYPEERASE_PAGES;
    erase.Banks = FLASH_BANK_1;
    erase.Page = NVM_PAGE_NUMBER_DATA;
    erase.NbPages = 1;

    uint32_t error = 0;
    HAL_FLASHEx_Erase(&erase, &error);
    if (error != NVM_ERASE_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    // Write serial number to flash
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NVM_ADDR_SERIAL_NUMBER, nvm_serial_number_raw) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    // Write startup config to flash
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NVM_ADDR_STP_CONFIG, nvm_stp_config_raw) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    // Write nominal bitrate to flash
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NVM_ADDR_STP_NOM_BITRATE, nvm_stp_nom_bitrate_raw) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    // Write data bitrate to flash
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NVM_ADDR_STP_DATA_BITRATE, nvm_stp_data_bitrate_raw) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    // Write standard filter to flash
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NVM_ADDR_STP_FILTER_STD, nvm_stp_filter_std_raw) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    // Write extended filter to flash
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NVM_ADDR_STP_FILTER_EXT, nvm_stp_filter_ext_raw) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    // Lock the flash
    HAL_FLASH_Lock();
    return HAL_OK;
}
