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
#include "nvm.h"
#include "slcan.h"

// Memory status
enum NvmMemoryStatus
{
    NVM_MEMORY_WRITTEN = 0xA,
    NVM_MEMORY_CLEARED = 0xF    /* Flash memory store 0xFF when cleared */
};

#define NVM_PAGE_NUMBER_DATA      (62)                          /* Page number of data area (see RM0444-3.3.1) */
#define NVM_ERASE_OK              (0xFFFFFFFF)
#define NVM_ADDR_ORIGIN           (0x0801F000)                  /* Start address of data area in flash */
#define NVM_ADDR_SERIAL_NUMBER    (NVM_ADDR_ORIGIN + 0x000UL)
#define NVM_ADDR_STP_CONFIG       (NVM_ADDR_ORIGIN + 0x008UL)   /* Auto startup configuration */
#define NVM_ADDR_STP_NOM_BITRATE  (NVM_ADDR_ORIGIN + 0x010UL)   /* Nominal bitrate at STartuP */
#define NVM_ADDR_STP_DATA_BITRATE (NVM_ADDR_ORIGIN + 0x018UL)
#define NVM_ADDR_STP_FILTER_CODE  (NVM_ADDR_ORIGIN + 0x020UL)
#define NVM_ADDR_STP_FILTER_MASK  (NVM_ADDR_ORIGIN + 0x028UL)

// Note: Integrity check relies solely on a 4-bit status nibble. A single bit-flip
// in the payload will silently pass NVM_IS_WRITTEN. No CRC or checksum is implemented.
#define NVM_EXTRACT_MEM_STS(val)  ((uint8_t)(((val) >> 60) & 0x0F))
#define NVM_IS_WRITTEN(val)       (NVM_EXTRACT_MEM_STS(val) == NVM_MEMORY_WRITTEN)
#define NVM_WRITE_MEM_STS(val)    ((((uint64_t)val) & 0x0FFFFFFFFFFFFFFF) | (((uint64_t)NVM_MEMORY_WRITTEN) << 60))

// Private variables
static uint64_t nvm_serial_number_raw;
static uint64_t nvm_stp_config_raw;
static uint64_t nvm_stp_nom_bitrate_raw;
static uint64_t nvm_stp_data_bitrate_raw;
static uint64_t nvm_stp_filter_code_raw;
static uint64_t nvm_stp_filter_mask_raw;

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
    nvm_stp_filter_code_raw =   *(uint64_t *)NVM_ADDR_STP_FILTER_CODE;
    nvm_stp_filter_mask_raw =   *(uint64_t *)NVM_ADDR_STP_FILTER_MASK;
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
    uint64_t prev_serial_number_raw = nvm_serial_number_raw;
    nvm_serial_number_raw = NVM_WRITE_MEM_STS(num);
    if (nvm_write_to_flash() != HAL_OK)
    {
        nvm_serial_number_raw = prev_serial_number_raw;
        return HAL_ERROR;
    }

    return HAL_OK;
}

// Apply auto startup configuration
// Note: No rollback on partial failure — if a later step returns HAL_ERROR,
// earlier settings may already be applied to live modules. Callers should treat
// HAL_ERROR as an indication that the system state is partially configured.
HAL_StatusTypeDef nvm_apply_startup_cfg(void)
{
    // Check if the memory is written
    if (!NVM_IS_WRITTEN(nvm_stp_config_raw)) return HAL_ERROR;
    if (!NVM_IS_WRITTEN(nvm_stp_nom_bitrate_raw)) return HAL_ERROR;
    if (!NVM_IS_WRITTEN(nvm_stp_data_bitrate_raw)) return HAL_ERROR;
    if (!NVM_IS_WRITTEN(nvm_stp_filter_code_raw)) return HAL_ERROR;
    if (!NVM_IS_WRITTEN(nvm_stp_filter_mask_raw)) return HAL_ERROR;

    // Read and apply the main configuration
    uint8_t startup_mode = (uint8_t)(nvm_stp_config_raw & 0xFF);

    if (startup_mode == SLCAN_AUTO_STARTUP_OFF)
        return HAL_OK;

    if (SLCAN_AUTO_STARTUP_INVALID <= startup_mode)
        return HAL_ERROR;

    uint8_t filter_mode = (uint8_t)((nvm_stp_config_raw >> 8) & 0xFF);

    if (SLCAN_FILTER_INVALID <= filter_mode)
        return HAL_ERROR;

    slcan_set_filter_mode(filter_mode);

    uint8_t timestamp_mode = (uint8_t)((nvm_stp_config_raw >> 16) & 0xFF);

    if (SLCAN_TIMESTAMP_INVALID <= timestamp_mode)
        return HAL_ERROR;

    slcan_set_timestamp_mode(timestamp_mode);

    uint16_t report_reg = (uint16_t)((nvm_stp_config_raw >> 24) & 0xFFFF);
    slcan_set_report_mode(report_reg);

    // Read and apply bitrate
    // Prescaler is stored in 8 bits; project decision limits it to 255 or less
    struct CanBitrateCfg bitrate;
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
    slcan_set_filter_code(nvm_stp_filter_code_raw & 0xFFFFFFFF);
    slcan_set_filter_mask(nvm_stp_filter_mask_raw & 0xFFFFFFFF);

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

    if (0xFF < slcan_get_filter_mode()) return HAL_ERROR;
    if (0xFF < slcan_get_timestamp_mode()) return HAL_ERROR;
    startup_cfg = (startup_cfg | ((uint64_t)slcan_get_filter_mode() << 8));
    startup_cfg = (startup_cfg | ((uint64_t)slcan_get_timestamp_mode() << 16));
    startup_cfg = (startup_cfg | ((uint64_t)slcan_get_report_mode() << 24));
    startup_cfg = NVM_WRITE_MEM_STS(startup_cfg);

    // Make raw data for nominal bitrate
    // Prescaler is stored in 8 bits; project decision limits it to 255 or less
    uint64_t nom_bitrate = 0;

    if (0xFF < can_get_nominal_bitrate_cfg().prescaler) return HAL_ERROR;
    nom_bitrate = (nom_bitrate | (uint64_t)can_get_nominal_bitrate_cfg().prescaler);
    nom_bitrate = (nom_bitrate | ((uint64_t)can_get_nominal_bitrate_cfg().time_seg1 << 8));
    nom_bitrate = (nom_bitrate | ((uint64_t)can_get_nominal_bitrate_cfg().time_seg2 << 16));
    nom_bitrate = (nom_bitrate | ((uint64_t)can_get_nominal_bitrate_cfg().sjw << 24));
    nom_bitrate = NVM_WRITE_MEM_STS(nom_bitrate);

    // Make raw data for data bitrate
    // Prescaler is stored in 8 bits; project decision limits it to 255 or less
    uint64_t data_bitrate = 0;

    if (0xFF < can_get_data_bitrate_cfg().prescaler) return HAL_ERROR;
    data_bitrate = (data_bitrate | (uint64_t)can_get_data_bitrate_cfg().prescaler);
    data_bitrate = (data_bitrate | ((uint64_t)can_get_data_bitrate_cfg().time_seg1 << 8));
    data_bitrate = (data_bitrate | ((uint64_t)can_get_data_bitrate_cfg().time_seg2 << 16));
    data_bitrate = (data_bitrate | ((uint64_t)can_get_data_bitrate_cfg().sjw << 24));
    data_bitrate = NVM_WRITE_MEM_STS(data_bitrate);

    // Make raw data for filter code
    uint64_t filter_code = 0;
    filter_code = NVM_WRITE_MEM_STS((uint64_t)slcan_get_filter_code());

    // Make raw data for filter mask
    uint64_t filter_mask = 0;
    filter_mask = NVM_WRITE_MEM_STS((uint64_t)slcan_get_filter_mask());

    // Check if the configuration is the same
    if (startup_cfg == nvm_stp_config_raw)
        if (nom_bitrate == nvm_stp_nom_bitrate_raw && data_bitrate == nvm_stp_data_bitrate_raw)
            if (filter_code == nvm_stp_filter_code_raw && filter_mask == nvm_stp_filter_mask_raw)
                return HAL_OK;

    // Update the RAM data
    uint64_t prev_stp_config_raw =      nvm_stp_config_raw;
    uint64_t prev_stp_nom_bitrate_raw = nvm_stp_nom_bitrate_raw;
    uint64_t prev_stp_data_bitrate_raw = nvm_stp_data_bitrate_raw;
    uint64_t prev_stp_filter_code_raw = nvm_stp_filter_code_raw;
    uint64_t prev_stp_filter_mask_raw = nvm_stp_filter_mask_raw;
    nvm_stp_config_raw =      startup_cfg;
    nvm_stp_nom_bitrate_raw = nom_bitrate;
    nvm_stp_data_bitrate_raw = data_bitrate;
    nvm_stp_filter_code_raw = filter_code;
    nvm_stp_filter_mask_raw = filter_mask;

    // Write to the flash
    if (nvm_write_to_flash() != HAL_OK)
    {
        nvm_stp_config_raw =      prev_stp_config_raw;
        nvm_stp_nom_bitrate_raw = prev_stp_nom_bitrate_raw;
        nvm_stp_data_bitrate_raw = prev_stp_data_bitrate_raw;
        nvm_stp_filter_code_raw = prev_stp_filter_code_raw;
        nvm_stp_filter_mask_raw = prev_stp_filter_mask_raw;
        return HAL_ERROR;
    }
    
    return HAL_OK;
}

// Write the RAM data to the data area in the flash memory
// Note: No wear leveling — every write erases and rewrites the entire page,
// consuming one flash erase cycle (~10,000–100,000 cycles rated). Acceptable
// because config writes are infrequent in normal operation.
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
    if (HAL_FLASHEx_Erase(&erase, &error) != HAL_OK || error != NVM_ERASE_OK)
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
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NVM_ADDR_STP_FILTER_CODE, nvm_stp_filter_code_raw) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    // Write extended filter to flash
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, NVM_ADDR_STP_FILTER_MASK, nvm_stp_filter_mask_raw) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return HAL_ERROR;
    }

    // Lock the flash
    HAL_FLASH_Lock();
    return HAL_OK;
}
