/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of other
  contributors to this software may be used to endorse or promote products
  derived from this software without specific prior written permission.

  4. This software must only be used in a processor manufactured by Nordic
  Semiconductor ASA, or in a processor manufactured by a third party that
  is used in combination with a processor manufactured by Nordic Semiconductor.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/

#include "dfu.h"
#include "dfu_types.h"
#include "nrf_flash.h"
#include "nrf_sdm.h"
#include "bootloader_util.h"
#include "app_error.h"

#include "nrf51.h"
#include <stdbool.h>
#include <string.h>

#define MAX_NUMBER_INTERRUPTS   (32)
/*****************************************************************************
* Static globals
*****************************************************************************/
static dfu_bootloader_info_t g_bl_info;
static bool g_in_dfu = false;
/*****************************************************************************
* Static Functions
*****************************************************************************/
static uint32_t flash_write(uint32_t* destination, uint8_t* data, uint32_t length)
{
    /* max length of a write is 256 * 4 = 1024 bytes, split it. */
    uint32_t writes = 0;
    while (length > 0)
    {
        uint32_t write_len =  length;
        if (write_len > 1024)
            write_len = 1024;
        
        uint32_t error_code = sd_flash_write(destination + writes * 256, (uint32_t*) data, write_len / 4);
        if (error_code != NRF_SUCCESS)
        {
            return error_code;
        }
        uint32_t evt;
        while (sd_evt_get(&evt) != NRF_SUCCESS || (evt != NRF_EVT_FLASH_OPERATION_SUCCESS && evt != NRF_EVT_FLASH_OPERATION_ERROR));
        if (evt == NRF_EVT_FLASH_OPERATION_ERROR)
        {
            return NRF_ERROR_INVALID_ADDR;
        }
        
        length -= write_len;
        writes++;
        data += write_len;
    }
    
    return NRF_SUCCESS;
}

static uint32_t flash_erase(uint32_t page_addr, uint32_t length)
{
    for (uint32_t i = 0; i <= (length - 1) / PAGE_SIZE; ++i)
    {
        uint32_t error_code = sd_flash_page_erase((page_addr / PAGE_SIZE) + i);
        if (error_code != NRF_SUCCESS)
        {
            return error_code;
        }
        
        uint32_t evt;
        while (sd_evt_get(&evt) != NRF_SUCCESS || (evt != NRF_EVT_FLASH_OPERATION_SUCCESS && evt != NRF_EVT_FLASH_OPERATION_ERROR));
        if (evt == NRF_EVT_FLASH_OPERATION_ERROR)
        {
            return NRF_ERROR_INVALID_ADDR;
        }
    }
    
    return NRF_SUCCESS;
}

static void get_bootloader_info(dfu_bootloader_info_t* p_info)
{
    memcpy(p_info, 
            (dfu_bootloader_info_t*) BOOTLOADER_INFO_ADDRESS, 
            sizeof(dfu_bootloader_info_t));
}

static uint16_t crc16_compute(const uint8_t * p_data, uint32_t size, const uint16_t * p_crc)
{
    uint32_t i;
    uint16_t crc = (p_crc == NULL) ? 0xffff : *p_crc;

    for (i = 0; i < size; i++)
    {
        crc  = (unsigned char)(crc >> 8) | (crc << 8);
        crc ^= p_data[i];
        crc ^= (unsigned char)(crc & 0xff) >> 4;
        crc ^= (crc << 8) << 4;
        crc ^= ((crc & 0xff) << 4) << 1;
    }

    return crc;
}


static bool bank_is_valid(const uint8_t* bank_start, uint32_t size, uint16_t bank_crc)
{
    uint16_t crc = crc16_compute(bank_start, size, NULL);
    return (crc == bank_crc);
}

static void interrupts_disable(void)
{
    uint32_t interrupt_setting_mask;
    uint32_t irq;

    // Fetch the current interrupt settings.
    interrupt_setting_mask = NVIC->ISER[0];

    // Loop from interrupt 0 for disabling of all interrupts.
    for (irq = 0; irq < MAX_NUMBER_INTERRUPTS; irq++)
    {
        if (interrupt_setting_mask & (1 << irq))
        {
            // The interrupt was enabled, hence disable it.
            NVIC_DisableIRQ((IRQn_Type)irq);
        }
    }
}

static void start_bootloader(void)
{
    uint32_t err_code = sd_softdevice_disable();
    APP_ERROR_CHECK(err_code);

    err_code = sd_softdevice_vector_table_base_set(BOOTLOADER_START_ADDRESS);
    APP_ERROR_CHECK(err_code);

    interrupts_disable();
    bootloader_util_app_start(BOOTLOADER_START_ADDRESS);
}
/*****************************************************************************
* Interface Functions
*****************************************************************************/

uint32_t dfu_transfer_begin(dfu_bootloader_info_t* bl_info)
{
    if (g_in_dfu)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    switch (bl_info->dfu_type)
    {
        case DFU_TYPE_APP:
            bl_info->bank_addr = DFU_APP_BANK_ADDRESS;
            if (bl_info->size > DFU_APP_MAX_SIZE)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            break;
        case DFU_TYPE_BOOTLOADER:
            bl_info->bank_addr = DFU_BL_BANK_ADDRESS;
            if (bl_info->size > DFU_BL_MAX_SIZE)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            break;
        case DFU_TYPE_SOFTDEVICE:
            bl_info->bank_addr = DFU_SD_BANK_ADDRESS;
            if (bl_info->size > DFU_SD_MAX_SIZE)
            {
                return NRF_ERROR_INVALID_PARAM;
            }
            break;
    }

    dfu_bootloader_info_t old_bl_info;
    get_bootloader_info(&old_bl_info);

    uint32_t error_code;
    
    /* erase bank */ 
    error_code = flash_erase(bl_info->bank_addr, bl_info->size);
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    /* store bootloader info in last page */
    error_code = flash_erase(BOOTLOADER_INFO_ADDRESS, PAGE_SIZE);
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }
    
    error_code = flash_write((uint32_t*) BOOTLOADER_INFO_ADDRESS, (uint8_t*) bl_info, sizeof(dfu_bootloader_info_t));
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    /* make ram copy */
    memcpy(&g_bl_info, bl_info, sizeof(dfu_bootloader_info_t));

    g_in_dfu = true;
    
    NRF_UICR->BOOTLOADERADDR = BOOTLOADER_START_ADDRESS;
    
    return NRF_SUCCESS;
}

uint32_t dfu_transfer_data(uint32_t address, uint32_t length, uint8_t* data)
{
    if (!g_in_dfu)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (address < g_bl_info.start_addr ||
        address + length > g_bl_info.start_addr + g_bl_info.size)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    /* must be word aligned */
    if ((address & 0x03) != 0)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    if ((length & 0x03) != 0)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    return flash_write((uint32_t*) g_bl_info.bank_addr + (address - g_bl_info.start_addr)/ 4, data, length);
}

uint32_t dfu_end(void)
{
    if (!g_in_dfu)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (g_bl_info.using_crc && !bank_is_valid((uint8_t*) g_bl_info.bank_addr, g_bl_info.size, g_bl_info.image_crc))
    {
        return NRF_ERROR_INVALID_DATA;
    }

    g_in_dfu = false;
    
    if (g_bl_info.dfu_type == DFU_TYPE_BOOTLOADER)
    {
        /* do the copy directly */
        uint32_t error_code;
        error_code = flash_erase(BOOTLOADER_START_ADDRESS, BOOTLOADER_MAX_SIZE);
        if (error_code != NRF_SUCCESS)
        {
            return error_code;
        }
        
        /* copy bootloader from bank */
        return flash_write((uint32_t*) BOOTLOADER_START_ADDRESS, (uint8_t*) g_bl_info.bank_addr, g_bl_info.size);

        /* the device now has a new bootloader! */
    }
    else
    {
        /* Jesus, take the wheel */
        start_bootloader();
    }
    
    return NRF_SUCCESS;
}

