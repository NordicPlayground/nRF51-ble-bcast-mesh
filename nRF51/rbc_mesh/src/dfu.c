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

#include "nrf51.h"
#include <stdbool.h>

/*****************************************************************************
* Static globals
*****************************************************************************/
static dfu_bootloader_info_t g_bl_info;
/*****************************************************************************
* Static Functions
*****************************************************************************/
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
        if (interrupt_setting_mask & (IRQ_ENABLED << irq))
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

    err_code = sd_softdevice_vector_table_base_set(NRF_UICR->BOOTLOADERADDR);
    APP_ERROR_CHECK(err_code);

    interrupts_disable();
    bootloader_util_app_start(NRF_UICR->BOOTLOADERADDR);
}
/*****************************************************************************
* Interface Functions
*****************************************************************************/

uint32_t dfu_transfer_begin(dfu_bootloader_info_t* bl_info)
{
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
    if (old_bl_info.app_id == bl_info->app_id)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    /* erase bank */ 
    nrf_flash_erase((uint32_t*) bl_info->bank_addr, bl_info->size);

    /* store bootloader info in last page */
    nrf_flash_erase((uint32_t*) BOOTLOADER_INFO_ADDRESS, PAGE_SIZE);
    nrf_flash_store((uint32_t*) BOOTLOADER_INFO_ADDRESS, bl_info, sizeof(dfu_bootloader_info_t), 0);

    /* make ram copy */
    memcpy(&g_bl_info, bl_info, sizeof(dfu_bootloader_info_t));

    return NRF_SUCCESS;
}

uint32_t dfu_transfer_data(uint32_t address, uint32_t length, uint8_t* data)
{
    if (address < g_bl_info.start_addr ||
        address + length > g_bl_info.start_addr + g_bl_info.size)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    /* must be word aligned */
    if (address & 0x03 != 0)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    if (length & 0x03 != 0)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    nrf_flash_store((uint32_t*) g_bl_info.bank_addr, data, length, address - g_bl_info.start_addr);

    return NRF_SUCCESS;
}

uint32_t dfu_end(void)
{
    if (!bank_is_valid((uint8_t*) g_bl_info.bank_addr, g_bl_info.size, g_bl_info.image_crc))
    {
        return NRF_ERROR_INVALID_DATA;
    }

    if (g_bl_info.dfu_type == DFU_TYPE_BOOTLOADER)
    {
        /* do the copy directly */
        nrf_flash_erase((uint32_t*) BOOTLOADER_START_ADDRESS, BOOTLOADER_MAX_SIZE);
        nrf_flash_store((uint32_t*) BOOTLOADER_START_ADDRESS, (uint8_t*) g_bl_info.bank_addr, g_bl_info.size, 0);

        /* the device now has a new bootloader! */
    }
    else
    {
        /* Jesus, take the wheel */
        start_bootloader();
    }

}

