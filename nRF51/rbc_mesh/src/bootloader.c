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

#include "dfu_types.h"
#include "nrf_flash.h"

#include "nrf_sdm.h"
#include "nrf51.h"
#include "bootloader_util.h"
#include "boards.h"
#include <string.h>
#include <stdbool.h>

#define MAX_NUMBER_INTERRUPTS   (32)
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

/* generic application restart. Never returns */
static void exit_bootloader(void)
{   
    sd_softdevice_vector_table_base_set(APP_START_ADDRESS);
    
    bootloader_util_app_start(APP_START_ADDRESS);
}

static void init_leds(void)
{
    nrf_gpio_cfg_output(LED_START);
    NRF_GPIO->OUTCLR = (1 << LED_START);
}
/*****************************************************************************
* Interface Functions
*****************************************************************************/


int main(void)
{
    init_leds();
    get_bootloader_info(&g_bl_info);

    if (g_bl_info.using_crc && !bank_is_valid((uint8_t*) g_bl_info.bank_addr, g_bl_info.size, g_bl_info.image_crc))
    {
        NRF_GPIO->OUTCLR = (1 << LED_START);
        exit_bootloader();
    }
    
    /* do the flash */
    nrf_flash_erase((uint32_t*) g_bl_info.start_addr, g_bl_info.size);
    nrf_flash_store((uint32_t*) g_bl_info.start_addr, (uint8_t*) g_bl_info.bank_addr, g_bl_info.size, 0);
    
    NRF_GPIO->OUTCLR = (1 << LED_START);
    exit_bootloader();
}
