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

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "bootloader_mesh.h"
#include "bootloader_info.h"
#include "bootloader_rtc.h"
#include "dfu_types_mesh.h"
#include "rbc_mesh.h"
#include "transport.h"
#include "mesh_aci.h"

#include "app_error.h"
#include "nrf_gpio.h"

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    __disable_irq();
    NRF_GPIO->OUTSET = (1 << 7);
    NRF_GPIO->OUTCLR = (1 << 23);
    __BKPT(0);
    while (1);
}

void HardFault_Handler(uint32_t pc, uint32_t lr)
{
    NRF_GPIO->OUTSET = (1 << 7);
    NRF_GPIO->OUTCLR = (1 << 23);
    __BKPT(0);
}

/** Interrupt indicating new serial command */
void SWI2_IRQHandler(void)
{
    mesh_aci_command_check();
}

static void rx_cb(mesh_packet_t* p_packet)
{
    mesh_adv_data_t* p_adv_data = mesh_packet_adv_data_get(p_packet);
    if (p_adv_data->handle > RBC_MESH_APP_MAX_HANDLE)
    {
        NRF_GPIO->OUTCLR = (1 << 21);
        bootloader_rx((dfu_packet_t*) &p_adv_data->handle, p_adv_data->adv_data_length - 3, false);
        NRF_GPIO->OUTSET = (1 << 21);
    }
}

static void init_leds(void)
{
    nrf_gpio_range_cfg_output(0, 32);
    NRF_GPIO->OUTCLR = 0xFFFFFFFF;
    NRF_GPIO->OUTSET = (1 << 21) | (1 << 22) | (1 << 23) | (1 << 24);
}

int main(void)
{
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    while (!NRF_CLOCK->EVENTS_HFCLKSTARTED);
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    while (!NRF_CLOCK->EVENTS_LFCLKSTARTED);
    
    NRF_CLOCK->TASKS_CAL = 1;
    while (!NRF_CLOCK->EVENTS_DONE);
    
    NVIC_SetPriority(SWI2_IRQn, 2);
    NVIC_EnableIRQ(SWI2_IRQn);
    
    init_leds();
    rtc_init();
#ifdef SERIAL    
    mesh_aci_init();
#endif    
    transport_init(rx_cb, RBC_MESH_ACCESS_ADDRESS_BLE_ADV);
    bootloader_info_init((uint32_t*) BOOTLOADER_INFO_ADDRESS, (uint32_t*) (BOOTLOADER_INFO_ADDRESS - PAGE_SIZE));
    bootloader_init();

    while (1)
    {
        __WFE();
    }
}
