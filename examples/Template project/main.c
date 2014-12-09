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

#include "rbc_mesh.h"

#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
  

/**
* @brief Softdevice crash handler, resets chip
* 
* @param[in] pc Program counter at which the assert failed
* @param[in] line_num Line where the error check failed 
* @param[in] p_file_name File where the error check failed
*/
void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{
    NVIC_SystemReset();
}

/**
* @brief App error handle callback. Called whenever an APP_ERROR_CHECK() fails.
* 
* @param[in] error_code The error code sent to APP_ERROR_CHECK()
* @param[in] line_num Line where the error check failed 
* @param[in] p_file_name File where the error check failed
*/
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    NVIC_SystemReset();
}

/**
* @brief Hardfault handler, resets chip
*/
void HardFault_Handler(void)
{
    NVIC_SystemReset();
}

/**
* @brief Softdevice event handler, forwards SD events to rbc_mesh framework 
*/
void SD_EVT_IRQHandler(void)
{
    rbc_mesh_sd_irq_handler();
}

/**
* @brief RBC_MESH framework event handler. Defined in rbc_mesh.h. Handles
*   events coming from the mesh. 
*
* @param[in] evt RBC event propagated from framework
*/
void rbc_mesh_event_handler(rbc_mesh_event_t* evt)
{
    switch (evt->event_type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
            break;
    }
}


int main(void)
{
    uint32_t error_code = 
        sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_XTAL_75_PPM, sd_assert_handler);
    
    ble_enable_params_t ble_enable_params;
    ble_enable_params.gatts_enable_params.service_changed = 0;
    
    error_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(error_code);
   
    rbc_mesh_init_params_t init_params;
    
    init_params.access_addr = 0xA541A68F;
    init_params.adv_int_ms = 100;
    init_params.channel = 38;
    init_params.handle_count = 1;
    init_params.packet_format = RBC_MESH_PACKET_FORMAT_ORIGINAL;
    init_params.radio_mode = RBC_MESH_RADIO_MODE_BLE_1MBIT;
    
    error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);
    
    
    /* sleep */
    while (true)
    {
        sd_app_evt_wait();
    }
}

