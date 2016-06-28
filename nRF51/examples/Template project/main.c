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
#include "mesh_aci.h"

#include "softdevice_handler.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "boards.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define MESH_ACCESS_ADDR        (0xA541A68F)
#define MESH_INTERVAL_MIN_MS    (100)
#define MESH_CHANNEL            (38)
#define MESH_CLOCK_SRC          (NRF_CLOCK_LFCLKSRC_XTAL_75_PPM)

/** @brief General error handler. */
static inline void error_loop(void)
{
    __disable_irq();
    while (true)
    {
        __WFE();
    }
}

/**
* @brief Softdevice crash handler, never returns
*
* @param[in] pc Program counter at which the assert failed
* @param[in] line_num Line where the error check failed
* @param[in] p_file_name File where the error check failed
*/
void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{
    error_loop();
}

/**
* @brief App error handle callback. Called whenever an APP_ERROR_CHECK() fails.
*   Never returns.
*
* @param[in] error_code The error code sent to APP_ERROR_CHECK()
* @param[in] line_num Line where the error check failed
* @param[in] p_file_name File where the error check failed
*/
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    error_loop();
}

/** @brief Hardware fault handler. */
void HardFault_Handler(void)
{
    error_loop();
}

/**
* @brief Mesh framework event handler.
*
* @param[in] p_evt Mesh event propagated from framework.
*/
static void rbc_mesh_event_handler(rbc_mesh_event_t* p_evt)
{
    switch (p_evt->type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
        case RBC_MESH_EVENT_TYPE_TX:
        case RBC_MESH_EVENT_TYPE_INITIALIZED:
        case RBC_MESH_EVENT_TYPE_DFU_NEW_FW_AVAILABLE:
        case RBC_MESH_EVENT_TYPE_DFU_RELAY_REQ:
        case RBC_MESH_EVENT_TYPE_DFU_SOURCE_REQ:
        case RBC_MESH_EVENT_TYPE_DFU_START:
        case RBC_MESH_EVENT_TYPE_DFU_END:
        case RBC_MESH_EVENT_TYPE_DFU_BANK_AVAILABLE:
            break;
    }
}

int main(void)
{
    /* Enable Softdevice (including sd_ble before framework */
    SOFTDEVICE_HANDLER_INIT(MESH_CLOCK_SRC, NULL);
    softdevice_ble_evt_handler_set(rbc_mesh_ble_evt_handler);

#ifdef RBC_MESH_SERIAL

    /* only want to enable serial interface, and let external host setup the framework */
    mesh_aci_init();
    mesh_aci_start();

#else

    /* Initialize mesh. */
    rbc_mesh_init_params_t init_params;
    init_params.access_addr = MESH_ACCESS_ADDR;
    init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
    init_params.channel = MESH_CHANNEL;
    init_params.lfclksrc = MESH_CLOCK_SRC;
    init_params.tx_power = RBC_MESH_TXPOWER_0dBm;

    uint32_t error_code;
    error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);

    /* Enable handle 1 and 2 */
    error_code = rbc_mesh_value_enable(1);
    APP_ERROR_CHECK(error_code);
    error_code = rbc_mesh_value_enable(2);
    APP_ERROR_CHECK(error_code);
#endif

    rbc_mesh_event_t evt;
    while (true)
    {
        if (rbc_mesh_event_get(&evt) == NRF_SUCCESS)
        {
            rbc_mesh_event_handler(&evt);
            rbc_mesh_event_release(&evt);
        }

        sd_app_evt_wait();
    }
}

