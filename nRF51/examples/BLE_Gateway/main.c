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
#ifdef MESH_DFU
#include "dfu_app.h"
#endif
#ifdef BLINKY
#include "blinky.h"
#endif

#include "nrf_adv_conn.h"
#include "led_config.h"
#include "mesh_aci.h"

#include "softdevice_handler.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "boards.h"

#if NORDIC_SDK_VERSION >= 11
#include "nrf_nvic.h"
#endif

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define MESH_ACCESS_ADDR        (RBC_MESH_ACCESS_ADDRESS_BLE_ADV)   /**< Access address for the mesh to operate on. */
#define MESH_INTERVAL_MIN_MS    (100)                               /**< Mesh minimum advertisement interval in milliseconds. */
#define MESH_CHANNEL            (38)                                /**< BLE channel to operate on. Single channel only. */
#if NORDIC_SDK_VERSION >= 11
nrf_nvic_state_t nrf_nvic_state = {0};
static nrf_clock_lf_cfg_t m_clock_cfg = 
{
    .source = NRF_CLOCK_LF_SRC_XTAL,    
    .xtal_accuracy = NRF_CLOCK_LF_XTAL_ACCURACY_75_PPM
};

#define MESH_CLOCK_SOURCE       (m_clock_cfg)    /**< Clock source used by the Softdevice. For calibrating timeslot time. */
#else
#define MESH_CLOCK_SOURCE       (NRF_CLOCK_LFCLKSRC_XTAL_75_PPM)    /**< Clock source used by the Softdevice. For calibrating timeslot time. */
#endif

#ifdef NRF51
#define EXAMPLE_DFU_BANK_ADDR   (0x26000)
#endif
#ifdef NRF52
#define EXAMPLE_DFU_BANK_ADDR   (0x40000)
#endif



/**
* @brief General error handler.
*/
static void error_loop(void)
{
    led_config(0, 1);
    led_config(2, 1);
    led_config(3, 1);
    led_config(1, 1);
    
    __disable_irq(); /* Prevent the mesh from continuing operation. */
    while (true)
    {
        __WFE(); /* sleep */
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

void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info)
{
    error_loop();
}
/**
* @brief Softdevice event handler
*/
void sd_ble_evt_handler(ble_evt_t* p_ble_evt)
{
    rbc_mesh_ble_evt_handler(p_ble_evt);
    nrf_adv_conn_evt_handler(p_ble_evt);
}

/**
* @brief RBC_MESH framework event handler. Defined in rbc_mesh.h. Handles
*   events coming from the mesh. Sets LEDs according to data
*
* @param[in] evt RBC event propagated from framework
*/
static void rbc_mesh_event_handler(rbc_mesh_event_t* p_evt)
{   
    switch (p_evt->type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
            if (p_evt->params.rx.value_handle > 1)
                break;

            led_config(p_evt->params.rx.value_handle, p_evt->params.rx.p_data[0]);
            break;

        case RBC_MESH_EVENT_TYPE_TX:
            break;

        case RBC_MESH_EVENT_TYPE_INITIALIZED:
            /* init BLE gateway softdevice application: */
            nrf_adv_conn_init();
            break;
#ifdef MESH_DFU
        case RBC_MESH_EVENT_TYPE_DFU_BANK_AVAILABLE:
            dfu_bank_flash(p_evt->params.dfu.bank.dfu_type);
            break;

        case RBC_MESH_EVENT_TYPE_DFU_NEW_FW_AVAILABLE:
            dfu_request(p_evt->params.dfu.new_fw.dfu_type,
                &p_evt->params.dfu.new_fw.new_fwid,
                (uint32_t*) EXAMPLE_DFU_BANK_ADDR);
            break;

        case RBC_MESH_EVENT_TYPE_DFU_RELAY_REQ:
            dfu_relay(p_evt->params.dfu.relay_req.dfu_type,
                &p_evt->params.dfu.relay_req.fwid);
            break;

        case RBC_MESH_EVENT_TYPE_DFU_START:
        case RBC_MESH_EVENT_TYPE_DFU_END:
            break;
        case RBC_MESH_EVENT_TYPE_DFU_SOURCE_REQ:
            break;
#endif
        default:
            break;
    }
}


/**
* @brief Initialize GPIO pins, for LEDs and debugging
*/
void gpio_init(void)
{
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);

    for (uint32_t i = 0; i < LEDS_NUMBER; ++i)
    {
        nrf_gpio_pin_set(LED_START + i);
    }

#if defined(BOARD_PCA10001) || defined(BOARD_PCA10028) || defined(BOARD_PCA10040)
    nrf_gpio_range_cfg_output(9, 32);
#endif

#if defined(BOARD_PCA10028) || defined(BOARD_PCA10040)
    #ifdef BUTTONS
        nrf_gpio_cfg_input(BUTTON_1, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_2, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_3, NRF_GPIO_PIN_PULLUP);
        nrf_gpio_cfg_input(BUTTON_4, NRF_GPIO_PIN_PULLUP);
    #endif
#endif

}

/** @brief main function */
int main(void)
{
    /* init leds and pins */
    gpio_init();

    /* Enable Softdevice (including sd_ble before framework */
#ifdef NRF52
    SOFTDEVICE_HANDLER_INIT(&MESH_CLOCK_SOURCE,NULL); 
#else
    SOFTDEVICE_HANDLER_INIT(MESH_CLOCK_SOURCE,NULL); 
#endif    
    
    softdevice_ble_evt_handler_set(sd_ble_evt_handler); /* app-defined event handler, as we need to send it to the nrf_adv_conn module and the rbc_mesh */
    softdevice_sys_evt_handler_set(rbc_mesh_sd_evt_handler);

#ifdef RBC_MESH_SERIAL
    mesh_aci_init();
#endif
    
    
    
    rbc_mesh_init_params_t init_params;

    init_params.access_addr = MESH_ACCESS_ADDR;
    init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
    init_params.channel = MESH_CHANNEL;
    init_params.lfclksrc = MESH_CLOCK_SOURCE;
    init_params.tx_power = RBC_MESH_TXPOWER_0dBm ;
    
    uint32_t error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);

    /* request values for both LEDs on the mesh */
    for (uint32_t i = 0; i < 2; ++i)
    {
        error_code = rbc_mesh_value_enable(i);
        APP_ERROR_CHECK(error_code);
    }
    /* init BLE gateway softdevice application: */
    nrf_adv_conn_init();
    
#ifdef RBC_MESH_SERIAL
    APP_ERROR_CHECK(mesh_aci_start());
#endif

    
#ifdef BLINKY
    
    led_init ();
    rtc_1_init ();
    start_blink_interval_s(1);
    
#endif
    
    
    
    rbc_mesh_event_t evt;
    while (true)
    {
#ifdef BUTTONS
        for (uint32_t pin = BUTTON_START; pin <= BUTTON_STOP; ++pin)
        {
            if(nrf_gpio_pin_read(pin) == 0)
            {
                while(nrf_gpio_pin_read(pin) == 0);
                uint8_t mesh_data[1];
                uint32_t led_status = !!((pin - BUTTON_START) & 0x01); /* even buttons are OFF, odd buttons are ON */
                uint32_t led_offset = !!((pin - BUTTON_START) & 0x02); /* two buttons per led */

                mesh_data[0] = led_status;
                if (rbc_mesh_value_set(led_offset, mesh_data, 1) == NRF_SUCCESS)
                {
                    led_config(led_offset, led_status);
                }
            }
        }
#endif

       
        
          

        if (rbc_mesh_event_get(&evt) == NRF_SUCCESS)
        {   
            rbc_mesh_event_handler(&evt);
            rbc_mesh_event_release(&evt);
        }
    }
}

