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
#include "bootloader_app.h"

#include "nrf_adv_conn.h"
#include "led_config.h"
#include "mesh_aci.h"

#include "softdevice_handler.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "boards.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

/* Debug macros for debugging with logic analyzer */
#define SET_PIN(x) NRF_GPIO->OUTSET = (1 << (x))
#define CLEAR_PIN(x) NRF_GPIO->OUTCLR = (1 << (x))
#define TICK_PIN(x) do { SET_PIN((x)); CLEAR_PIN((x)); }while(0)

#define MESH_ACCESS_ADDR        (RBC_MESH_ACCESS_ADDRESS_BLE_ADV)
#define MESH_INTERVAL_MIN_MS    (100)
#define MESH_CHANNEL            (38)
#define MESH_CLOCK_SOURCE       (NRF_CLOCK_LFCLKSRC_XTAL_75_PPM)

/**
* @brief General error handler.
*/
static void error_loop(void)
{
    led_config(2, 1);
    led_config(3, 0);
    while (true)
    {
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

void HardFault_Handler(void)
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
static void rbc_mesh_event_handler(rbc_mesh_event_t* evt)
{
    TICK_PIN(28);
    switch (evt->event_type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
            if (evt->value_handle > 1)
                break;

            led_config(evt->value_handle, evt->data[0]);
            break;
        case RBC_MESH_EVENT_TYPE_TX:
            break;
        case RBC_MESH_EVENT_TYPE_INITIALIZED:
            /* init BLE gateway softdevice application: */
            nrf_adv_conn_init();
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

#if defined(BOARD_PCA10001) || defined(BOARD_PCA10028)
    nrf_gpio_range_cfg_output(0, 32);
#endif

#ifdef BOARD_PCA10028
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
    NRF_GPIO->OUTSET = (1 << 4);
    /* Enable Softdevice (including sd_ble before framework */
    SOFTDEVICE_HANDLER_INIT(MESH_CLOCK_SOURCE, NULL);
    softdevice_ble_evt_handler_set(sd_ble_evt_handler); /* app-defined event handler, as we need to send it to the nrf_adv_conn module and the rbc_mesh */
    softdevice_sys_evt_handler_set(rbc_mesh_sd_evt_handler);

#ifdef RBC_MESH_SERIAL

    /* only want to enable serial interface, and let external host setup the framework */
    mesh_aci_init();

#else

    rbc_mesh_init_params_t init_params;

    init_params.access_addr = MESH_ACCESS_ADDR;
    init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
    init_params.channel = MESH_CHANNEL;
    init_params.lfclksrc = MESH_CLOCK_SOURCE;

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

#endif
    NRF_GPIO->OUTCLR = (1 << 4);

#if !(defined(BUTTONS))
    /* fetch events */
    rbc_mesh_event_t evt;
    while (true)
    {
        if (rbc_mesh_event_get(&evt) == NRF_SUCCESS)
        {
            rbc_mesh_event_handler(&evt);
            rbc_mesh_packet_release(evt.data);
        }

        sd_app_evt_wait();
    }
#else
    uint8_t mesh_data[16] = {0,0};
    rbc_mesh_event_t evt;
    while (true)
    {
        // red off
        if(nrf_gpio_pin_read(BUTTON_1) == 0)
        {
            while(nrf_gpio_pin_read(BUTTON_1) == 0);
            mesh_data[0] = 0;
            rbc_mesh_value_set(0, mesh_data, 1);
            led_config(0, 0);
        }
        // red on
        if(nrf_gpio_pin_read(BUTTON_2) == 0)
        {
            while(nrf_gpio_pin_read(BUTTON_2) == 0);
            mesh_data[0] = 1;
            rbc_mesh_value_set(0, mesh_data, 1);
            led_config(0, 1);
        }
        // green off
        if(nrf_gpio_pin_read(BUTTON_3) == 0)
        {
            while(nrf_gpio_pin_read(BUTTON_3) == 0);
            mesh_data[0] = 0;
            rbc_mesh_value_set(1, mesh_data, 1);
            led_config(1, 0);
        }
        // green on
        if(nrf_gpio_pin_read(BUTTON_4) == 0)
        {
            while(nrf_gpio_pin_read(BUTTON_4) == 0);
            mesh_data[0] = 1;
            rbc_mesh_value_set(1, mesh_data, 1);
            led_config(1, 1);
        }

        if (rbc_mesh_event_get(&evt) == NRF_SUCCESS)
        {
            rbc_mesh_event_handler(&evt);
            rbc_mesh_packet_release(evt.data);
        }
    }
#endif

}

