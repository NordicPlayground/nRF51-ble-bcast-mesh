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

#include "led_config.h"
#include "mesh_aci.h"

#include "nrf_soc.h"
#include "softdevice_handler.h"
#include "nrf_assert.h"
#include "nrf_sdm.h"
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


/**
* @brief General error handler.
*/
static void error_loop(void)
{
    led_config(3, 1);
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
void assert_nrf_callback(uint16_t line_num, const uint8_t *file_name)
{
    error_loop();
}
/**
* @brief RBC_MESH framework event handler. Defined in rbc_mesh.h. Handles
*   events coming from the mesh. Sets LEDs according to data
*
* @param[in] evt RBC event propagated from framework
*/
void rbc_mesh_event_handler(rbc_mesh_event_t* evt)
{
    TICK_PIN(18);
    CLEAR_PIN(15);
    switch (evt->event_type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
            if (evt->value_handle > 2)
                break;
            led_config(evt->value_handle, evt->data[0]);
            break;
        case RBC_MESH_EVENT_TYPE_INITIALIZED:
            break;
    }
}



#if defined(BOARD_PCA10001) || defined(BOARD_PCA10028)
/* configure button interrupt for evkits */
static void gpiote_init(void)
{
    NRF_GPIO->PIN_CNF[BUTTON_START] = (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos)
                                | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                | (BUTTON_PULL << GPIO_PIN_CNF_PULL_Pos)
                                | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
    
    NRF_GPIO->PIN_CNF[BUTTON_START + 1] = (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos)
                                | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                | (BUTTON_PULL << GPIO_PIN_CNF_PULL_Pos)
                                | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
    

    /* GPIOTE interrupt handler normally runs in STACK_LOW priority, need to put it 
    in APP_LOW in order to use the mesh API */
    NVIC_SetPriority(GPIOTE_IRQn, 3);
    
    NVIC_EnableIRQ(GPIOTE_IRQn);
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
}

#endif

#if defined(BOARD_PCA10001) || defined(BOARD_PCA10028)
#ifndef RBC_MESH_SERIAL
void GPIOTE_IRQHandler(void)
{
    NRF_GPIOTE->EVENTS_PORT = 0;
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (NRF_GPIO->IN & (1 << (BUTTON_START + i)))
        {
            uint8_t val[28];
            uint16_t len;
            APP_ERROR_CHECK(rbc_mesh_value_get(i + 1, val, &len, NULL));
            val[0] = !val[0];
            led_config(i + 1, val[0]);
            APP_ERROR_CHECK(rbc_mesh_value_set(i + 1, &val[0], 1));
        }
    }

}
#endif
#endif

void test_app_init(void)
{   
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);
    
    for (uint32_t i = 0; i < LEDS_NUMBER; ++i)
    {
        nrf_gpio_pin_set(LED_START + i);
    }
    nrf_gpio_range_cfg_output(0, 32);
#if defined(BOARD_PCA10001) || defined(BOARD_PCA10028)
    
    nrf_gpio_range_cfg_input(BUTTON_START, BUTTON_STOP, BUTTON_PULL);
    gpiote_init();
#endif    

    led_config(0, 0);
    led_config(1, 0);
    led_config(2, 0);
}


int main(void)
{
    NRF_POWER->RESET = 1;
    SOFTDEVICE_HANDLER_INIT(NRF_CLOCK_LFCLKSRC_XTAL_75_PPM, rbc_mesh_sd_irq_handler);
    test_app_init();
    
#ifdef RBC_MESH_SERIAL
    mesh_aci_init();
#else    
    
    rbc_mesh_init_params_t init_params;

    init_params.access_addr = 0xA541A68F;
    init_params.adv_int_ms = 100;
    init_params.channel = 38;
    init_params.handle_count = 2;
    init_params.packet_format = RBC_MESH_PACKET_FORMAT_ORIGINAL;
    init_params.radio_mode = RBC_MESH_RADIO_MODE_BLE_1MBIT;
    
    volatile uint32_t error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);
    
    error_code = rbc_mesh_value_enable(1);
    APP_ERROR_CHECK(error_code);
    error_code = rbc_mesh_value_enable(2);
    APP_ERROR_CHECK(error_code);
#endif    
    
    sd_nvic_EnableIRQ(SD_EVT_IRQn);
    
    /* sleep */
    while (true)
    {
        sd_app_evt_wait();
    }
    

}

