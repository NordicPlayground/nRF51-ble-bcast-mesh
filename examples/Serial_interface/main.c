#include "rbc_mesh.h"

#include "mesh_aci.h"
#include "serial_handler.h"

#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "nrf_gpio.h"
#include "app_error.h"
#include "nrf_delay.h"
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
* @brief General error handler. Sets the LEDs to blink forever
*/
static void error_loop(void)
{
    SET_PIN(7);
    while (true)
    {
        nrf_delay_ms(500);
        SET_PIN(LED_0);
        CLEAR_PIN(LED_1);
        nrf_delay_ms(500);
        CLEAR_PIN(LED_0);
        SET_PIN(LED_1);
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
    serial_evt_t serial_evt;
    switch (evt->event_type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
            serial_evt.opcode = SERIAL_EVT_OPCODE_EVENT_CONFLICTING;
            break;
        
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
            serial_evt.opcode = SERIAL_EVT_OPCODE_EVENT_NEW;
            break;
        
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
            serial_evt.opcode = SERIAL_EVT_OPCODE_EVENT_UPDATE;
            break;
    }
    
    /* opcode + handle + addr type + addr = 9 */
    serial_evt.length = 9 + evt->data_len;
    
    /* all event parameter types are the same, just use event_update for all */
    serial_evt.params.event_update.addr_type = ADDR_TYPE_BLE_GAP_ADV_ADDR;
    memcpy(serial_evt.params.event_update.origin_addr, evt->originator_address.addr, BLE_GAP_ADDR_LEN);
    serial_evt.params.event_update.handle = evt->value_handle;
    memcpy(serial_evt.params.event_update.data, evt->data, evt->data_len);
    
    serial_handler_event_send(&serial_evt);
}


int main(void)
{
    uint32_t error_code = 
        sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_XTAL_75_PPM, sd_assert_handler);
    
    ble_enable_params_t ble_enable_params;
    ble_enable_params.gatts_enable_params.service_changed = 0;
    
    error_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(error_code);
    
    nrf_gpio_cfg_output(LED_0);
    
    mesh_aci_init();
    
    while (true)
    {
        mesh_aci_loop();
    }
}

