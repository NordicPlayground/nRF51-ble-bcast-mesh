#include "rbc_mesh.h"
#include "nrf_adv_conn.h"
#include "led_config.h"
#include "timeslot_handler.h"

#include "nrf_soc.h"
#include "nrf_assert.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "simple_uart.h"
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
static void error_loop(char* message)
{
    SET_PIN(7);
    while (true)
    {
        simple_uart_putstring((uint8_t*) message);
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
    char str[256];
    sprintf(str, "SD ERROR: PC: %d, LINE: %d, FILE: %s\n", 
        pc, 
        line_num, 
        p_file_name);
    
    error_loop(str);
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
    SET_PIN(7);
    
    char str[256];
    sprintf(str, "APP ERROR: CODE: %d, LINE: %d, FILE: %s\n", 
        error_code, 
        line_num, 
        p_file_name);
    
    error_loop(str);
}

void HardFault_Handler(void)
{
    error_loop("HARDFAULT\n");
}

/**
* @brief Softdevice event handler 
*/
void SD_EVT_IRQHandler(void)
{
    rbc_mesh_sd_irq_handler();
    
    ble_evt_t ble_evt;
    uint16_t len = sizeof(ble_evt);
    while (sd_ble_evt_get((uint8_t*) &ble_evt, &len) == NRF_SUCCESS)
    {
        nrf_adv_conn_evt_handler(&ble_evt);
    }
}

/**
* @brief RBC_MESH framework event handler. Defined in rbc_mesh.h. Handles
*   events coming from the mesh. Sets LEDs according to data
*
* @param[in] evt RBC event propagated from framework
*/
void rbc_mesh_event_handler(rbc_mesh_event_t* evt)
{
    TICK_PIN(28);
    switch (evt->event_type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
            /* use new value */
            APP_ERROR_CHECK(rbc_mesh_value_set(
                evt->value_handle, 
                evt->data, 
                evt->data_len));
        
        /* intended fall through */
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
        
            if (evt->value_handle > 1)
                break;
            
            led_config(evt->value_handle, evt->data[0]);
            break;
    }
}


/**
* @brief Initialize GPIO pins, for LEDs and debugging
*/
void gpio_init(void)
{   
    nrf_gpio_cfg_output(LED_0);
    nrf_gpio_cfg_output(LED_1);  
    
#ifdef BOARD_PCA10000
    nrf_gpio_cfg_output(LED_RGB_BLUE);  
    nrf_gpio_pin_set(LED_RGB_RED);
    nrf_gpio_pin_set(LED_RGB_GREEN);
    nrf_gpio_pin_set(LED_RGB_BLUE);
#endif

#ifdef BOARD_PCA10001 
    nrf_gpio_range_cfg_output(0, 32);
#endif    

    led_config(0, 0);
    led_config(1, 0);
}

/** @brief main function */
int main(void)
{
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, true);
    /* Enable Softdevice (including sd_ble before framework */
    uint32_t error_code = 
        sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_XTAL_75_PPM, sd_assert_handler);
    
    ble_enable_params_t ble_enable_params;
    ble_enable_params.gatts_enable_params.service_changed = 0;
    
    error_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(error_code);
    
    /* Enable mesh framework on channel 37, min adv interval at 100ms, 
        2 characteristics */
    error_code = rbc_mesh_init(0x3414A68F, 37, 2, 100);
    APP_ERROR_CHECK(error_code);
    
#if 1
    /* request values for both LEDs on the mesh */
    error_code = rbc_mesh_value_req(0);
    APP_ERROR_CHECK(error_code);
    error_code = rbc_mesh_value_req(1);
    APP_ERROR_CHECK(error_code);
#endif
    /* init leds and pins */
    gpio_init();
    
    /* enable softdevice IRQ */
    sd_nvic_EnableIRQ(SD_EVT_IRQn);
    
    /* init BLE gateway softdevice application: */
    nrf_adv_conn_init();
    
    /* sleep */
    while (true)
    {
        sd_app_evt_wait();
    }
    

}

