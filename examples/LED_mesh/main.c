#include "rbc_mesh.h"

#include "led_config.h"

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
static void error_loop(void)
{
    SET_PIN(7);
    while (true)
    {
        nrf_delay_ms(500);
        SET_PIN(LED_0);
        CLEAR_PIN(LED_1);
        nrf_delay_ms(500);
        SET_PIN(LED_1);
        CLEAR_PIN(LED_0);
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
    NVIC_SystemReset();
}

/**
* @brief Softdevice event handler 
*/
void SD_EVT_IRQHandler(void)
{
    rbc_mesh_sd_irq_handler();
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
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
        
            if (evt->value_handle > 2)
                break;
            
            led_config(evt->value_handle, evt->data[0]);
            break;
    }
}



#ifdef BOARD_PCA10001
/* configure button interrupt for evkits */
static void gpiote_init(void)
{
    NRF_GPIO->PIN_CNF[BUTTON_0] = (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos)
                                | (GPIO_PIN_CNF_DRIVE_S0S1 << GPIO_PIN_CNF_DRIVE_Pos)
                                | (BUTTON_PULL << GPIO_PIN_CNF_PULL_Pos)
                                | (GPIO_PIN_CNF_INPUT_Connect << GPIO_PIN_CNF_INPUT_Pos)
                                | (GPIO_PIN_CNF_DIR_Input << GPIO_PIN_CNF_DIR_Pos);
    
    NRF_GPIO->PIN_CNF[BUTTON_1] = (GPIO_PIN_CNF_SENSE_Low << GPIO_PIN_CNF_SENSE_Pos)
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

void GPIOTE_IRQHandler(void)
{
    NRF_GPIOTE->EVENTS_PORT = 0;
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (NRF_GPIO->IN & (1 << (BUTTON_0 + i)))
        {
            uint8_t val[28];
            uint16_t len;
            APP_ERROR_CHECK(rbc_mesh_value_get(i + 1, val, &len));
            val[0] = !val[0];
            led_config(i + 1, val[0]);
            APP_ERROR_CHECK(rbc_mesh_value_set(i + 1, &val[0], 1));
        }
    }

}

void test_app_init(void)
{   
    nrf_gpio_cfg_output(LED_0);
    nrf_gpio_cfg_output(LED_1);  
    
#ifdef BOARD_PCA10000
    nrf_gpio_pin_clear(LED_RGB_RED);
    nrf_gpio_pin_clear(LED_RGB_GREEN);
    nrf_gpio_pin_clear(LED_RGB_BLUE);
#endif

#ifdef BOARD_PCA10001 
    nrf_gpio_range_cfg_output(0, 32);
    nrf_gpio_cfg_input(BUTTON_0, BUTTON_PULL);
    nrf_gpio_cfg_input(BUTTON_1, BUTTON_PULL);  
    gpiote_init();
#endif    

    led_config(1, 0);
    led_config(2, 0);
}


int main(void)
{
    uint32_t error_code = 
        sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_XTAL_75_PPM, sd_assert_handler);
    
    test_app_init();
    
    error_code = rbc_mesh_init(0xA541A68F, 38, 2, 100);
    APP_ERROR_CHECK(error_code);
    
    error_code = rbc_mesh_value_req(1);
    APP_ERROR_CHECK(error_code);
    error_code = rbc_mesh_value_req(2);
    APP_ERROR_CHECK(error_code);
    
    
    sd_nvic_EnableIRQ(SD_EVT_IRQn);
    
    /* sleep */
    while (true)
    {
        sd_app_evt_wait();
    }
    

}

