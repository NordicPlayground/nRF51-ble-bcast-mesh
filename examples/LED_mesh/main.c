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

#define ADDR_OFFSET             3
#define LENGTH_OFFSET           1
#define MESSAGE_TYPE_OFFSET     11
#define TRICKLE_ID_OFFSET       12
#define TRICKLE_VERSION_OFFSET  13
#define LED_CONFIG_OFFSET       14

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
            
            #ifdef BOARD_PCA10000
                CLEAR_PIN(LED_RGB_BLUE);
                nrf_delay_ms(20);
                SET_PIN(LED_RGB_BLUE);
            #endif
            break;
    }
}



#ifdef BOARD_PCA10001
/* configure button interrupt for evkits */
static void gpiote_init(void)
{
    
    NRF_GPIOTE->POWER = 0;
    NRF_GPIOTE->POWER = 1;

    
	NRF_GPIOTE->CONFIG[0] = 	GPIOTE_CONFIG_MODE_Event 		<< GPIOTE_CONFIG_MODE_Pos       |
                                GPIOTE_CONFIG_POLARITY_HiToLo 	<< GPIOTE_CONFIG_POLARITY_Pos   |
                                BUTTON_0						<< GPIOTE_CONFIG_PSEL_Pos;
    
	NRF_GPIOTE->CONFIG[1] = 	GPIOTE_CONFIG_MODE_Event 		<< GPIOTE_CONFIG_MODE_Pos       |
                                GPIOTE_CONFIG_POLARITY_Toggle   << GPIOTE_CONFIG_POLARITY_Pos   |
                                BUTTON_1						<< GPIOTE_CONFIG_PSEL_Pos;
    
    NRF_GPIOTE->INTENCLR = 0;
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk;
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN1_Msk;
    __NOP();
    __NOP();
    __NOP();
    NRF_GPIOTE->EVENTS_IN[0] = 0;
    NRF_GPIOTE->EVENTS_IN[1] = 0;
    sd_nvic_ClearPendingIRQ(GPIOTE_IRQn);
    sd_nvic_EnableIRQ(GPIOTE_IRQn);
}

void GPIOTE_IRQHandler(void)
{
    sd_nvic_ClearPendingIRQ(GPIOTE_IRQn);
    uint8_t val[] = {nrf_gpio_pin_read(BUTTON_0), nrf_gpio_pin_read(BUTTON_1)};
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (NRF_GPIOTE->EVENTS_IN[i])
        {
            NRF_GPIOTE->EVENTS_IN[i] = 0; 
            led_config(i, val[i]);
            APP_ERROR_CHECK(rbc_mesh_value_set(i, &val[i], 1));
        }
    }
}

#endif



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

    led_config(0, 0);
    led_config(1, 0);
}


int main(void)
{
    uint32_t error_code = 
        sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_XTAL_75_PPM, sd_assert_handler);
    
    ble_enable_params_t ble_enable_params;
    ble_enable_params.gatts_enable_params.service_changed = 0;
    
    error_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(error_code);
    
    error_code = rbc_mesh_init(0x3414A68F, 37, 2, 100);
    APP_ERROR_CHECK(error_code);
    
    error_code = rbc_mesh_value_req(0);
    APP_ERROR_CHECK(error_code);
    error_code = rbc_mesh_value_req(1);
    APP_ERROR_CHECK(error_code);
    
    test_app_init();
    
    sd_nvic_EnableIRQ(SD_EVT_IRQn);
    
    /* sleep */
    while (true)
    {
        sd_app_evt_wait();
#ifdef BOARD_PCA10001        
        /*if (NRF_GPIOTE->EVENTS_IN[0] || NRF_GPIOTE->EVENTS_IN[1])
        {
            sd_nvic_ClearPendingIRQ(GPIOTE_IRQn);
            uint8_t val[] = {nrf_gpio_pin_read(BUTTON_0), nrf_gpio_pin_read(BUTTON_1)};
            for (uint8_t i = 0; i < 2; ++i)
            {
                if (NRF_GPIOTE->EVENTS_IN[i])
                {
                    NRF_GPIOTE->EVENTS_IN[i] = 0; 
                    led_config(i, val[i]);
                    APP_ERROR_CHECK(rbc_mesh_value_set(i, &val[i], 1));
                }
            }
        }*/
#endif
    }
    

}

