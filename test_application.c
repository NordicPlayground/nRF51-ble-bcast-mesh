#include "test_application.h"

#include "rebroadcast.h"
#include "nrf_adv_conn.h"
#include "trickle_common.h"

#include "nrf_soc.h"
#include "app_error.h"
#include "nrf_assert.h"
#include "nrf_sdm.h"
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

/* Aliases for LEDs */
#ifdef BOARD_PCA10000
    #define LED_0 LED_RGB_RED
    #define LED_1 LED_RGB_BLUE
#endif


static uint8_t led_data;



void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{
    
    SET_PIN(PIN_ABORTED);
    uint8_t* name_ptr = (uint8_t*) p_file_name;
    while (*name_ptr != '\0')
    {
        PIN_OUT(name_ptr[0], 8);
        
        ++name_ptr;
        TICK_PIN(0);
        TICK_PIN(0);
        TICK_PIN(0);
        TICK_PIN(0);
        TICK_PIN(0);
        TICK_PIN(0);
        TICK_PIN(0);
        TICK_PIN(0);
    }
    while (true)
    {
        PIN_OUT(line_num, 16);
        nrf_delay_ms(500);
        SET_PIN(LED_0);
        CLEAR_PIN(LED_1);
        nrf_delay_ms(500);
        SET_PIN(LED_1);
        CLEAR_PIN(LED_0);
    }
}

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{

    SET_PIN(PIN_ABORTED);
    while (true)
    {
        PIN_OUT(error_code, 32);
        nrf_delay_ms(500);
        SET_PIN(LED_0);
        CLEAR_PIN(LED_1);
        nrf_delay_ms(500);
        SET_PIN(LED_1);
        CLEAR_PIN(LED_0);
    }
}

void SD_IRQHandler(void)
{
    rbc_sd_irq_handler();
    
    ble_evt_t ble_evt;
    uint16_t len = sizeof(ble_evt);
    while (sd_ble_evt_get((uint8_t*) &ble_evt, &len))
    {
        nrf_adv_conn_evt_handler(&ble_evt);
    }
}

void rbc_event_handler(rbc_event_t* evt)
{
    TICK_PIN(28);
    switch (evt->event_type)
    {
        case RBC_EVENT_TYPE_NEW_VAL:
            break;
        
        case RBC_EVENT_TYPE_UPDATE_VAL:
            break;
        
        case RBC_EVENT_TYPE_DELETE_VAL:
            break;
        
        case RBC_EVENT_TYPE_CONFLICTING_VAL:
            break;
        
        case RBC_EVENT_TYPE_DISCARDED_VAL:
            break;
    }
}


static void led_config(void)
{
    if ((led_data & 0x01) > 0)
    {
        nrf_gpio_pin_set(LED_0);
    }
    else
    {
        nrf_gpio_pin_clear(LED_0);
    }
    
    if ((led_data & 0x02) > 0)
    {
        nrf_gpio_pin_set(LED_1);
    }
    else
    {
        nrf_gpio_pin_clear(LED_1);
    }
} 

#if BOARD_PCA10003
/* configure button interrupt for evkits */
static void gpiote_init(void)
{
	NRF_GPIOTE->CONFIG[0] = 	GPIOTE_CONFIG_MODE_Event 		<< GPIOTE_CONFIG_MODE_Pos       |
                                GPIOTE_CONFIG_POLARITY_HiToLo 	<< GPIOTE_CONFIG_POLARITY_Pos   |
                                BUTTON_0						<< GPIOTE_CONFIG_PSEL_Pos;
    
	NRF_GPIOTE->CONFIG[1] = 	GPIOTE_CONFIG_MODE_Event 		<< GPIOTE_CONFIG_MODE_Pos       |
                                GPIOTE_CONFIG_POLARITY_HiToLo 	<< GPIOTE_CONFIG_POLARITY_Pos   |
                                BUTTON_1						<< GPIOTE_CONFIG_PSEL_Pos;
    
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk | GPIOTE_INTENSET_IN1_Msk;
    //NVIC_EnableIRQ(GPIOTE_IRQn);
}
#endif

#if BOARD_PCA10003
void GPIOTE_IRQHandler(void)
{
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (NRF_GPIOTE->EVENTS_IN[i])
        {
            NRF_GPIOTE->EVENTS_IN[i] = 0; 
            
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

#ifdef BOARD_PCA10003 
    nrf_gpio_range_cfg_output(0, 32);
    nrf_gpio_cfg_input(BUTTON_0, BUTTON_PULL);
    nrf_gpio_cfg_input(BUTTON_1, BUTTON_PULL);  
    gpiote_init();
#endif    

    led_data = 0;
    led_config();
}


int main(void)
{
    test_app_init();
    uint32_t error_code = 
        sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_XTAL_250_PPM, sd_assert_handler);
    
    ble_enable_params_t ble_enable_params;
    ble_enable_params.gatts_enable_params.service_changed = 0;
    
    error_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(error_code);
    
    error_code = rbc_init(RBC_ACCESS_ADDRESS_BLE_ADV, 37, 2, 100);
    APP_ERROR_CHECK(error_code);
    
    uint8_t data[] = {0, 1, 2, 3, 4, 5};
    
    error_code = rbc_value_set(1, data, 6);
    APP_ERROR_CHECK(error_code);
    
    
    /* dummy connectable advertiser softdevice application: */
    //nrf_adv_conn_init();
    
    /* sleep */
    while (true)
    {
#if USE_SOFTDEVICE        
        sd_app_evt_wait();
#else
        //_WFE();
#endif        
    }
    

}

