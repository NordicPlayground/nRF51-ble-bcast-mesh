#include "test_application.h"

#include "rebroadcast.h"
#include "nrf_adv_conn.h"

#include "nrf_soc.h"
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

static void app_mesh_event_handler(rbc_event_t* evt)
{
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
    rbc_init(app_mesh_event_handler);
    
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

