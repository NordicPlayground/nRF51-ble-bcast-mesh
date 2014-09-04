#include "radio_control.h"
#include "trickle.h"
#include "trickle_common.h"


#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "simple_uart.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define ADDR_OFFSET 3
#define LENGTH_OFFSET 1
#define MESSAGE_TYPE_OFFSET 11
#define TRICKLE_ID_OFFSET 12
#define LED_CONFIG_OFFSET 13

/* Aliases for LEDs */
#ifdef BOARD_PCA10000
    #define LED_0 LED_RGB_RED
    #define LED_1 LED_RGB_BLUE
#endif




uint8_t address[6];
trickle_t trickle;
uint8_t rx_data[256];

uint8_t tx_data[] = 
{
    0x02, /* ADV_NONCONN_IND */
    0x14, /* LENGTH */
    0x00, /* PADDING */
    0x00, 0xCE, 0xAB, 0x48, 0xDE, 0xAC, /* ADDRESS */
    0x04, /* ADV_DATA LENGTH */
    0xFF, /* MANUFACTURER SPECIFIC */
    MESSAGE_TYPE_TRICKLE_LED_CONFIG, /* MESSAGE_TYPE */
    0x00, /* TRICKLE_ID */
    0x00,  /* LED CONFIG */
    0x08, /* LENGTH */
    0x09, /* DEVICE NAME */
    'T', 'r', 'i', 'c', 'k', 'l', 'e' /* DEVICE NAME */
};
    

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    SET_PIN(PIN_ABORTED);
    while (true);
}


   
static void led_config(uint8_t led_data)
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
    
    char buf[256];
    
    sprintf(&buf[0], "LED CHANGED: 0x%X\n", led_data & 0x03);
    
    simple_uart_putstring((uint8_t*) buf);
} 


static void make_address(void)
{
    uint64_t addr = (((uint64_t) NRF_FICR->DEVICEADDR[0] << 32) | NRF_FICR->DEVICEADDR[1]);
    for (uint8_t i = 0; i < 6; ++i)
    {
        address[i] = (addr >> i * 8) & 0xFF;
    }
    memcpy(&tx_data[ADDR_OFFSET], address, 6);
}

void trickle_tx(void)
{
    radio_tx(tx_data);
    TICK_PIN(PIN_TRICKLE_TX);
}


void eval_msg(uint8_t* data)
{
    TICK_PIN(PIN_RX);
    if (NRF_RADIO->CRCSTATUS == 0)
    {
        return;
    }
    
    /* check if the radio was disabled while trying to receive */
    if (radio_aborted)
    {
        radio_aborted = 0;
        TICK_PIN(PIN_ABORTED);
        return;
    }
    
    uint8_t length = data[1];
    if (length != 0x14)
    {
        TICK_PIN(PIN_ABORTED);
        return;
    }
    uint8_t adv_type = data[0];
    if (adv_type != 2)
    {
        TICK_PIN(PIN_ABORTED);
        return;
    }
    uint8_t msg_type = data[MESSAGE_TYPE_OFFSET];
    if (msg_type != MESSAGE_TYPE_TRICKLE_LED_CONFIG)
    {
        TICK_PIN(PIN_ABORTED);
        return;
    }
    
    uint8_t trickle_id = data[TRICKLE_ID_OFFSET];
    uint8_t led_configuration = data[LED_CONFIG_OFFSET];
    
    if (trickle_id == tx_data[TRICKLE_ID_OFFSET])
    {
        trickle_rx_consistent(&trickle);
        TICK_PIN(PIN_CONSISTENT);
    }
    else if (msg_type == MESSAGE_TYPE_TRICKLE_LED_CONFIG)
    {
        if (trickle_id >= tx_data[TRICKLE_ID_OFFSET] + 1)
        {
            tx_data[TRICKLE_ID_OFFSET] = trickle_id;
            tx_data[LED_CONFIG_OFFSET] = led_configuration;
            led_config(tx_data[LED_CONFIG_OFFSET]);
            
        }
        
        trickle_rx_inconsistent(&trickle);
        TICK_PIN(PIN_INCONSISTENT);
    }
}

#ifdef BOARD_PCA10003
/* configure button interrupt for evkits */
static void gpiote_init(void)
{
	NRF_GPIOTE->CONFIG[0] = 	GPIOTE_CONFIG_MODE_Event 		<< GPIOTE_CONFIG_MODE_Pos       |
                                GPIOTE_CONFIG_POLARITY_HiToLo 	<< GPIOTE_CONFIG_POLARITY_Pos   |
                                BUTTON_0						<< GPIOTE_CONFIG_PSEL_Pos;
    
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk;
    NVIC_EnableIRQ(GPIOTE_IRQn);
}
#endif

void GPIOTE_IRQHandler(void)
{
#ifdef BOARD_PCA10003 
    
    if (NRF_GPIOTE->EVENTS_IN[0]) /* button 0 */
    {
        NRF_GPIOTE->EVENTS_IN[0] = 0;
        ++tx_data[TRICKLE_ID_OFFSET];
        ++tx_data[LED_CONFIG_OFFSET];
        led_config(tx_data[LED_CONFIG_OFFSET]);
        trickle_timer_reset(&trickle);
        TICK_PIN(PIN_BUTTON);
    }
#endif    
}



int main(void)
{
    SET_PIN(PIN_CPU_IN_USE);
    make_address();
    simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, true);
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    radio_init();
	//SCB->SCR |= SCB_SCR_SEVONPEND_Msk;  /* Enable Event on pending interrupt */
    trickle.i_max = 2000; /* max 200 seconds (100ms * 20) */
    trickle.i_min = 100; /* min 100ms */
    trickle.i = 400;
    trickle.k = 3;
    trickle.tx_cb = &trickle_tx;
    
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

    led_config(0);

    trickle_init(&trickle);
    radio_rx(rx_data);
    
    CLEAR_PIN(PIN_CPU_IN_USE);
    /* sleep */
    while (true)
    {
        __WFE();        
    }
    

}
