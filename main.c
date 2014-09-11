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
#define TRICKLE_VERSION_OFFSET 13
#define LED_CONFIG_OFFSET 14

/* Aliases for LEDs */
#ifdef BOARD_PCA10000
    #define LED_0 LED_RGB_RED
    #define LED_1 LED_RGB_BLUE
#endif




static uint8_t address[6];
static trickle_t trickle[2];
static uint8_t payload[2];
static uint8_t version[2];
static uint8_t led_data;

uint8_t rx_data[256];

/* TX DATA WITH ONE BYTE PAYLOAD */
uint8_t tx_data[] = 
{
    0x02, /* ADV_NONCONN_IND */
    0x15, /* LENGTH */
    0x00, /* PADDING */
    0x00, 0xCE, 0xAB, 0x48, 0xDE, 0xAC, /* ADDRESS */
    
    0x04, /* ADV_DATA LENGTH */
    0xFF, /* MANUFACTURER SPECIFIC */
    MESSAGE_TYPE_TRICKLE_LED_CONFIG, /* MESSAGE TYPE */
    0x00, /* TRICKLE_ID */
    0x00,  /* TRICKLE_VERSION */
    0x00,  /* LED CONFIG (PAYLOAD) */
    
    0x08, /* LENGTH */
    0x09, /* DEVICE NAME */
    'T', 'r', 'i', 'c', 'k', 'l', 'e' /* DEVICE NAME */
};
    

void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    SET_PIN(PIN_ABORTED);
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

static void set_tx_trickle_data(uint8_t trickle_id, uint8_t payload, uint8_t version)
{
    tx_data[TRICKLE_ID_OFFSET] = trickle_id;
    tx_data[LED_CONFIG_OFFSET] = payload;
    tx_data[TRICKLE_VERSION_OFFSET] = version;
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
    
    //char buf[256];
    
    //sprintf(&buf[0], "LED CHANGED: 0x%X\n", led_data & 0x03);
    
    //simple_uart_putstring((uint8_t*) buf);
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

void trickle_tx0(void)
{
    SET_PIN(PIN_TX0);
    set_tx_trickle_data(trickle[0].id, payload[0], version[0]);
    radio_tx(tx_data);
    TICK_PIN(PIN_TRICKLE_TX);
    CLEAR_PIN(PIN_TX0);
}

void trickle_tx1(void)
{
    SET_PIN(PIN_TX1);
    set_tx_trickle_data(trickle[1].id, payload[1], version[1]);
    radio_tx(tx_data);
    TICK_PIN(PIN_TRICKLE_TX);
    CLEAR_PIN(PIN_TX1);
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
    if (length != 0x15)
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
    uint8_t curr_version = data[TRICKLE_VERSION_OFFSET];
    uint8_t led_configuration = data[LED_CONFIG_OFFSET];
    
    if (curr_version == version[trickle_id])
    {
        trickle_rx_consistent(&trickle[trickle_id]);
        TICK_PIN(PIN_CONSISTENT);
    }
    else if (msg_type == MESSAGE_TYPE_TRICKLE_LED_CONFIG)
    {
        /* version 0 is not repeated as the number overflows, is treated as a special case */
        /* try to adapt to overflow */
        if (curr_version >= version[trickle_id] + 1 || version[trickle_id] == 0 || (curr_version < 64 && version[trickle_id] > 192))
        {
            version[trickle_id] = curr_version;
            
            led_data &= ~(1 << trickle_id);
            led_data |= ((led_configuration & 0x01) << trickle_id);
            
            led_config();
        }
        
        trickle_rx_inconsistent(&trickle[trickle_id]);
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
    
	NRF_GPIOTE->CONFIG[1] = 	GPIOTE_CONFIG_MODE_Event 		<< GPIOTE_CONFIG_MODE_Pos       |
                                GPIOTE_CONFIG_POLARITY_HiToLo 	<< GPIOTE_CONFIG_POLARITY_Pos   |
                                BUTTON_1						<< GPIOTE_CONFIG_PSEL_Pos;
    
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk | GPIOTE_INTENSET_IN1_Msk;
    NVIC_EnableIRQ(GPIOTE_IRQn);
}
#endif

#ifdef BOARD_PCA10003 
void GPIOTE_IRQHandler(void)
{
    for (uint8_t i = 0; i < 2; ++i)
    {
        if (NRF_GPIOTE->EVENTS_IN[i])
        {
            NRF_GPIOTE->EVENTS_IN[i] = 0; 
            ++payload[i];
            ++version[i];
            if (version[i] == 0)
                ++version[i];
            
            led_data &= ~(1 << i);
            led_data |= ((payload[i] & 0x01) << i);
            led_config();
            trickle_timer_reset(&trickle[i]);
            TICK_PIN(PIN_BUTTON);
        }
    } 
}
#endif   



int main(void)
{
    SET_PIN(PIN_CPU_IN_USE);
    make_address();
    //simple_uart_config(RTS_PIN_NUMBER, TX_PIN_NUMBER, CTS_PIN_NUMBER, RX_PIN_NUMBER, true);
    NRF_CLOCK->TASKS_LFCLKSTART = 1;
    NRF_CLOCK->TASKS_HFCLKSTART = 1;
    radio_init();
    trickle_setup();
    
    /* setup the two trickle objects */
    
    /* button 0 */
    trickle[0].i_max = 2000; /* max 200 seconds (100ms * 2000) */
    trickle[0].i_min = 100; /* min 100ms */
    trickle[0].i = 400;
    trickle[0].k = 3;
    trickle[0].tx_cb = &trickle_tx0;
    trickle[0].id = 0;
    
    /* button 1 */
    trickle[1].i_max = 2000; /* max 200 seconds (100ms * 2000) */
    trickle[1].i_min = 100; /* min 100ms */
    trickle[1].i = 400;
    trickle[1].k = 3;
    trickle[1].tx_cb = &trickle_tx1;
    trickle[1].id = 1;
    
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

    trickle_init(&trickle[0]);
    trickle_init(&trickle[1]);
    radio_rx(rx_data);
    
    CLEAR_PIN(PIN_CPU_IN_USE);
    /* sleep */
    while (true)
    {
        __WFE();        
    }
    

}
