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
#include "serial_handler.h"
#include "serial_queue.h"
#include "spi_slave.h"
#include "timeslot_handler.h"

#include "nrf_gpio.h"
#include "app_error.h"
#include "app_util_platform.h"
#include <string.h>
/** @brief Pin configuration for the application */
#define PIN_MISO            (28)
#define PIN_MOSI            (25)
#define PIN_SCK             (29)
#define PIN_CSN             (24)
#define PIN_RDYN            (20)

#define SERIAL_LENGTH_POS   (0)
#define SERIAL_OPCODE_POS   (1)

/*****************************************************************************
* Static types
*****************************************************************************/

typedef enum
{
    SERIAL_STATE_IDLE,
    SERIAL_STATE_WAIT_FOR_QUEUE,
    SERIAL_STATE_TRANSMIT
} serial_state_t;

/*****************************************************************************
* Static globals
*****************************************************************************/

static serial_queue_t rx_q;
static serial_queue_t tx_q;

static uint8_t dummy_data = 0;
static serial_data_t rx_buffer;
static serial_data_t tx_buffer;

static serial_state_t serial_state;
static bool has_pending_tx = false;

/*****************************************************************************
* Static functions
*****************************************************************************/

static void enable_pin_listener(bool enable)
{
    if (enable)
    {
        NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    }
    else
    {
        NRF_GPIOTE->INTENCLR = GPIOTE_INTENCLR_PORT_Msk;
    }
}

static void prepare_rx(void)
{
    uint32_t error_code;
    error_code = spi_slave_buffers_set(&dummy_data, 
                                      rx_buffer.buffer, 
                                      1, 
                                      SERIAL_DATA_MAX_LEN - 2);
    APP_ERROR_CHECK(error_code);
    has_pending_tx = false;
}

/**
* @brief Called when master requests send, or we want to notify master
*/
static void do_transmit(void)
{
    uint8_t* tx_ptr = &dummy_data;
    uint8_t tx_len = 0;
    uint32_t error_code;
    
    serial_state = SERIAL_STATE_TRANSMIT;
    
    /* don't want GPIOTE to interrupt again before after packet is 
    successfully transmitted. */
    enable_pin_listener(false);
    
    bool ordered_buffer = false;
    
    if (!serial_queue_is_empty(&tx_q))
    {
        if (serial_queue_dequeue(&tx_q, &tx_buffer))
        {
            tx_len = tx_buffer.buffer[0] + 2;
            tx_ptr = (uint8_t*) &tx_buffer;
            memset(rx_buffer.buffer, 0, SERIAL_DATA_MAX_LEN);
            error_code = spi_slave_buffers_set(tx_ptr,
                                              rx_buffer.buffer, 
                                              tx_len, 
                                              sizeof(rx_buffer));
            APP_ERROR_CHECK(error_code);
            ordered_buffer = true;
            has_pending_tx = true;
        }
    }
    if (!ordered_buffer)
    {
        /* don't need to wait for SPIS mutex */
        NRF_GPIO->OUTCLR = (1 << PIN_RDYN);
    }
        
    
    nrf_gpio_pin_set(0);
    nrf_gpio_pin_clear(0);
    
    /* wait for SPI driver to finish buffer set operation */
}

static void gpiote_init(void)
{
    NRF_GPIO->PIN_CNF[PIN_CSN] = (GPIO_PIN_CNF_SENSE_Low        << GPIO_PIN_CNF_SENSE_Pos)
                               | (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | (GPIO_PIN_CNF_PULL_Disabled    << GPIO_PIN_CNF_PULL_Pos)
                               | (GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                               | (GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos);
   
#if 0
    nrf_gpio_cfg_input(PIN_CSN, NRF_GPIO_PIN_PULLUP);
    
    
    NRF_GPIOTE->CONFIG[0] = GPIOTE_CONFIG_MODE_Event 		<< GPIOTE_CONFIG_MODE_Pos 
                          | GPIOTE_CONFIG_POLARITY_HiToLo 	<< GPIOTE_CONFIG_POLARITY_Pos 
                          | PIN_CSN                         << GPIOTE_CONFIG_PSEL_Pos;
    
    //NRF_GPIOTE->EVENTS_IN[0] = 0;
    NRF_GPIOTE->INTENSET  = GPIOTE_INTENSET_IN0_Set << GPIOTE_INTENSET_IN0_Pos;,
#endif  
    nrf_gpio_cfg_output(0);
    nrf_gpio_cfg_output(1);
    NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_EnableIRQ(GPIOTE_IRQn);
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Msk;
    //nrf_gpio_cfg_sense_input(PIN_CSN, NRF_GPIO_PIN_PULLDOWN, NRF_GPIO_PIN_SENSE_LOW);
    enable_pin_listener(true);
}



/*****************************************************************************
* System callbacks
*****************************************************************************/

/**
* @brief IRQ handler for treating incoming SPI message. The master will not
*   start sending before we lower the RDYN pin.
*/
void GPIOTE_IRQHandler(void)
{    
    nrf_gpio_pin_set(0);
    nrf_gpio_pin_clear(0);
    if (NRF_GPIOTE->EVENTS_PORT)//(true || serial_state == SERIAL_STATE_IDLE)
    {
        NRF_GPIOTE->EVENTS_PORT = 0;
        if ((NRF_GPIO->IN & (1 << PIN_CSN)) == 0 && serial_state == SERIAL_STATE_IDLE)
        {
            if (serial_queue_is_full(&rx_q))
            {
                /* wait until the application pops an event off the rx queue */
                serial_state = SERIAL_STATE_WAIT_FOR_QUEUE;
            }
            else
            {
                do_transmit();
            }
        }
    }
}

/**
* @brief SPI event handler, from SPI driver 
*/
void spi_event_handler(spi_slave_evt_t evt)
{
    switch (evt.evt_type)
    {
        case SPI_SLAVE_BUFFERS_SET_DONE:
            if (has_pending_tx)
            {
                NRF_GPIO->OUTCLR = (1 << PIN_RDYN);
            }
            has_pending_tx = false;
            break;
        
        case SPI_SLAVE_XFER_DONE:
            NRF_GPIO->OUTSET = (1 << PIN_RDYN);
            nrf_gpio_pin_set(1);
            nrf_gpio_pin_clear(1);
            /* handle incoming */
            if (rx_buffer.buffer[SERIAL_LENGTH_POS] > 0)
            {
                serial_queue_enqueue(&rx_q, &rx_buffer);
                
                /* notify ACI handler */
                async_event_t async_evt;
                async_evt.callback.generic = mesh_aci_command_check;
                async_evt.type = EVENT_TYPE_GENERIC;
                timeslot_queue_async_event(&async_evt);
            }
            
            if (serial_queue_is_empty(&tx_q))
            {
                serial_state = SERIAL_STATE_IDLE;
                prepare_rx();
                enable_pin_listener(true);
            }
            else if (serial_queue_is_full(&rx_q))
            {
                prepare_rx();
                serial_state = SERIAL_STATE_WAIT_FOR_QUEUE;
            }
            else
            {
                do_transmit();
            }
            
            break;
        
        default:
            /* no implementation necessary */
            break;
    }
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

void serial_handler_init(void)
{
    serial_queue_init(&tx_q);
    serial_queue_init(&rx_q);
    
    nrf_gpio_cfg_output(PIN_RDYN);  
    nrf_gpio_pin_set(PIN_RDYN);
    serial_state = SERIAL_STATE_IDLE;
    
    spi_slave_config_t spi_config;
    spi_config.bit_order = SPIM_LSB_FIRST;
    spi_config.mode = SPI_MODE_0;
    spi_config.def_tx_character = 0;
    spi_config.orc_tx_character = 0;
    spi_config.pin_csn = PIN_CSN;
    spi_config.pin_miso = PIN_MISO;
    spi_config.pin_mosi = PIN_MOSI;
    spi_config.pin_sck = PIN_SCK;
    
    
    
    APP_ERROR_CHECK(spi_slave_init(&spi_config));
    APP_ERROR_CHECK(spi_slave_evt_handler_register(spi_event_handler));

    gpiote_init();
    
    /* set initial buffers, dummy in tx */
    has_pending_tx = false;
    prepare_rx();
}

bool serial_handler_event_send(serial_evt_t* evt)
{
    if (serial_queue_is_full(&tx_q))
    {
        return false;
    }
    
    enable_pin_listener(false);
    NVIC_DisableIRQ(SPI1_TWI1_IRQn); /* critical section */
    
    serial_data_t raw_data;
    raw_data.status_byte = 0;
    memcpy(raw_data.buffer, evt, evt->length + 1);
    serial_queue_enqueue(&tx_q, &raw_data);
    
    if (serial_queue_is_full(&rx_q)) 
    {
        enable_pin_listener(true);
        serial_state = SERIAL_STATE_WAIT_FOR_QUEUE;
    }
    else if (serial_state == SERIAL_STATE_IDLE)
    {
        do_transmit();
    }
    
    NVIC_EnableIRQ(SPI1_TWI1_IRQn); /* critical section complete */
    return true;
}

bool serial_handler_command_get(serial_cmd_t* cmd)
{
    /* want to do this without being pre-empted, as there's a potential
    race condition */
    NVIC_DisableIRQ(SPI1_TWI1_IRQn);
    enable_pin_listener(false);
    serial_data_t temp;
    if (!serial_queue_dequeue(&rx_q, &temp))
    {
        enable_pin_listener(true);
        NVIC_EnableIRQ(SPI1_TWI1_IRQn);
        return false;
    }
    if (temp.buffer[0] > 0)
    {
        memcpy(cmd, temp.buffer, temp.buffer[0] + 1);
    }

    
    /* just made room in the queue */
    if (serial_state == SERIAL_STATE_WAIT_FOR_QUEUE)
    {
        /* would have race condition here with several irq contexts */
        do_transmit();
    }
    else if (serial_state == SERIAL_STATE_IDLE)
    {
        /* only want GPIOTE IRQ in IDLE */
        enable_pin_listener(true);
    }
    
    NVIC_EnableIRQ(SPI1_TWI1_IRQn);
    return true;
}

