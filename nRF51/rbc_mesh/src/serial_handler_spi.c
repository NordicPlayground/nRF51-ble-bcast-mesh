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
#include "spi_slave.h"
#include "event_handler.h"
#include "fifo.h"

#include "nrf_gpio.h"
#include "app_error.h"
#include "app_util_platform.h"
#include <string.h>
/** @brief Pin configuration for the application */
#define PIN_MISO                (28)
#define PIN_MOSI                (25)
#define PIN_SCK                 (29)
#define PIN_CSN                 (24)
#define PIN_RDYN                (20)


#define SERIAL_LENGTH_POS       (0)
#define SERIAL_OPCODE_POS       (1)

#define SERIAL_QUEUE_SIZE       (4)

#define SERIAL_REQN_GPIOTE_CH   (0)

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

static fifo_t rx_fifo;
static fifo_t tx_fifo;
static serial_data_t rx_fifo_buffer[SERIAL_QUEUE_SIZE];
static serial_data_t tx_fifo_buffer[SERIAL_QUEUE_SIZE];


static uint8_t dummy_data = 0;
static serial_data_t rx_buffer;
static serial_data_t tx_buffer;

static serial_state_t serial_state;
static bool has_pending_tx = false;
static bool doing_tx = false;
static bool suspend = false;

/*****************************************************************************
* Static functions
*****************************************************************************/
static void mesh_aci_command_check_cb(void* p_context)
{
    mesh_aci_command_check();
}

static void enable_pin_listener(bool enable)
{
    if (enable)
    {
        NRF_GPIOTE->EVENTS_IN[SERIAL_REQN_GPIOTE_CH] = 0;
        NRF_GPIOTE->INTENSET = (1 << SERIAL_REQN_GPIOTE_CH);
    }
    else
    {
        NRF_GPIOTE->INTENCLR = (1 << SERIAL_REQN_GPIOTE_CH);
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

    if (fifo_pop(&tx_fifo, &tx_buffer) == NRF_SUCCESS)
    {
        tx_len = tx_buffer.buffer[SERIAL_LENGTH_POS] + 2;
        tx_ptr = (uint8_t*) &tx_buffer;
        memset(rx_buffer.buffer, 0, SERIAL_DATA_MAX_LEN);
        error_code = spi_slave_buffers_set(tx_ptr,
                                          rx_buffer.buffer,
                                          tx_len,
                                          sizeof(rx_buffer));
        APP_ERROR_CHECK(error_code);
        ordered_buffer = true;
        has_pending_tx = true;
        doing_tx = true;
    }
    if (!ordered_buffer)
    {
        /* don't need to wait for SPIS mutex */
        NRF_GPIO->OUTCLR = (1 << PIN_RDYN);
    }
    /* wait for SPI driver to finish buffer set operation */
}

static void gpiote_init(void)
{
    NRF_GPIO->PIN_CNF[PIN_CSN] = (GPIO_PIN_CNF_SENSE_Disabled   << GPIO_PIN_CNF_SENSE_Pos)
                               | (GPIO_PIN_CNF_DRIVE_S0S1       << GPIO_PIN_CNF_DRIVE_Pos)
                               | (GPIO_PIN_CNF_PULL_Pulldown    << GPIO_PIN_CNF_PULL_Pos)
                               | (GPIO_PIN_CNF_INPUT_Connect    << GPIO_PIN_CNF_INPUT_Pos)
                               | (GPIO_PIN_CNF_DIR_Input        << GPIO_PIN_CNF_DIR_Pos);

    NRF_GPIOTE->CONFIG[SERIAL_REQN_GPIOTE_CH] = (PIN_CSN << GPIOTE_CONFIG_PSEL_Pos)
                                              | (GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)
                                              | (GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos);


    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_IN0_Msk;
    NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_EnableIRQ(GPIOTE_IRQn);
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
    if (NRF_GPIOTE->EVENTS_IN[SERIAL_REQN_GPIOTE_CH] && serial_state == SERIAL_STATE_IDLE)
    {
        if (fifo_is_full(&rx_fifo))
        {
            /* wait until the application pops an event off the rx queue */
            serial_state = SERIAL_STATE_WAIT_FOR_QUEUE;
        }
        else
        {
            do_transmit();
        }
    }
    NRF_GPIOTE->EVENTS_IN[SERIAL_REQN_GPIOTE_CH] = 0;
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
            if (doing_tx)
            {
                doing_tx = false;
                if (evt.tx_amount < tx_buffer.buffer[SERIAL_LENGTH_POS] + 2)
                {
                    /* master failed to receive our event. Re-send it. */
                    serial_handler_event_send((serial_evt_t*)tx_buffer.buffer);
                }
            }
            /* handle incoming */
            if (rx_buffer.buffer[SERIAL_LENGTH_POS] > 0)
            {
                if (fifo_push(&rx_fifo, &rx_buffer) == NRF_SUCCESS)
                {

                    /* notify ACI handler */
                    async_event_t async_evt;
                    memset(&async_evt, 0, sizeof(async_event_t));
                    async_evt.callback.generic.cb = mesh_aci_command_check_cb;
                    async_evt.type = EVENT_TYPE_GENERIC;
                    event_handler_push(&async_evt);
                }
                else
                {
                    APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
                }
            }
            if (suspend)
            {
                return;
            }
            if (fifo_is_empty(&tx_fifo))
            {
                serial_state = SERIAL_STATE_IDLE;
                prepare_rx();
                enable_pin_listener(true);
            }
            else if (fifo_is_full(&rx_fifo))
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
    has_pending_tx = false;
    /* init packet queues */
    tx_fifo.array_len = SERIAL_QUEUE_SIZE;
    tx_fifo.elem_array = tx_fifo_buffer;
    tx_fifo.elem_size = sizeof(serial_data_t);
    tx_fifo.memcpy_fptr = NULL;
    fifo_init(&tx_fifo);
    rx_fifo.array_len = SERIAL_QUEUE_SIZE;
    rx_fifo.elem_array = rx_fifo_buffer;
    rx_fifo.elem_size = sizeof(serial_data_t);
    rx_fifo.memcpy_fptr = NULL;
    fifo_init(&rx_fifo);

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
}

uint32_t serial_handler_credit_available(void)
{
    return fifo_get_len(&rx_fifo);
}

void serial_wait_for_completion(void)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    suspend = true;

    extern void SPI1_TWI1_IRQHandler(void);

    while (serial_state != SERIAL_STATE_IDLE)
    {
        SPI1_TWI1_IRQHandler();
    }

    suspend = false;
    _ENABLE_IRQS(was_masked);
}

bool serial_handler_event_send(serial_evt_t* evt)
{
    if (fifo_is_full(&tx_fifo))
    {
        return false;
    }

    enable_pin_listener(false);
    NVIC_DisableIRQ(SPI1_TWI1_IRQn); /* critical section */

    serial_data_t raw_data;
    raw_data.status_byte = 0;
    memcpy(raw_data.buffer, evt, evt->length + 1);
    fifo_push(&tx_fifo, &raw_data);

    if (fifo_is_full(&rx_fifo))
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
    if (fifo_pop(&rx_fifo, &temp) != NRF_SUCCESS)
    {
        enable_pin_listener(true);
        NVIC_EnableIRQ(SPI1_TWI1_IRQn);
        return false;
    }
    if (temp.buffer[SERIAL_LENGTH_POS] > 0)
    {
        memcpy(cmd, temp.buffer, temp.buffer[SERIAL_LENGTH_POS] + 1);
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

