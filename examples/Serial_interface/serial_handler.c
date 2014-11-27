#include "serial_handler.h"
#include "serial_queue.h"
#include "spi_slave.h"

#include "nrf_gpio.h"
#include "app_error.h"
#include "app_util_platform.h"

/** @brief Pin configuration for the application */
#define PIN_MISO            (12)
#define PIN_MOSI            (13)
#define PIN_SCK             (14)
#define PIN_CSN             (15)
#define PIN_RDYN            (16)

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

static serial_state_t serial_state;

/*****************************************************************************
* Static functions
*****************************************************************************/

/**
* @brief Called when master requests send, or we want to notify master
*/
static void do_transmit(void)
{
    uint8_t* tx_ptr = &dummy_data;
    serial_data_t tx_buffer;
    uint8_t tx_len = 0;
    uint32_t error_code;
    
    serial_state = SERIAL_STATE_TRANSMIT;
    
    /* don't want GPIOTE to interrupt again before after packet is 
    successfully transmitted. */
    NVIC_DisableIRQ(GPIOTE_IRQn);
    
    if (!serial_queue_is_empty(&tx_q))
    {
        if (serial_queue_dequeue(&tx_q, &tx_buffer))
        {
            tx_len = sizeof(serial_data_t);
            tx_ptr = (uint8_t*) &tx_buffer;
        }
    }
    
    error_code = spi_slave_buffers_set(tx_ptr, 
                                      rx_buffer.buffer, 
                                      tx_len, 
                                      sizeof(rx_buffer));
        
    APP_ERROR_CHECK(error_code);
    
    /* wait for SPI driver to finish buffer set operation */
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

/**
* @brief SPI event handler, from SPI driver 
*/
void spi_event_handler(spi_slave_evt_t evt)
{
    switch (evt.evt_type)
    {
        case SPI_SLAVE_BUFFERS_SET_DONE:
            /* ready to transmit */
            NRF_GPIO->OUTCLR = (1 << PIN_RDYN);
            break;
        
        case SPI_SLAVE_XFER_DONE:
            /* handle incoming */
            if (rx_buffer.buffer[SERIAL_LENGTH_POS] > 0)
            {
                serial_queue_enqueue(&rx_q, &rx_buffer);
            }
            
            if (serial_queue_is_empty(&tx_q))
            {
                serial_state = SERIAL_STATE_IDLE;
                NVIC_EnableIRQ(GPIOTE_IRQn);
            }
            else if (serial_queue_is_full(&rx_q))
            {
                serial_state = SERIAL_STATE_WAIT_FOR_QUEUE;
            }
            else
            {
                do_transmit();
            }
            
            break;
        
        default:
            /* no implementation necessary */
    }
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

void serial_handler_init(serial_cmd_handler cmd_handler)
{
    serial_queue_init(&tx_q);
    serial_queue_init(&rx_q);
    
    nrf_gpio_cfg_output(PIN_RDYN);  
    nrf_gpio_pin_write(PIN_RDYN, 1);
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
    
    /* GPIOTE */
    NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_EnableIRQ(GPIOTE_IRQn);
}

bool serial_handler_event_send(serial_evt_t* evt)
{
    if (serial_queue_is_full(&tx_q))
    {
        return false;
    }
    
    NVIC_DisableIRQ(GPIOTE_IRQn);
    NVIC_DisableIRQ(SPI1_TWI1_IRQn); /* critical section */
    
    serial_queue_enqueue(&tx_q, (serial_data_t*) evt);
    
    if (serial_queue_is_full(&rx_q)) 
    {
        serial_state = SERIAL_STATE_WAIT_FOR_QUEUE;
    }
    else
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
    NVIC_DisableIRQ(GPIOTE_IRQn);
    
    if (!serial_queue_dequeue(&rx_q, (serial_data_t*) cmd))
    {
        return false;
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
        NVIC_EnableIRQ(GPIOTE_IRQn);
    }
    
    NVIC_EnableIRQ(SPI1_TWI1_IRQn);
    return true;
}

