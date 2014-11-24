#include "serial_handler.h"
#include "serial_queue.h"
#include "spi_slave.h"

#include "app_error.h"
#include "app_util_platform.h"

/*****************************************************************************
* Static globals
*****************************************************************************/

static serial_queue_t rx_q;
static serial_queue_t tx_q;

static uint8_t dummy_data = 0;
static serial_data_t rx_buffer;

static uint32_t pin_miso;
static uint32_t pin_mosi;
static uint32_t pin_csn;
static uint32_t pin_sck;
static uint32_t pin_rdyn;

/*****************************************************************************
* Static functions
*****************************************************************************/
static void do_transmit(void)
{
    uint8_t* tx_ptr = &dummy_data;
    serial_data_t tx_buffer;
    uint8_t tx_len = 0;
    uint32_t error_code;
    
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
    
    /* ready to transmit */
    NRF_GPIO->OUTCLR = (1 << pin_rdyn);
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
    if ((NRF_GPIO->IN & (1 << pin_csn)) == 0)
    {
        if (!serial_queue_is_full(&rx_q))
        {
            
        }
    }
}

void spi_event_handler(spi_slave_evt_t evt)
{
    
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

void serial_handler_init(serial_cmd_handler cmd_handler)
{
    
    
    
}

void serial_handler_event_send(serial_evt_t* evt)
{
    
}

