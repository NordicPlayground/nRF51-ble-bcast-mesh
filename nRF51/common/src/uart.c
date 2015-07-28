/*----------------------------------------------------------------------------*/
/* uart.c                                                                     */
/* 38400 bps, 8 data bits, 1 stop bit, No Flow Control                        */
/*----------------------------------------------------------------------------*/

#include <stdbool.h>

#include "nrf.h"
#include "uart.h"
#include "nrf_gpio.h"
#include "boards.h"

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*----------------------------------------------------------------------------*/

#define BAUD_RATE  (UART_BAUDRATE_BAUDRATE_Baud38400 << UART_BAUDRATE_BAUDRATE_Pos)

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*----------------------------------------------------------------------------*/
void uart_init(void)
{
    // Configure UART0 pins.
    nrf_gpio_cfg_output(TX_PIN_NUMBER);
    nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);

    NRF_UART0->PSELTXD         = TX_PIN_NUMBER;
    NRF_UART0->PSELRXD         = RX_PIN_NUMBER;
    NRF_UART0->BAUDRATE        = BAUD_RATE;

    // Clean out possible events from earlier operations
    NRF_UART0->EVENTS_RXDRDY   = 0;
    NRF_UART0->EVENTS_TXDRDY   = 0;
    NRF_UART0->EVENTS_ERROR    = 0;

    // Activate UART.
    NRF_UART0->ENABLE          = UART_ENABLE_ENABLE_Enabled;
    NRF_UART0->INTENSET        = 0;
    NRF_UART0->TASKS_STARTTX   = 1;
    NRF_UART0->TASKS_STARTRX   = 1;
}

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*----------------------------------------------------------------------------*/
void uart_putc(uint8_t ch) {
    NRF_UART0->TXD = ch;
    while (NRF_UART0->EVENTS_TXDRDY != 1) {/* spin */};
    NRF_UART0->EVENTS_TXDRDY = 0;
}

/*----------------------------------------------------------------------------*/
/*                                                                            */
/*----------------------------------------------------------------------------*/
void uart_puts(uint8_t * str) {

    while(*str) {
        NRF_UART0->TXD = *str;
        while (NRF_UART0->EVENTS_TXDRDY != 1) {/* spin */};
        NRF_UART0->EVENTS_TXDRDY = 0;
        ++str;
    }

    NRF_UART0->TXD = '\n';
    while (NRF_UART0->EVENTS_TXDRDY != 1) {/* spin */};
    NRF_UART0->EVENTS_TXDRDY = 0;
}
