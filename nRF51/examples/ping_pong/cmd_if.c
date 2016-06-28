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

#include "cmd_if.h"

#include "app_uart.h"
#include "boards.h"

static cmd_rx_cb_t m_rx_cb;

/**
* @brief Handle incoming character on the control channel (UART or RTT).
*   Holds text until a \r or \n has been received, upon which it calls cmd_rx()
*/
static void char_rx(uint8_t c)
{
    static uint8_t rx_buf[32];
    static uint8_t* pp = rx_buf;

    if (c != '\r' && c != '\n')
    {
        *(pp++) = c;
    }
    else
    {
        *(pp++) = 0;
    }
    uint32_t len = (uint32_t)(pp - rx_buf);
    if (len >= sizeof(rx_buf) || c == '\r' || c == '\n') /* end of command */
    {
        if (len > 0)
            m_rx_cb(rx_buf, len);
        pp = rx_buf;
    }
}

/** Handler for incoming UART events */
void uart_event_handler(app_uart_evt_t * p_app_uart_event)
{
    uint8_t c;
    switch (p_app_uart_event->evt_type)
    {
        case APP_UART_DATA_READY:
            while (app_uart_get(&c) == NRF_SUCCESS)
            {
                /* log single character */
                char_rx(c);
            }
            break;
        default:
            break;
    }
}


void cmd_init(cmd_rx_cb_t rx_cb)
{
    uint32_t error_code;
    app_uart_comm_params_t uart_params;
    uart_params.baud_rate = UART_BAUDRATE_BAUDRATE_Baud460800;
    uart_params.cts_pin_no = CTS_PIN_NUMBER;
    uart_params.rts_pin_no = RTS_PIN_NUMBER;
    uart_params.rx_pin_no = RX_PIN_NUMBER;
    uart_params.tx_pin_no = TX_PIN_NUMBER;
    uart_params.flow_control = APP_UART_FLOW_CONTROL_DISABLED;
    uart_params.use_parity = false;
    APP_UART_FIFO_INIT(&uart_params, 8, 512, uart_event_handler, APP_IRQ_PRIORITY_LOW, error_code);
    APP_ERROR_CHECK(error_code);

    m_rx_cb = rx_cb;
}
