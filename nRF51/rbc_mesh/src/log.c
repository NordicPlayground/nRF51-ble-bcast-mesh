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

#include "log.h"
#include "fifo.h"
#include "nrf51.h"
#include "boards.h"
#include "nrf_error.h"
#include "nrf_soc.h"
#include <stddef.h>

#define QUEUE_LEN       (8)

static const char m_overflow_str[] = "WARNING: LOST LOG EVENT DUE TO OVERFLOW\n";

static fifo_t m_fifo;
static char* mpp_fifo_queue[QUEUE_LEN];
bool m_queue_overflow;

static void string_tx(char* str)
{
    char* p_char = &str[0];

    while (*p_char != '\0')
    {
        NRF_UART0->TXD = (uint8_t)(*p_char);

        while (NRF_UART0->EVENTS_TXDRDY != 1)
        {
            /* Wait for TXD data to be sent. */
        }

        NRF_UART0->EVENTS_TXDRDY = 0;

        p_char++;
    }
}

void log_init(uint32_t baud_rate, bool hwfc)
{
    m_fifo.elem_array = mpp_fifo_queue;
    m_fifo.elem_size = sizeof(char*);
    m_fifo.array_len = QUEUE_LEN;
    m_fifo.memcpy_fptr = NULL;
    fifo_init(&m_fifo);

    /* snippet from the old SDK module simple_uart */
    nrf_gpio_cfg_output(TX_PIN_NUMBER);
    nrf_gpio_cfg_input(RX_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);

    NRF_UART0->PSELTXD = TX_PIN_NUMBER;
    NRF_UART0->PSELRXD = RX_PIN_NUMBER;

    if (hwfc)
    {
        nrf_gpio_cfg_output(RTS_PIN_NUMBER);
        nrf_gpio_cfg_input(CTS_PIN_NUMBER, NRF_GPIO_PIN_NOPULL);
        NRF_UART0->PSELCTS = CTS_PIN_NUMBER;
        NRF_UART0->PSELRTS = RTS_PIN_NUMBER;
        NRF_UART0->CONFIG  = (UART_CONFIG_HWFC_Enabled << UART_CONFIG_HWFC_Pos);
    }

    NRF_UART0->BAUDRATE      = (baud_rate << UART_BAUDRATE_BAUDRATE_Pos);
    NRF_UART0->ENABLE        = (UART_ENABLE_ENABLE_Enabled << UART_ENABLE_ENABLE_Pos);
    NRF_UART0->TASKS_STARTTX = 1;
    NRF_UART0->TASKS_STARTRX = 1;
    NRF_UART0->EVENTS_RXDRDY = 0;
}

inline uint32_t log_push(const char* str)
{
    uint32_t ret_val = fifo_push(&m_fifo, &str);
    if (ret_val != NRF_SUCCESS)
    {
        m_queue_overflow = true;
    }
    __SEV(); /* signal event, in case loop function didn't get any */
    return ret_val;
}

void log_process(void)
{
    static char* str = NULL;
    while (fifo_pop(&m_fifo, &str) == NRF_SUCCESS && str != NULL)
    {
        string_tx(str);
        
        if (m_queue_overflow)
        {
            m_queue_overflow = (log_push(m_overflow_str) != NRF_SUCCESS);
        }
    }
}

