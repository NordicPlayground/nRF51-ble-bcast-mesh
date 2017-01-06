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

#include "radio_control.h"
#include "rbc_mesh_common.h"
#include "timeslot.h"
#include "trickle.h"
#include "fifo.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "toolchain.h"
#include "rbc_mesh.h"
#include "mesh_packet.h"

#include <stdbool.h>
#include <string.h>

/** The default access address the radio operates on. Represented as logical address 0. */
#define RADIO_DEFAULT_ADDRESS           (0x8E89BED6)

#define LIGHTWEIGHT_RADIO               (1)

#define RADIO_RX_TIMEOUT                (150 + 80)

#define RADIO_EVENT(evt)                (NRF_RADIO->evt == 1)

#define PPI_CH_STOP_RX_ABORT            (TIMER_PPI_CH_START + 4)

#define DEBUG_RADIO_SET_STATE(state) do {\
    DEBUG_RADIO_CLEAR_PIN(PIN_RADIO_STATE_TX);\
    DEBUG_RADIO_CLEAR_PIN(PIN_RADIO_STATE_RX);\
    DEBUG_RADIO_CLEAR_PIN(PIN_RADIO_STATE_IDLE);\
    DEBUG_RADIO_SET_PIN(state);\
    }while (0)

/**
* Internal enum denoting radio state.
*/
typedef enum
{
    RADIO_STATE_RX,
    RADIO_STATE_TX,
    RADIO_STATE_DISABLED,
    RADIO_STATE_NEVER_USED
} radio_state_t;

/*****************************************************************************
* Static globals
*****************************************************************************/

/** Global radio state */
static radio_state_t    m_radio_state = RADIO_STATE_NEVER_USED;

static fifo_t           m_radio_fifo;
static radio_event_t    m_radio_fifo_queue[RBC_MESH_RADIO_QUEUE_LENGTH];
static radio_idle_cb_t  m_idle_cb;
static radio_rx_cb_t    m_rx_cb;
static radio_tx_cb_t    m_tx_cb;
static uint32_t         m_alt_aa = RADIO_DEFAULT_ADDRESS;
/*****************************************************************************
* Static functions
*****************************************************************************/
static void purge_preemptable(void)
{
    uint32_t events_in_queue = fifo_get_len(&m_radio_fifo);
    while (events_in_queue > 1)
    {
        radio_event_t current_evt;
        if (fifo_peek(&m_radio_fifo, &current_evt) == NRF_SUCCESS &&
            current_evt.event_type == RADIO_EVENT_TYPE_RX_PREEMPTABLE)
        {
            /* event is preemptable, stop it */
            fifo_pop(&m_radio_fifo, NULL);

            radio_disable();
            while (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled);
            NRF_RADIO->EVENTS_END = 0;

            /* propagate failed rx event */
            m_rx_cb(current_evt.packet_ptr, false, 0xFFFFFFFF, 100);
            --events_in_queue;
        }
        else
        {
            break;
        }
    }
}

static void radio_channel_set(uint8_t ch)
{
    if (ch <= 10)
    {
        NRF_RADIO->FREQUENCY = 4 + ch * 2;
    }
    else if (ch <= 36)
    {
        NRF_RADIO->FREQUENCY = 6 + ch * 2;
    }
    else
    {
        uint32_t adv_freqs[] = {2, 26, 80};
        NRF_RADIO->FREQUENCY = adv_freqs[(ch - 37)];
    }

    NRF_RADIO->DATAWHITEIV = ch & RADIO_DATAWHITEIV_DATAWHITEIV_Msk;

}

static void setup_event(radio_event_t* p_evt)
{
    NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk | RADIO_SHORTS_END_DISABLE_Msk | RADIO_SHORTS_ADDRESS_RSSISTART_Msk;
    radio_channel_set(p_evt->channel);
    NRF_RADIO->PACKETPTR = (uint32_t) p_evt->packet_ptr;
    NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->PREFIX0 |= (((m_alt_aa >> 24) << 8) & 0x0000FF00);
    NRF_RADIO->BASE1    = ((m_alt_aa <<  8) & 0xFFFFFF00);

    if (p_evt->event_type == RADIO_EVENT_TYPE_TX)
    {
        DEBUG_RADIO_SET_STATE(PIN_RADIO_STATE_TX);
        NRF_RADIO->TXADDRESS = p_evt->access_address;
        NRF_RADIO->TXPOWER  = p_evt->tx_power;
        NRF_RADIO->TASKS_TXEN = 1;
        m_radio_state = RADIO_STATE_TX;
        
    }
    else
    {
        DEBUG_RADIO_SET_STATE(PIN_RADIO_STATE_RX);
        if (m_alt_aa != RADIO_DEFAULT_ADDRESS)
        {
            /* only enable alt-addr if it's different */
            NRF_RADIO->RXADDRESSES = 0x03;
        }
        else
        {
            NRF_RADIO->RXADDRESSES = 0x01;
        }
        NRF_RADIO->TASKS_RXEN = 1;
        m_radio_state = RADIO_STATE_RX;
    }
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

void radio_init(radio_idle_cb_t idle_cb,
                radio_rx_cb_t   rx_cb,
                radio_tx_cb_t   tx_cb)
{
    /* Reset all states in the radio peripheral */
    NRF_RADIO->POWER            = ((RADIO_POWER_POWER_Disabled << RADIO_POWER_POWER_Pos) & RADIO_POWER_POWER_Msk);
    NRF_RADIO->POWER            = ((RADIO_POWER_POWER_Enabled  << RADIO_POWER_POWER_Pos) & RADIO_POWER_POWER_Msk);

    /* Set radio configuration parameters */
    NRF_RADIO->TXPOWER      = ((RADIO_TXPOWER_TXPOWER_0dBm << RADIO_TXPOWER_TXPOWER_Pos) & RADIO_TXPOWER_TXPOWER_Msk);
    NRF_RADIO->MODE 	    = ((RADIO_MODE_MODE_Ble_1Mbit << RADIO_MODE_MODE_Pos) & RADIO_MODE_MODE_Msk);

    NRF_RADIO->FREQUENCY 	    = 2;					// Frequency bin 2, 2402MHz, channel 37.
    NRF_RADIO->DATAWHITEIV      = 37;					// NOTE: This value needs to correspond to the frequency being used


    /* Configure Access Address  */
    NRF_RADIO->PREFIX0	= ((RADIO_DEFAULT_ADDRESS >> 24) & 0x000000FF);
    NRF_RADIO->BASE0    = ((RADIO_DEFAULT_ADDRESS <<  8) & 0xFFFFFF00);
    NRF_RADIO->PREFIX0 |= (((m_alt_aa >> 24) << 8) & 0x0000FF00);
    NRF_RADIO->BASE1    = ((m_alt_aa <<  8) & 0xFFFFFF00);
    NRF_RADIO->TXADDRESS    = 0x00;			    // Use logical address 0 (prefix0 + base0) = 0x8E89BED6 when transmitting
    NRF_RADIO->RXADDRESSES  = 0x01;				// Enable reception on logical address 0 (PREFIX0 + BASE0)

    /* PCNF-> Packet Configuration. Now we need to configure the sizes S0, S1 and length field to match the datapacket format of the advertisement packets. */
    NRF_RADIO->PCNF0 =  (
                          (((1UL) << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk)    // length of S0 field in bytes 0-1.
                        | (((2UL) << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk)    // length of S1 field in bits 0-8.
                        | (((6UL) << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk)    // length of length field in bits 0-8.
                      );

    /* Packet configuration */
    NRF_RADIO->PCNF1 =  (
                          (((37UL)                          << RADIO_PCNF1_MAXLEN_Pos)  & RADIO_PCNF1_MAXLEN_Msk)   // maximum length of payload in bytes [0-255]
                        | (((0UL)                           << RADIO_PCNF1_STATLEN_Pos) & RADIO_PCNF1_STATLEN_Msk)	// expand the payload with N bytes in addition to LENGTH [0-255]
                        | (((3UL)                           << RADIO_PCNF1_BALEN_Pos)   & RADIO_PCNF1_BALEN_Msk)    // base address length in number of bytes.
                        | (((RADIO_PCNF1_ENDIAN_Little)     << RADIO_PCNF1_ENDIAN_Pos)  & RADIO_PCNF1_ENDIAN_Msk)   // endianess of the S0, LENGTH, S1 and PAYLOAD fields.
                        | (((RADIO_PCNF1_WHITEEN_Enabled)   << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk)	// enable packet whitening
                      );

    /* CRC config */
    NRF_RADIO->CRCPOLY = ((0x00065B << RADIO_CRCPOLY_CRCPOLY_Pos) & RADIO_CRCPOLY_CRCPOLY_Msk);    // CRC polynomial function
    NRF_RADIO->CRCCNF = (((RADIO_CRCCNF_SKIPADDR_Skip) << RADIO_CRCCNF_SKIPADDR_Pos) & RADIO_CRCCNF_SKIPADDR_Msk)
                      | (((RADIO_CRCCNF_LEN_Three)      << RADIO_CRCCNF_LEN_Pos)       & RADIO_CRCCNF_LEN_Msk);

    NRF_RADIO->CRCINIT = ((0x555555 << RADIO_CRCINIT_CRCINIT_Pos) & RADIO_CRCINIT_CRCINIT_Msk);    // Initial value of CRC
    /* Lock interframe spacing, so that the radio won't send too soon / start RX too early */
    NRF_RADIO->TIFS = 148;

    /* init radio packet fifo */
    if (m_radio_state == RADIO_STATE_NEVER_USED)
    {
        /* this flushes the queue */
        m_radio_fifo.array_len = RBC_MESH_RADIO_QUEUE_LENGTH;
        m_radio_fifo.elem_array = m_radio_fifo_queue;
        m_radio_fifo.elem_size = sizeof(radio_event_t);
        m_radio_fifo.memcpy_fptr = NULL;
        fifo_init(&m_radio_fifo);
    }

    m_radio_state = RADIO_STATE_DISABLED;
    NRF_RADIO->EVENTS_END = 0;

    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);
    m_idle_cb = idle_cb;
    m_rx_cb = rx_cb;
    m_tx_cb = tx_cb;

    DEBUG_RADIO_CLEAR_PIN(PIN_RADIO_STATE_RX);
    DEBUG_RADIO_CLEAR_PIN(PIN_RADIO_STATE_TX);

    if (fifo_is_empty(&m_radio_fifo))
    {
        m_idle_cb();
    }
    else
    {
        if (timeslot_is_in_ts())
        {
            NVIC_SetPendingIRQ(RADIO_IRQn);
        }
    }
}

void radio_alt_aa_set(uint32_t access_address)
{
    m_alt_aa = access_address;
}

uint32_t radio_order(radio_event_t* p_radio_event)
{
    if (p_radio_event == NULL)
    {
        return NRF_ERROR_NULL;
    }

    if (p_radio_event->event_type == RADIO_EVENT_TYPE_TX &&
        p_radio_event->access_address > 1)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    if (fifo_push(&m_radio_fifo, p_radio_event) != NRF_SUCCESS)
    {
        return NRF_ERROR_NO_MEM;
    }

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    if (timeslot_is_in_ts())
    {
        NVIC_SetPendingIRQ(RADIO_IRQn);
    }

    _ENABLE_IRQS(was_masked);
    return NRF_SUCCESS;
}

void radio_disable(void)
{
    NRF_RADIO->SHORTS = 0;
    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    NRF_RADIO->TASKS_DISABLE = 1;
    m_radio_state = RADIO_STATE_DISABLED;
    DEBUG_RADIO_SET_STATE(PIN_RADIO_STATE_IDLE);
}

/**
* IRQ handler for radio. Sends the radio around the state machine, ensuring secure radio state changes
*/
void radio_event_handler(void)
{
    if (NRF_RADIO->EVENTS_END)
    {
        bool crc_status = NRF_RADIO->CRCSTATUS;
        uint32_t crc = NRF_RADIO->RXCRC;
        uint8_t rssi = 100;

        if (NRF_RADIO->EVENTS_RSSIEND)
        {
            NRF_RADIO->EVENTS_RSSIEND = 0;
            rssi = NRF_RADIO->RSSISAMPLE;
        }

        radio_event_t prev_evt;
        NRF_RADIO->EVENTS_END = 0;

        /* pop the event that just finished */
        uint32_t error_code = fifo_pop(&m_radio_fifo, &prev_evt);
        APP_ERROR_CHECK(error_code);

        /* send to super space */
        if (prev_evt.event_type == RADIO_EVENT_TYPE_RX ||
            prev_evt.event_type == RADIO_EVENT_TYPE_RX_PREEMPTABLE)
        {
            m_rx_cb(prev_evt.packet_ptr, crc_status, crc, rssi);
        }
        else
        {
            m_tx_cb(prev_evt.packet_ptr);
        }

        DEBUG_RADIO_SET_STATE(PIN_RADIO_STATE_IDLE);
        m_radio_state = RADIO_STATE_DISABLED;
    }
    else
    {
        purge_preemptable();
    }

    if (m_radio_state == RADIO_STATE_DISABLED ||
        m_radio_state == RADIO_STATE_NEVER_USED)
    {
        radio_event_t evt;
        if (fifo_peek(&m_radio_fifo, &evt) == NRF_SUCCESS)
        {
            setup_event(&evt);
        }
        else
        {
            m_idle_cb();
        }
    }
}

