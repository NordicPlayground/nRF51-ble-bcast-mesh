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

#include "radio_control.h"
#include "timer_control.h"
#include "timeslot_handler.h"
#include "rbc_mesh_common.h"
#include "trickle.h"
#include "fifo.h"
#include "nrf.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "toolchain.h"

#include <stdbool.h>
#include <string.h>

#define RADIO_FIFO_QUEUE_SIZE           (8) /* must be power of two */

#define RADIO_RX_TIMEOUT                (150 + 80)

#define RADIO_EVENT(evt)                (NRF_RADIO->evt == 1)

#define PPI_CH_STOP_RX_ABORT            (TIMER_PPI_CH_START + 4)

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

/**
* Global radio state
*/
static radio_state_t radio_state = RADIO_STATE_NEVER_USED;

static fifo_t radio_fifo;
static radio_event_t radio_fifo_queue[RADIO_FIFO_QUEUE_SIZE];
static radio_idle_cb g_idle_cb;

static bool forced_wakeup = false;


/*****************************************************************************
* Static functions
*****************************************************************************/

static void rx_abort_cb(uint64_t timestamp);

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

static bool radio_will_go_to_disabled_state(void)
{
    radio_event_t current_evt, next_evt;
    fifo_peek(&radio_fifo, &current_evt);
    fifo_peek_at(&radio_fifo, &next_evt, 1);

    return ((fifo_get_len(&radio_fifo) == 2 &&
        (NRF_RADIO->EVENTS_READY) && !(NRF_RADIO->EVENTS_END)) ||
        (current_evt.channel != next_evt.channel));
}

static void setup_rx_timeout(uint64_t rx_start_time)
{
    timer_order_cb_ppi(TIMER_INDEX_RADIO, rx_start_time + RADIO_RX_TIMEOUT,
        rx_abort_cb,
        (uint32_t*) &(NRF_RADIO->TASKS_DISABLE));
    /* setup the address event to abort the rx timeout */
    NRF_PPI->CH[PPI_CH_STOP_RX_ABORT].EEP   = (uint32_t) &(NRF_RADIO->EVENTS_ADDRESS);
    NRF_PPI->CH[PPI_CH_STOP_RX_ABORT].TEP   = (uint32_t) &(NRF_TIMER0->TASKS_CAPTURE[TIMER_INDEX_RADIO]);
    NRF_PPI->CHENSET 			              = (1 << (PPI_CH_STOP_RX_ABORT));
}

/**
* One event just finished. Setup next event and propagate an
* event report to user space
*/
static void radio_transition_end(bool successful_transmission)
{
    bool crc_status = NRF_RADIO->CRCSTATUS;
    uint32_t crc = NRF_RADIO->RXCRC;

    /* pop the event that just finished */
    radio_event_t prev_evt;
    uint32_t error_code = fifo_pop(&radio_fifo, &prev_evt);
    APP_ERROR_CHECK(error_code);

    bool fly_through_disable = ((NRF_RADIO->SHORTS &
        (RADIO_SHORTS_DISABLED_RXEN_Msk | RADIO_SHORTS_DISABLED_TXEN_Msk)) > 0);

    NRF_RADIO->SHORTS = 0;

    DEBUG_RADIO_CLEAR_PIN(PIN_RADIO_STATE_RX);
    DEBUG_RADIO_CLEAR_PIN(PIN_RADIO_STATE_TX);


    if (fifo_is_empty(&radio_fifo))
    {
        radio_state = RADIO_STATE_DISABLED;
    }
    else
    {
        /* Take care of the upcoming event */

        bool start_manually = false;
        radio_event_t evt;
        fifo_peek(&radio_fifo, &evt);

        if (evt.start_time > 0)
        {
            uint32_t curr_time = timer_get_timestamp();

            if (evt.start_time < curr_time)
            {
                evt.start_time = 0;
            }
        }

        if (evt.start_time == 0)
        {
            NRF_RADIO->SHORTS = RADIO_SHORTS_READY_START_Msk;

            /* the ready->start shortcut doesn't work when we already are in IDLE */
            if (prev_evt.event_type == evt.event_type &&
                prev_evt.channel == evt.channel)
            {
                start_manually = true;
            }
        }
        else
        {
            timer_order_cb_ppi(TIMER_INDEX_RADIO, evt.start_time, setup_rx_timeout, (uint32_t*) &(NRF_RADIO->TASKS_START));
        }

        /* setup buffers and addresses */
        NRF_RADIO->PACKETPTR = (uint32_t) &evt.packet_ptr[0];

        if (evt.event_type == RADIO_EVENT_TYPE_RX ||
            evt.event_type == RADIO_EVENT_TYPE_RX_PREEMPTABLE)
        {
            DEBUG_RADIO_SET_PIN(PIN_RADIO_STATE_RX);
            NRF_RADIO->RXADDRESSES = evt.access_address;
            radio_state = RADIO_STATE_RX;

            NRF_RADIO->INTENSET = RADIO_INTENSET_ADDRESS_Msk;

            /* manually begin ramp up */
            if (!fly_through_disable)
            {
                radio_channel_set(evt.channel);
                NRF_RADIO->TASKS_RXEN = 1;
            }
        }
        else
        {
            DEBUG_RADIO_SET_PIN(PIN_RADIO_STATE_TX);
            NRF_RADIO->TXADDRESS = evt.access_address;
            radio_state = RADIO_STATE_TX;
            NRF_RADIO->INTENCLR = RADIO_INTENCLR_ADDRESS_Msk;

            /* manually begin ramp up */
            if (!fly_through_disable)
            {
                radio_channel_set(evt.channel);
                NRF_RADIO->TASKS_TXEN = 1;
            }
        }

        /* safe to kickstart it now */
        if (start_manually)
        {
            NRF_RADIO->TASKS_START = 1;
        }

        /* prepare shortcuts for next transmission */
        if (fifo_get_len(&radio_fifo) == 1)
        {
            NRF_RADIO->SHORTS |= RADIO_SHORTS_END_DISABLE_Msk;
        }
        else
        {
            /* More events after the upcoming one */
            radio_event_t next_evt;
            fifo_peek_at(&radio_fifo, &next_evt, 1);

            if (next_evt.event_type != evt.event_type ||
                next_evt.channel != evt.channel)
            {
                radio_channel_set(next_evt.channel);

                NRF_RADIO->SHORTS |= RADIO_SHORTS_END_DISABLE_Msk;

                /* make shortcut through disabled to accelerate the process */
                if (next_evt.event_type == RADIO_EVENT_TYPE_RX ||
                    next_evt.event_type == RADIO_EVENT_TYPE_RX_PREEMPTABLE)
                {
                    NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RXEN_Msk;
                }
                else /* shortcut to TX */
                {
                    NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
                }
            }
        }
    }

    /* send to super space */

    if (prev_evt.callback.tx != NULL)
    {
        CHECK_FP(prev_evt.callback.rx);
        if (prev_evt.event_type == RADIO_EVENT_TYPE_RX || 
            prev_evt.event_type == RADIO_EVENT_TYPE_RX_PREEMPTABLE)
        {
            (*prev_evt.callback.rx)(prev_evt.packet_ptr, successful_transmission && crc_status, crc);
            
            if (successful_transmission && crc_status)
                NRF_RADIO->EVENTS_END = 0;
        }
        else
        {
            (*prev_evt.callback.tx)(prev_evt.packet_ptr);
        }
    }
    else
    {
        APP_ERROR_CHECK(NRF_ERROR_NULL);
    }

    if (fifo_is_empty(&radio_fifo))
    {
        (*g_idle_cb)();
    }
}

static void rx_abort_cb(uint64_t timestamp)
{
    NRF_PPI->CHENCLR = (1 << (PPI_CH_STOP_RX_ABORT));
    radio_state = RADIO_STATE_DISABLED;
    radio_transition_end(false);
}

static void radio_wakeup(void)
{
    uint32_t events_in_queue = fifo_get_len(&radio_fifo);
    while (events_in_queue > 1)
    {
        radio_event_t current_evt;
        if (fifo_peek(&radio_fifo, &current_evt) == NRF_SUCCESS && 
            current_evt.event_type == RADIO_EVENT_TYPE_RX_PREEMPTABLE)
        {
            /* event is preemptable, stop it */
            fifo_pop(&radio_fifo, &current_evt);
            
            /* propagate failed rx event */
            (*current_evt.callback.rx)(current_evt.packet_ptr, false, 0xFFFFFFFF);
            radio_disable();
            --events_in_queue;
        }
        else
        {
            break;
        }
    }
    if (events_in_queue == 1)
    {
        /* order radio right away */
        radio_event_t radio_event;
        fifo_peek(&radio_fifo, &radio_event);

        radio_channel_set(radio_event.channel);

        NRF_RADIO->SHORTS = RADIO_SHORTS_END_DISABLE_Msk;

        if (radio_event.start_time > 0)
        {
            uint32_t curr_time = timer_get_timestamp();

            if (radio_event.start_time < curr_time)
            {
                radio_event.start_time = 0;
            }
        }

        NRF_RADIO->PACKETPTR = (uint32_t) &radio_event.packet_ptr[0];
        if (radio_event.event_type == RADIO_EVENT_TYPE_RX || 
            radio_event.event_type == RADIO_EVENT_TYPE_RX_PREEMPTABLE)
        {
            NRF_RADIO->TASKS_RXEN = 1;

            radio_state = RADIO_STATE_RX;
            NRF_RADIO->INTENSET = RADIO_INTENSET_ADDRESS_Msk;
            DEBUG_RADIO_SET_PIN(PIN_RADIO_STATE_RX);
        }
        else
        {
            NRF_RADIO->INTENCLR = RADIO_INTENCLR_ADDRESS_Msk;
            NRF_RADIO->TASKS_TXEN = 1;
            radio_state = RADIO_STATE_TX;
            DEBUG_RADIO_SET_PIN(PIN_RADIO_STATE_TX);
        }

        if (radio_event.start_time == 0)
        {
            NRF_RADIO->SHORTS |= RADIO_SHORTS_READY_START_Msk;
        }
        else
        {
            timer_order_cb_ppi(TIMER_INDEX_RADIO, radio_event.start_time, setup_rx_timeout, (uint32_t*) &(NRF_RADIO->TASKS_START));
        }

        NRF_RADIO->INTENSET = RADIO_INTENSET_END_Msk;

    }
    else
    {
        /* queue the event */
        if (!radio_will_go_to_disabled_state())
        {
            uint8_t queue_length = fifo_get_len(&radio_fifo);

            if (queue_length == 2)
            {
                /* this event will come straight after the current */

                /* get current event */
                radio_event_t ev;
                fifo_peek(&radio_fifo, &ev);

                /* get next event */
                radio_event_t radio_event;
                fifo_peek_at(&radio_fifo, &radio_event, 1);

                /* setup shorts */
                if ((ev.event_type != RADIO_EVENT_TYPE_TX) == (radio_event.event_type != RADIO_EVENT_TYPE_TX))
                {
                    NRF_RADIO->SHORTS &= ~(RADIO_SHORTS_END_DISABLE_Msk);
                }
                else if (radio_event.event_type == RADIO_EVENT_TYPE_TX)
                {
                    NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_TXEN_Msk;
                }
                else /* going to TX */
                {
                    NRF_RADIO->SHORTS |= RADIO_SHORTS_DISABLED_RXEN_Msk;
                }
            }
        }
    }
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

void radio_init(uint32_t access_address, radio_idle_cb idle_cb)
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
    NRF_RADIO->PREFIX0	    = ((access_address >> 24) & 0x000000FF);
    NRF_RADIO->BASE0 		= ((access_address << 8) & 0xFFFFFF00);
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
    if (radio_state == RADIO_STATE_NEVER_USED)
    {        
        /* this flushes the queue */ 
        radio_fifo.array_len = RADIO_FIFO_QUEUE_SIZE;
        radio_fifo.elem_array = radio_fifo_queue;
        radio_fifo.elem_size = sizeof(radio_event_t);
        radio_fifo.memcpy_fptr = NULL;
        fifo_init(&radio_fifo);
    }

    radio_state = RADIO_STATE_DISABLED;
    
    NVIC_ClearPendingIRQ(RADIO_IRQn);
    NVIC_EnableIRQ(RADIO_IRQn);
    g_idle_cb = idle_cb;
    
    DEBUG_RADIO_CLEAR_PIN(PIN_RADIO_STATE_RX);
    DEBUG_RADIO_CLEAR_PIN(PIN_RADIO_STATE_TX);
    
    if (fifo_is_empty(&radio_fifo))
    {
        (*g_idle_cb)();
    }
    else
    {        
        if (timeslot_is_in_ts())
        {
            forced_wakeup = true;
            NVIC_SetPendingIRQ(RADIO_IRQn);
        }
    }
}

bool radio_order(radio_event_t* radio_event)
{
    if (fifo_push(&radio_fifo, radio_event) != NRF_SUCCESS)
    {
        return false;
    }
    
    /* trigger radio callback */
    uint32_t was_masked;
    DISABLE_IRQS(was_masked);
    
    if (timeslot_is_in_ts())
    {
        forced_wakeup = true;
        NVIC_SetPendingIRQ(RADIO_IRQn);
    }
    
    if (!was_masked) ENABLE_IRQS();
    
    return true;
}



void radio_disable(void)
{
    DEBUG_RADIO_CLEAR_PIN(PIN_RADIO_STATE_RX);
    DEBUG_RADIO_CLEAR_PIN(PIN_RADIO_STATE_TX);
    NRF_RADIO->SHORTS = 0;
    NRF_RADIO->INTENCLR = 0xFFFFFFFF;
    NRF_RADIO->TASKS_DISABLE = 1;
    radio_state = RADIO_STATE_DISABLED;
    timer_abort(TIMER_INDEX_RADIO);
}

/**
* IRQ handler for radio. Sends the radio around the state machine, ensuring secure radio state changes
*/
void radio_event_handler(void)
{
    if (forced_wakeup)
    {
        forced_wakeup = false;
        /* triggered wakeup event */
        radio_wakeup();
    }
    if (RADIO_EVENT(EVENTS_END))
    {
        NRF_RADIO->EVENTS_END = 0;
        radio_transition_end(true);
    }
    switch (radio_state)
    {
        case RADIO_STATE_RX:
            if (RADIO_EVENT(EVENTS_ADDRESS))
            {
                timer_abort(TIMER_INDEX_RADIO);
            }

        case RADIO_STATE_TX:
            break;
        case RADIO_STATE_DISABLED:
            break;
        case RADIO_STATE_NEVER_USED:
            break;
    }

    NRF_RADIO->EVENTS_READY = 0;
    NRF_RADIO->EVENTS_ADDRESS = 0;
    NRF_RADIO->EVENTS_PAYLOAD = 0;
    NRF_RADIO->EVENTS_END = 0;
    NRF_RADIO->EVENTS_DISABLED = 0;
}

