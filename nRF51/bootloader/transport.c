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
#include <stddef.h>
#include <string.h>

#include "transport.h"
#include "radio_control.h"
#include "fifo.h"
#include "app_error.h"
#include "rand.h"
#include "bootloader_rtc.h"
#include "serial_handler.h"
#include "boards.h"

/******************************************************************************
* Static defines
******************************************************************************/
#define RADIO_RX_FIFO_LEN               (32)
#define RTC_MARGIN                      (2) /* ticks */
#define INTERVAL                        (3277) /* ticks */
#define REDUNDANCY_MAX                  (3)
#define TX_EVT_BITFIELD_HANDLE_START    (0xFFF0)
/******************************************************************************
* Static typedefs
******************************************************************************/
typedef struct
{
    mesh_packet_t* p_packet;
    uint32_t ticks_next;
    uint32_t ticks_start;
    uint8_t repeats;
    uint8_t count;
    uint8_t type;
    uint8_t redundancy;
} tx_t;
/******************************************************************************
* Static globals
******************************************************************************/
static rx_cb_t          m_rx_cb;
static tx_t             m_tx[TRANSPORT_TX_SLOTS];
static fifo_t           m_rx_fifo;
static mesh_packet_t*   m_rx_fifo_buf[RADIO_RX_FIFO_LEN];
static uint32_t         m_ticks_at_order_time;
static prng_t           m_prng;
static bool             m_started = false;
uint16_t                m_tx_evt_bitfield; /**< Bitfield of events for each handle in the reserved handle range 0xFFF0-0xFFFE. */
/******************************************************************************
* Static functions
******************************************************************************/
static void radio_tx_cb(uint8_t* p_data);
static void radio_rx_cb(uint8_t* p_data, bool success, uint32_t crc, uint8_t rssi);
static void radio_idle_cb(void);

static void set_next_tx(tx_t* p_tx)
{
    if (p_tx->type == TX_INTERVAL_TYPE_EXPONENTIAL)
    {
        uint32_t offset         = (INTERVAL <<  p_tx->count) - INTERVAL;
        uint32_t offset_next    = (INTERVAL << (p_tx->count + 1)) - INTERVAL;
        uint32_t diff = ((offset_next & RTC_MASK) - (offset & RTC_MASK)) & RTC_MASK;

        p_tx->ticks_next = (p_tx->ticks_start + offset +
            (rand_prng_get(&m_prng) % (diff / 2)) + diff / 2) & RTC_MASK;
    }
    else
    {
        const uint32_t interval_scaling = (p_tx->type == TX_INTERVAL_TYPE_REGULAR_SLOW ? 10 : 1);
        /* double interval for regulars */
        p_tx->ticks_next = (p_tx->ticks_start + (interval_scaling * 2 * INTERVAL * p_tx->count) +
            (rand_prng_get(&m_prng) % (interval_scaling * INTERVAL)) + interval_scaling * INTERVAL) & RTC_MASK;
    }
}

static void order_scan(void)
{
    radio_event_t evt;
    evt.event_type = RADIO_EVENT_TYPE_RX_PREEMPTABLE;
    if (!mesh_packet_acquire((mesh_packet_t**) &evt.packet_ptr))
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
    evt.access_address = 0;
    evt.channel = 37;
    if (radio_order(&evt) != NRF_SUCCESS)
    {
        mesh_packet_ref_count_dec((mesh_packet_t*) &evt.packet_ptr);
    }
}

static void radio_tx_cb(uint8_t* p_data)
{
#ifdef RBC_MESH_SERIAL
    mesh_adv_data_t* p_adv_data = mesh_packet_adv_data_get((mesh_packet_t*) p_data);
    if (p_adv_data)
    {
        /* Only do the tx event if the handle is set up for it. */
        if (p_adv_data->handle >= TX_EVT_BITFIELD_HANDLE_START &&
            transport_tx_evt_get(p_adv_data->handle))
        {
            serial_evt_t serial_evt;
            serial_evt.opcode = SERIAL_EVT_OPCODE_EVENT_TX;
            serial_evt.params.event_tx.handle = p_adv_data->handle;
            serial_evt.length = 3; /* opcode + Handle */
            serial_handler_event_send(&serial_evt);
        }
    }
#endif
    mesh_packet_ref_count_dec((mesh_packet_t*) p_data);
}

static void radio_rx_cb(uint8_t* p_data, bool success, uint32_t crc, uint8_t rssi)
{
    if (success &&
        fifo_push(&m_rx_fifo, &p_data) == NRF_SUCCESS)
    {
        NVIC_SetPendingIRQ(SWI0_IRQn);
    }
    else
    {
        mesh_packet_ref_count_dec((mesh_packet_t*) p_data);
    }
}

static void radio_idle_cb(void)
{
    if (m_started)
    {
        order_scan();
    }
}

static void order_next_rtc(void)
{
    const uint32_t ticks_now = NRF_RTC0->COUNTER;
    uint32_t earliest = (ticks_now - 1) & RTC_MASK;

    for (uint32_t i = 0; i < TRANSPORT_TX_SLOTS; ++i)
    {
        if (m_tx[i].p_packet != NULL &&
            ((m_tx[i].ticks_next - ticks_now) & RTC_MASK) <
            ((earliest - ticks_now) & RTC_MASK))
        {
            earliest = m_tx[i].ticks_next & RTC_MASK;
        }
    }

    if (earliest != ((ticks_now - 1) & RTC_MASK))
    {
        NRF_RTC0->INTENSET = (1 << (RTC_TRANSPORT_CH + RTC_INTENSET_COMPARE0_Pos));
        NRF_RTC0->EVENTS_COMPARE[RTC_TRANSPORT_CH] = 0;
        NRF_RTC0->CC[RTC_TRANSPORT_CH] = earliest;
        m_ticks_at_order_time = ticks_now;
    }
    else
    {
        NRF_RTC0->INTENCLR = (1 << (RTC_TRANSPORT_CH + RTC_INTENCLR_COMPARE0_Pos));
    }
}
/******************************************************************************
* IRQ handlers
******************************************************************************/
void SWI0_IRQHandler(void)
{
    mesh_packet_t* p_packet;
    while (fifo_pop(&m_rx_fifo, &p_packet) == NRF_SUCCESS)
    {
        APP_ERROR_CHECK_BOOL(mesh_packet_ref_count_get(p_packet) == 1);
        m_rx_cb(p_packet);
        mesh_packet_ref_count_dec(p_packet);
    }
}

void RADIO_IRQHandler(void)
{
    radio_event_handler();
}

/******************************************************************************
* Dummy functions
******************************************************************************/
bool timeslot_is_in_ts(void)
{
    return true;
}

/******************************************************************************
* Interface functions
******************************************************************************/
void transport_init(rx_cb_t rx_cb, uint32_t access_addr)
{
    m_started = false;
    m_rx_cb = rx_cb;
    m_tx_evt_bitfield = 0;
    memset(m_tx, 0, sizeof(tx_t) * TRANSPORT_TX_SLOTS);

    m_rx_fifo.elem_array = m_rx_fifo_buf;
    m_rx_fifo.elem_size = sizeof(mesh_packet_t*);
    m_rx_fifo.array_len = RADIO_RX_FIFO_LEN;
    m_rx_fifo.memcpy_fptr = NULL;
    fifo_init(&m_rx_fifo);
    mesh_packet_init();

    APP_ERROR_CHECK(rand_prng_seed(&m_prng));

    NVIC_SetPriority(SWI0_IRQn, 2);
    NVIC_EnableIRQ(SWI0_IRQn);

    NVIC_SetPriority(RADIO_IRQn, 0);

    radio_init(radio_idle_cb, radio_rx_cb, radio_tx_cb);
}

void transport_start(void)
{
    if (!m_started)
    {
        m_started = true;
        order_scan();
    }
}

bool transport_tx(mesh_packet_t* p_packet, uint8_t slot, uint8_t repeats, tx_interval_type_t type)
{
    if (type == TX_INTERVAL_TYPE_EXPONENTIAL && repeats > TX_REPEATS_EXPONENTIAL_MAX)
    {
        return false;
    }
    if (p_packet == NULL)
    {
        return false;
    }

    if (slot >= TRANSPORT_TX_SLOTS)
    {
        return false;
    }

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    tx_t* p_tx = &m_tx[slot];
    if (p_tx->p_packet)
    {
        mesh_packet_ref_count_dec(p_tx->p_packet);
    }
    mesh_packet_ref_count_inc(p_packet);
    p_tx->p_packet = p_packet;
    p_tx->repeats = repeats;
    p_tx->count = 0;
    p_tx->ticks_start = NRF_RTC0->COUNTER;
    p_tx->type = type;
    set_next_tx(p_tx);
    order_next_rtc();
    _ENABLE_IRQS(was_masked);
    return true;
}

void transport_tx_reset(uint8_t slot)
{
    if (slot < TRANSPORT_TX_SLOTS)
    {
        tx_t* p_tx = &m_tx[slot];
        p_tx->count = 0;
        p_tx->ticks_start = NRF_RTC0->COUNTER;
        set_next_tx(p_tx);
        order_next_rtc();
    }
}

void transport_tx_skip(uint8_t slot)
{
    NVIC_DisableIRQ(RTC0_IRQn);
    if (slot < TRANSPORT_TX_SLOTS)
    {
        tx_t* p_tx = &m_tx[slot];
        p_tx->redundancy++;
    }
    NVIC_EnableIRQ(RTC0_IRQn);
}

void transport_tx_abort(uint8_t slot)
{
    NVIC_DisableIRQ(RTC0_IRQn);

    if (slot < TRANSPORT_TX_SLOTS)
    {
        tx_t* p_tx = &m_tx[slot];
        mesh_packet_ref_count_dec(p_tx->p_packet);
        memset(p_tx, 0, sizeof(tx_t));

        order_next_rtc();
    }
    NVIC_EnableIRQ(RTC0_IRQn);
}

void transport_rtc_irq_handler(void)
{
    const uint32_t ticks_now = NRF_RTC0->COUNTER + RTC_MARGIN;

    for (uint32_t i = 0; i < TRANSPORT_TX_SLOTS; ++i)
    {
        if (m_tx[i].p_packet != NULL &&
            ((ticks_now - m_tx[i].ticks_next) & RTC_MASK) <=
            ((ticks_now - m_ticks_at_order_time) & RTC_MASK))
        {
            radio_event_t radio_evt;
            radio_evt.event_type = RADIO_EVENT_TYPE_TX;
            radio_evt.packet_ptr = (uint8_t*) m_tx[i].p_packet;
            radio_evt.access_address = 0;

#ifdef DEBUG_LEDS
            NRF_GPIO->OUT ^= LED_1;
#endif
            uint8_t radio_refs = 0;
            if (m_tx[i].redundancy < REDUNDANCY_MAX)
            {
                for (radio_evt.channel = 37; radio_evt.channel <= 39; ++radio_evt.channel)
                {
                    if (radio_order(&radio_evt) == NRF_SUCCESS)
                    {
                        radio_refs++;
                        mesh_packet_ref_count_inc(m_tx[i].p_packet);
                    }
                }
            }

            m_tx[i].redundancy = 0;

            if (m_tx[i].count++ == 0xFF)
            {
                m_tx[i].ticks_start = (m_tx[i].ticks_start + INTERVAL * 2 * 0x100) & RTC_MASK;
            }

            if (m_tx[i].count < m_tx[i].repeats || m_tx[i].repeats == TX_REPEATS_INF)
            {
                /* exponentially increasing intervals, geometric series. */
                set_next_tx(&m_tx[i]);
            }
            else
            {
                mesh_packet_ref_count_dec(m_tx[i].p_packet);
                memset(&m_tx[i], 0, sizeof(tx_t));
            }
        }
    }
    order_next_rtc();
}

void transport_tx_evt_set(uint16_t handle, bool value)
{
    if (value)
    {
        m_tx_evt_bitfield |= (1 << (handle - TX_EVT_BITFIELD_HANDLE_START));
    }
    else
    {
        m_tx_evt_bitfield &= ~(1 << (handle - TX_EVT_BITFIELD_HANDLE_START));
    }
}

bool transport_tx_evt_get(uint16_t handle)
{
    if (handle < TX_EVT_BITFIELD_HANDLE_START)
    {
        return false;
    }
    return !!(m_tx_evt_bitfield & (1 << (handle - TX_EVT_BITFIELD_HANDLE_START)));
}

