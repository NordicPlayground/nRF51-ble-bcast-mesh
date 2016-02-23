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
#include "event_handler.h"
#include "rbc_mesh_common.h"
#include "app_error.h"
#include "timeslot_handler.h"
#include "transport_control.h"
#include "mesh_packet.h"
#include "fifo.h"
#include "nrf_soc.h"
#include "toolchain.h"
#include <string.h>

#define EVENT_HANDLER_IRQ       (QDEC_IRQn)

#define ASYNC_EVENT_FIFO_QUEUE_SIZE (8)

static fifo_t g_async_evt_fifo;
static async_event_t g_async_evt_fifo_buffer[ASYNC_EVENT_FIFO_QUEUE_SIZE];
static fifo_t g_async_evt_fifo_ts;
static async_event_t g_async_evt_fifo_buffer_ts[ASYNC_EVENT_FIFO_QUEUE_SIZE];
static bool g_is_initialized;
static uint32_t g_critical = 0;

/**
* @brief execute asynchronous event, based on type
*/
static void async_event_execute(async_event_t* evt)
{

    switch (evt->type)
    {
        case EVENT_TYPE_TIMER:
            CHECK_FP(evt->callback.timer.cb);
            (*evt->callback.timer.cb)(evt->callback.timer.timestamp);
            break;
        case EVENT_TYPE_GENERIC:
            CHECK_FP(evt->callback.generic);
            (*evt->callback.generic)();
            break;
        case EVENT_TYPE_PACKET:
            tc_packet_handler(evt->callback.packet.payload,
                                evt->callback.packet.crc,
                                evt->callback.packet.timestamp,
                                evt->callback.packet.rssi);
        default:
            break;
    }
}

static bool event_fifo_pop(fifo_t* evt_fifo)
{
    SET_PIN(PIN_SWI0);
    async_event_t evt;
    uint32_t error_code = fifo_pop(evt_fifo, &evt);
    if (error_code == NRF_SUCCESS)
    {
        async_event_execute(&evt);
        CLEAR_PIN(PIN_SWI0);
        return true;
    }
    CLEAR_PIN(PIN_SWI0);
    return false;
}

/**
* @brief Async event dispatcher, works in APP LOW
*/
void QDEC_IRQHandler(void)
{
    while (true)
    {
        bool got_evt = false;

        got_evt |= event_fifo_pop(&g_async_evt_fifo);

        if (timeslot_get_end_time() > 0) /* in timeslot */
        {
            got_evt |= event_fifo_pop(&g_async_evt_fifo_ts);
        }

        if (!got_evt)
        {
            break;
        }
    }
}

void event_handler_init(void)
{
    if (g_is_initialized)
    {
        /* may be called twice when in serial mode, can safely skip the second time */
        return;
    }
    /* init event queues */
    g_async_evt_fifo.array_len = ASYNC_EVENT_FIFO_QUEUE_SIZE;
    g_async_evt_fifo.elem_array = g_async_evt_fifo_buffer;
    g_async_evt_fifo.elem_size = sizeof(async_event_t);
    g_async_evt_fifo.memcpy_fptr = NULL;
    fifo_init(&g_async_evt_fifo);

    g_async_evt_fifo_ts.array_len = ASYNC_EVENT_FIFO_QUEUE_SIZE;
    g_async_evt_fifo_ts.elem_array = g_async_evt_fifo_buffer_ts;
    g_async_evt_fifo_ts.elem_size = sizeof(async_event_t);
    g_async_evt_fifo_ts.memcpy_fptr = NULL;
    fifo_init(&g_async_evt_fifo_ts);

    NVIC_EnableIRQ(EVENT_HANDLER_IRQ);
    NVIC_SetPriority(EVENT_HANDLER_IRQ, 3);
    g_is_initialized = true;
}


uint32_t event_handler_push(async_event_t* evt)
{
    fifo_t* p_fifo = NULL;
    switch (evt->type)
    {
    case EVENT_TYPE_GENERIC:
    case EVENT_TYPE_PACKET:
        p_fifo = &g_async_evt_fifo;
        break;
    default:
        p_fifo = &g_async_evt_fifo_ts;
        break;
    }
    uint32_t result = fifo_push(p_fifo, evt);
    if (result != NRF_SUCCESS)
    {
        return result;
    }

    /* trigger IRQ */
    NVIC_SetPendingIRQ(EVENT_HANDLER_IRQ);

    return NRF_SUCCESS;
}



void event_handler_on_ts_end(void)
{
    fifo_flush(&g_async_evt_fifo_ts);
}

void event_handler_on_ts_begin(void)
{
    if (!fifo_is_empty(&g_async_evt_fifo) ||
        !fifo_is_empty(&g_async_evt_fifo_ts))
    {
        NVIC_SetPendingIRQ(EVENT_HANDLER_IRQ);
    }
}

void event_handler_critical_section_begin(void)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    if (!g_critical++)
    {
        NVIC_DisableIRQ(QDEC_IRQn);
    }
    _ENABLE_IRQS(was_masked);
}

void event_handler_critical_section_end(void)
{
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
    if (!--g_critical)
    {
        NVIC_EnableIRQ(QDEC_IRQn);
    }
    _ENABLE_IRQS(was_masked);
}

