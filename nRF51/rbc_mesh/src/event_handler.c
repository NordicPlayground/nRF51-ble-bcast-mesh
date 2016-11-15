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
#include "timeslot.h"
#include "transport_control.h"
#include "mesh_packet.h"
#include "fifo.h"
#include "nrf_soc.h"
#include "toolchain.h"
#include "handle_storage.h"
#include <string.h>
#include "rbc_mesh.h"


#define EVENT_HANDLER_IRQ       (QDEC_IRQn)



static fifo_t g_async_evt_fifo;

static async_event_t g_async_evt_fifo_buffer[RBC_MESH_INTERNAL_EVENT_QUEUE_LENGTH];
static fifo_t g_async_evt_fifo_ts;

static async_event_t g_async_evt_fifo_buffer_ts[RBC_MESH_INTERNAL_EVENT_QUEUE_LENGTH];
static bool g_is_initialized;
static uint32_t g_critical = 0;


#if defined(WITH_ACK_MASTER)
static uint32_t event_handler_counter[5] __attribute__((at(0x200031D0)))={0};
#endif

#if defined(WITHOUT_ACK_MASTER)
static uint32_t event_handler_counter[5]__attribute__((at(0x200028B8)))={0};
#endif

#if defined(WITH_ACK_SLAVE)
static uint32_t event_handler_counter[5] __attribute__((at(0x200028CC))) ={0};
#endif

#if defined(WITHOUT_ACK_SLAVE)
static uint32_t event_handler_counter[5] __attribute__((at(0x200028B4)))={0};
#endif

/**
* @brief execute asynchronous event, based on type
*/
static void async_event_execute(async_event_t* p_evt)
{

    switch (p_evt->type)
    {
        case EVENT_TYPE_TIMER:
            CHECK_FP(p_evt->callback.timer.cb);
            p_evt->callback.timer.cb(p_evt->callback.timer.timestamp);
        
            #if defined(WITH_ACK_MASTER) || defined (WITHOUT_ACK_MASTER)||defined(WITH_ACK_SLAVE)||defined(WITHOUT_ACK_SLAVE)
		    event_handler_counter[0]++;
            #endif
            break;
        case EVENT_TYPE_GENERIC:
            CHECK_FP(p_evt->callback.generic.cb);
            p_evt->callback.generic.cb(p_evt->callback.generic.p_context);
        
            #if defined(WITH_ACK_MASTER) || defined (WITHOUT_ACK_MASTER)||defined(WITH_ACK_SLAVE)||defined(WITHOUT_ACK_SLAVE)
		    event_handler_counter[1]++;
            #endif
        
            break;
        case EVENT_TYPE_PACKET:
            tc_packet_handler(p_evt->callback.packet.payload,
                              p_evt->callback.packet.crc,
                              p_evt->callback.packet.timestamp,
                              p_evt->callback.packet.rssi);
        
            #if defined(WITH_ACK_MASTER) || defined (WITHOUT_ACK_MASTER)||defined(WITH_ACK_SLAVE)||defined(WITHOUT_ACK_SLAVE)
		    event_handler_counter[2]++;
            #endif
        
            break;
        case EVENT_TYPE_SET_FLAG:
            handle_storage_flag_set(p_evt->callback.set_flag.handle,
                                    (handle_flag_t) p_evt->callback.set_flag.flag,
                                    p_evt->callback.set_flag.value);
            #if defined(WITH_ACK_MASTER) || defined (WITHOUT_ACK_MASTER)||defined(WITH_ACK_SLAVE)||defined(WITHOUT_ACK_SLAVE)
			event_handler_counter[3]++;
            #endif
        
            break;
        case EVENT_TYPE_TIMER_SCH:
            CHECK_FP(p_evt->callback.timer_sch.cb);
            p_evt->callback.timer_sch.cb(p_evt->callback.timer_sch.timestamp,
                                         p_evt->callback.timer_sch.p_context);
        
            #if defined(WITH_ACK_MASTER) || defined (WITHOUT_ACK_MASTER)||defined(WITH_ACK_SLAVE)||defined(WITHOUT_ACK_SLAVE)
		    event_handler_counter[4]++;
            #endif
            break;
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

        if (timeslot_is_in_ts()) /* in timeslot */
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
  
     g_async_evt_fifo.array_len =RBC_MESH_INTERNAL_EVENT_QUEUE_LENGTH ;
    g_async_evt_fifo.elem_array = g_async_evt_fifo_buffer;
    g_async_evt_fifo.elem_size = sizeof(async_event_t);
    g_async_evt_fifo.memcpy_fptr = NULL;
    fifo_init(&g_async_evt_fifo);

 
    g_async_evt_fifo_ts.array_len = RBC_MESH_INTERNAL_EVENT_QUEUE_LENGTH; 
    g_async_evt_fifo_ts.elem_array = g_async_evt_fifo_buffer_ts;
    g_async_evt_fifo_ts.elem_size = sizeof(async_event_t);
    g_async_evt_fifo_ts.memcpy_fptr = NULL;
    fifo_init(&g_async_evt_fifo_ts);

    NVIC_EnableIRQ(EVENT_HANDLER_IRQ);
#ifdef NRF51
    NVIC_SetPriority(EVENT_HANDLER_IRQ, 3);
#else
    NVIC_SetPriority(EVENT_HANDLER_IRQ, 6);
#endif
    g_is_initialized = true;
}


uint32_t event_handler_push(async_event_t* p_evt)
{
    if (p_evt == NULL)
    {
        return NRF_ERROR_NULL;
    }
    fifo_t* p_fifo = NULL;
    switch (p_evt->type)
    {
    case EVENT_TYPE_GENERIC:
    case EVENT_TYPE_PACKET:
    case EVENT_TYPE_SET_FLAG:
    case EVENT_TYPE_TIMER_SCH:
        p_fifo = &g_async_evt_fifo;
        break;
    case EVENT_TYPE_TIMER:
        p_fifo = &g_async_evt_fifo_ts;
        break;
    default:
        return NRF_ERROR_INVALID_PARAM;
    }
    uint32_t result = fifo_push(p_fifo, p_evt);
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

