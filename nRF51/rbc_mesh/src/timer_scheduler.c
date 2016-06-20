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
#include "timer_scheduler.h"
#include "event_handler.h"
#include "toolchain.h"
#include "rbc_mesh_common.h"
#include "nrf_error.h"
#include <stdio.h>

/** Time in us to regard as immidiate when firing several timers at once */
#define TIMER_MARGIN    (100)

/*****************************************************************************
* Local typedefs
*****************************************************************************/
typedef struct
{
    timer_event_t* p_head;
    uint32_t pending_reschedules;
} scheduler_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
static scheduler_t m_scheduler; /** Global scheduler instance */
/*****************************************************************************
* Static functions
*****************************************************************************/
static void timer_cb(timestamp_t timestamp);

static void add_evt(timer_event_t* p_evt)
{
    if (m_scheduler.p_head == NULL ||
        TIMER_OLDER_THAN(p_evt->timestamp, m_scheduler.p_head->timestamp))
    {
        p_evt->p_next = m_scheduler.p_head;
        m_scheduler.p_head = p_evt;
        return;
    }
    timer_event_t* p_temp = m_scheduler.p_head;

    while (p_temp->p_next &&
         TIMER_OLDER_THAN(p_temp->p_next->timestamp, p_evt->timestamp))
    {
        p_temp = p_temp->p_next;
    }

    p_evt->p_next = p_temp->p_next;
    p_temp->p_next = p_evt;
}

static uint32_t remove_evt(timer_event_t* p_evt)
{
    if (p_evt == m_scheduler.p_head)
    {
        m_scheduler.p_head = p_evt->p_next;
        p_evt->p_next = NULL;
        return NRF_SUCCESS;
    }

    timer_event_t* p_temp = m_scheduler.p_head;
    while (p_temp &&
           p_temp->p_next &&
           !TIMER_OLDER_THAN(p_evt->timestamp, p_temp->p_next->timestamp)
          )
    {
        if (p_temp->p_next == p_evt)
        {
            p_temp->p_next = p_evt->p_next;
            p_evt->p_next = NULL;
            return NRF_SUCCESS;
        }

        p_temp = p_temp->p_next;
    }

    return NRF_ERROR_NOT_FOUND;
}

static void fire_timers(timestamp_t time_now)
{
   if (m_scheduler.pending_reschedules)
   {
       return;
   }

   while (m_scheduler.p_head &&
       TIMER_OLDER_THAN(m_scheduler.p_head->timestamp, time_now + TIMER_MARGIN))
   {
       timer_event_t* p_evt = m_scheduler.p_head;
       async_event_t evt;
       evt.type = EVENT_TYPE_TIMER_SCH;
       evt.callback.timer_sch.cb = p_evt->cb;
       evt.callback.timer_sch.p_context = p_evt->p_context;
       evt.callback.timer_sch.timestamp = time_now;
       if (event_handler_push(&evt) != NRF_SUCCESS)
       {
           /* event queue full */
           return;
       }

        /* iterate */
        m_scheduler.p_head = p_evt->p_next;
        p_evt->p_next = NULL;

        if (p_evt->interval != 0)
        {
            do
            {
                p_evt->timestamp += p_evt->interval;
            } while (TIMER_OLDER_THAN(p_evt->timestamp, time_now + TIMER_MARGIN));

            add_evt(p_evt);
        }
   }
}

static void setup_timeout(timestamp_t time_now)
{
    if (m_scheduler.p_head)
    {
        if (TIMER_OLDER_THAN(time_now, m_scheduler.p_head->timestamp))
        {
            timer_order_cb(TIMER_INDEX_SCHEDULER, m_scheduler.p_head->timestamp, timer_cb, TIMER_ATTR_NONE);
        }
        else
        {
            timer_order_cb(TIMER_INDEX_SCHEDULER, time_now + TIMER_MARGIN, timer_cb, TIMER_ATTR_NONE);
        }
    }
}

static void async_schedule(void* p_context)
{
    TICK_PIN(3);
    timer_event_t* p_evt = (timer_event_t*) p_context;
    timestamp_t time_now = timer_now();
    add_evt(p_evt);

    fire_timers(time_now);
    setup_timeout(time_now);
}

static void async_remove(void* p_context)
{
    TICK_PIN(3);
    timestamp_t time_now = timer_now();
    remove_evt(p_context);
    setup_timeout(time_now);
}

static void async_reschedule(void* p_context)
{
    TICK_PIN(3);
    timestamp_t time_now = timer_now();
    remove_evt(p_context);
    add_evt(p_context);

    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    m_scheduler.pending_reschedules--;

    _ENABLE_IRQS(was_masked);

    fire_timers(time_now);
    setup_timeout(time_now);
}

static void timer_cb(timestamp_t timestamp)
{
    fire_timers(timestamp);
    setup_timeout(timestamp);
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
uint32_t timer_sch_init(void)
{
    m_scheduler.p_head = NULL;
    return NRF_SUCCESS;
}

uint32_t timer_sch_schedule(timer_event_t* p_timer_evt)
{
    if (p_timer_evt == NULL)
    {
        return NRF_ERROR_NULL;
    }
    p_timer_evt->p_next = NULL; /* sanitize linked list pointer */
    async_event_t evt;
    evt.type = EVENT_TYPE_GENERIC;
    evt.callback.generic.cb = async_schedule;
    evt.callback.generic.p_context = p_timer_evt;
    return event_handler_push(&evt);
}

uint32_t timer_sch_abort(timer_event_t* p_timer_evt)
{
    if (p_timer_evt == NULL)
    {
        return NRF_ERROR_NULL;
    }
    async_event_t evt;
    evt.type = EVENT_TYPE_GENERIC;
    evt.callback.generic.cb = async_remove;
    evt.callback.generic.p_context = p_timer_evt;
    return event_handler_push(&evt);
}

uint32_t timer_sch_reschedule(timer_event_t* p_timer_evt, timestamp_t new_timeout)
{
    if (p_timer_evt == NULL)
    {
        return NRF_ERROR_NULL;
    }

    async_event_t evt;
    evt.type = EVENT_TYPE_GENERIC;
    evt.callback.generic.cb = async_reschedule;
    evt.callback.generic.p_context = p_timer_evt;
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);

    uint32_t error_code = event_handler_push(&evt);
    if (error_code == NRF_SUCCESS)
    {
        m_scheduler.pending_reschedules++;
        p_timer_evt->timestamp = new_timeout;
    }
    _ENABLE_IRQS(was_masked);
    return error_code;
}
