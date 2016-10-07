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

#include "timer.h"

#include "event_handler.h"
#include "toolchain.h"
#include "app_error.h"

#include "nrf.h"

#define TIMER_COMPARE_COUNT     (3)

/** Time from timeslot API starts the TIMER0 until we are sure we have had time to set all timeouts. */
#define TIMER_TS_BEGIN_MARGIN_US    (120)
/*****************************************************************************
* Static globals
*****************************************************************************/
/** Bitfield for the attributes given to the timer */
static timer_attr_t     m_attributes[TIMER_COMPARE_COUNT];
/** Array of function pointers for callbacks for each timer. */
static timer_callback_t m_callbacks[TIMER_COMPARE_COUNT];
/** Array of PPI tasks to trigger on timeout. */
static uint32_t*        mp_ppi_tasks[TIMER_COMPARE_COUNT];
/** Timestamps set for each timeout.  */
static timestamp_t      m_timeouts[TIMER_COMPARE_COUNT];
/** Time from which the TIMER0 is started. */
static timestamp_t      m_reference_time;
/** Time captured at the end of the previous timeslot. */
static timestamp_t      m_ts_end_time;
/** Timeslot currently in progress. */
static bool             m_is_in_ts;
/** Timer mutex. */
static uint32_t         m_timer_mut;
/*****************************************************************************
* Static functions
*****************************************************************************/
static void timer_set(uint8_t timer, timestamp_t timeout)
{
    APP_ERROR_CHECK_BOOL(timer < 2);
    NRF_TIMER0->INTENSET  = (1 << (TIMER_INTENSET_COMPARE0_Pos + timer));
    NRF_TIMER0->CC[timer] = timeout;
    NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
    (void)NRF_TIMER0->EVENTS_COMPARE[timer];
}

/** Implement mutex lock, the SD-mut is hidden behind SVC, and cannot be used in IRQ level <= 1.
    While the mutex is locked, the timer is unable to receive timer interrupts, and the
    timers may safely be changed */
static inline void timer_mut_lock(void)
{
    _DISABLE_IRQS(m_timer_mut);
}

/** Implement mutex unlock, the SD-mut is hidden behind SVC, and cannot be used in IRQ level <= 1 */
static inline void timer_mut_unlock(void)
{
    _ENABLE_IRQS(m_timer_mut);
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
void timer_event_handler(void)
{
    for (uint32_t i = 0; i < TIMER_COMPARE_COUNT; ++i)
    {
        if (NRF_TIMER0->EVENTS_COMPARE[i])
        {
            timer_callback_t cb = m_callbacks[i];
            APP_ERROR_CHECK_BOOL(cb != NULL);
            m_callbacks[i] = NULL;
            NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + i));
            timestamp_t time_now = timer_now();
            if (m_attributes[i] & TIMER_ATTR_SYNCHRONOUS)
            {
                cb(time_now);
            }
            else
            {
                async_event_t evt;
                evt.type = EVENT_TYPE_TIMER;
                evt.callback.timer.cb = cb;
                evt.callback.timer.timestamp = time_now;
                event_handler_push(&evt);
            }
            NRF_TIMER0->EVENTS_COMPARE[i] = 0;
            (void) NRF_TIMER0->EVENTS_COMPARE[i];
            if (i == 0)
            {
                break;
            }
        }
    }
}

uint32_t timer_order_cb(uint8_t timer,
                        timestamp_t time,
                        timer_callback_t callback,
                        timer_attr_t attributes)
{
    if (timer >= TIMER_COMPARE_COUNT)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if ((attributes & (TIMER_ATTR_SYNCHRONOUS | TIMER_ATTR_TIMESLOT_LOCAL)) != attributes)
    {
        return NRF_ERROR_INVALID_FLAGS;
    }

    timer_mut_lock();

    m_callbacks[timer] = callback;
    m_timeouts[timer] = time;
    m_attributes[timer] = attributes;
    mp_ppi_tasks[timer] = NULL;

    if (m_is_in_ts)
    {
        timer_set(timer, TIMER_DIFF(time, m_reference_time));
    }

    timer_mut_unlock();

    return NRF_SUCCESS;
}

uint32_t timer_order_cb_ppi(uint8_t timer,
                            timestamp_t time,
                            timer_callback_t callback,
                            uint32_t* p_task,
                            timer_attr_t attributes)
{
    if (timer >= TIMER_COMPARE_COUNT)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if ((attributes & (TIMER_ATTR_SYNCHRONOUS | TIMER_ATTR_TIMESLOT_LOCAL)) != attributes)
    {
        return NRF_ERROR_INVALID_FLAGS;
    }

    timer_mut_lock();

    m_callbacks[timer] = callback;
    m_timeouts[timer] = time;
    mp_ppi_tasks[timer] = p_task;
    m_attributes[timer] = attributes;

    if (m_is_in_ts)
    {
        timer_set(timer, TIMER_DIFF(time, m_reference_time));
        /* Setup PPI */
        NRF_PPI->CH[TIMER_PPI_CH_START + timer].EEP   = (uint32_t) &(NRF_TIMER0->EVENTS_COMPARE[timer]);
        NRF_PPI->CH[TIMER_PPI_CH_START + timer].TEP   = (uint32_t) p_task;
        NRF_PPI->CHENSET 			                  = (1 << (TIMER_PPI_CH_START + timer));
    }

    timer_mut_unlock();

    return NRF_SUCCESS;
}

uint32_t timer_order_ppi(uint8_t timer, timestamp_t time, uint32_t* p_task, timer_attr_t attributes)
{
    if (timer >= TIMER_COMPARE_COUNT)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if ((attributes & (TIMER_ATTR_TIMESLOT_LOCAL)) != attributes)
    {
        return NRF_ERROR_INVALID_FLAGS;
    }

    timer_mut_lock();

    m_timeouts[timer] = time;
    mp_ppi_tasks[timer] = p_task;
    m_attributes[timer] = attributes;

    if (m_is_in_ts)
    {
        /* Setup PPI */
        NRF_PPI->CH[TIMER_PPI_CH_START + timer].EEP   = (uint32_t) &(NRF_TIMER0->EVENTS_COMPARE[timer]);
        NRF_PPI->CH[TIMER_PPI_CH_START + timer].TEP   = (uint32_t) p_task;
        NRF_PPI->CHENSET 			                  = (1 << (TIMER_PPI_CH_START + timer));
    }

    timer_mut_unlock();

    return NRF_SUCCESS;
}

uint32_t timer_abort(uint8_t timer)
{
    if (timer >= TIMER_COMPARE_COUNT)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (mp_ppi_tasks[timer] == NULL && m_callbacks[timer] == NULL)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    timer_mut_lock();
    if (timer < TIMER_COMPARE_COUNT)
    {
        m_callbacks[timer] = NULL;
        m_attributes[timer] = TIMER_ATTR_NONE;
        if (m_is_in_ts)
        {
            NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + timer));
            NRF_PPI->CHENCLR = (1 << (TIMER_PPI_CH_START + timer));
        }
    }
    timer_mut_unlock();

    return NRF_SUCCESS;
}

timestamp_t timer_now(void)
{
    timer_mut_lock();
    timestamp_t time = 0;
    if (m_is_in_ts)
    {
        NRF_TIMER0->EVENTS_COMPARE[TIMER_INDEX_TIMESTAMP] = 0;
        NRF_TIMER0->TASKS_CAPTURE[TIMER_INDEX_TIMESTAMP] = 1;
        time = NRF_TIMER0->CC[TIMER_INDEX_TIMESTAMP] + m_reference_time;
    }
    else
    {
        /* return the end of the previous TS */
        time = m_ts_end_time;
    }
    timer_mut_unlock();
    return time;
}

void timer_on_ts_begin(timestamp_t timeslot_start_time)
{
    /* executed in STACK_LOW */

    for (uint32_t i = 0; i < TIMER_COMPARE_COUNT; ++i)
    {
        NRF_TIMER0->CC[i] = 0;
        NRF_TIMER0->EVENTS_COMPARE[i] = 0;
        (void) NRF_TIMER0->EVENTS_COMPARE[i];
        /* Timer already timed out, execute immediately. */
        if (
            TIMER_DIFF(m_timeouts[i], m_reference_time) <
            TIMER_DIFF(timeslot_start_time + TIMER_TS_BEGIN_MARGIN_US, m_reference_time)
           )
        {
            if (m_callbacks[i] != NULL)
            {
                timer_callback_t cb = m_callbacks[i];
                m_callbacks[i] = NULL;
                if (m_attributes[i] & TIMER_ATTR_SYNCHRONOUS)
                {
                    cb(timeslot_start_time);
                }
                else
                {
                    async_event_t evt;
                    evt.type = EVENT_TYPE_TIMER;
                    evt.callback.timer.cb = cb;
                    evt.callback.timer.timestamp = timeslot_start_time;
                    event_handler_push(&evt);
                }
            }
            if (mp_ppi_tasks[i] != NULL)
            {
                /* trigger expired tasks */
                *mp_ppi_tasks[i] = 1;
                mp_ppi_tasks[i] = NULL;
            }
        }
        else
        {
            /* reschedule timers to fit new reference */
            if (m_callbacks[i] != NULL)
            {
                timer_set(i, TIMER_DIFF(m_timeouts[i], timeslot_start_time));
            }
        }
    }

    /* only enable interrupts if we're not locked. */
    if (!m_timer_mut)
    {
#if defined(NRF51) || defined(NRF52)
        NVIC_EnableIRQ(TIMER0_IRQn);
#endif
    }

    m_reference_time = timeslot_start_time;
    m_is_in_ts = true;
}

void timer_on_ts_end(timestamp_t timeslot_end_time)
{
    /* executed in STACK_LOW */
    /* purge ts-local timers */
    for (uint32_t i = 0; i < TIMER_COMPARE_COUNT; ++i)
    {
        if (m_attributes[i] & TIMER_ATTR_TIMESLOT_LOCAL)
        {
            m_attributes[i] = TIMER_ATTR_NONE;
            m_callbacks[i] = NULL;
            mp_ppi_tasks[i] = NULL;
        }
    }
    m_ts_end_time = timeslot_end_time;
    m_is_in_ts = false;
}

