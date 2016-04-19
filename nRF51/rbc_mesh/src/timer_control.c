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

#include "timer_control.h"
#include "rbc_mesh_common.h"

#include "timeslot_handler.h"
#include "event_handler.h"

#include "app_error.h"
#include "nrf_soc.h"

#include "nrf51_bitfields.h"

#define TIMER_SAFE_START()    do\
    {\
        uint32_t was_masked;\
        _DISABLE_IRQS(was_masked);\
        if (!m_mut++ && is_in_ts)\
        {\
            NVIC_DisableIRQ(TIMER0_IRQn);\
        }\
        _ENABLE_IRQS(was_masked);\
    } while (0)
    
#define TIMER_SAFE_END()    do\
    {\
        uint32_t was_masked;\
        _DISABLE_IRQS(was_masked);\
        if (!--m_mut && is_in_ts)\
        {\
            NVIC_EnableIRQ(TIMER0_IRQn);\
        }\
        _ENABLE_IRQS(was_masked);\
    } while (0)
    

#define TIMER_COMPARE_COUNT     (3)
#define TIMEOUT_MARGIN_US       (200)

/*****************************************************************************
* Static globals
*****************************************************************************/

static uint8_t active_callbacks = 0;

/* bitmap indicating that callback should be executed in handlers interrupt
 context, instead of swi context */
static uint8_t sync_exec_bitmap = 0;

static timer_callback callbacks[3];

static bool is_in_ts = false;
static uint8_t m_mut = 0;
/*****************************************************************************
* Static functions
*****************************************************************************/
/*****************************************************************************
* Interface functions
*****************************************************************************/

void timer_event_handler(void)
{
    bool handled = false;
    for (uint32_t i = 0; i < 3; ++i)
    {
        if (!is_in_ts)
        {
            return;
        }
        if (NRF_TIMER0->EVENTS_COMPARE[i])
        {
            NRF_TIMER0->EVENTS_COMPARE[i] = 0;

            if (active_callbacks & (1 << i))
            {
                NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + i));
                timer_callback cb = callbacks[i];
                active_callbacks &= ~(1 << i);
                handled = true;
                CHECK_FP(cb);

                if (sync_exec_bitmap & (1 << i))
                {
                    sync_exec_bitmap &= ~(1 << i);
                    (*cb)(NRF_TIMER0->CC[i]);
                }
                else
                {
                    /* propagate evt */
                    async_event_t evt;
                    evt.type = EVENT_TYPE_TIMER;
                    evt.callback.timer.cb = cb;
                    evt.callback.timer.timestamp = NRF_TIMER0->CC[i];
                    event_handler_push(&evt);
                }
            }
        }
    }
    if (!handled)
    {
        //APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
    }
}

void timer_order_cb(uint8_t timer, uint32_t time, timer_callback callback)
{
    TIMER_SAFE_START();
    if (is_in_ts)
    {
        uint64_t time_now = timer_get_timestamp();
        if (time > time_now + TIMEOUT_MARGIN_US)
        {
            callbacks[timer] = callback;
            active_callbacks |= (1 << timer);
            NRF_TIMER0->CC[timer] = time;
            NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
            NRF_TIMER0->INTENSET  = (1 << (TIMER_INTENSET_COMPARE0_Pos + timer));
        }
        else
        {
            async_event_t evt;
            evt.type = EVENT_TYPE_TIMER;
            evt.callback.timer.cb = callback;
            evt.callback.timer.timestamp = time_now;
            event_handler_push(&evt);
        }
    }
    TIMER_SAFE_END();
}

void timer_order_cb_sync_exec(uint8_t timer, uint32_t time, timer_callback callback)
{
    TIMER_SAFE_START();
    if (is_in_ts)
    {
        uint64_t time_now = timer_get_timestamp();
        if (time > time_now + TIMEOUT_MARGIN_US)
        {
            sync_exec_bitmap |= (1 << timer);

            callbacks[timer] = callback;
            active_callbacks |= (1 << timer);
            NRF_TIMER0->CC[timer] = time;
            NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
            NRF_TIMER0->INTENSET  = (1 << (TIMER_INTENSET_COMPARE0_Pos + timer));
        }
        else
        {
            callback(time_now);
        }
    }
    TIMER_SAFE_END();
}

void timer_order_cb_ppi(uint8_t timer, uint32_t time, timer_callback callback, uint32_t* task)
{
    TIMER_SAFE_START(); 
    if (is_in_ts)
    {
        uint64_t time_now = timer_get_timestamp();
        if (time > time_now + TIMEOUT_MARGIN_US)
        {
            callbacks[timer] = callback;
            active_callbacks |= (1 << timer);
            NRF_TIMER0->CC[timer] = time;
            NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
            NRF_TIMER0->INTENSET  = (1 << (TIMER_INTENSET_COMPARE0_Pos + timer));
            /* Setup PPI */
            NRF_PPI->CH[TIMER_PPI_CH_START + timer].EEP   = (uint32_t) &(NRF_TIMER0->EVENTS_COMPARE[timer]);
            NRF_PPI->CH[TIMER_PPI_CH_START + timer].TEP   = (uint32_t) task;
            NRF_PPI->CHENSET 			                  = (1 << (TIMER_PPI_CH_START + timer));
        }
        else
        {
            async_event_t evt;
            evt.type = EVENT_TYPE_TIMER;
            evt.callback.timer.cb = callback;
            evt.callback.timer.timestamp = time_now;
            event_handler_push(&evt);
            *task = 1;
        }
    }
    TIMER_SAFE_END();
}

void timer_order_ppi(uint8_t timer, uint32_t time, uint32_t* task)
{
    TIMER_SAFE_START(); 
    if (is_in_ts)
    {
        NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
        NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + timer));
        NRF_TIMER0->CC[timer] = time;

        /* Setup PPI */
        NRF_PPI->CH[TIMER_PPI_CH_START + timer].EEP   = (uint32_t) &(NRF_TIMER0->EVENTS_COMPARE[timer]);
        NRF_PPI->CH[TIMER_PPI_CH_START + timer].TEP   = (uint32_t) task;
        NRF_PPI->CHENSET 			                  = (1 << (TIMER_PPI_CH_START + timer));
    }
    TIMER_SAFE_END();
}

void timer_abort(uint8_t timer)
{
    TIMER_SAFE_START();
    if (is_in_ts)
    {
        if (timer < TIMER_COMPARE_COUNT)
        {
            NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + timer));
            active_callbacks &= ~(1 << timer);
            NRF_PPI->CHENCLR = (1 << (TIMER_PPI_CH_START + timer));
        }
    }
    TIMER_SAFE_END();
}

uint32_t timer_get_timestamp(void)
{
    TIMER_SAFE_START(); 
    uint32_t time = 0;
    if (is_in_ts)
    {
        NRF_TIMER0->EVENTS_COMPARE[TIMER_INDEX_TIMESTAMP] = 0;
        NRF_TIMER0->TASKS_CAPTURE[TIMER_INDEX_TIMESTAMP] = 1;
        time = NRF_TIMER0->CC[TIMER_INDEX_TIMESTAMP];
    }
    TIMER_SAFE_END();
    return time;
}

void timer_init(void)
{
    active_callbacks = 0;
}

void timer_on_ts_begin(void)
{
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    NRF_TIMER0->EVENTS_COMPARE[1] = 0;
    NRF_TIMER0->EVENTS_COMPARE[2] = 0;
    NRF_TIMER0->EVENTS_COMPARE[3] = 0;
    active_callbacks = 0;
    sync_exec_bitmap = 0;
    is_in_ts = true;
}

void timer_on_ts_end(void)
{
    is_in_ts = false;
}
