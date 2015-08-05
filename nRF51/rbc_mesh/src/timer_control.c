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

#include "timer_control.h"
#include "rbc_mesh_common.h"

#include "timeslot_handler.h"
#include "event_handler.h"

#include "app_error.h"
#include "nrf_soc.h"

#include "nrf51_bitfields.h"



/*****************************************************************************
* Static globals
*****************************************************************************/

static uint8_t active_callbacks;

static uint8_t reference_channel = 0xFF;

static uint8_t never_used_bitmap = 0xFF;

/* bitmap indicating that callback should be executed in handlers interrupt
 context, instead of swi context */
static uint8_t sync_exec_bitmap = 0;

static int32_t reference_offset;

static timer_callback callbacks[4];

static uint32_t reference_point = 0;

/*****************************************************************************
* Static functions
*****************************************************************************/

/** @brief cycle through timer slots to find one that is available */
static uint8_t get_available_timer(void)
{
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (NRF_TIMER0->EVENTS_COMPARE[i] == 1 || (never_used_bitmap & (1 << i)))
        {
            /* wipe out all information that may cause misfire */
            never_used_bitmap &= ~(1 << i);
            NRF_PPI->CHENCLR = (1 << (TIMER_PPI_CH_START + i));
            active_callbacks &= ~(1 << i);
            NRF_TIMER0->EVENTS_COMPARE[i] = 0;
            return i;
        }
    }

    return 0xFF;
}



/*****************************************************************************
* Interface functions
*****************************************************************************/

void timer_event_handler(void)
{
    /* check if this is about the reference time */
    if (reference_channel != 0xFF && NRF_TIMER0->EVENTS_COMPARE[reference_channel])
    {
        NRF_TIMER0->EVENTS_COMPARE[reference_channel] = 0;
        reference_point = NRF_TIMER0->CC[reference_channel] + reference_offset;
        NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + reference_channel));
        NRF_PPI->CHENCLR  = (1 << (TIMER_PPI_CH_START + reference_channel));

        reference_channel = 0xFF;
        reference_offset = 0;
    }

    for (uint8_t i = 0; i < 4; ++i)
    {
        if ((active_callbacks & (1 << i)) && NRF_TIMER0->EVENTS_COMPARE[i])
        {
            timer_callback cb = callbacks[i];
            active_callbacks &= ~(1 << i);
            NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + i));


            CHECK_FP(cb);

            if (sync_exec_bitmap & (1 << i))
            {
                sync_exec_bitmap &= ~(1 << i);
                (*cb)();
            }
            else
            {
                /* propagate evt */
                async_event_t evt;
                evt.type = EVENT_TYPE_TIMER;
                evt.callback.timer = cb;
                event_handler_push(&evt);
            }
        }
    }

}

uint8_t timer_order_cb(uint32_t time, timer_callback callback)
{
    uint8_t timer = get_available_timer();

    if (timer == 0xFF)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }

    NRF_TIMER0->CC[timer] = reference_point + time;
    NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
    NRF_TIMER0->INTENSET  = (1 << (TIMER_INTENSET_COMPARE0_Pos + timer));
    callbacks[timer] = callback;
    active_callbacks |= (1 << timer);


    return timer;
}

uint8_t timer_order_cb_sync_exec(uint32_t time, timer_callback callback)
{
    /* just calling timer_order_cb and setting flag here creates a race condition,
        needs full impl.*/
    uint8_t timer = get_available_timer();

    if (timer == 0xFF)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }

    sync_exec_bitmap |= (1 << timer);

    NRF_TIMER0->CC[timer] = reference_point + time;
    NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
    NRF_TIMER0->INTENSET  = (1 << (TIMER_INTENSET_COMPARE0_Pos + timer));
    callbacks[timer] = callback;
    active_callbacks |= (1 << timer);


    return timer;
}

uint8_t timer_order_cb_ppi(uint32_t time, timer_callback callback, uint32_t* task)
{
    uint8_t timer = get_available_timer();

    if (time == 0xFF)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
    NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
    NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + timer));
    NRF_TIMER0->CC[timer] = reference_point + time;

    NRF_TIMER0->INTENSET = (1 << (TIMER_INTENSET_COMPARE0_Pos + timer));
    callbacks[timer] = callback;
    active_callbacks |= (1 << timer);

    /* Setup PPI */
    NRF_PPI->CH[TIMER_PPI_CH_START + timer].EEP   = (uint32_t) &(NRF_TIMER0->EVENTS_COMPARE[timer]);
	NRF_PPI->CH[TIMER_PPI_CH_START + timer].TEP   = (uint32_t) task;
	NRF_PPI->CHENSET 			                  = (1 << (TIMER_PPI_CH_START + timer));


    return timer;
}

uint8_t timer_order_ppi(uint32_t time, uint32_t* task)
{
    uint8_t timer = get_available_timer();

    if (time == 0xFF)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }

    NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
    NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + timer));
    NRF_TIMER0->CC[timer] = reference_point + time;

    /* Setup PPI */
    NRF_PPI->CH[TIMER_PPI_CH_START + timer].EEP   = (uint32_t) &(NRF_TIMER0->EVENTS_COMPARE[timer]);
	NRF_PPI->CH[TIMER_PPI_CH_START + timer].TEP   = (uint32_t) task;
	NRF_PPI->CHENSET 			                  = (1 << (TIMER_PPI_CH_START + timer));

    return timer;
}

void timer_abort(uint8_t timer_index)
{
    if (timer_index < 4)
    {
        NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + timer_index));
        active_callbacks &= ~(1 << timer_index);
        NRF_PPI->CHENCLR = (1 << (TIMER_PPI_CH_START + timer_index));
        never_used_bitmap |= (1 << timer_index);
    }
}

uint32_t timer_get_timestamp(void)
{
    uint8_t timer = get_available_timer();

    if (timer == 0xFF)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }

    NRF_TIMER0->TASKS_CAPTURE[timer] = 1;


    uint32_t stamp = NRF_TIMER0->CC[timer];

    never_used_bitmap |= (1 << timer);

    return stamp;
}

void timer_reference_point_trigger(uint32_t* trigger_event, int32_t time_offset)
{
    uint8_t timer = get_available_timer();
    if (timer == 0xFF)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }

    reference_channel = timer;
    reference_offset = time_offset;

    NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
    NRF_TIMER0->INTENSET = (1 << (TIMER_INTENSET_COMPARE0_Pos + timer));

    /* Setup PPI */
	NRF_PPI->CH[timer].EEP   = (uint32_t) trigger_event;
    NRF_PPI->CH[timer].TEP   = (uint32_t) &(NRF_TIMER0->TASKS_CAPTURE[timer]);
	NRF_PPI->CHENSET 		 = (1 << (TIMER_PPI_CH_START + timer));
}

uint32_t timer_get_reference_point(void)
{
    return reference_point;
}

void timer_reference_point_set(uint32_t ref_point)
{
    reference_point = ref_point;
}

void timer_init(void)
{
    timer_reference_point_set(0);
    never_used_bitmap = 0xFF;
    NRF_TIMER0->EVENTS_COMPARE[0] = 0;
    NRF_TIMER0->EVENTS_COMPARE[1] = 0;
    NRF_TIMER0->EVENTS_COMPARE[2] = 0;
    NRF_TIMER0->EVENTS_COMPARE[3] = 0;
    NVIC_EnableIRQ(TIMER0_IRQn);

    active_callbacks = 0;
}
