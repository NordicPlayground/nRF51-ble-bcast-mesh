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

/**
* @file: Timeslot handler imitation for the bootloader to work without the softdevice
*/

#include "timeslot_handler.h"
#include "timer_control.h"
#include "radio_control.h"
#include "mesh_packet.h"
#include "transport_control.h"
#include "event_handler.h"
#include "version_handler.h"
#include "nrf_soc.h"

#define TS_LENGTH   0x80000000 /* artificial timeslot length to prevent TIMER0 rollover */


static uint64_t g_global_time;
/*****************************************************************************
* Static Functions
*****************************************************************************/
static void s_timeslot_begin(void);

static void s_timer_init(void)
{
    NRF_TIMER0->BITMODE = 3;
    NRF_TIMER0->PRESCALER = 4;
    
    NVIC_SetPriority(TIMER0_IRQn, 0);
    NVIC_EnableIRQ(TIMER0_IRQn);
    
    NRF_TIMER0->TASKS_CLEAR = 1;
    NRF_TIMER0->TASKS_START = 1;
}

static void s_radio_init(void)
{
    NVIC_SetPriority(RADIO_IRQn, 0);
    NVIC_EnableIRQ(RADIO_IRQn);
}

static void s_timeslot_end(uint64_t timestamp)
{
    event_handler_on_ts_end();
    NRF_TIMER0->TASKS_CLEAR = 1;
    g_global_time += TS_LENGTH;
    
    s_timeslot_begin();
}

static void s_timeslot_begin(void)
{
    mesh_packet_on_ts_begin();
    event_handler_on_ts_begin();
    timer_init();
    tc_on_ts_begin();
    vh_on_timeslot_begin();
    timer_order_cb_sync_exec(TIMER_INDEX_TS_END, TS_LENGTH, s_timeslot_end);
    
}
/*****************************************************************************
* Interrupt handlers
*****************************************************************************/

/* The interrupt handlers are normally baked into the softdevice, but we must
 implement our own. */

void TIMER0_IRQHandler(void)
{
    timer_event_handler();
}

void RADIO_IRQHandler(void)
{
    radio_event_handler();
}
/*****************************************************************************
* Interface Functions
*****************************************************************************/
void timeslot_handler_init(nrf_clock_lfclksrc_t clock_source)
{
    g_global_time = 0;
    s_timer_init();
    s_radio_init();
    
    s_timeslot_begin();
}

void ts_sd_event_handler(uint32_t event)
{
    /* no implementation needed */
}

uint64_t timeslot_get_global_time(void)
{
    return g_global_time;
}

uint64_t timeslot_get_end_time(void)
{
    return g_global_time + TS_LENGTH;
}

void timeslot_restart(void)
{
    s_timeslot_end(0);
    s_timeslot_begin();
}

bool timeslot_is_in_ts(void)
{
    return true;
}
