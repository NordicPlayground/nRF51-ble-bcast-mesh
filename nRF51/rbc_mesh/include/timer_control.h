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

#ifndef _TIMER_CONTROL_H__
#define _TIMER_CONTROL_H__
#include <stdint.h>

/**
* @file HF Timer module abstraction. Allocates timers and schedules PPI
*   signals in PPI channels 8-11. Can also manually do callbacks at timeout,
*   should only be used for applications with soft realtime requirements. 
*   Callbacks may be executed both synchronously (in STACK LOW) and in SWI0 
*   (APP LOW). May set a reference point, to which all time is executed after.
* 
* @note All order functions return the index of the capture register they were 
*   allocated in, in the range 0-3.
*/

#define TIMER_PPI_CH_START  8
#define TIMER_TIMEOUT_INVALID (0xFFFFFFFF)

#define TIMER_INDEX_TS_END      (0)
#define TIMER_INDEX_VH          (1)
#define TIMER_INDEX_RADIO       (2)
#define TIMER_INDEX_TIMESTAMP   (3)

/** @brief Callback type for callbacks at finished timers */
typedef void(*timer_callback)(uint64_t timestamp);

/** @brief hardware event handler, should be called at all TIMER0 events */
void timer_event_handler(void);

/** @brief order a timer with callback, execute in APP LOW */
void timer_order_cb(uint8_t timer, uint32_t time, timer_callback callback);

/** @brief order a timer with callback, execute in event handler context */
void timer_order_cb_sync_exec(uint8_t timer, uint32_t time, timer_callback callback);

/** @brief order a timer with both async callback and PPI to trigger HW task */
void timer_order_cb_ppi(uint8_t timer, uint32_t time, timer_callback callback, uint32_t* task);

/** @brief order a timer with a PPI channel to trigger HW task */
void timer_order_ppi(uint8_t timer, uint32_t time, uint32_t* task);

/** 
* @brief abort timer with given index. 
*/
void timer_abort(uint8_t timer);

/**
* @brief Get current timestamp from HF timer. This timestamp is directly 
*   related to the time used to order timers, and may be used to order 
*   relative timers.
*/
uint32_t timer_get_timestamp(void);

/** 
* @brief initialize timer module. Must be called at the beginning of each 
*   SD granted timeslot. Flushes all timer slots.
*/
void timer_init(void);


void timer_on_ts_begin(void);

void timer_on_ts_end(void);

#endif /* _TIMER_CONTROL_H__ */
