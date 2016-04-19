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

#ifndef _TIMESLOT_HANDLER_H__
#define _TIMESLOT_HANDLER_H__
#include "radio_control.h"
#include "timer_control.h"
#include "nrf_sdm.h"
#include <stdint.h>
#include <stdbool.h>

/**
* @file Module responsible for providing a safe interface to Softdevice 
*   timeslot API. Handles all system events, makes sure all timeslots are 
*   ended in time, provides some simple functions for manipulating the way 
*   timeslots behave. 
*/



/** @brief event handler for softdevice events */
void ts_sd_event_handler(uint32_t evt);

/** @brief initialize timeslot handler. Only called once */
void timeslot_handler_init(nrf_clock_lfclksrc_t lfclksrc);

/** 
* @brief order a timeslot as soon as possible.
* 
* @param[in] length_us Desired length of timeslot in microseconds
* @param[in] immediately Whether to wait for current timeslot to end before 
*   ordering. At the end of the timeslot, the last request is the one that's
*   processed.
*/
void timeslot_order_earliest(uint32_t length_us, bool immediately);

/** 
* @brief order a timeslot some time after the one before it
* 
* @param[in] length_us Desired length of timeslot in microseconds
* @param[in] distance_us Distance between start of previous and current timeslot
* @param[in] immediately Whether to wait for current timeslot to end before 
*   ordering. At the end of the timeslot, the last request is the one that's
*   processed.
*/
void timeslot_order_normal(uint32_t length_us, uint32_t distance_us, bool immediately);

/**
* @brief Extend current timeslot by some extra time 
*/
void timeslot_extend(uint32_t extra_time_us);

/** @brief Forcibly stop the timeslot execution */
void timeslot_stop(void);

/** @brief immediately end the current timeslot, and order a new one */
void timeslot_restart(void);

/** @brief returns the timestamp sampled at the beginning of the timeslot */
uint64_t timeslot_get_global_time(void);

/** @brief returns the timestamp the timeslot is set to end at */
uint64_t timeslot_get_end_time(void);

bool timeslot_is_in_ts(void);

#endif /* _TIMESLOT_HANDLER_H__ */
