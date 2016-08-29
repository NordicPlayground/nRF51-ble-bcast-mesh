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
#ifndef TIMESLOT_H__
#define TIMESLOT_H__

#include <stdint.h>
#include <stdbool.h>

#include "timer.h"
#include "nrf_sdm.h"

/**
 * @{
 * @defgroup TIMESLOT Timeslot handler
 *   Module responsible for providing a safe interface to Softdevice
 *   Timeslot API. Handles all system events, makes sure all timeslots are
 *   ended in time, provides some simple functions for manipulating the way
 *   timeslots behave.
 */

/**
 * Event handler for softdevice events.
 *
 * @param[in] evt Softdevice event to process.
 */
void timeslot_sd_event_handler(uint32_t evt);

/**
 * Initialize timeslot handler.
 *
 * @param[in] lfclksrc Low frequency clock source, as given to the softdevice.
 *
 * @return NRF_SUCCESS The timeslot module was successfully initialized.
 * @return NRF_ERROR_INVALID_STATE The timeslot module has already been
 *         initialized.
 */
#if (NORDIC_SDK_VERSION >= 11)
uint32_t timeslot_init(nrf_clock_lf_cfg_t lfclksrc);
#else
uint32_t timeslot_init(nrf_clock_lfclksrc_t lfclksrc);
#endif

/** Forcibly stop the timeslot execution */
void timeslot_stop(void);

/** Restart the current timeslot. */
void timeslot_restart(void);

/**
 * Resume a stopped timeslot.
 *
 * @return NRF_SUCCESS The timeslot was successfully resumed.
 * @return NRF_ERROR_INVALID_STATE There is already a timesot in progress.
 */
uint32_t timeslot_resume(void);

/**
 * Get the timestamp sampled at the beginning of the timeslot.
 *
 * @return The start of the current timeslot.
 */
timestamp_t timeslot_start_time_get(void);

/**
 * Get the timestamp for the projected end of the current timeslot.
 *
 * @return The projected end of the current timeslot.
 */
timestamp_t timeslot_end_time_get(void);

/**
 * Get the remaining time of the current timeslot.
 *
 * @return The remaining time of the current timeslot.
 */
timestamp_t timeslot_remaining_time_get(void);

/**
 * Get whether the framework is currently in a timeslot.
 *
 * @return Whether the framework is currently in a timeslot.
 */
bool timeslot_is_in_ts(void);

/** @} */

#endif /* TIMESLOT_H__ */
