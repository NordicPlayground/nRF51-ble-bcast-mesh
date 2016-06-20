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
#ifndef TIMER_H__
#define TIMER_H__
#include <stdint.h>
#include <stdbool.h>

/**
* @defgroup TIMER HF Timer module abstraction.
* Allocates timers and schedules PPI signals in PPI channels 8-10. Can also do callbacks at timeout.
*/

/** First channel to use for timer PPI triggering */
#define TIMER_PPI_CH_START  (8)
/** Invalid timestamp */
#define TIMER_TIMEOUT_INVALID (0xFFFFFFFF)

/** Timer index for end of timeslot timing */
#define TIMER_INDEX_TS_END      (0)
/** Timer index for scheduler timing */
#define TIMER_INDEX_SCHEDULER   (1)
/** Timer index for radio timing */
#define TIMER_INDEX_RADIO       (2)
/** Timer index for getting timestamps */
#define TIMER_INDEX_TIMESTAMP   (3)

/** Get timestamp - ref, including rollover. */
#define TIMER_DIFF(timestamp, reference) ((uint32_t)(timestamp-reference) > UINT32_MAX / 2 ? (uint32_t)(reference-timestamp) : (uint32_t)(timestamp-reference))

/** Check whether the given time is older than the reference */
#define TIMER_OLDER_THAN(time, ref) (((uint32_t) (time)) - ((uint32_t) (ref)) > UINT32_MAX / 2)

/** Timestamp type for all time-values */
typedef uint32_t timestamp_t;

/**
 * Attribute bitfield fields to apply to a timer callback.
 */
typedef enum
{
    TIMER_ATTR_NONE             = 0x00, /**< No special attributes. */
    TIMER_ATTR_TIMESLOT_LOCAL   = 0x01, /**< The timer should not be transferred to the next timeslot if it fails to fire within the current timeslot. */
    TIMER_ATTR_SYNCHRONOUS      = 0x02, /**< The timer callback should be executed in TIMER0-IRQ context. Only applies to timers with callbacks. */
} timer_attr_t;

/** Callback type for callbacks at finished timers */
typedef void(*timer_callback_t)(timestamp_t timestamp);

/** Hardware event handler, should be called at all TIMER0 events. */
void timer_event_handler(void);

/**
 * Order a timer with callback.
 *
 * @param[in] timer Timer index to register timeout on. Overrides any previous timeout on this index.
 * @param[in] time Timestamp at which the action should trigger.
 * @param[in] callback Function pointer to the callback function called when the timer triggers.
 * @param[in] attributes Timer attributes to apply to the timer event. May be used as a bitfield.
 *
 * @return NRF_SUCCESS The callback was successfully scheduled.
 * @return NRF_ERROR_INVALID_FLAGS The supplied attributes were invalid.
 * @return NRF_ERROR_INVALID_PARAM The timer parameter was outside the range of the timer capture registers.
 */
uint32_t timer_order_cb(uint8_t timer,
                        timestamp_t time,
                        timer_callback_t callback,
                        timer_attr_t attributes);

/**
 * Order a timer with callback and a PPI task.
 *
 * @param[in] timer Timer index to register timeout on. Overrides any previous timeout on this index.
 * @param[in] time Timestamp at which the action should trigger.
 * @param[in] callback Function pointer to the callback function called when the timer triggers.
 * @param[in] p_task PPI task to trigger when the timer expires.
 * @param[in] attributes Timer attributes to apply to the timer event. May be used as a bitfield.
 *
 * @return NRF_SUCCESS The callback and PPI task were successfully scheduled.
 * @return NRF_ERROR_INVALID_FLAGS The supplied attributes were invalid.
 * @return NRF_ERROR_INVALID_PARAM The timer parameter was outside the range of the timer capture registers.
 */
uint32_t timer_order_cb_ppi(uint8_t timer,
                            timestamp_t time,
                            timer_callback_t callback,
                            uint32_t* p_task,
                            timer_attr_t attributes);

/**
 * Order a timer with a PPI task.
 *
 * @param[in] timer Timer index to register timeout on. Overrides any previous timeout on this index.
 * @param[in] time Timestamp at which the action should trigger.
 * @param[in] p_task PPI task to trigger when the timer expires.
 * @param[in] attributes Timer attributes to apply to the timer event. May be used as a bitfield.
 *
 * @return NRF_SUCCESS The PPI task was successfully scheduled.
 * @return NRF_ERROR_INVALID_FLAGS The supplied attributes were invalid.
 * @return NRF_ERROR_INVALID_PARAM The timer parameter was outside the range of the timer capture registers.
 */
uint32_t timer_order_ppi(uint8_t timer,
                         timestamp_t time,
                         uint32_t* p_task,
                         timer_attr_t attributes);

/**
 * Abort timer with given index.
 *
 * @param[in] timer Index to abort.
 *
 * @return NRF_SUCCESS The timer was successfully aborted.
 * @return NRF_ERROR_INVALID_PARAM The timer parameter was outside the range of the timer capture registers.
 * @return NRF_ERROR_NOT_FOUND The given timer wasn't scheduled.
 */
uint32_t timer_abort(uint8_t timer);

/**
* Get current timestamp from HF timer. This timestamp is directly
*   related to the time used to order timers, and may be used to order
*   relative timers.
*
* @return 32bit timestamp relative to global epoch.
*/
timestamp_t timer_now(void);

/**
* Initialize timer hardware. Must be called at the beginning of each
*   SD granted timeslot. Flushes all timer slots.
*
* @param[in] timeslot_start_time Timestamp for the start of the current
*            timeslot.
*/
void timer_on_ts_begin(timestamp_t timeslot_start_time);

/**
* Halt timer module operation. Must be called at the end of each
*   SD granted timeslot.
*
* @param[in] timeslot_end_time Timestamp for the end of the current
*            timeslot.
*/
void timer_on_ts_end(timestamp_t timeslot_end_time);

#endif /* TIMER_H__ */
