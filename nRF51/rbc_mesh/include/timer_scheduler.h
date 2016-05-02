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
#ifndef TIMER_SCHEDULER_H__
#define TIMER_SCHEDULER_H__

#include <stdint.h>

#include "timer.h"

/**
 * @defgroup TIMER_SCHEDULER Asynchronous event scheduler.
 * Scalable event scheduling on the high frequency timer.
 * @{
 */

/**
 * Constant to use in the interval field of @ref timer_event_t if the timer is
 * to be regarded a single shot-timer.
 */
#define TIMER_EVENT_INTERVAL_SINGLE_SHOT    (0)

/** Function type for the generic scheduler timeout callback */
typedef void (*timer_sch_callback_t)(timestamp_t timestamp, void * p_context);

/**
 * Timer event structure for schedulable timer
 */
typedef struct timer_event
{
    timestamp_t         timestamp; /**< Timestamp at which to fire. Is updated by the scheduler if periodic.  */
    timer_sch_callback_t    cb;    /**< Callback function to call when the timer fires. Called asynchronously. */
    timestamp_t         interval;  /**< Interval in us between each fire for periodic timers, or 0 if single-shot */
    void *              p_context; /**< Pointer to data passed on to the callback. */
    struct timer_event* p_next;    /**< Pointer to next event in linked list. Only for internal usage. */
} timer_event_t;

/**
 * Initialize the scheduler module.
 *
 * @return NRF_SUCCESS The timer scheduler was successfully initialized.
 */
uint32_t timer_sch_init(void);

/**
 * Schedule a timer event.
 *
 * @param[in] p_timer_evt A pointer to a statically allocated timer event, which will be used as
 *  context for the schedulable event.
 *
 * @warning The structure parameters should not change after the structure has been given to the
 *  scheduler, as this may cause a race condition. If a change in timing is needed, please use the
 *  @ref timer_sch_reschedule function. If any other changes are needed, abort the event, change the
 *  parameter, and schedule it again.
 *
 * @return NRF_SUCCESS The event has been scheduled successfully.
 * @return NRF_ERROR_NULL The given event is a NULL-pointer.
 * @return NRF_ERROR_NO_MEM The asynchronous bearer event scheduler has run out of space, and the action
 *  cannot be scheduled for processing. Allow the bearer event module to process its queue.
 */
uint32_t timer_sch_schedule(timer_event_t* p_timer_evt);

/**
 * Abort a previously scheduled event.
 *
 * @param[in] p_timer_evt A pointer to a previously scheduled event.
 *
 * @return NRF_SUCCESS The event was successfully aborted.
 * @return NRF_ERROR_NULL The given event is a NULL-pointer.
 * @return NRF_ERROR_NO_MEM The asynchronous bearer event scheduler has run out of space, and the action
 *  cannot be scheduled for processing. Allow the bearer event module to process its queue.
 */
uint32_t timer_sch_abort(timer_event_t* p_timer_evt);

/**
 * Reschedule a previously scheduled event.
 *
 * @param[in] p_timer_evt A pointer to a previously scheduled event.
 * @param[in] new_timestamp When the event should time out, instead of the old time.
 *
 * @return NRF_SUCCESS The event was successfully aborted.
 * @return NRF_ERROR_NULL The given event is a NULL-pointer.
 * @return NRF_ERROR_NO_MEM The asynchronous bearer event scheduler has run out of space, and the action
 *  cannot be scheduled for processing. Allow the bearer event module to process its queue.
 */
uint32_t timer_sch_reschedule(timer_event_t* p_timer_evt, timestamp_t new_timestamp);

/** @} */

#endif /* TIMER_SCHEDULER_H__ */
