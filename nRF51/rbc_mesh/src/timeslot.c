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
#include "timeslot.h"

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "radio_control.h"
#include "timer.h"
#include "transport_control.h"
#include "event_handler.h"
#include "rbc_mesh_common.h"

#ifdef MESH_DFU
#include "dfu_app.h"
#include "mesh_flash.h"
#endif

#include "app_error.h"
#include "nrf_sdm.h"
#include "nrf_soc.h"


#define TIMESLOT_END_SAFETY_MARGIN_US       (1000)          /**< Allocated time between end timer timeout and actual timeslot end. */
#define TIMESLOT_SLOT_LENGTH_US             (10000)         /**< Base timeslot length. */
#define TIMESLOT_SLOT_EXTEND_LENGTH_US      (10000)       /**< Base extension length. */
#define TIMESLOT_SLOT_EMERGENCY_LENGTH_US   (6000)          /**< Base timeslot length for when a regular request is denied. */
#define TIMESLOT_TIMEOUT_DEFAULT_US         (50000)         /**< Timeout supplied to SD for "earliest" request. */
#define TIMESLOT_MAX_LENGTH_US              (10000000UL)    /**< The upper limit for timeslot extensions. */
#define TIMESLOT_MAX_LENGTH_FIRST_US        (10000UL)    /**< The upper limit for timeslot extensions for the first timeslot. */
#define RTC_MAX_TIME_TICKS                  (0xFFFFFF)      /**< RTC-clock rollover time. */

/*****************************************************************************
* Local type definitions
*****************************************************************************/
/**
 * Types of forced command sent to the signal handler.
 */
typedef enum
{
    TS_FORCED_COMMAND_NONE,     /** No command for the signal handler. */
    TS_FORCED_COMMAND_STOP,     /** Stop the current timeslot, and don't order a new one. */
    TS_FORCED_COMMAND_RESTART,  /** Stop the current timeslot, and ordern a new one as early as possible */
} ts_forced_command_t;

/*****************************************************************************
* Static globals
*****************************************************************************/

/** Timeslot earliest request */
static nrf_radio_request_t m_radio_request_earliest =
                {
                    .request_type = NRF_RADIO_REQ_TYPE_EARLIEST,
                    .params.earliest =
                    {
#if (NORDIC_SDK_VERSION >= 11)
                        .hfclk = NRF_RADIO_HFCLK_CFG_XTAL_GUARANTEED,
#else
                        .hfclk = NRF_RADIO_HFCLK_CFG_DEFAULT,
#endif
                        .priority = NRF_RADIO_PRIORITY_NORMAL,
                        .length_us = TIMESLOT_SLOT_LENGTH_US,
                        .timeout_us = TIMESLOT_TIMEOUT_DEFAULT_US
                    }
                };

static nrf_radio_signal_callback_return_param_t m_ret_param; /** Return parameter for SD radio signal handler. */
static timestamp_t          m_timeslot_length           = 0; /** Length of current timeslot (including extensions). */
static timestamp_t          m_start_time                = 0; /** Start time for current timeslot. */
static timestamp_t          m_negotiate_timeslot_length = TIMESLOT_SLOT_LENGTH_US; /** Length of the currently attempted TS length. */
static bool                 m_is_in_callback            = false; /** Code is in callback-context. */
static bool                 m_is_in_timeslot            = false; /** A timeslot is currently in progress. */
static bool                 m_framework_initialized     = false; /** The timeslot_init function has been called. */
static bool                 m_end_timer_triggered       = false; /** The timeslot end timer has been triggered, and the timeslot is about to end. */
static ts_forced_command_t  m_timeslot_forced_command   = TS_FORCED_COMMAND_NONE; /** Forced command, checked in radio signal callback. */
static uint32_t             m_lfclk_ppm                 = 250; /** The set drift accuracy for the LF clock source. */
static uint32_t             m_timeslot_count            = 0;

/*****************************************************************************
* Static Functions
*****************************************************************************/
static uint32_t end_timer_margin(void)
{
    return (m_timeslot_length * m_lfclk_ppm) / 1000000 + TIMESLOT_END_SAFETY_MARGIN_US;
}

static void ts_order_earliest(timestamp_t length_us)
{
    if (m_is_in_callback)
    {
        m_radio_request_earliest.params.earliest.length_us = length_us;
        m_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        m_ret_param.params.request.p_next = &m_radio_request_earliest;
    }
    else
    {
        sd_radio_request(&m_radio_request_earliest);
    }
    m_timeslot_length = length_us;
}

static void ts_extend(timestamp_t extra_time_us)
{
    if (m_is_in_callback)
    {
        if (m_timeslot_length + extra_time_us > TIMESLOT_MAX_LENGTH_US)
        {
            extra_time_us = TIMESLOT_MAX_LENGTH_US - m_timeslot_length;
        }
        m_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;
        m_ret_param.params.extend.length_us = extra_time_us;
    }
}

void start_time_update(void)
{
    static uint64_t s_last_rtc_value = 0;
    uint64_t rtc_time = NRF_RTC0->COUNTER;

    /* First run, no delta. */
    if(m_timeslot_count == 0)
    {
        s_last_rtc_value = rtc_time;
    }

    uint64_t delta_rtc_time;
    if(s_last_rtc_value > rtc_time)
    {
        delta_rtc_time = RTC_MAX_TIME_TICKS - s_last_rtc_value + rtc_time;
    }
    else
    {
        delta_rtc_time = rtc_time - s_last_rtc_value;
    }

    /* scale to microseconds */
    m_start_time += ((delta_rtc_time * 1000000) >> 15);
    s_last_rtc_value = rtc_time;
}

static void timeslot_end(void)
{
    radio_disable();
    timer_on_ts_end(timeslot_end_time_get());
    m_is_in_timeslot = false;
    m_is_in_callback = false;
    m_end_timer_triggered = false;
    CLEAR_PIN(PIN_IN_TS);
    CLEAR_PIN(PIN_IN_CB);
    
#ifdef NRF52
    NRF_TIMER0->TASKS_STOP = 0;
    NRF_TIMER0->TASKS_SHUTDOWN = 1;
    for (uint32_t i = 0; i < 6; i++)
    {
        NRF_TIMER0->EVENTS_COMPARE[i] = 0;
        NRF_TIMER0->CC[i] = 0;
    }
    NRF_TIMER0->INTENCLR = 0xFFFFFFFF;
    NVIC_ClearPendingIRQ(TIMER0_IRQn);
#endif
}

/*****************************************************************************
* System callback functions
*****************************************************************************/

/**
* Timeslot related events callback
*   Called whenever the softdevice tries to change the original course of actions
*   related to the timeslots
*/
void timeslot_sd_event_handler(uint32_t evt)
{
    switch (evt)
    {
        case NRF_EVT_RADIO_SESSION_IDLE:
            if (m_timeslot_forced_command != TS_FORCED_COMMAND_STOP)
            {
                ts_order_earliest(TIMESLOT_SLOT_LENGTH_US);
            }
            break;

        case NRF_EVT_RADIO_SESSION_CLOSED:
            break;

        case NRF_EVT_RADIO_BLOCKED:
            /* Something in the softdevice is blocking our requests,
               go into emergency mode, where slots are short, in order to
               avoid complete lockout. */
            ts_order_earliest(TIMESLOT_SLOT_EMERGENCY_LENGTH_US);
            break;

        case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
            APP_ERROR_CHECK_BOOL(false);
            break;

        case NRF_EVT_RADIO_CANCELED:
            ts_order_earliest(TIMESLOT_SLOT_LENGTH_US);
            break;
        default:
            break;
    }
}

/**
* Timeslot end guard timer callback.
*/
static void end_timer_handler(timestamp_t timestamp)
{
    m_end_timer_triggered = true;
}

/** Radio signal callback handler taking care of all SD radio signals */
static nrf_radio_signal_callback_return_param_t* radio_signal_callback(uint8_t sig)
{
    static uint32_t requested_extend_time = 0;
    static uint32_t successful_extensions = 0;
    SET_PIN(PIN_IN_CB);
    m_is_in_callback = true;

    switch (m_timeslot_forced_command)
    {
        case TS_FORCED_COMMAND_STOP:
            m_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_END;
            m_timeslot_count = 0;
            timeslot_end();
            return &m_ret_param;

        case TS_FORCED_COMMAND_RESTART:
            ts_order_earliest(TIMESLOT_SLOT_LENGTH_US);
            timeslot_end();
            m_timeslot_forced_command = TS_FORCED_COMMAND_NONE;
            return &m_ret_param;

        default:
            break;
    }

    m_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
    switch (sig)
    {
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
        {
            SET_PIN(PIN_IN_TS);
            m_is_in_timeslot = true;
            m_end_timer_triggered = false;
            successful_extensions = 0;

            start_time_update();

            /* notify other modules */
            event_handler_on_ts_begin();
            timer_on_ts_begin(m_start_time);
            tc_on_ts_begin();

            m_negotiate_timeslot_length = TIMESLOT_SLOT_EXTEND_LENGTH_US;

            timer_order_cb(TIMER_INDEX_TS_END, timeslot_start_time_get() + m_timeslot_length - end_timer_margin(),
                    end_timer_handler, (timer_attr_t) (TIMER_ATTR_SYNCHRONOUS | TIMER_ATTR_TIMESLOT_LOCAL));

            /* attempt to extend our time right away */
            ts_extend(m_negotiate_timeslot_length);

            /* increase timeslot-count, but skip =0 on rollover */
            if (!++m_timeslot_count)
            {
                m_timeslot_count++;
            }

            break;
        }
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
            /* send to radio control module */
            radio_event_handler();
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
            /* send to timer control module */
            timer_event_handler();
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
            m_timeslot_length += requested_extend_time;
            requested_extend_time = 0;
            ++successful_extensions;

            timer_abort(TIMER_INDEX_TS_END);

            timer_order_cb(TIMER_INDEX_TS_END, timeslot_start_time_get() + m_timeslot_length - end_timer_margin(),
                    end_timer_handler, (timer_attr_t) (TIMER_ATTR_SYNCHRONOUS | TIMER_ATTR_TIMESLOT_LOCAL));

            m_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;

            if (m_timeslot_count == 1)
            {
                if (m_timeslot_length + m_negotiate_timeslot_length < TIMESLOT_MAX_LENGTH_FIRST_US)
                {
                    ts_extend(m_negotiate_timeslot_length);
                }
            }
            else
            {
                if (m_timeslot_length + m_negotiate_timeslot_length < TIMESLOT_MAX_LENGTH_US)
                {
                    ts_extend(m_negotiate_timeslot_length);
                }
            }

            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
            m_negotiate_timeslot_length >>= 1;
            if (m_negotiate_timeslot_length > 1000)
            {
                ts_extend(m_negotiate_timeslot_length);
            }
            break;

        default:
            APP_ERROR_CHECK_BOOL(false);
    }

    if (m_end_timer_triggered)
    {
        ts_order_earliest(TIMESLOT_SLOT_LENGTH_US);
        timeslot_end();
    }
    else if (m_ret_param.callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND)
    {
        requested_extend_time = m_ret_param.params.extend.length_us;
    }
    else
    {
#ifdef MESH_DFU
        mesh_flash_op_execute(timeslot_remaining_time_get());
#endif
        requested_extend_time = 0;
    }

    m_is_in_callback = false;
    CLEAR_PIN(PIN_IN_CB);
    return &m_ret_param;
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/

#if (NORDIC_SDK_VERSION >= 11)
uint32_t timeslot_init(nrf_clock_lf_cfg_t lfclksrc)
#else
uint32_t timeslot_init(nrf_clock_lfclksrc_t lfclksrc)
#endif
{
    if (m_framework_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

#if (NORDIC_SDK_VERSION >= 11)
    switch (lfclksrc.xtal_accuracy)
    {
        case NRF_CLOCK_LF_XTAL_ACCURACY_20_PPM:
            m_lfclk_ppm = 20;
            break;
        case NRF_CLOCK_LF_XTAL_ACCURACY_30_PPM:
            m_lfclk_ppm = 30;
            break;
        case NRF_CLOCK_LF_XTAL_ACCURACY_50_PPM:
            m_lfclk_ppm = 50;
            break;
        case NRF_CLOCK_LF_XTAL_ACCURACY_75_PPM:
            m_lfclk_ppm = 75;
            break;
        case NRF_CLOCK_LF_XTAL_ACCURACY_100_PPM:
            m_lfclk_ppm = 100;
            break;
        case NRF_CLOCK_LF_XTAL_ACCURACY_150_PPM:
            m_lfclk_ppm = 150;
            break;
        case NRF_CLOCK_LF_XTAL_ACCURACY_250_PPM:
            m_lfclk_ppm = 250;
            break;
        case NRF_CLOCK_LF_XTAL_ACCURACY_500_PPM:
            m_lfclk_ppm = 500;
            break;
        default:
            m_lfclk_ppm = 250;
    }
#else
    switch (lfclksrc)
    {
        case NRF_CLOCK_LFCLKSRC_XTAL_100_PPM:
            m_lfclk_ppm = 100;
            break;
        case NRF_CLOCK_LFCLKSRC_XTAL_150_PPM:
            m_lfclk_ppm = 150;
            break;
        case NRF_CLOCK_LFCLKSRC_XTAL_20_PPM:
            m_lfclk_ppm = 20;
            break;
        case NRF_CLOCK_LFCLKSRC_XTAL_250_PPM:
            m_lfclk_ppm = 250;
            break;
        case NRF_CLOCK_LFCLKSRC_XTAL_30_PPM:
            m_lfclk_ppm = 30;
            break;
        case NRF_CLOCK_LFCLKSRC_XTAL_500_PPM:
            m_lfclk_ppm = 500;
            break;
        case NRF_CLOCK_LFCLKSRC_XTAL_50_PPM:
            m_lfclk_ppm = 50;
            break;
        case NRF_CLOCK_LFCLKSRC_XTAL_75_PPM:
            m_lfclk_ppm = 75;
            break;
        default: /* all RC-sources are 250 */
            m_lfclk_ppm = 250;
    }
#endif

    m_is_in_callback = false;
    m_framework_initialized = true;

    m_timeslot_length = TIMESLOT_SLOT_LENGTH_US;

    return NRF_SUCCESS;
}

void timeslot_stop(void)
{
    APP_ERROR_CHECK(sd_radio_session_close());

    m_timeslot_forced_command = TS_FORCED_COMMAND_STOP;
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
#ifdef NRF51
    if (m_is_in_timeslot)
    {
        NVIC_SetPendingIRQ(RADIO_IRQn);
    }
#endif
    _ENABLE_IRQS(was_masked);
}

void timeslot_restart(void)
{
    m_timeslot_forced_command = TS_FORCED_COMMAND_RESTART;
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
#ifdef NRF51
    if (m_is_in_timeslot)
    {
        NVIC_SetPendingIRQ(RADIO_IRQn);
    }
#endif
    _ENABLE_IRQS(was_masked);
}

uint32_t timeslot_resume(void)
{
    APP_ERROR_CHECK(sd_radio_session_open(&radio_signal_callback));
    m_timeslot_forced_command = TS_FORCED_COMMAND_NONE;

    if (timeslot_is_in_ts())
    {
        return NRF_ERROR_INVALID_STATE;
    }
    ts_order_earliest(TIMESLOT_SLOT_LENGTH_US);
    return NRF_SUCCESS;
}

timestamp_t timeslot_start_time_get(void)
{
    return m_start_time;
}

timestamp_t timeslot_end_time_get(void)
{
    if (!m_is_in_timeslot)
    {
        return 0;
    }

    return m_timeslot_length + m_start_time;
}

timestamp_t timeslot_remaining_time_get(void)
{
    if (!m_is_in_timeslot)
    {
        return 0;
    }
    return TIMER_DIFF((m_timeslot_length + m_start_time), timer_now());
}

bool timeslot_is_in_ts(void)
{
    return m_is_in_timeslot;
}

