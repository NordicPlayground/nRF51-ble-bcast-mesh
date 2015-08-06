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

#include "timeslot_handler.h"

#include "radio_control.h"
#include "trickle.h"
#include "rbc_mesh_common.h"
#include "mesh_srv.h"
#include "timer_control.h"
#include "transport_control.h"
#include "event_handler.h"

#include "nrf_sdm.h"
#include "app_error.h"
#include "nrf_assert.h"
#include "nrf_soc.h"
#include "fifo.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define TIMESLOT_END_SAFETY_MARGIN_US   (2000)
#define TIMESLOT_SLOT_LENGTH            (10000)
#define TIMESLOT_SLOT_EXTEND_LENGTH     (50000)
#define TIMESLOT_SLOT_EMERGENCY_LENGTH  (3000) /* will fit between two conn events */
#define TIMESLOT_MAX_LENGTH             (20000000UL) /* 20s */


/*****************************************************************************
* Local type definitions
*****************************************************************************/



/*****************************************************************************
* Static globals
*****************************************************************************/

/**
* Timeslot request structures
*/


static nrf_radio_request_t radio_request_normal =
                {
                    .request_type = NRF_RADIO_REQ_TYPE_NORMAL,
                    .params.normal =
                    {
                        .hfclk = NRF_RADIO_HFCLK_CFG_DEFAULT,
                        .priority = NRF_RADIO_PRIORITY_NORMAL,
                        .distance_us = 10000,
                        .length_us = TIMESLOT_SLOT_LENGTH
                    }
                };

static nrf_radio_request_t radio_request_earliest =
                {
                    .request_type = NRF_RADIO_REQ_TYPE_EARLIEST,
                    .params.earliest =
                    {
                        .hfclk = NRF_RADIO_HFCLK_CFG_DEFAULT,
                        .priority = NRF_RADIO_PRIORITY_NORMAL,
                        .length_us = TIMESLOT_SLOT_LENGTH,
                        .timeout_us = 50000 /* 10ms */
                    }
                };



static nrf_radio_signal_callback_return_param_t g_ret_param;
//static nrf_radio_signal_callback_return_param_t g_final_ret_param;

static bool g_is_in_callback = true;

static uint64_t g_timeslot_length;
static uint32_t g_timeslot_end_timer;
static uint64_t g_next_timeslot_length;
static uint64_t g_start_time_ref = 0;
static bool g_is_in_timeslot = false;
static bool g_framework_initialized = false;
static bool g_end_timer_triggered = false;
static uint32_t g_negotiate_timeslot_length = TIMESLOT_SLOT_LENGTH;


static volatile uint32_t ts_count = 0;

/*****************************************************************************
* Static Functions
*****************************************************************************/



/*****************************************************************************
* System callback functions
*****************************************************************************/

/**
* @brief Timeslot related events callback
*   Called whenever the softdevice tries to change the original course of actions
*   related to the timeslots
*/
void ts_sd_event_handler(void)
{
    uint32_t evt;
    SET_PIN(4);
    while (sd_evt_get(&evt) == NRF_SUCCESS)
    {
        PIN_OUT(evt, 32);
        switch (evt)
        {
            case NRF_EVT_RADIO_SESSION_IDLE:
                timeslot_order_earliest(TIMESLOT_SLOT_LENGTH, true);
                break;

            case NRF_EVT_RADIO_SESSION_CLOSED:
                APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA);

                break;

            case NRF_EVT_RADIO_BLOCKED:
                /* something in the softdevice is blocking our requests,
                go into emergency mode, where slots are short, in order to
                avoid complete lockout */
                timeslot_order_earliest(TIMESLOT_SLOT_EMERGENCY_LENGTH, true);
                break;

            case NRF_EVT_RADIO_SIGNAL_CALLBACK_INVALID_RETURN:
                APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA);
                break;

            case NRF_EVT_RADIO_CANCELED:
                timeslot_order_earliest(TIMESLOT_SLOT_LENGTH, true);
                break;
            default:
                APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
        }
    }
    CLEAR_PIN(4);
}

/**
* @brief Timeslot end guard timer callback. Attempts to extend the timeslot.
*/
static void end_timer_handler(void)
{
    g_end_timer_triggered = true;
}


/**
* @brief Radio signal callback handler taking care of all signals in searching
*   mode
*/
static nrf_radio_signal_callback_return_param_t* radio_signal_callback(uint8_t sig)
{
    static uint32_t requested_extend_time = 0;
    static uint32_t successful_extensions = 0;
    static uint64_t last_rtc_value = 0;
    static uint64_t time_now = 0;
    g_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
    g_is_in_callback = true;
    
    SET_PIN(3);

    switch (sig)
    {
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
        {
            SET_PIN(2);

            g_is_in_timeslot = true;
            g_end_timer_triggered = false;
            successful_extensions = 0;

            timer_init();

            g_negotiate_timeslot_length = TIMESLOT_SLOT_EXTEND_LENGTH;//g_timeslot_length;
            g_timeslot_length = g_next_timeslot_length;

            g_timeslot_end_timer =
                timer_order_cb_sync_exec(g_timeslot_length - TIMESLOT_END_SAFETY_MARGIN_US,
                    end_timer_handler);

            /* attempt to extend our time right away */
            timeslot_extend(g_negotiate_timeslot_length);

            /* sample RTC timer for trickle timing */
            uint32_t rtc_time = NRF_RTC0->COUNTER;

            /*First time the offset should be added*/
            if(last_rtc_value == 0)
            {
                last_rtc_value = g_start_time_ref;
            }

            /* Calculate delta rtc time */
            uint64_t delta_rtc_time;
            if(last_rtc_value > rtc_time)
            {
                delta_rtc_time = 0xFFFFFF - last_rtc_value + rtc_time;
            }
            else
            {
                delta_rtc_time = rtc_time - last_rtc_value;
            }
            /* Store last rtc time */
            last_rtc_value = rtc_time;


            /* scale to become us */
            time_now += ((delta_rtc_time << 15) / 1000);

            transport_control_timeslot_begin(time_now);


            break;
        }
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_RADIO:
            /* send to radio control module */
            TICK_PIN(PIN_RADIO_SIGNAL);
            radio_event_handler();
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_TIMER0:
            /* send to timer control module */
            TICK_PIN(PIN_TIMER_SIGNAL);
            timer_event_handler();
            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_SUCCEEDED:
            g_timeslot_length += requested_extend_time;
            requested_extend_time = 0;
            ++successful_extensions;

            timer_abort(g_timeslot_end_timer);

            g_timeslot_end_timer =
                timer_order_cb_sync_exec(g_timeslot_length - TIMESLOT_END_SAFETY_MARGIN_US,
                    end_timer_handler);
            g_end_timer_triggered = false;
            g_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;

            TICK_PIN(1);
            if (g_timeslot_length + g_negotiate_timeslot_length < TIMESLOT_MAX_LENGTH)
            {
                timeslot_extend(g_negotiate_timeslot_length);
            }
            else
            {
                /* done extending, check for new trickle event */
                transport_control_step();
            }

            break;

        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_EXTEND_FAILED:
            g_negotiate_timeslot_length >>= 1;
            TICK_PIN(1);
            if (g_negotiate_timeslot_length > 1000)
            {
                timeslot_extend(g_negotiate_timeslot_length);
            }
            else
            {
                /* done extending, check for new trickle event */
                transport_control_step();
            }
            break;

        default:
            APP_ERROR_CHECK(NRF_ERROR_INVALID_STATE);
    }



    if (g_end_timer_triggered)
    {
        timeslot_order_earliest(TIMESLOT_SLOT_LENGTH, true);
        g_is_in_timeslot = false;
        g_end_timer_triggered = false;
        CLEAR_PIN(2);
    }
    else if (g_ret_param.callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND)
    {
        requested_extend_time = g_ret_param.params.extend.length_us;
    }
    else
    {
        requested_extend_time = 0;
    }
    g_is_in_callback = false;

    CLEAR_PIN(3);
    return &g_ret_param;
}


/*****************************************************************************
* Interface Functions
*****************************************************************************/

void timeslot_handler_init(void)
{
    if (g_framework_initialized)
    {
        /* may happen with serial interface, can safely skip redundant inits */
        return;
    }
    uint32_t error;

    g_is_in_callback = false;
    g_framework_initialized = true;


    error = sd_nvic_EnableIRQ(SD_EVT_IRQn);
    APP_ERROR_CHECK(error);

    error = sd_radio_session_open(&radio_signal_callback);
    APP_ERROR_CHECK(error);
    g_start_time_ref = NRF_RTC0->COUNTER;
    g_timeslot_length = TIMESLOT_SLOT_LENGTH;
    timeslot_order_earliest(g_timeslot_length, true);
}



void timeslot_order_earliest(uint32_t length_us, bool immediately)
{
    if (immediately)
    {
        radio_request_earliest.params.earliest.length_us = length_us;
        g_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        g_ret_param.params.request.p_next = &radio_request_earliest;

        g_next_timeslot_length = length_us;

        if (!g_is_in_callback)
        {
            uint32_t error_code = sd_radio_request(&radio_request_earliest);
            APP_ERROR_CHECK(error_code);
        }
    }
    else
    {
        radio_request_earliest.params.earliest.length_us = length_us;

        g_next_timeslot_length = length_us;
    }
}


void timeslot_order_normal(uint32_t length_us, uint32_t distance_us, bool immediately)
{
    if (immediately)
    {
        radio_request_normal.params.normal.length_us = length_us;
        radio_request_normal.params.normal.distance_us = distance_us;
        g_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        g_ret_param.params.request.p_next = &radio_request_normal;

        g_next_timeslot_length = length_us;

        if (!g_is_in_callback)
        {
            sd_radio_request(&radio_request_normal);
        }
    }
    else
    {
        radio_request_normal.params.normal.length_us = length_us;
        radio_request_normal.params.normal.distance_us = distance_us;

        g_next_timeslot_length = length_us;
    }
}

void timeslot_extend(uint32_t extra_time_us)
{
    if (g_is_in_callback)
    {
        if (g_timeslot_length + extra_time_us > TIMESLOT_MAX_LENGTH)
        {
            extra_time_us = TIMESLOT_MAX_LENGTH - g_timeslot_length;
        }
        g_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND;
        g_ret_param.params.extend.length_us = extra_time_us;
    }
}


uint32_t timeslot_get_remaining_time(void)
{
    if (!g_is_in_timeslot)
    {
        return 0;
    }

    uint32_t timestamp = timer_get_timestamp();
    if (timestamp > g_timeslot_length - TIMESLOT_END_SAFETY_MARGIN_US)
    {
        return 0;
    }
    else
    {
        return (g_timeslot_length - TIMESLOT_END_SAFETY_MARGIN_US - timestamp);
    }
}

uint64_t timeslot_get_end_time(void)
{
    if (!g_is_in_timeslot)
    {
        return 0;
    }

    return g_timeslot_length;
}
