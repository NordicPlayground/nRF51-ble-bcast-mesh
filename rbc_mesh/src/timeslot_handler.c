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

#include "nrf_sdm.h"
#include "app_error.h"
#include "nrf_assert.h"
#include "nrf_soc.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#define USE_SWI_FOR_PROCESSING          (1)


#define TIMESLOT_END_SAFETY_MARGIN_US   (200)
#define TIMESLOT_SLOT_LENGTH            (100000)
#define TIMESLOT_SLOT_EMERGENCY_LENGTH  (3000) /* will fit between two conn events */
#define TIMESLOT_MAX_LENGTH             (1000000) /* 1s */

#if USE_SWI_FOR_PROCESSING
#define ASYNC_EVENT_FIFO_QUEUE_SIZE (8)
#define ASYNC_EVENT_FIFO_QUEUE_MASK (ASYNC_EVENT_FIFO_QUEUE_SIZE - 1)      
#endif

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
                        .timeout_us = 10000 /* 10ms */
                    }
                };
                
                
                  
static nrf_radio_signal_callback_return_param_t g_ret_param;
//static nrf_radio_signal_callback_return_param_t g_final_ret_param;

static bool g_is_in_callback = true;
                
static uint32_t g_timeslot_length;      
static uint32_t g_timeslot_end_timer;      
static uint32_t g_next_timeslot_length;    
static uint32_t g_start_time_ref = 0;     
static uint32_t g_is_in_timeslot = false; 
static uint32_t g_framework_initialized = false;                
static uint32_t g_negotiate_timeslot_length = TIMESLOT_SLOT_LENGTH;

#if USE_SWI_FOR_PROCESSING
static uint8_t event_fifo_head = 0;
static uint8_t event_fifo_tail = 0;       
static async_event_t async_event_fifo_queue[ASYNC_EVENT_FIFO_QUEUE_SIZE];

static async_event_t evt;                
#endif                
                
/*****************************************************************************
* Static Functions
*****************************************************************************/

/***** ASYNC EVENT QUEUE *****/
/**@TODO: add generic implementation shared with radio fifo */

#pragma diag_suppress 177 /* silence "not used" warnings */
#if USE_SWI_FOR_PROCESSING
static bool event_fifo_full(void)
{
    return ((event_fifo_tail + ASYNC_EVENT_FIFO_QUEUE_SIZE) == event_fifo_head);
}

static bool event_fifo_empty(void)
{
    return (event_fifo_head == event_fifo_tail);
}

static uint8_t event_fifo_get_length(void)
{
    return (event_fifo_head - event_fifo_tail) & 0xFF;
}

static uint8_t event_fifo_put(async_event_t* evt)
{
    if (event_fifo_full())
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
    
    async_event_t* head = &async_event_fifo_queue[event_fifo_head & ASYNC_EVENT_FIFO_QUEUE_MASK];
    
    memcpy(head, evt, sizeof(async_event_t));
    
    return ((event_fifo_head++) & ASYNC_EVENT_FIFO_QUEUE_MASK);
}

static uint32_t event_fifo_get(async_event_t* evt)
{
    if (event_fifo_empty())
    {
        return NRF_ERROR_NULL;
    }
    if (evt != NULL)
    {
        async_event_t* tail = &async_event_fifo_queue[event_fifo_tail & ASYNC_EVENT_FIFO_QUEUE_MASK];
        
        memcpy(evt, tail, sizeof(async_event_t));
    }    
    ++event_fifo_tail;
    return NRF_SUCCESS;
}


static uint32_t event_fifo_peek_at(async_event_t* evt, uint8_t offset)
{
    if (event_fifo_get_length() < offset)
    {
        return NRF_ERROR_NULL;
    }
    
    async_event_t* tail = &async_event_fifo_queue[(event_fifo_tail + offset) & ASYNC_EVENT_FIFO_QUEUE_MASK];
    
    memcpy(evt, tail, sizeof(async_event_t));
    
    return NRF_SUCCESS;
}

static uint32_t event_fifo_peek(async_event_t* evt)
{
    return event_fifo_peek_at(evt, 0);
}

static void event_fifo_flush(void)
{
    event_fifo_tail = event_fifo_head;
}
#endif

/**
* @brief execute asynchronous event, based on type
*/
static void async_event_execute(async_event_t* evt)
{
    switch (evt->type)
    {
        case EVENT_TYPE_RADIO_RX:
            (*evt->callback.radio_rx.function)(evt->callback.radio_rx.data);
            break;
        case EVENT_TYPE_RADIO_TX:
            (*evt->callback.radio_tx)();
            break;
        case EVENT_TYPE_TIMER:
            (*evt->callback.timer)();
            break;
        case EVENT_TYPE_GENERIC:
            (*evt->callback.generic)();
            break;
        case EVENT_TYPE_PACKET:
            mesh_srv_packet_process(&evt->callback.packet);
        default:
            break;
    }
}


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
    SET_PIN(6);
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
    CLEAR_PIN(6);
}

/**
* @brief Timeslot end guard timer callback. Attempts to extend the timeslot. 
*/
static void end_timer_handler(void)
{
    /*static uint8_t noise_val = 0xA7;
    noise_val ^= 0x55 + (noise_val >> 1);
    
    uint32_t extend_len = ((g_emergency_timeslot_mode)?
                                TIMESLOT_SLOT_EMERGENCY_LENGTH : 
                                TIMESLOT_SLOT_LENGTH + noise_val * 50);*/
    timeslot_order_earliest(((g_timeslot_length > 100000)? 100000 : g_timeslot_length), true);
}
    


#if USE_SWI_FOR_PROCESSING

/**
* @brief Async event dispatcher, works in APP LOW
*/
void SWI0_IRQHandler(void)
{
    while (!event_fifo_empty() && (g_is_in_timeslot || !g_framework_initialized))
    {
        if (event_fifo_get(&evt) == NRF_SUCCESS)
        {
            event_fifo_get(NULL); /* bump tail */
            async_event_execute(&evt);
        }
    }
}
#endif


/**
* @brief Radio signal callback handler taking care of all signals in searching 
*   mode
*/
static nrf_radio_signal_callback_return_param_t* radio_signal_callback(uint8_t sig)
{
    g_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
    g_is_in_callback = true;
    static uint32_t requested_extend_time = 0;
    static uint32_t successful_extensions = 0;  
    static uint8_t noise_val = 0x5F;
    SET_PIN(PIN_SYNC_TIME);
    
    uint64_t time_now = 0;
    
    switch (sig)
    {
        case NRF_RADIO_CALLBACK_SIGNAL_TYPE_START:
            NVIC_ClearPendingIRQ(SWI0_IRQn);
            g_is_in_timeslot = true;
            
            event_fifo_flush();
            timer_init();
            SET_PIN(2);
            successful_extensions = 0;
        
            g_negotiate_timeslot_length = g_timeslot_length;
        
            g_timeslot_length = g_next_timeslot_length;
        
            g_timeslot_end_timer = 
                timer_order_cb_sync_exec(g_timeslot_length - TIMESLOT_END_SAFETY_MARGIN_US, 
                    end_timer_handler);
            
            
            /* attempt to extend our time right away */
            timeslot_extend(g_negotiate_timeslot_length);
            
#if USE_SWI_FOR_PROCESSING
            NVIC_EnableIRQ(SWI0_IRQn);
            NVIC_SetPriority(SWI0_IRQn, 3);
#endif       
        
            /* sample RTC timer for trickle timing */
            time_now = NRF_RTC0->COUNTER - g_start_time_ref;
            
            /* scale to become us */     
            time_now = ((time_now << 15) / 1000);
            
            transport_control_timeslot_begin(time_now);
        
            break;
        
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
            g_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_NONE;
        
            timer_abort(g_timeslot_end_timer);
        
            g_timeslot_end_timer = 
                timer_order_cb_sync_exec(g_timeslot_length - TIMESLOT_END_SAFETY_MARGIN_US, 
                    end_timer_handler);
            
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
            g_negotiate_timeslot_length >>= 2;
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
    
    
    g_is_in_callback = false;
    if (g_ret_param.callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_EXTEND)
    {
        requested_extend_time = g_ret_param.params.extend.length_us;
    }
    else if (g_ret_param.callback_action == NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END)
    {
        CLEAR_PIN(2);
        g_is_in_timeslot = false;
        event_fifo_flush();
    }
    else
    {
        requested_extend_time = 0;
    }
    
    CLEAR_PIN(PIN_SYNC_TIME);
    return &g_ret_param;
}


/*****************************************************************************
* Interface Functions
*****************************************************************************/

void timeslot_handler_init(void)
{
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
            sd_radio_request(&radio_request_earliest);
        }
    }
    else
    {
        radio_request_earliest.params.earliest.length_us = length_us;
        //g_final_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        //g_final_ret_param.params.request.p_next = &radio_request_earliest;
        
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
        //g_final_ret_param.callback_action = NRF_RADIO_SIGNAL_CALLBACK_ACTION_REQUEST_AND_END;
        //g_final_ret_param.params.request.p_next = &radio_request_normal;
        
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


void timeslot_queue_async_event(async_event_t* evt)
{
#if USE_SWI_FOR_PROCESSING
    NVIC_EnableIRQ(SWI0_IRQn);
    NVIC_SetPriority(SWI0_IRQn, 3);
    event_fifo_put(evt);
    NVIC_SetPendingIRQ(SWI0_IRQn);
#else
    /* execute immediately */
    async_event_execute(evt);
#endif
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

uint32_t timeslot_get_end_time(void)
{
    if (!g_is_in_timeslot)
    {
        return 0;
    }
    
    return g_timeslot_length;
}
