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
#ifndef _EVENT_HANDLER_H__
#define _EVENT_HANDLER_H__
#include "radio_control.h"
#include "timer.h"
#include "timer_scheduler.h"
#include <stdint.h>
#include <stdbool.h>

/**
* @brief Asynchronous event definitions
*/
typedef enum
{
    EVENT_TYPE_TIMER,
    EVENT_TYPE_TIMER_SCH,
    EVENT_TYPE_GENERIC,
    EVENT_TYPE_PACKET,
    EVENT_TYPE_SET_FLAG
} event_type_t;

/** @brief callback type for generic asynchronous events */
typedef void(*generic_cb_t)(void* p_context);

/**
* @brief Asynchronous event type.
*/
typedef struct
{
    event_type_t type;
    union
    {
        struct
        {
            uint8_t* payload; /* packet to be processed */
            uint32_t crc;
            uint32_t timestamp;
            uint8_t rssi;
        } packet;
        struct
        {
            timer_callback_t cb;/*void return */
            timestamp_t timestamp;
        } timer;
        struct
        {
            timer_sch_callback_t cb;
            timestamp_t timestamp;
            void* p_context;
        } timer_sch;
        struct
        {
            generic_cb_t cb;
            void* p_context;
        } generic; /*void return */
        struct
        {
            uint16_t handle;
            uint8_t flag;
            bool value;
        } set_flag;
    } callback;
} async_event_t;


void event_handler_init(void);

/** @brief Queue an asynchronous event for execution later */
uint32_t event_handler_push(async_event_t* evt);

/** @brief called from ts handler upon ts exit */
void event_handler_on_ts_end(void);

/** @brief called from ts handler upon ts begin */
void event_handler_on_ts_begin(void);

void event_handler_critical_section_begin(void);

void event_handler_critical_section_end(void);

#endif /* _EVENT_HANDLER_H__ */

