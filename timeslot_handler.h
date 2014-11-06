#ifndef _TIMESLOT_HANDLER_H__
#define _TIMESLOT_HANDLER_H__
#include "rbc_database.h"
#include "radio_control.h"
#include "timer_control.h"
#include <stdint.h>


/**
* Asynchronous event definintions
*/
typedef enum
{
    EVENT_TYPE_TIMER,
    EVENT_TYPE_RADIO_RX,
    EVENT_TYPE_RADIO_TX,
    EVENT_TYPE_GENERIC
} event_type_t;

typedef void(*generic_cb)(void);

typedef struct
{
    event_type_t type;
    union
    {
        struct 
        {
            radio_rx_cb function;
            uint8_t* data;
        }radio_rx;
        radio_tx_cb radio_tx;/*void*/
        timer_callback timer;/*void*/
        generic_cb generic; /*void*/
    } callback;
} async_event_t;



void ts_sd_event_handler(void);

void timeslot_handler_init(void);

void timeslot_order_earliest(uint32_t length_us, bool immediately);

void timeslot_order_normal(uint32_t length_us, uint32_t distance_us, bool immediately);

void timeslot_extend(uint32_t extra_time_us);

void timeslot_queue_async_event(async_event_t* evt);

uint32_t timeslot_get_remaining_time(void);

#endif /* _TIMESLOT_HANDLER_H__ */
