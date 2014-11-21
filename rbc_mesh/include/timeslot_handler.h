#ifndef _TIMESLOT_HANDLER_H__
#define _TIMESLOT_HANDLER_H__
#include "mesh_srv.h"
#include "radio_control.h"
#include "timer_control.h"
#include <stdint.h>

/**
* @file Module responsible for providing a safe interface to Softdevice 
*   timeslot API. Handles all system events, makes sure all timeslots are 
*   ended in time, provides some simple functions for manipulating the way 
*   timeslots behave. Also provides an interface for executing events 
*   in an asynchronous manner (in a FIFO fashion)
*/

/**
* @brief Asynchronous event definitions
*/
typedef enum
{
    EVENT_TYPE_TIMER,
    EVENT_TYPE_RADIO_RX,
    EVENT_TYPE_RADIO_TX,
    EVENT_TYPE_GENERIC,
    EVENT_TYPE_PACKET
} event_type_t;

/** @brief callback type for generic asynchronous events */
typedef void(*generic_cb)(void);

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
            radio_rx_cb function;
            uint8_t* data;
        }radio_rx;
        packet_t packet; /* packet to be processed */
        radio_tx_cb radio_tx;/*void return */
        timer_callback timer;/*void return */
        generic_cb generic; /*void return */
    } callback;
} async_event_t;


/** @brief event handler for softdevice events */
void ts_sd_event_handler(void);

/** @brief initialize timeslot handler. Only called once */
void timeslot_handler_init(void);

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

/** @brief Queue an asynchronous event for execution later */
void timeslot_queue_async_event(async_event_t* evt);

/** @brief returns the time in us until current timeslot ends */
uint32_t timeslot_get_remaining_time(void);

#endif /* _TIMESLOT_HANDLER_H__ */
