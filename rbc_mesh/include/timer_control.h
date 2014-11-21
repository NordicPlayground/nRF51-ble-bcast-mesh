#ifndef _TIMER_CONTROL_H__
#define _TIMER_CONTROL_H__
#include <stdint.h>

/**
* @file HF Timer module abstraction. Allocates timers and schedules PPI
*   signals in PPI channels 8-11. Can also manually do callbacks at timeout,
*   should only be used for applications with soft realtime requirements. 
*   Callbacks may be executed both synchronously (in STACK LOW) and in SWI0 
*   (APP LOW). May set a reference point, to which all time is executed after.
* 
* @note All order functions return the index of the capture register they were 
*   allocated in, in the range 0-3.
*/

#define TIMER_PPI_CH_START  8

/** @brief Callback type for callbacks at finished timers */
typedef void(*timer_callback)();

/** @brief hardware event handler, should be called at all TIMER0 events */
void timer_event_handler(void);

/** @brief order a timer with callback, execute in APP LOW */
uint8_t timer_order_cb(uint32_t time, timer_callback callback);

/** @brief order a timer with callback, execute in event handler context */
uint8_t timer_order_cb_sync_exec(uint32_t time, timer_callback callback);

/** @brief order a timer with both async callback and PPI to trigger HW task */
uint8_t timer_order_cb_ppi(uint32_t time, timer_callback callback, uint32_t* task);

/** @brief order a timer with a PPI channel to trigger HW task */
uint8_t timer_order_ppi(uint32_t time, uint32_t* task);

/** 
* @brief abort timer with given index. This index is the same as returned from 
*   order function.
*/
void timer_abort(uint8_t timer_index);

/**
* @brief Get current timestamp from HF timer. This timestamp is directly 
*   related to the time used to order timers, and may be used to order 
*   relative timers.
*/
uint32_t timer_get_timestamp(void);

/** 
* @brief Use PPI to trigger reference point to which all timers will be 
*   relative to
*/
void timer_reference_point_trigger(uint32_t* trigger_event, int32_t time_offset);

/** @brief Get reference point previously set by application, or 0 if not set*/
uint32_t timer_get_reference_point(void);

/** @brief Set timer reference point manually */
void timer_reference_point_set(uint32_t ref_point);

/** 
* @brief initialize timer module. Must be called at the beginning of each 
*   SD granted timeslot. Flushes all timer slots.
*/
void timer_init(void);

#endif /* _TIMER_CONTROL_H__ */
