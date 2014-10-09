#ifndef _TIMER_CONTROL_H__
#define _TIMER_CONTROL_H__
#include <stdint.h>

#define TIMER_PPI_CH_START  8

typedef void(*timer_callback)();

void timer_event_handler(void);

uint8_t timer_order_cb(uint32_t time, timer_callback callback);

uint8_t timer_order_cb_ppi(uint32_t time, timer_callback callback, uint32_t* task);

uint8_t timer_order_ppi(uint32_t time, uint32_t* task);

void timer_abort(uint8_t timer_index);

uint32_t timer_get_timestamp(void);

void timer_reference_point_trigger(uint32_t* trigger_event, int32_t time_offset);

uint32_t timer_get_reference_point(void);


#endif /* _TIMER_CONTROL_H__ */
