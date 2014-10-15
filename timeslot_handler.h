#ifndef _TIMESLOT_HANDLER_H__
#define _TIMESLOT_HANDLER_H__
#include "drip_control.h"
#include <stdint.h>


void broadcast_event_handler(void);

void timeslot_handler_init(void);

void timeslot_order_earliest(uint32_t length_us, bool immediately);

void timeslot_order_normal(uint32_t length_us, uint32_t distance_us, bool immediately);

void timeslot_extend(uint32_t extra_time_us);

uint32_t timeslot_get_remaining_time(void);

#endif /* _TIMESLOT_HANDLER_H__ */
