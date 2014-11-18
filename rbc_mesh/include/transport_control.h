#ifndef _TRANSPORT_CONTROL_H__
#define _TRANSPORT_CONTROL_H__
#include <stdint.h>


void transport_control_timeslot_begin(uint32_t global_timer_value);

void transport_control_step(void);

#endif /* _TRANSPORT_CONTROL_H__ */
