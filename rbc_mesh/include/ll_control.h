#ifndef _LL_CONTROL_H__
#define _LL_CONTROL_H__
#include <stdint.h>


void ll_control_timeslot_begin(uint32_t global_timer_value);

void ll_control_step(void);

#endif /* _LL_CONTROL_H__ */
