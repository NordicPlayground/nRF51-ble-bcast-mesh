#ifndef _TIMESLOT_HANDLER_H__
#define _TIMESLOT_HANDLER_H__
#include <stdint.h>

void timeslot_handler_init(void);


/**
* Start ordering timeslots in a periodic fashion. @param time_period_us indicates the time from 
* the beginning of the previous timeslot until the time when the first periodic timeslot should 
* start (in µs). A time_period of 0 orders a timeslot "as soon as possible".
*/
void timeslot_handler_start_periodic(uint32_t time_period_us);

#endif /* _TIMESLOT_HANDLER_H__ */
