#ifndef _TRANSPORT_CONTROL_H__
#define _TRANSPORT_CONTROL_H__
#include <stdint.h>

/**
* @file This module takes care of all lower level packet processing and 
*   schedules the radio for transmission. Acts as the link between the radio
*   and the mesh service.
*/

/** 
* @brief Called at the beginning of a timeslot with a timestamp in order to let
*   the system catch up with any lost time between timeslots
* 
* @param[in] global_timer_value The timestamp to use as reference for whether
*   there is anything to process.
*/

void transport_control_timeslot_begin(uint32_t global_timer_value);

/**
* @brief Force a check for timed out values
*/
void transport_control_step(void);

#endif /* _TRANSPORT_CONTROL_H__ */
