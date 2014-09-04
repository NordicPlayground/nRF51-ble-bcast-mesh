#ifndef _RADIO_CONTROL_H__
#define _RADIO_CONTROL_H__
#include <stdint.h>

extern uint8_t radio_aborted;


void radio_init(void);

void radio_tx(uint8_t* data);

/* receive data into given buffer. Buffer must be at least 255 bytes to ensure safe behavior */
void radio_rx(uint8_t* data);



#endif /* _RADIO_CONTROL_H__*/
