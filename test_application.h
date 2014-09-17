#ifndef _TEST_APPLICATION_H__
#define _TEST_APPLICATION_H__
#include <stdint.h>

#define MESSAGE_TYPE_TRICKLE_LED_CONFIG 0x72


/* RX/TX buffer */
extern uint8_t rx_data[256];
extern uint8_t tx_data[256];

/**
* Checks the given buffer against current trickle state
*/
void eval_msg(uint8_t* msg);

void test_app_init(void);

#endif /* _TEST_APPLICATION_H__ */
