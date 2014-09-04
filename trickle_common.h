#ifndef _TRICKLE_COMMON_H__
#define _TRICKLE_COMMON_H__
#include <stdint.h>

#define MESSAGE_TYPE_TRICKLE_LED_CONFIG 0x72
#define MESSAGE_TYPE_TRICKLE_RESET 0x73

#ifdef BOARD_PCA10000
    #define PIN_TRICKLE_TX      LED_RGB_GREEN
    #define TICK_PIN(x) 
    #define SET_PIN(x) 
    #define CLEAR_PIN(x) 
#else
    #define PIN_TRICKLE_TX      0
    
    #define TICK_PIN(x) NRF_GPIO->OUTSET = (1 << (x)); \
													NRF_GPIO->OUTCLR = (1 << (x))
     
    #define SET_PIN(x) NRF_GPIO->OUTSET = (1 << (x))
    #define CLEAR_PIN(x) NRF_GPIO->OUTCLR = (1 << (x))
#endif

#define PIN_NEW_INTERVAL    1
#define PIN_CPU_IN_USE      2
#define PIN_CONSISTENT      3
#define PIN_INCONSISTENT    4
#define PIN_RX              5
#define PIN_BUTTON          6
#define PIN_ABORTED         7

/* RX/TX buffer */
extern uint8_t rx_data[256];
extern uint8_t tx_data[256];

/**
* Checks the given buffer against current trickle state
*/
void eval_msg(uint8_t* msg);

#endif /* _TRICKLE_COMMON_H__ */
