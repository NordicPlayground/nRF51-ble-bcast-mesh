#ifndef _RBC_MESH_COMMON_H__
#define _RBC_MESH_COMMON_H__
#include <stdint.h>


/******************************************************************************
* Debug related defines
******************************************************************************/
#ifdef BOARD_PCA10000
    #define PIN_MESH_TX      LED_RGB_GREEN
    #define TICK_PIN(x) 
    #define SET_PIN(x) 
    #define CLEAR_PIN(x) 
#else
    #define PIN_MESH_TX      (0)
    
    #define TICK_PIN(x) NRF_GPIO->OUTSET = (1 << (x)); \
                                                    __nop();\
                                                    __nop();\
													NRF_GPIO->OUTCLR = (1 << (x))
     
    #define SET_PIN(x) NRF_GPIO->OUTSET = (1 << (x))
    #define CLEAR_PIN(x) NRF_GPIO->OUTCLR = (1 << (x))
#endif

#define PIN_SEARCHING       (1)
#define PIN_CPU_IN_USE      (2)
#define PIN_CONSISTENT      (3)
#define PIN_INCONSISTENT    (4)
#define PIN_RX              (5)
#define PIN_BUTTON          (6)
#define PIN_ABORTED         (7)

#define PIN_INT0            (25)
#define PIN_INT1            (26)
#define PIN_TX0             (27)
#define PIN_TX1             (28)
#define PIN_SYNC_TIME       (29)

#define PIN_RADIO_SIGNAL    (3)
#define PIN_TIMER_SIGNAL    (4)
#define PIN_IN_TIMESLOT     (6)

#define PIN_BIT_H           (25)
#define PIN_BIT_L           (28)

//#define PIN_OUT(val,bitcount)   
#define PIN_OUT(val,bitcount)      for (uint8_t i = 0; i < (bitcount); ++i){ if (((val) >> ((bitcount) - 1 - i) & 0x01)) { TICK_PIN(PIN_BIT_H); } else { TICK_PIN(PIN_BIT_L); } }


#define DEBUG_DB                    (0)
#define DEBUG_TIMESLOT              (1)
#define DEBUG_TIMESLOT_HANDLER      (1)

#if DEBUG_DB
#define DEBUG_PIN_DB(x)   TICK_PIN(x)
#else
#define DEBUG_PIN_DB(x)
#endif

#if DEBUG_TIMESLOT
#define DEBUG_PIN_TIMESLOT(x)   TICK_PIN(x)
#else
#define DEBUG_PIN_TIMESLOT(x)
#endif

#if DEBUG_TIMESLOT_HANDLER
#define DEBUG_PIN_TH(x)   TICK_PIN(x)
#else
#define DEBUG_PIN_TH(x)
#endif








#endif /* _RBC_MESH_COMMON_H__ */
