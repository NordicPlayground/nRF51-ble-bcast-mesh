/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of other
  contributors to this software may be used to endorse or promote products
  derived from this software without specific prior written permission.

  4. This software must only be used in a processor manufactured by Nordic
  Semiconductor ASA, or in a processor manufactured by a third party that
  is used in combination with a processor manufactured by Nordic Semiconductor.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/

#ifndef _RBC_MESH_COMMON_H__
#define _RBC_MESH_COMMON_H__
#include <stdint.h>

#define RBC_MESH_DEBUG  (0)

/******************************************************************************
* Debug related defines
******************************************************************************/
#ifdef BOARD_PCA10000
    #define TICK_PIN(x) 
    #define SET_PIN(x) 
    #define CLEAR_PIN(x) 
#else
    #if RBC_MESH_DEBUG
        #define TICK_PIN(x) NRF_GPIO->OUTSET = (1 << (x)); \
                                                        __nop();\
                                                        __nop();\
                                                        NRF_GPIO->OUTCLR = (1 << (x))
         
        #define SET_PIN(x) NRF_GPIO->OUTSET = (1 << (x))
        #define CLEAR_PIN(x) NRF_GPIO->OUTCLR = (1 << (x))
    #else
        #define TICK_PIN(x) 
        #define SET_PIN(x) 
        #define CLEAR_PIN(x) 
    #endif
#endif

#define PIN_MESH_TX         (0)
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

#define DEBUG_RADIO         (0)

#define PIN_RADIO_STATE_RX  (1)
#define PIN_RADIO_STATE_TX  (2)
#define PIN_RADIO_STATE_IDLE (3)

#if DEBUG_RADIO
    #define DEBUG_RADIO_SET_PIN(x) NRF_GPIO->OUTSET = (1 << (x))
    #define DEBUG_RADIO_CLEAR_PIN(x) NRF_GPIO->OUTCLR = (1 << (x))
#else
    #define DEBUG_RADIO_SET_PIN(x)
    #define DEBUG_RADIO_CLEAR_PIN(x)
#endif


#define PIN_BIT_H           (25)
#define PIN_BIT_L           (28)

#if RBC_MESH_DEBUG
    #define PIN_OUT(val,bitcount)      for (uint8_t i = 0; i < (bitcount); ++i){ if (((val) >> ((bitcount) - 1 - i) & 0x01)) { TICK_PIN(PIN_BIT_H); } else { TICK_PIN(PIN_BIT_L); } }
#else
    #define PIN_OUT(val,bitcount)   
#endif

        
#define CHECK_FP(fp) if ((uint32_t)fp < 0x18000UL || (uint32_t)fp > 0x20000000UL){APP_ERROR_CHECK(NRF_ERROR_INVALID_ADDR);}
        
#endif /* _RBC_MESH_COMMON_H__ */
