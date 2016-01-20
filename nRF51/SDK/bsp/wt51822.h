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
#ifndef WT51822_H
#define WT51822_H

#define LED_START      3
#define LED_1          3
#define LED_2          4
#define LED_3          5
#define LED_STOP       5

#define BSP_LED_0      LED_1
#define BSP_LED_1      LED_2
#define BSP_LED_2      LED_3

#define BUTTON_START   0
#define SW_1           0
#define SW_2           1
#define SW_3           2
#define BUTTON_STOP    2
#define BUTTON_PULL    NRF_GPIO_PIN_PULLUP

#define BSP_BUTTON_0   SW_1
#define BSP_BUTTON_1   SW_2
#define BSP_BUTTON_2   SW_3

#define BUTTONS_NUMBER 3
#define LEDS_NUMBER    3
#define BUTTONS_MASK   0x00000007

#define BSP_BUTTON_0_MASK (1<<BSP_BUTTON_0)
#define BSP_BUTTON_1_MASK (1<<BSP_BUTTON_1)
#define BSP_BUTTON_2_MASK (1<<BSP_BUTTON_2)

#define BUTTONS_LIST { SW_1, SW_2, SW_3 }
#define LEDS_LIST { LED_1, LED_2, LED_3 }

#define BSP_LED_0_MASK    (1<<LED_1)
#define BSP_LED_1_MASK    (1<<LED_2)
#define BSP_LED_2_MASK    (1<<LED_3)

#define LEDS_MASK      (BSP_LED_0_MASK | BSP_LED_1_MASK | BSP_LED_2_MASK)
/* all LEDs are lit when GPIO is low */
#define LEDS_INV_MASK  LEDS_MASK


#define RX_PIN_NUMBER  13
#define TX_PIN_NUMBER  12
#define CTS_PIN_NUMBER 14
#define RTS_PIN_NUMBER 15
#define HWFC           true

#define SER_CON_RX_PIN              13
#define SER_CON_TX_PIN              12
#define SER_CON_CTS_PIN             14
#define SER_CON_RTS_PIN             15

#endif
