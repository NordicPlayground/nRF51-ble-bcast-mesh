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

#include <stdint.h>
#include "blinky.h"
#include "nrf_gpio.h"
#include "boards.h"
#include "nrf_soc.h"

#if NORDIC_SDK_VERSION >= 11
#include "nrf_nvic.h"
#endif



void led_init ()
{
    
#if defined(BOARD_PCA10031)
    nrf_gpio_cfg_output(LED_RGB_BLUE);
#elif defined(BOARD_PCA10028)|| defined(BOARD_PCA10040)
    nrf_gpio_cfg_output(LED_4);
    nrf_gpio_cfg_output(LED_3);
#endif
    

}


void rtc_1_init ()
{       
NRF_RTC1->PRESCALER = 327;
NRF_RTC1->TASKS_CLEAR = 1;
NRF_RTC1->EVENTS_COMPARE[1]=0;
NRF_RTC1->INTENSET = (1<< (RTC_INTENSET_COMPARE1_Pos + 0));
    
    
#if defined(NRF51) 
    NVIC_SetPriority(RTC1_IRQn , 3); 
#elif defined(NRF52) 
    NVIC_SetPriority(RTC1_IRQn , 6);   
#endif 

NVIC_EnableIRQ (RTC1_IRQn);
    
}


void start_blink_interval_s(uint8_t second )
{

NRF_RTC1->CC[1] = (uint32_t)(100*second) ; // 100 for 1 sec 
NRF_RTC1->TASKS_START = 1;

} 

void RTC1_IRQHandler()
{

if (NRF_RTC1->EVENTS_COMPARE[1] == 1)
 {
    NRF_RTC1->EVENTS_COMPARE[1]=0;
	(void)NRF_RTC1->EVENTS_COMPARE[1]; 
    NRF_RTC1->TASKS_CLEAR = 1;
     
#if defined(BOARD_PCA10031)
    LEDS_INVERT(1 << LED_RGB_BLUE );
#elif defined(BOARD_PCA10028)|| defined(BOARD_PCA10040)
    LEDS_INVERT(1 << LED_4  ); 
    LEDS_INVERT(1 << LED_3  ); 
     
#endif 
 }

}
