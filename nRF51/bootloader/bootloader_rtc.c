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
#include "bootloader_rtc.h"
#include "transport.h"
#include "bootloader.h"


void RTC0_IRQHandler(void)
{
    if ((NRF_RTC0->EVENTS_COMPARE[RTC_TRANSPORT_CH]) &&
        (NRF_RTC0->INTENSET & (1 << (RTC_TRANSPORT_CH + RTC_INTENSET_COMPARE0_Pos))))
    {
        NRF_RTC0->EVENTS_COMPARE[RTC_TRANSPORT_CH] = 0;
        transport_rtc_irq_handler();
    }
    if ((NRF_RTC0->EVENTS_COMPARE[RTC_BL_STATE_CH]) &&
        (NRF_RTC0->INTENSET & (1 << (RTC_BL_STATE_CH + RTC_INTENSET_COMPARE0_Pos))))
    {
        NRF_RTC0->EVENTS_COMPARE[RTC_BL_STATE_CH] = 0;
        NRF_RTC0->INTENCLR = (1 << (RTC_BL_STATE_CH + RTC_INTENSET_COMPARE0_Pos));
        bootloader_timeout();
    }
}

void rtc_init(void)
{
    NRF_RTC0->PRESCALER = 0;
    NRF_RTC0->EVTENSET  = (1 << (RTC_TRANSPORT_CH + RTC_EVTEN_COMPARE0_Pos))
                        | (1 << (RTC_BL_STATE_CH  + RTC_EVTEN_COMPARE0_Pos));
    NRF_RTC0->TASKS_CLEAR = 1;
    NRF_RTC0->TASKS_START = 1;
    NVIC_SetPriority(RTC0_IRQn, 1);
    NVIC_EnableIRQ(RTC0_IRQn);
}
