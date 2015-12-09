#include "bootloader_rtc.h"
#include "transport.h"
#include "bootloader_mesh.h"


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
        bootloader_rtc_irq_handler();
    }
}

void rtc_init(void)
{
    NRF_RTC0->PRESCALER = 0;
    NRF_RTC0->EVTENSET  = (1 << (RTC_TRANSPORT_CH + RTC_EVTEN_COMPARE0_Pos)) 
                        | (1 << (RTC_BL_STATE_CH  + RTC_EVTEN_COMPARE0_Pos));
    NRF_RTC0->TASKS_CLEAR = 1;
    NRF_RTC0->TASKS_START = 1;
    NVIC_SetPriority(RTC0_IRQn, 2);
    NVIC_EnableIRQ(RTC0_IRQn);
}
