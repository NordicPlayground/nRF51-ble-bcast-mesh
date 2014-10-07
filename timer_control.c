#include "timer_control.h"

#include "app_error.h"
#include "nrf51_bitfields.h"
#include "nrf_soc.h"



/*****************************************************************************
* Static globals
*****************************************************************************/
static uint32_t timeout_list[3];

static uint8_t active_callbacks;

static uint8_t reference_channel = 0xFF;

static int32_t reference_offset;

static timer_callback callbacks[3];

static uint32_t reference_point = 0;

/*****************************************************************************
* Static functions
*****************************************************************************/

static uint8_t get_available_timer(void)
{    
    for (uint8_t i = 0; i < 4; ++i)
    {
        if (NRF_TIMER0->EVENTS_COMPARE[i] == 1)
        {
            /* wipe out all information that may cause misfire */
            NRF_PPI->CHENCLR |= (1 << (TIMER_PPI_CH_START + i));
            active_callbacks &= ~(1 << i);
            NRF_TIMER0->EVENTS_COMPARE[i] = 0;
            return i;
        }
    }
    
    return 0xFF;
}



/*****************************************************************************
* Interface functions
*****************************************************************************/

void timer_event_handler(void)
{
    /* check if this is about the reference time */
    if (reference_channel != 0xFF && NRF_TIMER0->EVENTS_COMPARE[reference_channel])
    {
        NRF_TIMER0->EVENTS_COMPARE[reference_channel] = 0;
        reference_point = NRF_TIMER0->CC[reference_channel] + reference_offset;
        timeout_list[reference_channel] = 0;
        NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + reference_channel));
        NRF_PPI->CHENCLR |= (1 << (TIMER_PPI_CH_START + reference_channel));
        
        reference_channel = 0xFF;
        reference_offset = 0;
    }

    for (uint8_t i = 0; i < 3; ++i)
    {
        if ((active_callbacks & (1 << i)) && NRF_TIMER0->EVENTS_COMPARE[i])
        {
            (*callbacks[i])();
            active_callbacks &= ~(1 << i);
            timeout_list[i] = 0;
        }
    }
            
}

uint8_t timer_order_cb(uint32_t time, timer_callback callback)
{
    uint8_t timer = get_available_timer();
    
    if (timer == 0xFF)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
    
    NRF_TIMER0->CC[timer] = reference_point + time;
    NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
    NRF_TIMER0->INTENSET = (1 << (TIMER_INTENSET_COMPARE0_Pos + timer));
    callbacks[timer] = callback;
    active_callbacks |= (1 << timer);
    
    return timer;
}

uint8_t timer_order_cb_ppi(uint32_t time, timer_callback callback, uint32_t* task)
{
    uint8_t timer = get_available_timer();
    
    if (time == 0xFF)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
    NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
    NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + timer));
    NRF_TIMER0->CC[timer] = reference_point + time;
    
    NRF_TIMER0->INTENSET = (1 << (TIMER_INTENSET_COMPARE0_Pos + timer));
    callbacks[timer] = callback;
    active_callbacks |= (1 << timer);
    
    /* Setup PPI */
    NRF_PPI->CH[TIMER_PPI_CH_START + timer].EEP   = (uint32_t) &(NRF_TIMER0->EVENTS_COMPARE[timer]);
	NRF_PPI->CH[TIMER_PPI_CH_START + timer].TEP   = (uint32_t) task;
	NRF_PPI->CHENSET 			                 |= (1 << (TIMER_PPI_CH_START + timer));
    
    return timer;
}

uint8_t timer_order_ppi(uint32_t time, uint32_t* task)
{
    uint8_t timer = get_available_timer();
    
    if (time == 0xFF)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
    
    NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
    NRF_TIMER0->INTENCLR = (1 << (TIMER_INTENCLR_COMPARE0_Pos + timer));
    NRF_TIMER0->CC[timer] = reference_point + time;
    
    /* Setup PPI */
    NRF_PPI->CH[TIMER_PPI_CH_START + timer].EEP   = (uint32_t) &(NRF_TIMER0->EVENTS_COMPARE[timer]);
	NRF_PPI->CH[TIMER_PPI_CH_START + timer].TEP   = (uint32_t) task;
	NRF_PPI->CHENSET 			                 |= (1 << (TIMER_PPI_CH_START + timer));
    
    return timer;
}


uint32_t timer_get_timestamp(void)
{
    uint8_t timer = get_available_timer();
    
    if (timer == 0xFF)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
    
    NRF_TIMER0->TASKS_CAPTURE[timer];
    
    NRF_TIMER0->EVENTS_COMPARE[timer] = 1;
    return NRF_TIMER0->CC[timer];
}    

void timer_reference_point_trigger(uint32_t* trigger_event, int32_t time_offset)
{
    uint8_t timer = get_available_timer();
    if (timer == 0xFF)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }
    
    reference_channel = timer;
    reference_offset = time_offset;
    
    NRF_TIMER0->EVENTS_COMPARE[timer] = 0;
    NRF_TIMER0->INTENSET = (1 << (TIMER_INTENSET_COMPARE0_Pos + timer));
        
    /* Setup PPI */
	NRF_PPI->CH[timer].EEP   = (uint32_t) trigger_event;
    NRF_PPI->CH[timer].TEP   = (uint32_t) &(NRF_TIMER0->TASKS_CAPTURE[timer]);
	NRF_PPI->CHENSET 		|= (1 << (TIMER_PPI_CH_START + timer));    
}

uint32_t timer_get_reference_point(void)
{
    return reference_point;
}



