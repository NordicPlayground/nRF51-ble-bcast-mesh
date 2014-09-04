#include "trickle.h"
#include "trickle_common.h"

#include "boards.h"
#include "nrf51_bitfields.h"



static trickle_t* local_trickle;
uint8_t rng_vals[64];
uint8_t rng_index;

void RTC0_IRQHandler(void)
{
    SET_PIN(PIN_CPU_IN_USE);
    
    /* t is triggered */
    if (NRF_RTC0->EVENTS_COMPARE[0])
    {
        NRF_RTC0->EVENTS_COMPARE[0] = 0;
        if (local_trickle->c < local_trickle->k)
        {
            local_trickle->tx_cb();
        }
    }
    if (NRF_RTC0->EVENTS_COMPARE[1]) /* i is triggered */
    {
        NRF_RTC0->EVENTS_COMPARE[1] = 0;
        local_trickle->i = (local_trickle->i * 2 < local_trickle->i_max * local_trickle->i_min)?
                                local_trickle->i * 2 : 
                                local_trickle->i_max * local_trickle->i_min;
        trickle_interval_begin(local_trickle);
        TICK_PIN(PIN_NEW_INTERVAL);
    }     
    CLEAR_PIN(PIN_CPU_IN_USE);   
}

void trickle_init(trickle_t* trickle)
{
    local_trickle = trickle;
    
    /* Fill rng pool */
    for (uint8_t i = 0; i < 64; ++i)
    {
        NRF_RNG->EVENTS_VALRDY = 0;
        NRF_RNG->TASKS_START = 1;
        while (!NRF_RNG->EVENTS_VALRDY);
        rng_vals[i] = NRF_RNG->VALUE;
    }
    rng_index = 0;
	NRF_RTC0->PRESCALER = 32; /* 1 tick = 0.9929 ms, rounding to 1ms.. */
    
    NRF_RTC0->INTENSET |= (1 << RTC_INTENSET_COMPARE0_Pos)|(1 << RTC_INTENSET_COMPARE1_Pos);
    NVIC_EnableIRQ(RTC0_IRQn);
    //NVIC_SetPriority(RTC0_IRQn, 1); /* below radio interrupt priority */
    
    trickle_interval_begin(trickle);
}

void trickle_interval_begin(trickle_t* trickle)
{
    trickle->c = 0;
    
    uint32_t i_half = trickle->i / 2;
    uint32_t rand_number =  ((uint32_t) rng_vals[(rng_index++) & 0x3F])       |
                            ((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 8  |
                            ((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 16 |
                            ((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 24;
    trickle->t = i_half + (rand_number % i_half);
    
	NRF_RTC0->PRESCALER = 32; /* 1 tick = 0.9929 ms, rounding to 1ms.. */
    
    NRF_RTC0->TASKS_START = 1;
    NRF_RTC0->TASKS_CLEAR = 1;
    NRF_RTC0->EVENTS_COMPARE[0] = 0;
    NRF_RTC0->EVENTS_COMPARE[1] = 0;
    NRF_RTC0->INTENSET |= (1 << RTC_INTENSET_COMPARE0_Pos)|(1 << RTC_INTENSET_COMPARE1_Pos);
    
    
    NRF_RTC0->CC[0] = trickle->t;
    NRF_RTC0->CC[1] = trickle->i;    
}

void trickle_rx_consistent(trickle_t* trickle)
{
    ++trickle->c;
}

void trickle_rx_inconsistent(trickle_t* trickle)
{
    if (trickle->i > trickle->i_min)
    {
        trickle_timer_reset(trickle);
    }       
}

void trickle_timer_reset(trickle_t* trickle)
{
    trickle->i = trickle->i_min; 
    trickle_interval_begin(trickle);
}
