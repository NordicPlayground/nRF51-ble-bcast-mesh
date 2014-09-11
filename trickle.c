#include "trickle.h"
#include "trickle_common.h"
#include "app_timer.h"

#include "boards.h"
#include "nrf51_bitfields.h"

#define MAX_TRICKLE_INSTANCES 8

#define APP_TIMER_PRESCALER 32

#define TRICKLE_FLAGS_T_DONE_Pos    (0)
#define TRICKLE_FLAGS_DISCARDED_Pos (1)

uint8_t rng_vals[64];
uint8_t rng_index;


static void trickle_timeout_handler(void * trickle_ptr)
{
    trickle_t* trickle = (trickle_t*) trickle_ptr;
    SET_PIN(PIN_CPU_IN_USE);
    
    /* check which timer we just triggered */
    if (trickle->trickle_flags & (1 << TRICKLE_FLAGS_T_DONE_Pos)) /* interval timer */
    {
        
        /* double value of i */
        trickle->i = (trickle->i * 2 < trickle->i_max * trickle->i_min)?
                        trickle->i * 2 : 
                        trickle->i_max * trickle->i_min;
        
        trickle_interval_begin(trickle);
    }
    else /* t (tx timer) */
    {
        trickle->trickle_flags |= (1 << TRICKLE_FLAGS_T_DONE_Pos);
        
        if (trickle->c < trickle->k)
        {
            trickle->tx_cb();
        }
        
        if (APP_TIMER_TICKS((trickle->i - trickle->t), APP_TIMER_PRESCALER) > 5 && trickle->i > trickle->t)
        {
            APP_ERROR_CHECK(app_timer_start(trickle->timer_id, APP_TIMER_TICKS((trickle->i - trickle->t), APP_TIMER_PRESCALER), trickle));
        }
        else
        {
            APP_ERROR_CHECK(app_timer_start(trickle->timer_id, APP_TIMER_TICKS(5, APP_TIMER_PRESCALER), trickle)); /* min tick count is 5 */
        }
           
    }
    CLEAR_PIN(PIN_CPU_IN_USE);
}


void trickle_setup(void)
{
    APP_TIMER_INIT(APP_TIMER_PRESCALER, MAX_TRICKLE_INSTANCES, MAX_TRICKLE_INSTANCES, false);   
    
    /* bump priority of app_timer interrupt SWI0, as it is required to operate in the middle of a GPIOTE interrupt */
    //NVIC_SetPriority(SWI0_IRQn, 4);
    
    /* Fill rng pool */
    for (uint8_t i = 0; i < 64; ++i)
    {
        NRF_RNG->EVENTS_VALRDY = 0;
        NRF_RNG->TASKS_START = 1;
        while (!NRF_RNG->EVENTS_VALRDY);
        rng_vals[i] = NRF_RNG->VALUE;
    } 
    rng_index = 0;
}

void trickle_init(trickle_t* trickle)
{
	uint32_t error = app_timer_create(&trickle->timer_id, APP_TIMER_MODE_SINGLE_SHOT, trickle_timeout_handler);
    APP_ERROR_CHECK(error);
    
    trickle_interval_begin(trickle);
}

void trickle_interval_begin(trickle_t* trickle)
{
    trickle->c = 0;
    
    uint32_t rand_number =  ((uint32_t) rng_vals[(rng_index++) & 0x3F])       |
                            ((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 8  |
                            ((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 16 |
                            ((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 24;
    
    uint32_t i_half = trickle->i / 2;
    trickle->t = i_half + (rand_number % i_half);
    
    trickle->trickle_flags &= ~(1 << TRICKLE_FLAGS_T_DONE_Pos);

    APP_ERROR_CHECK(app_timer_start(trickle->timer_id, APP_TIMER_TICKS(trickle->t, APP_TIMER_PRESCALER), trickle));
    
    
    TICK_PIN(PIN_NEW_INTERVAL);
    TICK_PIN((PIN_INT0 + trickle->id));
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
    trickle->trickle_flags &= ~(1 << TRICKLE_FLAGS_T_DONE_Pos);
    APP_ERROR_CHECK(app_timer_stop(trickle->timer_id));
    trickle->i = trickle->i_min; 
    trickle_interval_begin(trickle);
}
