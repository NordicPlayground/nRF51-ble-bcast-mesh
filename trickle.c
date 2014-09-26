#include "trickle.h"
#include "trickle_common.h"

#include "boards.h"
#include "nrf51_bitfields.h"
#include <string.h>


#define MAX_TRICKLE_INSTANCES 8

#define APP_TIMER_PRESCALER 16

#define RX_PROPAGATION_TIME         (377)


#define TRICKLE_FLAGS_T_DONE_Pos    (0)
#define TRICKLE_FLAGS_DISCARDED_Pos (1)

/*****************************************************************************
* Static Globals
*****************************************************************************/


static uint32_t g_trickle_time; /* global trickle time that all time variables are relative to */

static uint8_t rng_vals[64];
static uint8_t rng_index;

/*global parameters for trickle behavior, set in trickle_setup() */
static uint32_t g_i_min, g_i_max;
static uint8_t g_k;

/*****************************************************************************
* Static Functions
*****************************************************************************/

#if !USE_SOFTDEVICE
void TIMER0_IRQHandler(void)
{
    if (NRF_TIMER0->EVENTS_COMPARE[0])
    {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        NRF_TIMER0->CC[0] = TRICKLE_INTERVAL_US;
        TICK_PIN(PIN_SYNC_TIME);
        
        trickle_step();
    }
}
#endif

static void trickle_interval_begin(trickle_t* trickle)
{
    trickle->c = 0;
    
    uint32_t rand_number =  ((uint32_t) rng_vals[(rng_index++) & 0x3F])       |
                            ((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 8  |
                            ((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 16 |
                            ((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 24;
    
    uint32_t i_half = trickle->i_relative / 2;
    trickle->t = g_trickle_time * TRICKLE_INTERVAL_US / 1000 + i_half + (rand_number % i_half);
    trickle->i = g_trickle_time * TRICKLE_INTERVAL_US / 1000 + trickle->i_relative;
    
    trickle->trickle_flags &= ~(1 << TRICKLE_FLAGS_T_DONE_Pos);
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/


void trickle_setup(uint32_t i_min, uint32_t i_max, uint8_t k)
{
    g_i_min = i_min;
    g_i_max = i_max;
    g_k = k;
    
    rng_index = 0;
    
    /* Fill rng pool */
    for (uint8_t i = 0; i < 64; ++i)
    {
        NRF_RNG->EVENTS_VALRDY = 0;
        NRF_RNG->TASKS_START = 1;
        while (!NRF_RNG->EVENTS_VALRDY);
        rng_vals[i] = NRF_RNG->VALUE;
    }
}


void trickle_time_increment(void)
{
    /* step global time */
    ++g_trickle_time;
    return;
}



uint32_t trickle_init(trickle_t* trickle)
{
    
    if (trickle->i < g_i_min || 
        trickle->i > g_i_min * g_i_max)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    trickle->trickle_flags = 0;
    
    trickle_interval_begin(trickle);
    
    return NRF_SUCCESS;
}

void trickle_rx_consistent(trickle_t* trickle)
{    
    ++trickle->c;
}

void trickle_rx_inconsistent(trickle_t* trickle)
{        
    if (trickle->i_relative > g_i_min)
    {
        trickle_timer_reset(trickle);
    }
}

void trickle_timer_reset(trickle_t* trickle)
{    
    trickle->trickle_flags &= ~(1 << TRICKLE_FLAGS_T_DONE_Pos);
    trickle->i_relative = g_i_min; 
        
    trickle_interval_begin(trickle);
}

void trickle_register_tx(trickle_t* trickle)
{
    trickle->trickle_flags |= TRICKLE_FLAGS_T_DONE_Pos;
}

void trickle_step(trickle_t* trickle, bool* out_do_tx)
{
    if (trickle->trickle_flags & (1 << TRICKLE_FLAGS_T_DONE_Pos)) /* i is next timeout for this instance */
    {
        if (1000 * trickle->i / TRICKLE_INTERVAL_US <= g_trickle_time)
        {
            /* double value of i */
            trickle->i_relative = (trickle->i_relative * 2 < g_i_max * g_i_min)?
                            trickle->i_relative * 2 : 
                            g_i_max * g_i_min;
            
            trickle_interval_begin(trickle);
        }
    }
    else /* t is next timeout for this instance */
    {
        if (1000 * trickle->t / TRICKLE_INTERVAL_US <= g_trickle_time)
        {
            if (trickle->c < g_k)
            {
                *out_do_tx = true;
            }
            else /* no tx this interval, tell trickle to prepare 
                for interval timeout*/
            {
                trickle->trickle_flags |= (1 << TRICKLE_FLAGS_T_DONE_Pos);
            }
        }
    }
}
