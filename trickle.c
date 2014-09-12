#include "trickle.h"
#include "trickle_common.h"

#include "boards.h"
#include "nrf51_bitfields.h"
#include <string.h>

typedef struct
{
    uint32_t        t;              /* Absolute value of t. Equals g_trickle_time (at set time) + t_relative */
    uint32_t        i;              /* Absolute value of i. Equals g_trickle_time (at set time) + i_relative */
    uint32_t        i_relative;     /* Relative value of i. Represents the actual i value in RFC6206 */
    uint32_t        i_min;          /* Minimum interval in ms */
    uint16_t        i_max;          /* Maximum doubling of i_min. The biggest interval is (i_min * i_max) */
    uint8_t         c;              /* Consistent messages counter */
    uint8_t         k;              /* Redundancy constant */
    trickle_id      id;             /* Global ID of the trickle instance. Is the same in the entire network */
    trickle_tx_cb   tx_cb;          /* Callback function to be called when trickle wants to broadcast */    
    uint8_t         trickle_flags;  /* Bitfield for various flags used for housekeeping */
} trickle_t;

#define MAX_TRICKLE_INSTANCES 8

#define APP_TIMER_PRESCALER 16

#define RX_PROPAGATION_TIME         (377)

#define SYNC_TIMER_INTERVAL_MS      (10)

#define TRICKLE_FLAGS_T_DONE_Pos    (0)
#define TRICKLE_FLAGS_DISCARDED_Pos (1)

/* container for the trickle instances */
static trickle_t trickle_instances[MAX_TRICKLE_INSTANCES];
static uint32_t g_trickle_time; /* global trickle time that all time variables are relative to */

uint8_t rng_vals[64];
uint8_t rng_index;
uint8_t is_synced;

/* pointer to next synchronized trickle instance to be executed */
trickle_t* next_trickle;



static uint32_t interval_begin(trickle_t* trickle);


static void trickle_step(void)
{
    /* step global time */
    ++g_trickle_time;
    
    trickle_t* trickle = NULL;
    
    /* select an instance to broadcast */
    for (uint32_t i = 0; i < MAX_TRICKLE_INSTANCES; ++i)
    {
        uint8_t instance_can_broadcast = 0;
        if (trickle_instances[i].trickle_flags & (1 << TRICKLE_FLAGS_DISCARDED_Pos))
            continue;
        
        /* check whether the instance is able to broadcast */
        if (trickle_instances[i].trickle_flags & (1 << TRICKLE_FLAGS_T_DONE_Pos)) /* i is next timeout for this instance */
        {
            if (trickle_instances[i].i / SYNC_TIMER_INTERVAL_MS <= g_trickle_time)
            {
                /* double value of i */
                trickle_instances[i].i_relative = (trickle_instances[i].i_relative * 2 < trickle_instances[i].i_max * trickle_instances[i].i_min)?
                                trickle_instances[i].i_relative * 2 : 
                                trickle_instances[i].i_max * trickle_instances[i].i_min;
                
                interval_begin(&trickle_instances[i]);
            }
        }
        else /* t is next timeout for this instance */
        {
            if (trickle_instances[i].t / SYNC_TIMER_INTERVAL_MS <= g_trickle_time)
            {
                if (trickle_instances[i].c < trickle_instances[i].k)
                {
                    instance_can_broadcast = 1;
                }
                trickle_instances[i].trickle_flags |= (1 << TRICKLE_FLAGS_T_DONE_Pos);
            }
        }
        
        /* select the most urgent instance for broadcasting */
        if (instance_can_broadcast)
        {
            if (trickle != NULL)
            {
                if (trickle->i_relative > trickle_instances[i].i)
                {
                    trickle = &trickle_instances[i];
                }
            }
            else
            {
                trickle = &trickle_instances[i];
            }
        }
    }    
    
    /* do the send */
    if (trickle != NULL)
    {
        trickle->tx_cb();
    }
}

void TIMER0_IRQHandler(void)
{
    
    if (NRF_TIMER0->EVENTS_COMPARE[0])
    {
        NRF_TIMER0->EVENTS_COMPARE[0] = 0;
        NRF_TIMER0->CC[0] = 1000 * SYNC_TIMER_INTERVAL_MS;
        TICK_PIN(PIN_SYNC_TIME);
        
        trickle_step();
    }
}
    
static trickle_t* get_trickle(trickle_id id)
{
    for (uint32_t i = 0; i < MAX_TRICKLE_INSTANCES; ++i)
    {
        if (trickle_instances[i].id == id)
        {
            return &trickle_instances[i];
        }
    }
    return NULL;
}


/************************************************************************************
* Static pointer version of public functions
************************************************************************************/

static uint32_t interval_begin(trickle_t* trickle)
{
    /* if the system hasn't synced up with anyone else, just start own sync up */
    static uint8_t intervals = 0;
    if (is_synced == 0 && trickle->c == 0 && intervals > 2)
    {
        NRF_TIMER0->TASKS_START = 1;
        NRF_TIMER0->TASKS_CLEAR = 1;
    }
    ++intervals;
    trickle->c = 0;
    
    uint32_t rand_number =  ((uint32_t) rng_vals[(rng_index++) & 0x3F])       |
                            ((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 8  |
                            ((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 16 |
                            ((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 24;
    
    uint32_t i_half = trickle->i_relative / 2;
    trickle->t = g_trickle_time * SYNC_TIMER_INTERVAL_MS + i_half + (rand_number % i_half);
    trickle->i = g_trickle_time * SYNC_TIMER_INTERVAL_MS + trickle->i_relative;
    
    trickle->trickle_flags &= ~(1 << TRICKLE_FLAGS_T_DONE_Pos);
    
    TICK_PIN(PIN_NEW_INTERVAL);
    TICK_PIN((PIN_INT0 + trickle->id));
    
    return NRF_SUCCESS;
}

static uint32_t timer_reset(trickle_t* trickle)
{
    trickle->trickle_flags &= ~(1 << TRICKLE_FLAGS_T_DONE_Pos);
    trickle->i_relative = trickle->i_min; 
        
    return interval_begin(trickle);
}

void trickle_setup(void)
{
    /* setup timers */
    NRF_TIMER0->POWER = 1;
    NRF_TIMER0->PRESCALER = 4; // 1MHz, period = 1us
    NRF_TIMER0->INTENSET = TIMER_INTENSET_COMPARE0_Msk;
    NRF_TIMER0->SHORTS = TIMER_SHORTS_COMPARE0_CLEAR_Msk;
    NRF_TIMER0->CC[0] = 1000 * SYNC_TIMER_INTERVAL_MS; // 10ms
    NRF_TIMER0->TASKS_CLEAR = 1;
    NVIC_EnableIRQ(TIMER0_IRQn);
    
    for (uint8_t i = 0; i < MAX_TRICKLE_INSTANCES; ++i)
    {
        trickle_instances[i].trickle_flags |= (1 << TRICKLE_FLAGS_DISCARDED_Pos);
    }
    
    is_synced = 0;
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

uint32_t trickle_init(trickle_init_t* trickle)
{
    trickle_t* trickle_instance = NULL;
    for (uint32_t i = 0; i < MAX_TRICKLE_INSTANCES; ++i)
    {
        if (trickle_instances[i].trickle_flags & (1 << TRICKLE_FLAGS_DISCARDED_Pos))
        {
            trickle_instance = &trickle_instances[i];
            break;
        }
    }
    
    if (trickle_instance == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }
    
    if (trickle->i < trickle->i_min || 
        trickle->i > trickle->i_min * trickle->i_max ||
        trickle->k == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    
    trickle_instance->i_min         = trickle->i_min;
    trickle_instance->i_max         = trickle->i_max;
    trickle_instance->i_relative    = trickle->i;
    trickle_instance->id            = trickle->id;
    trickle_instance->k             = trickle->k;
    trickle_instance->trickle_flags = 0;
    trickle_instance->tx_cb         = trickle->tx_cb;
    
    return interval_begin(trickle_instance);
}

uint32_t trickle_discard(trickle_id id)
{
    trickle_t* trickle = get_trickle(id);
    
    if (trickle == NULL)
        return NRF_ERROR_NULL;
    
    trickle->trickle_flags |= (1 << TRICKLE_FLAGS_DISCARDED_Pos);
    
    return NRF_SUCCESS;    
}

uint32_t trickle_interval_begin(trickle_id id)
{
    trickle_t* trickle = get_trickle(id);
    
    if (trickle == NULL)
        return NRF_ERROR_NULL;
    
    return interval_begin(trickle);
}

uint32_t trickle_rx_consistent(trickle_id id)
{
    trickle_t* trickle = get_trickle(id);
    
    if (trickle == NULL)
        return NRF_ERROR_NULL;
    
    ++trickle->c;
    
    return NRF_SUCCESS;
}

uint32_t trickle_rx_inconsistent(trickle_id id)
{
    trickle_t* trickle = get_trickle(id);
    
    if (trickle == NULL)
        return NRF_ERROR_NULL;
    
    if (trickle->i_relative > trickle->i_min)
    {
        timer_reset(trickle);
    }
    
    return NRF_SUCCESS;
}

uint32_t trickle_timer_reset(trickle_id id)
{
    trickle_t* trickle = get_trickle(id);
    
    if (trickle == NULL)
        return NRF_ERROR_NULL;
    
    return timer_reset(trickle);
}

void trickle_sync(uint16_t msg_len)
{
    NRF_TIMER0->CC[0] = 1000 * SYNC_TIMER_INTERVAL_MS - RX_PROPAGATION_TIME - (8 * msg_len + 80);
    NRF_TIMER0->TASKS_START = 1;
    NRF_TIMER0->TASKS_CLEAR = 1;
    is_synced = 1;
}
