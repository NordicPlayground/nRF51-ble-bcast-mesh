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

#include "trickle.h"
#include "rbc_mesh_common.h"
#include "app_error.h"
#include "rand.h"
#include "timer.h"

#include "nrf_soc.h"
#ifdef NRF51
#include "nrf51_bitfields.h"
#else
#include "nrf.h"
#endif
#include <string.h>

#define TIME_MARGIN (1000)
/*****************************************************************************
* Static Globals
*****************************************************************************/

/*global parameters for trickle behavior, set in trickle_setup() */
static uint32_t g_i_min, g_i_max;
static uint8_t g_k;

static prng_t g_rand;

/*****************************************************************************
* Static Functions
*****************************************************************************/
/**
* @brief Do calculations for beginning of a trickle interval. Is called from
*   trickle_step function.
*/
static void trickle_interval_begin(trickle_t* trickle)
{
    if (trickle_is_enabled(trickle))
    {
        trickle->c = 0;
        trickle->i += trickle->i_relative;
    }
}

static void refresh_t(trickle_t* trickle, uint32_t time_now)
{
    static uint32_t time_prev = 0;
    if (TIMER_OLDER_THAN(time_now, time_prev))
    {
        time_now = time_prev;
    }
    time_prev = time_now;

    uint32_t rand_number = rand_prng_get(&g_rand);

    uint32_t i_half = trickle->i_relative >> 1;

    trickle->t = trickle->i + i_half + (rand_number % i_half);
}

static void check_interval(trickle_t* trickle, uint32_t time_now)
{
    if (!TIMER_OLDER_THAN(time_now, trickle->i) && trickle_is_enabled(trickle))
    {
        if (trickle->i_relative < g_i_max * g_i_min)
            trickle->i_relative <<= 1;
        else
            trickle->i_relative = g_i_max * g_i_min;
        /* we've started a new interval since we last touched this trickle */
        trickle->c = 0;
        trickle->i = trickle->i_relative + time_now;
    }
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/
void trickle_setup(uint32_t i_min, uint32_t i_max, uint8_t k)
{
    g_i_min = i_min;
    g_i_max = i_max;
    g_k = k;

    rand_prng_seed(&g_rand);
}

void trickle_rx_consistent(trickle_t* trickle, uint32_t time_now)
{
    if (trickle_is_enabled(trickle))
    {
        TICK_PIN(PIN_CONSISTENT);
        check_interval(trickle, time_now);
        if (trickle->c + 1 != TRICKLE_C_DISABLED)
        {
            ++trickle->c;
        }
    }
}

void trickle_rx_inconsistent(trickle_t* trickle, uint32_t time_now)
{
    TICK_PIN(PIN_INCONSISTENT);
    if (trickle->i_relative > g_i_min)
    {
        trickle_timer_reset(trickle, time_now);
    }
}

void trickle_timer_reset(trickle_t* trickle, uint32_t time_now)
{
    trickle->i = time_now;
    trickle->i_relative = g_i_min;

    refresh_t(trickle, time_now);
    trickle_interval_begin(trickle);
}

void trickle_tx_register(trickle_t* trickle, uint32_t time_now)
{
    if (TIMER_OLDER_THAN(trickle->i, time_now))
    {
        trickle->i = time_now;
    }
    refresh_t(trickle, time_now); /* order next t */
}

void trickle_tx_timeout(trickle_t* trickle, bool* out_do_tx, uint32_t time_now)
{
    if (!trickle_is_enabled(trickle))
    {
        *out_do_tx = false;
    }
    else
    {
        *out_do_tx = (trickle->c < g_k);
        check_interval(trickle, time_now);
        if (!(*out_do_tx))
        {
            /* will never get a call to tx_register, order next t manually */
            refresh_t(trickle, time_now);
        }
    }
}

void trickle_disable(trickle_t* trickle)
{
    trickle->c = TRICKLE_C_DISABLED;
}

void trickle_enable(trickle_t* trickle)
{
    if (trickle->c == TRICKLE_C_DISABLED)
    {
        trickle->c = 0;
        trickle_timer_reset(trickle, 0);
    }
}

bool trickle_is_enabled(trickle_t* trickle)
{
    return (trickle->c != TRICKLE_C_DISABLED);
}
