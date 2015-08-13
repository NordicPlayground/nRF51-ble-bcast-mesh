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

  4. This software must only be used in a processor manufactured by Nordic
  Semiconductor ASA, or in a processor manufactured by a third party that
  is used in combination with a processor manufactured by Nordic Semiconductor.


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

#include "nrf_soc.h"
#include "nrf51_bitfields.h"
#include <string.h>


#define TRICKLE_RNG_POOL_SIZE   (64)

/*****************************************************************************
* Static Globals
*****************************************************************************/

static uint8_t rng_vals[64];
static uint8_t rng_index = 0;

/*global parameters for trickle behavior, set in trickle_setup() */
static uint32_t g_i_min, g_i_max;
static uint8_t g_k;

/*****************************************************************************
* Static Functions
*****************************************************************************/

/**
* @brief Do calculations for beginning of a trickle interval. Is called from
*   trickle_step function.
*/
static void trickle_interval_begin(trickle_t* trickle)
{
    trickle->c = 0;

    trickle->i += trickle->i_relative;
}

static void refresh_t(trickle_t* trickle)
{
    uint32_t rand_number = 0;
    rand_number |= ((uint32_t) rng_vals[(rng_index++) & 0x3F]);
    rand_number |= (((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 8);
    rand_number |= (((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 16);
    rand_number |= (((uint32_t) rng_vals[(rng_index++) & 0x3F]) << 24);

    uint64_t i_half = trickle->i_relative / 2;
    trickle->t = trickle->i + i_half + (rand_number % i_half);
}


static void check_interval(trickle_t* trickle, uint64_t time_now)
{
    if (time_now >= trickle->i)
    {
        /* we've started a new interval since we last touched this trickle */
        if (trickle->i_relative < g_i_max * g_i_min)
            trickle->i_relative *= 2;
        else
            trickle->i_relative = g_i_max * g_i_min;

        trickle->c = 0;
        uint32_t delta = (time_now - trickle->i) / trickle->i_relative;
        trickle->i += trickle->i_relative * (delta + 1);
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
    uint32_t error_code;
    rng_index = 0;

    /* Fill rng pool */
    uint8_t bytes_available;
    do
    {
        error_code =
            sd_rand_application_bytes_available_get(&bytes_available);
        APP_ERROR_CHECK(error_code);
        if (bytes_available > 0)
        {
            uint8_t byte_count =
                ((bytes_available > TRICKLE_RNG_POOL_SIZE - rng_index)?
                (TRICKLE_RNG_POOL_SIZE - rng_index) :
                (bytes_available));

            error_code =
                sd_rand_application_vector_get(&rng_vals[rng_index],
                byte_count);
            APP_ERROR_CHECK(error_code);

            rng_index += byte_count;
        }
    } while (rng_index < TRICKLE_RNG_POOL_SIZE);

}

void trickle_rx_consistent(trickle_t* trickle, uint64_t time_now)
{
    TICK_PIN(PIN_CONSISTENT);
    check_interval(trickle, time_now);
    ++trickle->c;
}

void trickle_rx_inconsistent(trickle_t* trickle, uint64_t time_now)
{
    TICK_PIN(PIN_INCONSISTENT);
    if (trickle->i_relative > g_i_min)
    {
        trickle_timer_reset(trickle, time_now);
    }
}

void trickle_timer_reset(trickle_t* trickle, uint64_t time_now)
{
    trickle->i = time_now;
    trickle->i_relative = g_i_min;

    trickle_interval_begin(trickle);
    refresh_t(trickle);
}

void trickle_tx_register(trickle_t* trickle)
{
    refresh_t(trickle); /* order next t */
}

void trickle_tx_timeout(trickle_t* trickle, bool* out_do_tx, uint64_t time_now)
{
    *out_do_tx = (trickle->c < g_k);
    check_interval(trickle, time_now);
    if (!(*out_do_tx))
    {
        /* will never get a call to tx_register, order next t manually */
        refresh_t(trickle);
    }
}

uint64_t trickle_next_processing_get(trickle_t* trickle, uint64_t time_now)
{
    check_interval(trickle, time_now);
    return trickle->t;
}
