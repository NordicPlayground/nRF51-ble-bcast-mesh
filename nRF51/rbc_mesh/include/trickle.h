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

#ifndef _TRICKLE_H__
#define _TRICKLE_H__
#ifdef NRF51
#include "nrf.h"
#else
#include "nrf.h"
#endif
#include "toolchain.h"
#include <stdint.h>
#include <stdbool.h>
/**
* @file Implementation of Trickle algorithm described in IETF RFC6206
*   http://tools.ietf.org/html/rfc6206
*/

#define TRICKLE_C_DISABLED  (0xFF)

/**
* @brief trickle instance type. Contains all values necessary for maintaining
*   an isolated version of the algorithm
*/
typedef __packed_armcc struct
{
    uint32_t        t;              /* Absolute value of t. Equals g_trickle_time (at set time) + t_relative */
    uint32_t        i;              /* Absolute value of i. Equals g_trickle_time (at set time) + i_relative */
    uint32_t        i_relative;     /* Relative value of i. Represents the actual i value in IETF RFC6206 */
    uint8_t         c;              /* Consistent messages counter */
} __packed_gcc trickle_t;


/** 
* @brief Setup the algorithm. Is only called once, and before all other trickle
*   related functions.
*/
void trickle_setup(uint32_t i_min, uint32_t i_max, uint8_t k);

/**
* @brief Register a consistent RX on the given trickle algorithm instance.
*   Increments the instance's C value.
*/
void trickle_rx_consistent(trickle_t* id, uint32_t time_now);

/**
* @brief register an inconsistent RX on the given trickle algorithm instance.
*   Resets interval time.
*/
void trickle_rx_inconsistent(trickle_t* id, uint32_t time_now);

/**
* @brief reset interval timer for the given trickle algorithm instance.
*/
void trickle_timer_reset(trickle_t* trickle, uint32_t time_now);

/**
* @brief register a successful TX on the given trickle algorithm instance.
*/
void trickle_tx_register(trickle_t* trickle, uint32_t time_now);

/**
* @brief Check timeouts and check whether a TX on the trickle instance is 
*   necessary.
* 
* @param[in] trickle pointer to trickle algorithm instance object.
* @param[out] out_do_tx returns whether the trickle instance is due for a TX
*/
void trickle_tx_timeout(trickle_t* trickle, bool* out_do_tx, uint32_t time_now);

/**
* @brief Disable the given trickle instance. It will always report that it is 
*   not to perform a transmit when checked.
*/
void trickle_disable(trickle_t* trickle);

/** 
* @brief enable a previously disabled trickle value
*/
void trickle_enable(trickle_t* trickle);

/**
* @brief return whether the given trickle instance is enabled, and should be
*   considered for transmission.
*/
bool trickle_is_enabled(trickle_t* trickle);

#endif /* _TRICKLE_H__ */
