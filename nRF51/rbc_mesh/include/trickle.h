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

#ifndef _TRICKLE_H__
#define _TRICKLE_H__
#include "nrf51.h"
#include <stdint.h>
#include <stdbool.h>
/**
* @file Implementation of Trickle algorithm described in IETF RFC6206
*   http://tools.ietf.org/html/rfc6206
*/

/**
* @brief trickle instance type. Contains all values necessary for maintaining
*   an isolated version of the algorithm
*/
typedef struct
{
    uint64_t        t;              /* Absolute value of t. Equals g_trickle_time (at set time) + t_relative */
    uint64_t        volatile i;              /* Absolute value of i. Equals g_trickle_time (at set time) + i_relative */
    uint64_t        volatile i_relative;     /* Relative value of i. Represents the actual i value in IETF RFC6206 */
    uint8_t         c;              /* Consistent messages counter */
    uint8_t         trickle_flags;  /* Bitfield for various flags used for housekeeping */
} trickle_t;


/** 
* @brief Setup the algorithm. Is only called once, and before all other trickle
*   related functions.
*/
void trickle_setup(uint32_t i_min, uint32_t i_max, uint8_t k);

/** 
* @brief increment global trickle timestamp by one 
*/
void trickle_time_increment(void);

/**
* @brief set absolute value of global trickle timestamp
*/
void trickle_time_update(uint64_t time);

/**
* @brief initialize a trickle algorithm instance. Prepares all flags and 
*   values used for book keeping
*/
void trickle_init(trickle_t* trickle);

/**
* @brief Register a consistent RX on the given trickle algorithm instance.
*   Increments the instance's C value.
*/
void trickle_rx_consistent(trickle_t* id);

/**
* @brief register an inconsistent RX on the given trickle algorithm instance.
*   Resets interval time.
*/
void trickle_rx_inconsistent(trickle_t* id);

/**
* @brief reset interval timer for the given trickle algorithm instance.
*/
void trickle_timer_reset(trickle_t* trickle);

/**
* @brief register a successful TX on the given trickle algorithm instance.
*/
void trickle_register_tx(trickle_t* trickle);

/**
* @brief Check timeouts and check whether a TX on the trickle instance is 
*   necessary.
* 
* @param[in] trickle pointer to trickle algorithm instance object.
* @param[out] out_do_tx returns whether the trickle instance is due for a TX
*/
void trickle_step(trickle_t* trickle, bool* out_do_tx);

/**
* @brief get current global trickle timestamp
*/
uint64_t trickle_timestamp_get(void);

/**
* @brief get the next time the indicated trickle instance is required to do some processing
*/
uint64_t trickle_next_processing_get(trickle_t* trickle);

#endif /* _TRICKLE_H__ */
