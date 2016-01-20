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

#ifndef RAND_H__
#define RAND_H__

#include <stdint.h>

/**
 * @defgroup RAND Random number generator
 * This module abstracts the HW RNG, as well as providing a simple PRNG for non-cryptographic
 * random numbers.
 * @{
 */


/** PRNG instance structure. */
typedef struct
{
    uint32_t a; /**< PRNG state variable A */
    uint32_t b; /**< PRNG state variable B */
    uint32_t c; /**< PRNG state variable C */
    uint32_t d; /**< PRNG state variable D */
} prng_t;

/**
 * Seed a PRNG-instance with random values from the HW RNG module.
 *
 * @note The prng-structure is not seeded by default, and all new instances should be seeded
 *       before use. The probability of cycles shorter than 2^64 are assumed neglectable, but
 *       it is still advisable to reseed the prng with regular intervals.
 *
 * @param[in,out] p_prng A pointer to a prng_t structure to be seeded.
 *
 * @return NRF_SUCCESS The PRNG structure was successfully seeded.
 */
uint32_t rand_prng_seed(prng_t* p_prng);

/**
 * Get a 32bit pseudo-random value from the given PRNG instance.
 *
 * @warning The PRNG must NEVER be used for cryptographic purposes, as it is considered unsafe.
 *
 * @param[in,out] p_prng The PRNG instance to get a value from. The instance is altered for each
 *                get.
 *
 * @return A 32bit pseudo-random value.
 */
uint32_t rand_prng_get(prng_t* p_prng);

/**
 * Get a variable length array of random bytes from the HW RNG module.
 *
 * @param[out] p_result An array to copy the random values to. Must be at least of length len.
 * @param[in] len The number of bytes to be copied into the p_result parameter.
 *
 * @return NRF_SUCCESS The p_result parameter was successfully filled with len number of random bytes.
 */
uint32_t rand_hw_rng_get(uint8_t* p_result, uint16_t len);

/** @} */

#endif /* RAND_H__ */
