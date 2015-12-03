/* Copyright (c) Nordic Semiconductor ASA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

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
