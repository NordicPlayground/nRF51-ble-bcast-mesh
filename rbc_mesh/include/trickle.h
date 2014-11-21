#ifndef _TRICKLE_H__
#define _TRICKLE_H__
#include <stdint.h>
#include "nrf51.h"
#include "app_timer.h"
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
    uint32_t        t;              /* Absolute value of t. Equals g_trickle_time (at set time) + t_relative */
    uint32_t        volatile i;              /* Absolute value of i. Equals g_trickle_time (at set time) + i_relative */
    uint32_t        volatile i_relative;     /* Relative value of i. Represents the actual i value in IETF RFC6206 */
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
void trickle_time_update(uint32_t time);

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
uint32_t trickle_timestamp_get(void);

/**
* @brief get the next time the indicated trickle instance is required to do some processing
*/
uint32_t trickle_next_processing_get(trickle_t* trickle);

#endif /* _TRICKLE_H__ */
