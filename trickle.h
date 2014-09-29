#ifndef _TRICKLE_H__
#define _TRICKLE_H__
#include <stdint.h>
#include "nrf51.h"
#include "app_timer.h"

typedef struct
{
    uint32_t        t;              /* Absolute value of t. Equals g_trickle_time (at set time) + t_relative */
    uint32_t        i;              /* Absolute value of i. Equals g_trickle_time (at set time) + i_relative */
    uint32_t        i_relative;     /* Relative value of i. Represents the actual i value in RFC6206 */
    uint8_t         c;              /* Consistent messages counter */
    uint8_t         trickle_flags;  /* Bitfield for various flags used for housekeeping */
} trickle_t;


void trickle_setup(uint32_t i_min, uint32_t i_max, uint8_t k);

void trickle_time_increment(void);

void trickle_init(trickle_t* trickle);

void trickle_rx_consistent(trickle_t* id);

void trickle_rx_inconsistent(trickle_t* id);

void trickle_timer_reset(trickle_t* trickle);

void trickle_register_tx(trickle_t* trickle);

void trickle_step(trickle_t* trickle, bool* out_do_tx);

#endif /* _TRICKLE_H__ */
