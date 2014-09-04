#ifndef _TRICKLE_H__
#define _TRICKLE_H__
#include <stdint.h>
#include "nrf51.h"

typedef void (*trickle_tx_cb)(void);

typedef struct
{
    uint32_t        i_min;  /* Minimum interval in ms */
    uint16_t        i_max;  /* Maximum doubling of i_min. The biggest interval is i_min * i_max */
    uint32_t        i;      /* Current interval length */
    uint8_t         c;      /* Consistent messages counter */
    uint8_t         k;      /* Redundancy constant */
    uint32_t        t;      /* Timing variable */
    trickle_tx_cb  tx_cb;  /* Callback function to be called when trickle wants to broadcast */
} trickle_t;

/*void trickle_random_pool_generate(void);*/
void trickle_init(trickle_t* trickle);
void trickle_interval_begin(trickle_t* trickle);
void trickle_rx_consistent(trickle_t* trickle);
void trickle_rx_inconsistent(trickle_t* trickle);
void trickle_timer_reset(trickle_t* trickle);



#endif /* _TRICKLE_H__ */
