#ifndef _TRICKLE_H__
#define _TRICKLE_H__
#include <stdint.h>
#include "nrf51.h"
#include "app_timer.h"

typedef void (*trickle_tx_cb)(void);

typedef uint32_t trickle_id;

typedef struct
{
    uint32_t        i;              /* Current interval length */
    uint32_t        i_min;          /* Minimum interval in ms */
    uint16_t        i_max;          /* Maximum doubling of i_min. The biggest interval is (i_min * i_max) */
    uint8_t         c;              /* Consistent messages counter */
    uint8_t         k;              /* Redundancy constant */
    trickle_id      id;             /* Global ID of the trickle instance. Is the same in the entire network */
    trickle_tx_cb   tx_cb;          /* Callback function to be called when trickle wants to broadcast */    
} trickle_init_t;



void trickle_setup(void);

trickle_tx_cb trickle_step(void);

uint32_t trickle_init(trickle_init_t* trickle);

uint32_t trickle_discard(trickle_id id);

uint32_t trickle_interval_begin(trickle_id trickle);

uint32_t trickle_rx_consistent(trickle_id id);

uint32_t trickle_rx_inconsistent(trickle_id id);

uint32_t trickle_timer_reset(trickle_id trickle);

void trickle_sync(uint16_t msg_len);



#endif /* _TRICKLE_H__ */
