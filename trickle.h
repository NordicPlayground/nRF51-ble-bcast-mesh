#ifndef _TRICKLE_H__
#define _TRICKLE_H__
#include <stdint.h>
#include "nrf51.h"
#include "app_timer.h"

typedef void (*trickle_tx_cb)(void);

typedef struct _trickle
{
    uint32_t        i;      /* Current interval length */
    uint32_t        i_min;  /* Minimum interval in ms */
    uint16_t        i_max;  /* Maximum doubling of i_min. The biggest interval is (i_min * i_max) */
    uint8_t         c;      /* Consistent messages counter */
    uint8_t         k;      /* Redundancy constant */
    uint32_t        t;      /* Timing variable */
    uint32_t        id;     /* Trickle id, chosen by the user. Should be unique for the mesh. */
    trickle_tx_cb   tx_cb;  /* Callback function to be called when trickle wants to broadcast */
    
    /* House keeping varaibles, should be initialized to 0: */
    app_timer_id_t  timer_id;   /* Internal timer id for the trickle_instance. Used for house keeping. Should be initialized to 0 */
    uint8_t         trickle_flags; /* Internal flag field for housekeeping. Should be initialized to 0 */
    
} trickle_t;

void trickle_setup(void);
void trickle_init(trickle_t* trickle);
void trickle_interval_begin(trickle_t* trickle);
void trickle_rx_consistent(trickle_t* trickle);
void trickle_rx_inconsistent(trickle_t* trickle);
void trickle_timer_reset(trickle_t* trickle);
void trickle_sync(void);



#endif /* _TRICKLE_H__ */
