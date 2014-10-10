#include "conn_evt.h"
#include "radio_control.h"
#include "timer_control.h"
#include "timeslot_handler.h"



/*****************************************************************************
* Local type definitions
*****************************************************************************/

typedef enum
{
    CONN_STATE_SEARCH, /* Always RX, try to find master */
    CONN_STATE_MASTER, /* Periodic anchor send */
    CONN_STATE_MASTER_SEARCH, /* Master state with search between conn events*/
    CONN_STATE_CONNECT, /* Always RX, try to find and sync masters */
    CONN_STATE_SYNC, /* Always RX, tell all except master to sync to self */
    CONN_STATE_SLAVE, /* Periodic anchor RX, occasional adv */
    CONN_STATE_RELAY, /* Periodic anchor RX, followed by anchor TX */
    CONN_STATE_HANDOVER /* Syncing to new master */
} conn_state_t;


/*****************************************************************************
* Static globals
*****************************************************************************/

#define CONN_INT_DEFAULT_US    (100000)

static conn_state_t g_conn_state = CONN_STATE_SEARCH;


/*****************************************************************************
* Static functions
*****************************************************************************/
static void search_start(void);
static void master_start(void);
static void master_search_start(void);
static void connect_start(void);
static void sync_start(void);
static void slave_start(void);
static void relay_start(void); 
static void handover_start(void);



static void master_search_start(void)
{
    g_conn_state = CONN_STATE_MASTER_SEARCH;
    
    
    
}


static void search_result(uint8_t* data)
{
    if (data == NULL)
    {
        master_search_start();
    }
    else
    {
        connect_start();
    }
}

static void search_start(void)
{
    g_conn_state = CONN_STATE_SEARCH;
    
    radio_event_t radio_search;
    radio_search.access_address = 1;
    radio_search.channel = 37;
    radio_search.event_type = RADIO_EVENT_TYPE_RX;
    radio_search.start_time = 0;
    radio_search.callback.rx = search_result;
    
    radio_order(&radio_search);
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

void conn_evt_init(void)
{
    
}


void conn_evt_start(void)
{
    switch (g_conn_state)
    {
        case CONN_STATE_SEARCH:
            start_search();
            break;
        
        case CONN_STATE_MASTER:
            
            break;
        
        case CONN_STATE_MASTER_SEARCH:
               
            break;
        
        case CONN_STATE_CONNECT:
            
            break;
        
        case CONN_STATE_SYNC:
            
            break;
        
        case CONN_STATE_SLAVE:
            
            break;
        
        case CONN_STATE_RELAY:
            
            break;
        
        case CONN_STATE_HANDOVER:
            
            break;
    }
}