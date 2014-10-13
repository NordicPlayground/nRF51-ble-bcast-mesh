#include "ll_control.h"
#include "radio_control.h"
#include "timer_control.h"
#include "timeslot_handler.h"
#include "trickle_common.h"
#include <string.h>

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


typedef void(*action_complete_cb)(void);

/*****************************************************************************
* Static globals
*****************************************************************************/
#define ADDRESS_PROPAGATION_US          (45)
#define MASTER_TIME_OFFSET_US           (260)
#define MESSAGE_SLOT_US                 (350)
#define T_IFS_US                        (150)
#define MESSAGE_PERIOD_US               (MESSAGE_SLOT_US + T_IFS_US)
#define PRECOMPUTE_TIME_US              (100)
#define MAX_DRIFT_US(ms_since_last_sync) ((ms_since_last_sync) / 20)

#define CONN_EVT_CONTROL_PACKETS        (2)
#define CONN_EVT_DATA_PACKETS           (6)
#define PACKETS_IN_CONN_EVT             (CONN_EVT_CONTROL_PACKETS + CONN_EVT_DATA_PACKETS)

#define CONN_EVT_LEN_US                 (PACKETS_IN_CONN_EVT * (MESSAGE_SLOT_US + T_IFS_US) + PRECOMPUTE_TIME_US)
#define CONN_INT_LEN_DEFAULT_US         (100000) /* 100ms */
#define SEARCH_TIMEOUT_US               (1000000) /* 1s */

#define SLAVE_ADV_PERIOD                (2)

/*** PACKET OFFSETS, TO BE PORTED TO EXTERNAL MODULE ***/
#define PACKET_OPCODE_POS               (3)
#define PACKET_MESH_HANDLE_POS          (PACKET_OPCODE_POS + 1)
#define PACKET_MASTER_HANDLE_POS        (PACKET_MESH_HANDLE_POS + 4)
#define PACKET_INTERVAL_POS             (PACKET_MASTER_HANDLE_POS + 4)
#define PACKET_OFFSET_POS               (PACKET_INTERVAL_POS + 2)
#define PACKET_SCA_POS                  (PACKET_OFFSET_POS + 2)

#define PACKET_TYPE_LL_CONTROL          (0x03)
#define PACKET_OPCODE_LL_MASTER_ANCHOR  (0x0E)
#define PACKET_OPCODE_LL_MASTER_OVERTAKE (0x0F)
#define PACKET_LL_MASTER_ANCHOR_LENGTH  (0x0C)

static conn_state_t g_conn_state = CONN_STATE_SEARCH;

/* TEST ANCHOR; SHOULD BE PORTED TO EXTERNAL MODULE LATER */
static uint8_t master_anchor[] = 
{
    PACKET_TYPE_LL_CONTROL, 0x00, PACKET_LL_MASTER_ANCHOR_LENGTH, /* PDU header */
    PACKET_OPCODE_LL_MASTER_ANCHOR, /* OPCODE */
    0x00, 0x01, 0x02, 0x03, /* Mesh handle */
    0xA0, 0xA1, 0xA2, 0xA3, /* master handle */
    0x50, 0x00, /* inverval = 100ms */
    0x00, 0x00, /* offset = 0 */
    0x20 /* Sleep clock accuracy of 250ppm */
};

/* TEST OVERTAKE; SHOULD BE PORTED TO EXTERNAL MODULE LATER */
static uint8_t master_overtake[] = 
{
    PACKET_TYPE_LL_CONTROL, 0x00, PACKET_LL_MASTER_ANCHOR_LENGTH, /* PDU header */
    PACKET_OPCODE_LL_MASTER_OVERTAKE, /* OPCODE  */
    0x00, 0x01, 0x02, 0x03, /* Mesh handle */
    0xA0, 0xA1, 0xA2, 0xA3, /* master handle */
    0x50, 0x00, /* inverval = 100ms */
    0x00, 0x00, /* offset = 0 */
    0x20 /* Sleep clock accuracy of 250ppm */
};

static uint8_t conn_evt_rx_timer_index;

static uint32_t conn_interval = 0;
static uint32_t my_master = 0;

/*****************************************************************************
* Static functions
*****************************************************************************/

static inline bool packet_is_anchor(uint8_t* data)
{
    return (data[0] == PACKET_TYPE_LL_CONTROL && 
        data[2] == PACKET_LL_MASTER_ANCHOR_LENGTH &&
        data[PACKET_OPCODE_POS] == PACKET_OPCODE_LL_MASTER_ANCHOR);
        
}

static void conn_evt_rx_cb(uint8_t* data)
{
    timer_abort(conn_evt_rx_timer_index);
    memcpy(master_anchor, data, 17);
}

static void conn_evt_no_anchor(void)
{
    // TODO
}

static void conn_evt_master_rx_cb(uint8_t* data)
{
    if (data[PACKET_OPCODE_POS] == PACKET_OPCODE_LL_MASTER_OVERTAKE)
    {
        /* TODO */
    }
}


/********* SEPARATE ACTIONS IN LL STATEMACHINE **********/
/* Events that are repeated through several states */

static void ll_action_conn_evt_relay(action_complete_cb end_cb)
{
    /* rx anchor */
    timer_reference_point_trigger((uint32_t*) &(NRF_RADIO->EVENTS_ADDRESS), -ADDRESS_PROPAGATION_US);
    
    radio_event_t radio_search;
    radio_search.access_address = 1;
    radio_search.callback.rx = conn_evt_rx_cb;
    radio_search.channel = 37;
    radio_search.event_type = RADIO_EVENT_TYPE_RX;
    radio_search.start_time = 0;
    
    radio_order(&radio_search);
    
    conn_evt_rx_timer_index = 
        timer_order_cb_ppi(700, conn_evt_no_anchor, (uint32_t*) &(NRF_RADIO->TASKS_STOP));
    
    /* tx anchor */
    radio_event_t radio_anchor;
    radio_anchor.access_address = 0;
    radio_anchor.callback.tx = NULL;
    radio_anchor.channel = 37;
    radio_anchor.event_type = RADIO_EVENT_TYPE_TX;
    radio_anchor.packet_ptr = master_anchor;
    radio_anchor.start_time = MESSAGE_PERIOD_US;
    
    radio_order(&radio_anchor);
    
    /* TODO: DATA SLOTS HERE */
}

static void ll_action_conn_evt_master(action_complete_cb end_cb)
{
    /* tx anchor */
    
    radio_event_t radio_anchor;
    radio_anchor.access_address = 0;
    radio_anchor.callback.tx = NULL;
    radio_anchor.channel = 37;
    radio_anchor.event_type = RADIO_EVENT_TYPE_TX;
    radio_anchor.packet_ptr = master_anchor;
    radio_anchor.start_time = MASTER_TIME_OFFSET_US;
    
    radio_order(&radio_anchor);
    
    radio_event_t radio_rsp_rx;
    radio_rsp_rx.access_address = 1;
    radio_rsp_rx.callback.rx = conn_evt_rx_cb;
    radio_rsp_rx.channel = 37;
    radio_rsp_rx.event_type = RADIO_EVENT_TYPE_RX;
    radio_rsp_rx.start_time = MASTER_TIME_OFFSET_US + MESSAGE_PERIOD_US;
    
    radio_order(&radio_rsp_rx);
    /* TODO: DATA SLOTS HERE */
}

static void ll_action_conn_evt_slave(action_complete_cb end_cb)
{
    static uint8_t adv_period_counter = 0;
    /* rx anchor */
    timer_reference_point_trigger((uint32_t*) &(NRF_RADIO->EVENTS_ADDRESS), -ADDRESS_PROPAGATION_US);
    
    radio_event_t radio_search;
    radio_search.access_address = 1;
    radio_search.callback.rx = conn_evt_rx_cb;
    radio_search.channel = 37;
    radio_search.event_type = RADIO_EVENT_TYPE_RX;
    radio_search.start_time = 0;
    
    radio_order(&radio_search);
    
    conn_evt_rx_timer_index = 
        timer_order_cb_ppi(700, conn_evt_no_anchor, (uint32_t*) &(NRF_RADIO->TASKS_STOP));
    
    
    ++adv_period_counter;
    
    if ((adv_period_counter & (SLAVE_ADV_PERIOD - 1)) == 0)
    {
        /* tx anchor */
        radio_event_t radio_anchor;
        radio_anchor.access_address = 0;
        radio_anchor.callback.tx = NULL;
        radio_anchor.channel = 37;
        radio_anchor.event_type = RADIO_EVENT_TYPE_TX;
        radio_anchor.packet_ptr = master_anchor;
        radio_anchor.start_time = MESSAGE_SLOT_US + T_IFS_US;
        
        radio_order(&radio_anchor);
    }
    
    /* TODO: DATA SLOTS HERE */
}

/********* STATES **********/

static void search_start(void);
static void master_start(void);
static void master_search_start(void);
static void connect_start(void);
static void sync_start(void);
static void slave_start(void);
static void relay_start(void); 
static void handover_start(void);


static void master_rx(uint8_t* data)
{
    if (data[0] != 0x03)
        return;
    
}

static void master_search_start(void)
{
    g_conn_state = CONN_STATE_MASTER_SEARCH;
    
    
    uint32_t timeslot_remaining = timeslot_get_remaining_time();
    if (timeslot_remaining < CONN_INT_LEN_DEFAULT_US + CONN_EVT_LEN_US)
    {
        timeslot_extend(CONN_INT_LEN_DEFAULT_US + CONN_EVT_LEN_US - timeslot_remaining);
    }
    
    radio_disable();
    
    ll_action_conn_evt_master();
    
    /* TODO: ADD SEARCH */
    
}

static void sync_rx(uint8_t* data)
{
    static bool has_children = false;
    if (packet_is_anchor(data))
    {
        uint32_t incoming_master = 0;
        memcpy(&incoming_master, &data[PACKET_MASTER_HANDLE_POS], 4);
        
        if (incoming_master == my_master)
        {
            radio_disable(); /* abort current response attempt */
            if (has_children)
            {
                relay_start();
            }
            else
            {
                slave_start();
            }
        }
        else
        {
            has_children = true;
            radio_event_t radio_search;
            radio_search.access_address = 1;
            radio_search.channel = 37;
            radio_search.event_type = RADIO_EVENT_TYPE_RX;
            radio_search.start_time = 0;
            radio_search.callback.rx = sync_rx;
            
            radio_order(&radio_search);
            
            radio_event_t radio_response;
            radio_response.access_address = 0;
            radio_response.callback.tx = NULL;
            radio_response.channel = 37;
            radio_response.event_type = RADIO_EVENT_TYPE_TX;
            radio_response.packet_ptr = master_overtake;
            radio_response.start_time = 0; /* as soon as possible after RX */
            
            radio_order(&radio_response);
        }
        
    }
            
        
}

static void sync_start(void)
{
    g_conn_state = CONN_STATE_SYNC;
    
    uint32_t timeslot_remaining = timeslot_get_remaining_time();
    if (timeslot_remaining < conn_interval)
    {
        timeslot_extend(conn_interval - timeslot_remaining);
    }
    
    memcpy(&master_overtake[PACKET_MASTER_HANDLE_POS], &my_master, 4);
    memcpy(&master_anchor[PACKET_MASTER_HANDLE_POS], &my_master, 4);
    
    radio_event_t radio_search;
    radio_search.access_address = 1;
    radio_search.channel = 37;
    radio_search.event_type = RADIO_EVENT_TYPE_RX;
    radio_search.start_time = 0;
    radio_search.callback.rx = sync_rx;
    
    radio_order(&radio_search);
    
    radio_event_t radio_response;
    radio_response.access_address = 0;
    radio_response.callback.tx = NULL;
    radio_response.channel = 37;
    radio_response.event_type = RADIO_EVENT_TYPE_TX;
    radio_response.packet_ptr = master_overtake;
    radio_response.start_time = 0; /* as soon as possible after RX */
    
    radio_order(&radio_response);
    
    timer_order_cb(SEARCH_TIMEOUT_US, search_start);
}

static void connect_rx(uint8_t* data)
{
    if (packet_is_anchor(data))
    {
        uint32_t this_master = 0;
        memcpy(&this_master, &data[PACKET_MASTER_HANDLE_POS], 4);
        
        if (my_master < this_master)
        {
            my_master = this_master;
        }
        else if (my_master == this_master) /* one period has passed */
        {
            sync_start();
        }
    }
}

static void connect_start(void)
{
    g_conn_state = CONN_STATE_CONNECT;
    
    uint32_t timeslot_remaining = timeslot_get_remaining_time();
    if (timeslot_remaining < conn_interval * 2)
    {
        timeslot_extend(conn_interval * 2 - timeslot_remaining);
    }
    
    radio_event_t radio_search;
    radio_search.access_address = 1;
    radio_search.channel = 37;
    radio_search.event_type = RADIO_EVENT_TYPE_RX;
    radio_search.start_time = 0;
    radio_search.callback.rx = connect_rx;
    
    radio_order(&radio_search);
    
    
    timer_order_cb(SEARCH_TIMEOUT_US, search_start);
}

static void search_rx(uint8_t* data)
{
    if (data == NULL)
    {
        master_search_start();
    }
    else if (packet_is_anchor(data))
    {
        conn_interval = (data[PACKET_INTERVAL_POS] | 
                data[PACKET_INTERVAL_POS + 1] << 8);
        
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
    radio_search.callback.rx = search_rx;
    
    radio_order(&radio_search);
    
    
    uint32_t timeslot_remaining = timeslot_get_remaining_time();
    
    if (timeslot_remaining < SEARCH_TIMEOUT_US + 60)
    {
        timeslot_extend(SEARCH_TIMEOUT_US - timeslot_remaining);
    }
    
    timer_order_cb(SEARCH_TIMEOUT_US, master_search_start);
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

void ll_control_init(void)
{
    
}

void ll_control_reset(void)
{

}

void ll_control_slot_start(void)
{
    PIN_OUT(g_conn_state, 32);
    switch (g_conn_state)
    {
        case CONN_STATE_SEARCH:
            search_start();
            break;
        
        case CONN_STATE_MASTER_SEARCH:
        case CONN_STATE_MASTER:
            
            g_conn_state = CONN_STATE_MASTER;
        
               
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
