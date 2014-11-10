#include "ll_control.h"

#include "boards.h"
#include "nrf_delay.h"

#include "radio_control.h"
#include "timer_control.h"
#include "mesh_srv.h"
#include "rbc_mesh.h"
#include "timeslot_handler.h"
#include "rbc_mesh_common.h"
#include "ble_gap.h"
#include "nrf_soc.h"
#include <string.h>

/**@file
* 
* @brief Controller for radio_control, timer_control and timeslot_handler.
*   Acts as an abstraction of the lower link layer to higher layer modules
*/

#define PACKET_TYPE_LEN             (1)
#define PACKET_LENGTH_LEN           (1)
#define PACKET_ADDR_LEN             (BLE_GAP_ADDR_LEN)

#define PACKET_TYPE_POS             (0)
#define PACKET_LENGTH_POS           (1)
#define PACKET_PADDING_POS          (2)
#define PACKET_ADDR_POS             (3)
#define PACKET_DATA_POS             (PACKET_ADDR_POS + PACKET_ADDR_LEN)


#define PACKET_TYPE_ADV_NONCONN     (0x02)

#define PACKET_TYPE_MASK            (0x0F)
#define PACKET_LENGTH_MASK          (0x3F)
#define PACKET_ADDR_TYPE_MASK       (0x40)

#define PACKET_DATA_MAX_LEN         (28)
#define PACKET_MAX_CHAIN_LEN        (1) /**@TODO: May be increased when RX 
                                        callback packet chain handling is implemented.*/

/* minimum time to be left in the timeslot for there to be any point in ordering the radio */
#define RADIO_SAFETY_TIMING_US      (500)


static uint8_t tx_data[(PACKET_DATA_MAX_LEN + PACKET_DATA_POS) * PACKET_MAX_CHAIN_LEN];
static uint8_t timer_index = 0xFF;
static uint32_t global_time = 0;
static uint32_t step_time = 0;

static void search_callback(uint8_t* data);
static void trickle_step_callback(void);

static void order_search(void)
{
    radio_event_t search_event;
    search_event.access_address = 1; /* RX: treat as bitfield */
    search_event.callback.rx = search_callback;
    rbc_mesh_channel_get(&search_event.channel);
    search_event.event_type = RADIO_EVENT_TYPE_RX;
    search_event.start_time = 0;
    
    radio_order(&search_event);   
}

static inline void packet_create_from_data(uint8_t* data, packet_t* packet)
{
    /* advertisement package */
    packet->data = &data[PACKET_DATA_POS];
    packet->length = (data[PACKET_LENGTH_POS] & PACKET_LENGTH_MASK);
    
    memcpy(packet->sender.addr, &data[PACKET_ADDR_POS], PACKET_ADDR_LEN);
    
    /* addr type */
    bool addr_is_random = (data[PACKET_TYPE_POS] & PACKET_ADDR_TYPE_MASK);
    
    if (addr_is_random)
    {
        bool is_static = ((packet->sender.addr[5] & 0xC0) == 0xC0);
        if (is_static)
        {
            packet->sender.addr_type = BLE_GAP_ADDR_TYPE_RANDOM_STATIC;
        }
        else
        {
            bool is_resolvable = ((packet->sender.addr[5] & 0xC0) == 0x40);
            packet->sender.addr_type = (is_resolvable? 
                BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_RESOLVABLE :
                BLE_GAP_ADDR_TYPE_RANDOM_PRIVATE_NON_RESOLVABLE);
        }
    }
    else
    {
        packet->sender.addr_type = BLE_GAP_ADDR_TYPE_PUBLIC;
    }
}


static inline bool packet_is_data_packet(uint8_t* data)
{
    return ((data[PACKET_TYPE_POS] & PACKET_TYPE_MASK) 
        == PACKET_TYPE_ADV_NONCONN);
}

/**
* @brief Handle incoming packets 
*/
static void search_callback(uint8_t* data)
{
    TICK_PIN(PIN_RX);
    
    /* check if timeslot is about to end */
    uint8_t dummy;
    
    /* Do the radio order in a critical section to keep the timeslot handler 
    from ending the timeslot while we're working on the hardware, as it 
    causes a hardfault */
    sd_nvic_critical_region_enter(&dummy);
    
    uint32_t radio_time_left = timeslot_get_remaining_time();
    
    if (radio_time_left > RADIO_SAFETY_TIMING_US)
    {
        /* setup next RX */
        order_search();
        /* setup timer again, in case things have changed */    
        TICK_PIN(PIN_RX);
        uint32_t new_processing_timeout;
        if (mesh_srv_get_next_processing_time(&new_processing_timeout) != NRF_SUCCESS)
        {
            timer_abort(timer_index);
        }
    }
    /* all hardware work done */
    sd_nvic_critical_region_exit(dummy);
    
    if (data == NULL || !packet_is_data_packet(data))
        return;
    
    
    TICK_PIN(1);
    
    packet_t packet;
    packet_create_from_data(data, &packet);
    mesh_srv_packet_process(&packet);
    
    /** @TODO: add packet chain handling */
}

/**
* @brief Handle trickle timing events 
*/
static void trickle_step_callback(void)
{
    /* check if timeslot is about to end */
    if (timeslot_get_remaining_time() < RADIO_SAFETY_TIMING_US)
        return;
    
    trickle_time_update(step_time);
    
    uint8_t temp_data[PACKET_DATA_MAX_LEN * PACKET_MAX_CHAIN_LEN];
    
    //PIN_OUT(global_time, 32);
    
    packet_t packet;
    packet.data = temp_data;
    bool has_anything_to_send = false;
    
    mesh_srv_packet_assemble(&packet, PACKET_DATA_MAX_LEN * PACKET_MAX_CHAIN_LEN, 
        &has_anything_to_send);
    
    if (has_anything_to_send)
    {
        TICK_PIN(PIN_MESH_TX);
        radio_disable();
        
        ble_gap_addr_t my_adv_addr;
        sd_ble_gap_address_get(&my_adv_addr);
        
        uint8_t packet_and_addr_type = PACKET_TYPE_ADV_NONCONN |
            ((my_adv_addr.addr_type == BLE_GAP_ADDR_TYPE_PUBLIC)?
            0 :
            PACKET_ADDR_TYPE_MASK);
        
        uint8_t* temp_data_ptr = &temp_data[0];
        uint8_t* tx_data_ptr = &tx_data[0];
        tx_data_ptr[PACKET_TYPE_POS] = packet_and_addr_type;
        
        /* Code structured for packet chaining, although this is yet 
         to be implemented. */
        do
        {
            uint8_t min_len = ((packet.length > PACKET_DATA_MAX_LEN)? 
                PACKET_DATA_MAX_LEN : 
                packet.length);
            
            tx_data_ptr[PACKET_PADDING_POS] = 0;
            tx_data_ptr[PACKET_LENGTH_POS] = (min_len + PACKET_ADDR_LEN);
            tx_data_ptr[PACKET_TYPE_POS] = packet_and_addr_type;
            
            memcpy(&tx_data_ptr[PACKET_ADDR_POS], my_adv_addr.addr, PACKET_ADDR_LEN);
            memcpy(&tx_data_ptr[PACKET_DATA_POS], &temp_data_ptr[0], min_len);
            
            radio_event_t tx_event;
            tx_event.access_address = 0;
            rbc_mesh_channel_get(&tx_event.channel);
            tx_event.event_type = RADIO_EVENT_TYPE_TX;
            tx_event.packet_ptr = &tx_data_ptr[0];
            tx_event.start_time = 0;
            tx_event.callback.tx = NULL;
            
            /* testing CRC bug */
            tx_data_ptr[min_len + PACKET_DATA_POS + 0] = 0x33;
            tx_data_ptr[min_len + PACKET_DATA_POS + 1] = 0xcA;
            tx_data_ptr[min_len + PACKET_DATA_POS + 2] = 0xB2;
            
            radio_order(&tx_event);
            /*
            temp_data_ptr += min_len;
            tx_data_ptr += min_len + PACKET_DATA_POS;
            packet.length -= min_len;
            
            tx_data_ptr[PACKET_TYPE_POS] = packet_and_addr_type;
            */
        } while (0);
        
        order_search(); /* search for the rest of the timeslot */
    }
    else
    {
        TICK_PIN(6);
    }
    
}

void ll_control_timeslot_begin(uint32_t global_timer_value)
{
    uint32_t aa;    
    rbc_mesh_access_address_get(&aa);
    
    radio_init(aa);
    //order_search();  
    
    global_time = global_timer_value;
    
    ll_control_step();
    order_search();
}

void ll_control_step(void)
{
    step_time = global_time + timer_get_timestamp();
    
    async_event_t async_evt;
    async_evt.callback.generic = trickle_step_callback;
    async_evt.type = EVENT_TYPE_GENERIC;
    timeslot_queue_async_event(&async_evt);
}

