#include "ll_control.h"
#include "radio_control.h"
#include "timer_control.h"
#include "rbc_database.h"
#include "rebroadcast.h"
#include "timeslot_handler.h"
#include "trickle_common.h"
#include "ble_gap.h"
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
#define PACKET_LENGTH_POS           (2)
#define PACKET_ADDR_POS             (PACKET_LENGTH_POS + PACKET_LENGTH_LEN)
#define PACKET_DATA_POS             (PACKET_ADDR_POS + PACKET_ADDR_LEN)


#define PACKET_TYPE_ADV_NONCONN     (0x02)

#define PACKET_TYPE_MASK            (0x0F)
#define PACKET_LENGTH_MASK          (0x3F)
#define PACKET_ADDR_TYPE_MASK       (0x40)

#define PACKET_DATA_MAX_LEN         (31)
#define PACKET_MAX_CHAIN_LEN        (1) /**@TODO: May be increased when RX 
                                        callback packet chain handling is implemented.*/

#define TRICKLE_TIME_PERIOD         (20000) /* 20ms */
#define TRICKLE_TIME_OFFSET         (2000) /* 2ms */


static uint8_t tx_data[(PACKET_DATA_MAX_LEN + PACKET_DATA_POS) * PACKET_MAX_CHAIN_LEN];
static uint8_t timer_index = 0xFF;
static uint32_t timeout_time = UINT32_MAX;
static uint32_t global_time = 0;

static void search_callback(uint8_t* data);
static void trickle_step_callback(void);

static void order_search(void)
{
    radio_event_t search_event;
    search_event.access_address = 1; /* RX: treat as bitfield */
    search_event.callback.rx = search_callback;
    rbc_channel_get(&search_event.channel);
    search_event.event_type = RADIO_EVENT_TYPE_RX;
    search_event.start_time = 0;
    
    radio_order(&search_event);   
}

static void setup_next_step_callback(void)
{
    uint32_t current_time = timer_get_timestamp();
    uint32_t end_of_timeslot = timeslot_get_remaining_time() + current_time;
    
    /**@TODO: Don't wake the processor just to process a trickle period change */
    mesh_srv_get_next_processing_time(&timeout_time);
    
    
    if (timeout_time - global_time + 1500 > end_of_timeslot)
    {
        timeslot_extend(timeout_time - global_time + 1500);
    }
    
    timer_index = timer_order_cb(timeout_time - global_time, trickle_step_callback);
}

static inline void packet_create_from_data(uint8_t* data, packet_t* packet)
{
    /* advertisement package */
    packet->data = &data[PACKET_DATA_POS];
    packet->length = (data[PACKET_LENGTH_POS - PACKET_ADDR_LEN] & 
        PACKET_LENGTH_MASK);
    
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

static void search_callback(uint8_t* data)
{
    TICK_PIN(PIN_RX);
    order_search();
    
    if (!packet_is_data_packet(data))
        return;
    
    packet_t packet;
    packet_create_from_data(data, &packet);
    mesh_srv_packet_process(&packet);
    
    /** @TODO: add packet chain handling */
    
    TICK_PIN(PIN_RX);
    
    uint32_t new_processing_timeout;
    mesh_srv_get_next_processing_time(&new_processing_timeout);
    
    if (new_processing_timeout < timeout_time)
    {
        timeout_time = new_processing_timeout;
        timer_abort(timer_index);
        timer_index = timer_order_cb(timeout_time, trickle_step_callback);
    }
}


static void trickle_step_callback(void)
{
    trickle_time_update(timeout_time + 1);
    
    uint8_t temp_data[PACKET_DATA_MAX_LEN * PACKET_MAX_CHAIN_LEN];
    
    packet_t packet;
    packet.data = temp_data;
    bool has_anything_to_send = false;
    
    mesh_srv_packet_assemble(&packet, PACKET_DATA_MAX_LEN * PACKET_MAX_CHAIN_LEN, 
        &has_anything_to_send);
    
    if (has_anything_to_send)
    {
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
            TICK_PIN(0);
            uint8_t min_len = ((packet.length > PACKET_DATA_MAX_LEN)? 
                PACKET_DATA_MAX_LEN : 
                packet.length);
            
            tx_data_ptr[PACKET_LENGTH_POS] = min_len;
            tx_data_ptr[PACKET_TYPE_POS] = packet_and_addr_type;
            
            
            
            memcpy(&tx_data_ptr[PACKET_DATA_POS], temp_data_ptr, min_len);
            
            radio_event_t tx_event;
            tx_event.access_address = 0;
            rbc_channel_get(&tx_event.channel);
            tx_event.event_type = RADIO_EVENT_TYPE_TX;
            tx_event.packet_ptr = tx_data_ptr;
            tx_event.start_time = 0;
            tx_event.callback.tx = NULL;
            
            if (packet.length > PACKET_DATA_MAX_LEN) 
            {
                tx_data_ptr[PACKET_TYPE_POS] |= (1 << 4); /* MD = 1 */
            }
            
            
            radio_order(&tx_event);
            
            temp_data_ptr += min_len;
            tx_data_ptr += min_len + PACKET_DATA_POS;
            packet.length -= min_len;
            
            tx_data_ptr[PACKET_TYPE_POS] = packet_and_addr_type;
            
        } while (packet.length > 0);
        order_search();
    }
    else
    {
        TICK_PIN(6);
    }
    
    setup_next_step_callback();
}

void ll_control_timeslot_begin(uint32_t global_timer_value)
{
    uint32_t aa;    
    rbc_access_address_get(&aa);
    
    radio_init(aa);
    order_search();  
    
    timeout_time = global_timer_value;
    global_time = global_timer_value;
    
    setup_next_step_callback();
}
