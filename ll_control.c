#include "ll_control.h"
#include "radio_control.h"
#include "timer_control.h"
#include "timeslot_handler.h"
#include "drip_control.h"
#include "trickle_common.h"
#include <string.h>

#define PACKET_DATA_POS         (3)
#define PACKET_LENGTH_POS       (2)
#define PACKET_TYPE_POS         (0)
#define PACKET_TYPE_DATA_START  (0x02)
#define PACKET_TYPE_DATA_CONT   (0x01)
#define PACKET_TYPE_LL_CONTROL  (0x03)

#define PACKET_DATA_MAX_LEN     (27)
#define PACKET_MAX_CHAIN_LEN    (4)

#define TRICKLE_TIME_PERIOD     (20000) /* 20ms */
#define TRICKLE_TIME_OFFSET     (2000) /* 2ms */


static uint8_t tx_data[(PACKET_DATA_MAX_LEN + PACKET_DATA_POS) * PACKET_MAX_CHAIN_LEN];
static uint8_t timer_index = 0xFF;
static uint32_t timeout_time = UINT32_MAX;
static uint32_t global_time = 0;

static void search_callback(uint8_t* data);
static void trickle_step_callback(void);

static void order_search(void)
{
    radio_event_t search_event;
    search_event.access_address = 1;
    search_event.callback.rx = search_callback;
    search_event.channel = 37;
    search_event.event_type = RADIO_EVENT_TYPE_RX;
    search_event.start_time = 0;
    
    radio_order(&search_event);   
}

static void setup_next_step_callback(void)
{
    uint32_t current_time = timer_get_timestamp();
    uint32_t end_of_timeslot = timeslot_get_remaining_time() + current_time;
    
    timeout_time = 1000 * drip_get_next_processing_time();
    
    if (timeout_time - global_time + 1500 > end_of_timeslot)
    {
        timeslot_extend(timeout_time - global_time + 1500);
    }
    
    timer_index = timer_order_cb(timeout_time - global_time, trickle_step_callback);
}

static inline void packet_create_from_data(uint8_t* data, packet_t* packet)
{
    packet->data = &data[PACKET_DATA_POS];
    packet->length = data[PACKET_LENGTH_POS];
    packet->sender = 0;
}


static inline bool packet_is_data_packet(uint8_t* data)
{
    return ((data[PACKET_TYPE_POS] & 0x03) == PACKET_TYPE_DATA_CONT || 
        (data[PACKET_TYPE_POS] & 0x03) == PACKET_TYPE_DATA_START);
}

static void search_callback(uint8_t* data)
{
    TICK_PIN(PIN_RX);
    order_search();
    
    if (!packet_is_data_packet(data))
        return;
    
    packet_t packet;
    packet_create_from_data(data, &packet);
    drip_packet_dissect(&packet);
    
    TICK_PIN(PIN_RX);
    
    uint32_t new_processing_timeout = 1000 * drip_get_next_processing_time();
    if (new_processing_timeout < timeout_time)
    {
        timeout_time = new_processing_timeout;
        timer_abort(timer_index);
        timer_index = timer_order_cb(1000 * timeout_time, trickle_step_callback);
    }
}


static void trickle_step_callback(void)
{
    trickle_time_update(timeout_time / 1000 + 1);
    
    uint8_t temp_data[PACKET_DATA_MAX_LEN * PACKET_MAX_CHAIN_LEN];
    
    packet_t packet;
    packet.data = temp_data;
    bool has_anything_to_send = false;
    
    drip_packet_assemble(&packet, PACKET_DATA_MAX_LEN * PACKET_MAX_CHAIN_LEN, &has_anything_to_send);
    
    if (has_anything_to_send)
    {
        radio_disable();
        
        uint8_t* temp_data_ptr = &temp_data[0];
        uint8_t* tx_data_ptr = &tx_data[0];
        tx_data_ptr[PACKET_TYPE_POS] = PACKET_TYPE_DATA_START;
        
        do
        {
            TICK_PIN(0);
            uint8_t min_len = ((packet.length > PACKET_DATA_MAX_LEN)? 
                PACKET_DATA_MAX_LEN : 
                packet.length);
            
            tx_data_ptr[PACKET_LENGTH_POS] = min_len;
            tx_data_ptr[PACKET_TYPE_POS] = PACKET_TYPE_DATA_START;
            
            memcpy(&tx_data_ptr[PACKET_DATA_POS], temp_data_ptr, min_len);
            
            radio_event_t tx_event;
            tx_event.access_address = 0;
            tx_event.channel = 37;
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
            
            tx_data_ptr[PACKET_TYPE_POS] = PACKET_TYPE_DATA_CONT;
            
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
    radio_init();
    order_search();  
    
    timeout_time = global_timer_value;
    global_time = global_timer_value;
    
    setup_next_step_callback();
}
