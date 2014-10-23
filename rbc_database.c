#include "rbc_database.h"
#include "timeslot_handler.h"
#include "trickle_common.h"
#include <stdbool.h>
#include <string.h>
#include "nrf_error.h"

/* TODO: change to global version */
#define VALUE_COUNT (8)

#define VARIATION_FLAG_LENGTH_MASK    (0x1F)
#define VARIATION_PACKET_FLAG_MASK    (0xC0)


/*****************************************************************************
* Static globals
*****************************************************************************/

static db_value_t g_value_pool[VALUE_COUNT];


/*****************************************************************************
* Static functions
*****************************************************************************/

static variation_t* variation_get_newest(variation_t* variation1, variation_t* variation2)
{
    if (variation1->version > variation2->version)
        return variation1;
    else
        return variation2;
}


/**
* Returns the length of a serialized version of the value
*/
static uint8_t value_get_length(db_value_t* value)
{
    /* length and version fields */
    uint8_t tot_len = 2;
    
    /* data */
    tot_len += value->variation.length;
    
    if (value->flags & (1 << DB_VALUE_FLAG_IS_BROADCAST_POS))
    {
        /* id */
        tot_len += 2;
    }
    else
    {
        /* target and id */
        tot_len += 3;
    }
    
    if (value->flags & (1 << DB_VALUE_FLAG_HAS_SOURCE_POS))
    {
        /* source address */
        tot_len += 2;
    }
    
    return tot_len;
}   


/**
* Serialize value into buffer. Does not check for buffer overflow, 
* must be done by callee
*/
static void value_place_in_buffer(db_value_t* value, uint8_t* buffer)
{
    uint8_t i = 0;
    buffer[i++] = (value->variation.length & VARIATION_FLAG_LENGTH_MASK) | 
                    (value->flags & VARIATION_FLAG_LENGTH_MASK);
    buffer[i++] = value->variation.version;
    
    if (value->flags & (1 << DB_VALUE_FLAG_IS_BROADCAST_POS))
    {
        buffer[i++] = (value->identifier.broadcast.id >> 8);
        buffer[i++] = (value->identifier.broadcast.id & 0xFF);
    }
    else
    {
        buffer[i++] = (value->identifier.unicast.target >> 8);
        buffer[i++] = (value->identifier.unicast.target & 0xFF);
        buffer[i++] = value->identifier.unicast.id;
    }
    
    if (value->flags & (1 << DB_VALUE_FLAG_HAS_SOURCE_POS))
    {
        buffer[i++] = (value->variation.source >> 8);
        buffer[i++] = (value->variation.source & 0xFF);
    }
    
    memcpy( &buffer[i], 
            value->variation.data, 
            value->variation.length & VARIATION_FLAG_LENGTH_MASK);
}
    
        

/*****************************************************************************
* Interface functions
*****************************************************************************/

void db_init(void)
{
    trickle_setup(100, 20, 3);
    for (uint16_t i = 0; i < VALUE_COUNT; ++i)
    {
        g_value_pool[i].flags = 0;
        g_value_pool[i].variation.length = 0;
        g_value_pool[i].variation.version = 0;
        g_value_pool[i].variation.source = 0;
        g_value_pool[i].variation.last_sender = 0;
    }
        
}

void db_packet_dissect(packet_t* packet)
{
    uint8_t index = 0;
    while (index < packet->length)
    {
        uint8_t flags = packet->data[index] & VARIATION_PACKET_FLAG_MASK;
        
        variation_t variation;
        db_value_t* value = NULL;
        uint16_t id = 0;
        uint16_t target = 0;
        
        
        variation.last_sender = packet->sender;
        
        variation.length = packet->data[index++] & VARIATION_FLAG_LENGTH_MASK;
        variation.version = packet->data[index++];
        
        if (variation.length > DB_VARIATION_MAX_LENGTH)
        {
            /* assume broken packet, abort. */
            /* This should never, ever happen */
            
            //APP_ERROR_CHECK(NRF_ERROR_INVALID_LENGTH);
        }
        
        /* identifier bytes are structured differently if the variation has 
        * its source embedded in the header, and if it is unicast */
        if (flags & DB_VALUE_FLAG_IS_BROADCAST_POS)
        {
            
            id = (packet->data[index++] << 8) | 
                (packet->data[index++]);
        }
        else /* unicast */
        {
            target = (packet->data[index++] << 8) | 
                (packet->data[index++]);
            id = packet->data[index++];
        }
        
        if (flags & DB_VALUE_FLAG_HAS_SOURCE_POS)
        {
            variation.source = (packet->data[index++] << 8) | 
                (packet->data[index++]);
        }
        
        
        memcpy(&variation.data[0], &packet->data[index], variation.length);
        
        index += variation.length;
        
        value = db_value_get(id, target);
        TICK_PIN(PIN_ABORTED);
        PIN_OUT(id, 16);   
        if (value == NULL)
        {
            TICK_PIN(PIN_ABORTED);
            value = db_value_alloc();
            value->flags = (flags & VARIATION_PACKET_FLAG_MASK);
            if (flags & DB_VALUE_FLAG_IS_BROADCAST_POS)
            {
                value->identifier.broadcast.id = id;
            }
            else
            {
                value->identifier.unicast.target = 
                    (packet->data[index++] << 8) | 
                    (packet->data[index++]);
            }
        }
        
        db_value_update_variation(value, &variation);
    }
}


db_value_t* db_value_get(uint16_t id, uint16_t target)
{
    for (uint16_t i = 0; i < VALUE_COUNT; ++i)
    {
        db_value_t* value = &g_value_pool[i];
        
        if ((value->flags & (1 << DB_VALUE_FLAG_ACTIVE_POS)) == 0)
            continue;
        
        if (value->flags & (1 << DB_VALUE_FLAG_IS_BROADCAST_POS))
        {
            if (value->identifier.broadcast.id == id)
                return value;
        }
        else /* UNICAST */
        {
            if (value->identifier.unicast.id == (id & 0x00FF) && 
                value->identifier.unicast.target == target)
                return value;
        }
    }
    
    return NULL;        
}

db_value_t* db_value_alloc(void)
{
    db_value_t* oldest = NULL;
    uint32_t oldest_i = 0;
    
    for (uint16_t i = 0; i < VALUE_COUNT; ++i)
    {
        /* early escape if a non-allocated object is found */
        if ((g_value_pool[i].flags & DB_VALUE_FLAG_ACTIVE_POS) == 0)
        {
            oldest = &g_value_pool[i];
            break;
        }
        
        if (g_value_pool[i].flags & (1 << DB_VALUE_FLAG_VOLATILE_POS))
            continue;
        
        
        if (g_value_pool[i].trickle.i > oldest_i)
        {
            oldest_i = g_value_pool[i].trickle.i;
            oldest = &g_value_pool[i];
        }
    }
    
    if (oldest == NULL)
    {
        /* No non-volatile objects in pool */
        return NULL;
    }
    
    /** @TODO: Send message to user space notifying of object erase */
    
    
    trickle_init(&oldest->trickle);
    
    oldest->flags |= (1 << DB_VALUE_FLAG_ACTIVE_POS);
    return oldest;
}

void db_value_delete(db_value_t* value)
{
    value->flags = 0;
}


void db_value_update_variation(db_value_t* value, variation_t* variation)
{
    bool is_newer = (value->variation.version < variation->version);
    
    TICK_PIN(PIN_ABORTED);
    if (value->variation.version == variation->version)
    {
        trickle_rx_consistent(&value->trickle);
    }
    else
    {
        trickle_rx_inconsistent(&value->trickle);
    } 

    /* send system values to lower layer */
    if (value->flags & (1 << DB_VALUE_FLAG_SYSTEM_POS))
    {
        TICK_PIN(PIN_ABORTED);
        //sync_value_rx(variation);
    }    
    
    /* update value construct */
    if (is_newer || (value->flags & (1 << DB_VALUE_FLAG_ACTIVE_POS)) == 0)
    {
        value->variation.length = variation->length;
        value->variation.version = variation->version;
        value->variation.source = variation->source;
        value->variation.last_sender = variation->last_sender;
        memcpy(&(value->variation.data[0]), &(variation->data[0]), variation->length);
    }
    
    
    if (!(value->flags & (1 << DB_VALUE_FLAG_SYSTEM_POS)))
    {
        /*TODO: send message to user space */
    }
}


void db_packet_assemble(packet_t* packet, uint8_t max_len, bool* has_anything_to_send)
{
    uint8_t packet_index = 0;
    *has_anything_to_send = false;
    
    for (uint16_t i = 0; i < VALUE_COUNT; ++i)
    {
        if ((g_value_pool[i].flags & (1 << DB_VALUE_FLAG_ACTIVE_POS)) == 0)
        {
            continue;
        }
        bool do_tx;
        trickle_step(&g_value_pool[i].trickle, &do_tx);
        
        if (do_tx)
        {
            uint8_t value_len = value_get_length(&g_value_pool[i]);
            
            /* Greedy approach, classic knapsack problem, a fitting-algorithm
            * should be considered. Also, objects are not properly prioritized. */
            if (value_len + packet_index < max_len)
            {
                if (g_value_pool[i].flags & (1 << DB_VALUE_FLAG_SYSTEM_POS))
                {
                    g_value_pool[i].variation.data[0] = g_value_pool[i].trickle.t - trickle_timestamp_get();
                }
                
                value_place_in_buffer(&g_value_pool[i], &packet->data[packet_index]);
                packet_index += value_len;
                *has_anything_to_send = true;
                trickle_register_tx(&g_value_pool[i].trickle);
                
                DEBUG_PIN_DB(3);
            }
        }
    }
     
    packet->length = packet_index - 1;
}

uint32_t db_get_next_processing_time(void)
{
    uint32_t next_time = UINT32_MAX;
    for (uint8_t i = 0; i < VALUE_COUNT; ++i)
    {
        if ((g_value_pool[i].flags & (1 << DB_VALUE_FLAG_ACTIVE_POS)) == 0)
        {
            continue;
        }
        uint32_t this_timeout = trickle_next_processing_get(&g_value_pool[i].trickle);
        
        if (this_timeout < next_time)
            next_time = this_timeout;
    }
    
    return next_time;
}

