#include "drip_control.h"
#include "timeslot_handler.h"
#include "trickle_common.h"
#include <stdbool.h>
#include <string.h>
#include "nrf_error.h"

/* TODO: change to global version */
#define DRIP_COUNT (8)

#define DROPLET_FLAG_LENGTH_MASK    (0x1F)
#define DROPLET_PACKET_FLAG_MASK    (0xC0)


/*****************************************************************************
* Static globals
*****************************************************************************/

static drip_t g_drip_pool[DRIP_COUNT];


/*****************************************************************************
* Static functions
*****************************************************************************/

static droplet_t* droplet_get_newest(droplet_t* droplet1, droplet_t* droplet2)
{
    if (droplet1->version > droplet2->version)
        return droplet1;
    else
        return droplet2;
}

static void drip_update_droplet(drip_t* drip, droplet_t* droplet)
{
    bool is_newer = (drip->droplet.version < droplet->version);
    
    TICK_PIN(PIN_ABORTED);
    if (drip->droplet.version == droplet->version)
    {
        trickle_rx_consistent(&drip->trickle);
    }
    else
    {
        trickle_rx_inconsistent(&drip->trickle);
    } 

    /* send system drips to lower layer */
    if (drip->flags & (1 << DRIP_FLAG_SYSTEM_POS))
    {
        TICK_PIN(PIN_ABORTED);
        //sync_drip_rx(droplet);
    }    
    
    /* update drip construct */
    if (is_newer || (drip->flags & (1 << DRIP_FLAG_ACTIVE_POS)) == 0)
    {
        drip->droplet.length = droplet->length;
        drip->droplet.version = droplet->version;
        drip->droplet.source = droplet->source;
        drip->droplet.last_sender = droplet->last_sender;
        memcpy(&(drip->droplet.data[0]), &(droplet->data[0]), droplet->length);
    }
    
    
    if (!(drip->flags & (1 << DRIP_FLAG_SYSTEM_POS)))
    {
        /*TODO: send message to user space */
    }
}

/**
* Returns the length of a serialized version of the drip
*/
static uint8_t drip_get_length(drip_t* drip)
{
    /* length and version fields */
    uint8_t tot_len = 2;
    
    /* data */
    tot_len += drip->droplet.length;
    
    if (drip->flags & (1 << DRIP_FLAG_IS_BROADCAST_POS))
    {
        /* id */
        tot_len += 2;
    }
    else
    {
        /* target and id */
        tot_len += 3;
    }
    
    if (drip->flags & (1 << DRIP_FLAG_HAS_SOURCE_POS))
    {
        /* source address */
        tot_len += 2;
    }
    
    return tot_len;
}   


/**
* Serialize drip into buffer. Does not check for buffer overflow, 
* must be done by callee
*/
static void drip_place_in_buffer(drip_t* drip, uint8_t* buffer)
{
    uint8_t i = 0;
    buffer[i++] = (drip->droplet.length & DROPLET_FLAG_LENGTH_MASK) | 
                    (drip->flags & DROPLET_FLAG_LENGTH_MASK);
    buffer[i++] = drip->droplet.version;
    
    if (drip->flags & (1 << DRIP_FLAG_IS_BROADCAST_POS))
    {
        buffer[i++] = (drip->identifier.broadcast.id >> 8);
        buffer[i++] = (drip->identifier.broadcast.id & 0xFF);
    }
    else
    {
        buffer[i++] = (drip->identifier.unicast.target >> 8);
        buffer[i++] = (drip->identifier.unicast.target & 0xFF);
        buffer[i++] = drip->identifier.unicast.id;
    }
    
    if (drip->flags & (1 << DRIP_FLAG_HAS_SOURCE_POS))
    {
        buffer[i++] = (drip->droplet.source >> 8);
        buffer[i++] = (drip->droplet.source & 0xFF);
    }
    
    memcpy( &buffer[i], 
            drip->droplet.data, 
            drip->droplet.length & DROPLET_FLAG_LENGTH_MASK);
}
    
        

/*****************************************************************************
* Interface functions
*****************************************************************************/

void drip_init(void)
{
    trickle_setup(100, 20, 3);
    for (uint16_t i = 0; i < DRIP_COUNT; ++i)
    {
        g_drip_pool[i].flags = 0;
        g_drip_pool[i].droplet.length = 0;
        g_drip_pool[i].droplet.version = 0;
        g_drip_pool[i].droplet.source = 0;
        g_drip_pool[i].droplet.last_sender = 0;
    }
        
}

void drip_packet_dissect(packet_t* packet)
{
    uint8_t index = 0;
    while (index < packet->length)
    {
        uint8_t flags = packet->data[index] & DROPLET_PACKET_FLAG_MASK;
        
        droplet_t droplet;
        drip_t* drip = NULL;
        uint16_t id = 0;
        uint16_t target = 0;
        
        
        droplet.last_sender = packet->sender;
        
        droplet.length = packet->data[index++] & DROPLET_FLAG_LENGTH_MASK;
        droplet.version = packet->data[index++];
        
        if (droplet.length > DROPLET_MAX_LENGTH)
        {
            /* assume broken packet, abort. */
            /* This should never, ever happen */
            
            //APP_ERROR_CHECK(NRF_ERROR_INVALID_LENGTH);
        }
        
        /* identifier bytes are structured differently if the droplet has 
        * its source embedded in the header, and if it is unicast */
        if (flags & DRIP_FLAG_IS_BROADCAST_POS)
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
        
        if (flags & DRIP_FLAG_HAS_SOURCE_POS)
        {
            droplet.source = (packet->data[index++] << 8) | 
                (packet->data[index++]);
        }
        
        
        memcpy(&droplet.data[0], &packet->data[index], droplet.length);
        
        index += droplet.length;
        
        drip = drip_get(id, target);
        TICK_PIN(PIN_ABORTED);
        PIN_OUT(id, 16);   
        if (drip == NULL)
        {
            TICK_PIN(PIN_ABORTED);
            drip = drip_allocate_new();
            drip->flags = (flags & DROPLET_PACKET_FLAG_MASK);
            if (flags & DRIP_FLAG_IS_BROADCAST_POS)
            {
                drip->identifier.broadcast.id = id;
            }
            else
            {
                drip->identifier.unicast.target = 
                    (packet->data[index++] << 8) | 
                    (packet->data[index++]);
            }
        }
        
        drip_update_droplet(drip, &droplet);
    }
}


drip_t* drip_get(uint16_t id, uint16_t target)
{
    for (uint16_t i = 0; i < DRIP_COUNT; ++i)
    {
        drip_t* drip = &g_drip_pool[i];
        
        if ((drip->flags & (1 << DRIP_FLAG_ACTIVE_POS)) == 0)
            continue;
        
        if (drip->flags & (1 << DRIP_FLAG_IS_BROADCAST_POS))
        {
            if (drip->identifier.broadcast.id == id)
                return drip;
        }
        else /* UNICAST */
        {
            if (drip->identifier.unicast.id == (id & 0x00FF) && 
                drip->identifier.unicast.target == target)
                return drip;
        }
    }
    
    return NULL;        
}

drip_t* drip_allocate_new(void)
{
    drip_t* oldest = NULL;
    uint32_t oldest_i = 0;
    
    for (uint16_t i = 0; i < DRIP_COUNT; ++i)
    {
        /* early escape if a non-allocated object is found */
        if ((g_drip_pool[i].flags & DRIP_FLAG_ACTIVE_POS) == 0)
        {
            oldest = &g_drip_pool[i];
            break;
        }
        
        if (g_drip_pool[i].flags & (1 << DRIP_FLAG_VOLATILE_POS))
            continue;
        
        
        if (g_drip_pool[i].trickle.i > oldest_i)
        {
            oldest_i = g_drip_pool[i].trickle.i;
            oldest = &g_drip_pool[i];
        }
    }
    
    if (oldest == NULL)
    {
        /* No non-volatile objects in pool, this violates usage 
        patterns, and should not happen. */
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM); 
    }
    
    /*TODO: Send message to user space notifying of object erase */
    
    
    trickle_init(&oldest->trickle);
    
    oldest->flags |= (1 << DRIP_FLAG_ACTIVE_POS);
    return oldest;
}

void drip_delete(drip_t* drip)
{
    drip->flags = 0;
}

void drip_packet_assemble(packet_t* packet, uint8_t max_len, bool* has_anything_to_send)
{
    uint8_t packet_index = 0;
    *has_anything_to_send = false;
    
    for (uint16_t i = 0; i < DRIP_COUNT; ++i)
    {
        if ((g_drip_pool[i].flags & (1 << DRIP_FLAG_ACTIVE_POS)) == 0)
        {
            continue;
        }
        bool do_tx;
        trickle_step(&g_drip_pool[i].trickle, &do_tx);
        
        if (do_tx)
        {
            uint8_t drip_len = drip_get_length(&g_drip_pool[i]);
            
            /* Greedy approach, classic knapsack problem, a fitting-algorithm
            * should be considered. Also, objects are not properly prioritized. */
            if (drip_len + packet_index < max_len)
            {
                if (g_drip_pool[i].flags & (1 << DRIP_FLAG_SYSTEM_POS))
                {
                    g_drip_pool[i].droplet.data[0] = g_drip_pool[i].trickle.t - trickle_timestamp_get();
                }
                
                drip_place_in_buffer(&g_drip_pool[i], &packet->data[packet_index]);
                packet_index += drip_len;
                *has_anything_to_send = true;
                trickle_register_tx(&g_drip_pool[i].trickle);
                
                DEBUG_PIN_DRIP(3);
            }
        }
    }
     
    packet->length = packet_index - 1;
}

uint32_t drip_get_next_processing_time(void)
{
    uint32_t next_time = UINT32_MAX;
    for (uint8_t i = 0; i < DRIP_COUNT; ++i)
    {
        if ((g_drip_pool[i].flags & (1 << DRIP_FLAG_ACTIVE_POS)) == 0)
        {
            continue;
        }
        uint32_t this_timeout = trickle_next_processing_get(&g_drip_pool[i].trickle);
        
        if (this_timeout < next_time)
            next_time = this_timeout;
    }
    
    return next_time;
}

