#ifndef _DRIP_CONTROL_H__
#define _DRIP_CONTROL_H__

#include "trickle.h"
#include <stdint.h>

#define DRIP_FLAG_SYSTEM_POS        (3)
#define DRIP_FLAG_VOLATILE_POS      (4)
#define DRIP_FLAG_ACTIVE_POS        (5)
#define DRIP_FLAG_HAS_SOURCE_POS    (6)
#define DRIP_FLAG_IS_BROADCAST_POS  (7)

#define DROPLET_MAX_LENGTH          (24)
#define DROPLET_MAX_PACKET_LENGTH   (DROPLET_MAX_LENGTH + 4)



typedef struct
{
    uint16_t sender;
    uint8_t length;
    uint8_t* data;
} packet_t;



typedef struct
{
    uint8_t length;
    uint8_t version;
    uint8_t data[DROPLET_MAX_LENGTH];
    uint16_t source;
    uint16_t last_sender; /* last node to send new info */
} droplet_t;


typedef struct
{
    union
    {
        struct 
        {
            uint16_t id;
        } broadcast;
        struct 
        {
            uint16_t target;
            uint8_t id;
        } unicast;
    } identifier;
    uint8_t flags; /* [0 - 3]: RFU | [4]: IS VOLATILE | [5]: IS ALLOCATED | [6]: HAS SOURCE INCLUDED | [7]: BROADCAST[1]/UNICAST[0] */
    droplet_t droplet; /* most recent version */
    trickle_t trickle;   
} drip_t;

void drip_init(void);

void drip_packet_dissect(packet_t* packet);

drip_t* drip_get(uint16_t id, uint16_t target);

drip_t* drip_allocate_new(void);

void drip_delete(drip_t* drip);

void drip_droplet_c(uint8_t* raw_data, uint8_t length);

void drip_packet_assemble(packet_t* packet, uint8_t max_len, bool* has_anything_to_send);


#endif /* _DRIP_CONTROL_H__ */
