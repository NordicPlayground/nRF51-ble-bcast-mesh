#ifndef _DRIP_CONTROL_H__
#define _DRIP_CONTROL_H__
#include "timeslot_handler.h"
#include <stdint.h>

#define DRIP_FLAG_HAS_SOURCE_POS    (6)
#define DRIP_FLAG_IS_BROADCAST_POS  (7)

typedef struct
{
    uint8_t length;
    uint8_t version;
    uint8_t* data;
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
    uint8_t flags; /* [0 - 5]: RFU | [6]: HAS SOURCE INCLUDED | [7]: BROADCAST[1]/UNICAST[0] */
    droplet_t droplet; /* most recent version */
    uint8_t trickle_id;   
} drip_t;


void drip_packet_dissect(packet_t* packet);

void drip_droplet_dissect(uint8_t* raw_data, uint8_t length);

void drip_packet_assemble(packet_t* packet, uint8_t max_len);

void drip_init(void);




#endif /* _DRIP_CONTROL_H__ */
