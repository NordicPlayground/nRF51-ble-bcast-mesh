#ifndef _RBC_DATABASE_H__
#define _RBC_DATABASE_H__

#include "trickle.h"
#include <stdint.h>

#define DB_VALUE_FLAG_SYSTEM_POS        (3)
#define DB_VALUE_FLAG_VOLATILE_POS      (4)
#define DB_VALUE_FLAG_ACTIVE_POS        (5)
#define DB_VALUE_FLAG_HAS_SOURCE_POS    (6)
#define DB_VALUE_FLAG_IS_BROADCAST_POS  (7)

#define DB_VARIATION_MAX_LENGTH          (27)
#define DB_MAX_PACKET_LENGTH            DB_VARIATION_MAX_LENGTH)



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
    uint8_t data[DB_VARIATION_MAX_LENGTH];
    uint16_t source;
    uint16_t last_sender; /* last node to send new info */
} variation_t;


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
    variation_t variation; /* most recent version */
    trickle_t trickle;   
} db_value_t;

void db_init(void);

void db_packet_dissect(packet_t* packet);

db_value_t* db_value_get(uint16_t id, uint16_t target);

db_value_t* db_value_alloc(void);

void db_value_update_variation(db_value_t* val, variation_t* variation);

void db_value_delete(db_value_t* val);

void db_packet_assemble(packet_t* packet, uint8_t max_len, bool* has_anything_to_send);

uint32_t db_get_next_processing_time(void);

#endif /* _RBC_DATABASE_H__ */
