#ifndef _RBC_DATABASE_H__
#define _RBC_DATABASE_H__

#include "trickle.h"
#include "ble_gap.h"
#include <stdint.h>

#define MAX_VALUE_COUNT                 (155)
#define MAX_VALUE_LENGTH                (27)


#define MESH_SRV_UUID                   (0x0001)
#define MESH_MD_CHAR_UUID               (0x0002)
#define MESH_VALUE_CHAR_UUID            (0x0003)

#define MESH_MD_CHAR_LEN                (10)
#define MESH_MD_CHAR_AA_OFFSET          (0)
#define MESH_MD_CHAR_ADV_INT_OFFSET     (4)
#define MESH_MD_CHAR_COUNT_OFFSET       (8)
#define MESH_MD_CHAR_CH_OFFSET          (9)


#define MESH_MD_FLAGS_USED_POS          (0)
#define MESH_MD_FLAGS_INITIALIZED_POS   (1)


typedef struct
{
    ble_gap_addr_t sender;
    uint8_t length;
    uint8_t* data;
} packet_t;


typedef struct
{
    uint16_t version_number;
    uint16_t char_value_handle;
    ble_gap_addr_t last_sender_addr;
    trickle_t trickle;
    uint8_t flags;
} mesh_char_metadata_t;


typedef struct 
{
    uint32_t mesh_access_addr;
    uint32_t mesh_adv_int_ms;
    uint8_t mesh_value_count;
    uint8_t mesh_channel;
} mesh_metadata_char_t;

uint32_t mesh_srv_init(uint8_t mesh_value_count, 
    uint32_t access_address, uint8_t channel, uint32_t adv_int_ms);

uint32_t mesh_srv_char_val_set(uint8_t index, uint8_t* data, uint16_t len);

uint32_t mesh_srv_char_val_get(uint8_t index, uint8_t* data, uint16_t* len);

uint32_t mesh_srv_char_md_get(mesh_metadata_char_t* metadata);

uint32_t mesh_srv_get_next_processing_time(uint32_t* time);

uint32_t mesh_srv_packet_process(packet_t* packet);

/**
* @brief fills databuffer in the provided packet object with any mesh service 
*    that is to be sent. Indicates sending with the anything_to_send flag
*
* @param[in,out] packet Packet to be filled. data member must point to 
*    existing databuffer (i.e. not NULL)
* @param[in] packet_max_len Maximum permitted length of message 
*    (length of packet->data)
* @param[out] has_anything_to_send Set to true if any operation was performed 
*    on the data buffer
*/
uint32_t mesh_srv_packet_assemble(packet_t* packet, 
    uint16_t packet_max_len, 
    bool* has_anything_to_send);


#endif /* _RBC_DATABASE_H__ */

