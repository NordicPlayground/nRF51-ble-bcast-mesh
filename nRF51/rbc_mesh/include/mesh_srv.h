/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of other
  contributors to this software may be used to endorse or promote products
  derived from this software without specific prior written permission.

  4. This software must only be used in a processor manufactured by Nordic
  Semiconductor ASA, or in a processor manufactured by a third party that
  is used in combination with a processor manufactured by Nordic Semiconductor.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/

#ifndef _MESH_SRV_H__
#define _MESH_SRV_H__

#include "trickle.h"
#include "ble.h"
#include <stdint.h>
#include <stdbool.h>

/**
* @file Module handling all data storage related functionality.
*   Stores the mesh values in the Softdevice GATT server, within a GATT 
*   service. Processes and assembles packet payloads for the mesh.
*/

#define MAX_VALUE_COUNT                 (155) /* The highest possible number of values stored in the mesh. */
#define MAX_VALUE_LENGTH                (28) /* The maximum number of bytes available in one mesh value. */

#define MESH_SRV_UUID                   (0x0001) /* Mesh service UUID */
#define MESH_MD_CHAR_UUID               (0x0002) /* Mesh metadata characteristic UUID */
#define MESH_VALUE_CHAR_UUID            (0x0003) /* Mesh value characteristic UUID */

#define MESH_MD_CHAR_LEN                (10) /* Total length of Mesh metadata characteristic data */
#define MESH_MD_CHAR_AA_OFFSET          (0) /* Metadata characteristic Access Address offset */
#define MESH_MD_CHAR_ADV_INT_OFFSET     (4) /* Metadata characteristic Advertisement interval offset */
#define MESH_MD_CHAR_COUNT_OFFSET       (8) /* Metadata characteristic value count offset */
#define MESH_MD_CHAR_CH_OFFSET          (9) /* Metadata characteristic channel offset */

#define MESH_MD_FLAGS_USED_POS          (0) /* Metadata flag: Used */
#define MESH_MD_FLAGS_INITIALIZED_POS   (1) /* Metadata flag: Initialized */
#define MESH_MD_FLAGS_IS_ORIGIN_POS     (2) /* Metadata flag: Is origin */


/**
* @brief Radio packet type 
*/
typedef struct
{
    ble_gap_addr_t sender;
    uint8_t length;
    uint8_t data[MAX_VALUE_LENGTH + 3];
    uint32_t rx_crc;
} packet_t;

/**
* @brief Mesh value metadata type 
*/
typedef struct
{
    uint16_t version_number; /* value version */
    uint8_t char_value_handle; /* value handle */
    uint8_t flags; /* value flags */
    uint32_t crc; /* previous value CRC */
    ble_gap_addr_t last_sender_addr; /*previous value originator address */
    trickle_t trickle; /* trickle instance for mesh value */
} mesh_char_metadata_t;

/**
* @brief Global mesh metadata characteristic type
*/
typedef struct 
{
    uint32_t mesh_access_addr; /* operating access address */
    uint32_t mesh_adv_int_ms; /* operating minimum advertisement interval */
    uint8_t mesh_value_count; /* Number of mesh values */
    uint8_t mesh_channel; /* Mesh channel */
} mesh_metadata_char_t;

/**
* @brief initialize mesh service, and all related data. Must be called before 
*   any other mesh_srv function.
*
* @param[in] mesh_value_count The number of mesh values to be allocated in the
*   service. Is static for the lifetime of the node in the mesh.
* @param[in] access_address The access address the node should try to find 
*   other mesh nodes on. All nodes in the same mesh must use the same access 
*   address.
* @param[in] channel The channel the node should try to find other mesh nodes 
*   on. All nodes in the same mesh must use the same channel.
* @param[in] adv_int_ms The minimum advertisement interval the node should 
*   utilize.
* 
* @return NRF_SUCCESS The mesh service has been correctly set up.
* @return NRF_ERROR_INVALID_PARAM The provided mesh_value_count parameter 
*   exceeds the maximum number of mesh values.
* @return NRF_ERROR_INVALID_STATE The mesh service has already been 
*   initialized.
*/
uint32_t mesh_srv_init(uint8_t mesh_value_count, 
    uint32_t access_address, uint8_t channel, uint32_t adv_int_ms);

/**
* @brief Set contents of the indicated mesh value. 
* 
* @param[in] index The index of the value that is to be changed.
* @param[in] data The data to be put in the indicated value slot.
* @param[in] len The length of the data provided. May not exceed 
*   MAX_VALUE_LENGTH.
* @param[in] update_sender Whether to overwrite the originator field for the 
*   given handle with own address. If false, the previous version's originator
*   will still stand as originator.
*
* @return NRF_SUCCESS The value was updated successfully.
* @return NRF_ERROR_INVALID_STATE The mesh service has not been initialized.
* @return NRF_ERROR_INVALID_ADDR The given index is not in the range of handles
*   given in the initialization.
* @return NRF_ERROR_INVALID_LENGTH The len parameter exce3eds MAX_VALUE_LENGTH
*/
uint32_t mesh_srv_char_val_set(uint8_t index, uint8_t* data, uint16_t len, bool update_sender);

/**
* @brief Get contents located at indicated value index.
*
* @param[in] index The index of the value that is to be read
* @param[out] data, buffer to copy value contents to. Must be at least 
*   MAX_VALUE_LENGTH long to ensure safe operation.
* @param[out] len Returns the length of the copied data
* @param[out] origin_addr Returns the BLE GAP address at which the current 
*   version of this handle was first broadcasted.
*
* @return NRF_SUCCESS The value was successfully read
* @return NRF_ERROR_INVALID_STATE The mesh service has not been initialized.
* @return NRF_ERROR_INVALID_ADDR The given index is not in the range of handles
*   given in the initialization.
*/
uint32_t mesh_srv_char_val_get(uint8_t index, uint8_t* data, uint16_t* len, ble_gap_addr_t* origin_addr);

/**
* @brief Get contents of mesh metadata characteristic
* 
* @param[out] metadata pointer to metadata type structure to copy the 
*   metadata into.
* 
* @return NRF_SUCCESS the metadata was successfully read 
* @return NRF_ERROR_INVALID_STATE The mesh service has not been initialized.
*/
uint32_t mesh_srv_char_md_get(mesh_metadata_char_t* metadata);

/**
* @brief Get the timestamp of the next time the mesh service has something to 
*   process, based on internal trickle instances.
* 
* @param[out] time Returns the timestamp at which the next processing is 
*   scheduled.
*
* @return NRF_SUCCESS The next processing time has succesfully been read.
* @return NRF_ERROR_INVALID_STATE The mesh service has not been initialized.
* @return NRF_ERROR_NOT_FOUND The mesh service has nothing to process. May 
*   indicate that there are no active mesh values.
*/
uint32_t mesh_srv_get_next_processing_time(uint64_t* time);

/**
* @brief Process the payload of a received packet. 
* 
* @param[in] packet Pointer to packet to be processed 
* 
* @return NRF_SUCCESS The packet was processed successfully
* @return NRF_ERROR_INVALID_STATE The mesh service has not been initialized.
* @return NRF_ERROR_INVALID_LENGTH The length of the mesh value in the packet 
*   is longer than MAX_VALUE_LENGTH
* @return NRF_ERROR_INVALID_ADDR The index indicated in the packet is not in
*   the range of handles this node services.
*/
uint32_t mesh_srv_packet_process(packet_t* packet);

/**
* @brief Register an update to the current softdevice connection handle. The 
*   mesh service needs this in order to provide GATT indication on new value
*   updates.
* 
* @param[in] conn_handle The new connection handle.
* 
* @return NRF_SUCCESS the conn_handle was successfully updated.
*/
uint32_t mesh_srv_conn_handle_update(uint16_t conn_handle);

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
* 
* @return NRF_SUCCESS the packet was successfully assembled
* @return NRF_ERROR_INVALID_STATE The mesh service has not been initialized.
*/
uint32_t mesh_srv_packet_assemble(packet_t* packet, 
    uint16_t packet_max_len, 
    bool* has_anything_to_send);
    
/**
* @brief Register a Softdevice GATT Write event. Called from the softdevice 
*   event handler. Will filter out all non-related handles.
*
* @param[in] evt The Softdevice produced GATT server write event
* 
* @return NRF_SUCCESS The GATT server event was successfully processed
* @return NRF_ERROR_INVALID_STATE The mesh service has not been initialized.
* @return NRF_ERROR_FORBIDDEN The service written to is not the mesh service.
* @return NRF_ERROR_INVALID_ADDR The indicated characteristic was in the range
*   of the mesh service, but did not belong to any characteristic values 
*   (may be a write to CCCD)
*/
uint32_t mesh_srv_gatts_evt_write_handle(ble_gatts_evt_write_t* evt);

/**
* @brief Initialize the mesh value with the given handle. Initializes 
*   its trickle algorithm instance, starting broadcasting of the value.
* 
* @param[in] index The index of the value that we wish to initialize.
*
* @return NRF_SUCCESS The value was successfully initilalized.
* @return NRF_ERROR_INVALID_STATE The mesh service has not been initialized.
* @return NRF_ERROR_INVALID_ADDR The indicated index is outside the range 
*   specified in the init function call.
*/
uint32_t mesh_srv_char_val_enable(uint8_t index);

/**
* @brief Flag the value as disabled. It will no longer be rebroadcasted.
* 
* @param[in] index The index of the value to be disabled.
* 
* @return NRF_SUCCESS The value was successfully disabled.
* @return NRF_ERROR_INVALID_STATE The mesh service has not been initialized.
* @return NRF_ERROR_INVALID_ADDR The indicated index is outside the range 
*   specified in the init function call.
*/
uint32_t mesh_srv_char_val_disable(uint8_t index);


#endif /* _MESH_SRV_H__ */

