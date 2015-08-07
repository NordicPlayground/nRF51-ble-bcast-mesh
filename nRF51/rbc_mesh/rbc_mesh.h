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

#ifndef _RBC_MESH_H__
#define _RBC_MESH_H__
#include <stdint.h>
#include <stdbool.h>
#include "nrf51.h"
#include "ble.h"

#define RBC_MESH_ACCESS_ADDRESS_BLE_ADV  (0x8E89BED6)
#define RBC_MESH_ADV_INT_MIN             (5)
#define RBC_MESH_ADV_INT_MAX             (60000)
#define RBC_MESH_VALUE_MAX_LEN           (28)
/** 
* @brief Rebroadcast value handle type 
*
* @detailed Handle type used to identify a value in the mesh, is consistent
*   throughout the network
*/
typedef uint16_t rbc_mesh_value_handle_t;
    
/** @brief Event type enum. Identifies framework generated events */
typedef enum
{
    RBC_MESH_EVENT_TYPE_UPDATE_VAL,      /** Another node has updated the value */
    RBC_MESH_EVENT_TYPE_CONFLICTING_VAL, /** Another node has a conflicting version of the value */
    RBC_MESH_EVENT_TYPE_NEW_VAL,         /** A previously unallocated value has been received and allocated */
    RBC_MESH_EVENT_TYPE_INITIALIZED      /** The framework has been initialized internally (most likely via serial interface) */
} rbc_mesh_event_type_t;

/** 
* @brief Rebroadcast framework generated event. Carries information about a
*   change to one database value.
*/
typedef struct
{
    rbc_mesh_event_type_t event_type;        /** See @ref rbc_mesh_event_type_t */
    rbc_mesh_value_handle_t value_handle;    /** Handle of the value the event is generated for */
    uint8_t* data;                      /** Current data array contained at the event handle location */
    uint8_t data_len;                   /** Length of data array */
    ble_gap_addr_t originator_address; /** GAP address of node where this version of the message appeared */
} rbc_mesh_event_t;


/** 
* @brief Enum for radio mode used in initialization. See nRF51 Series 
*   documentation for details on the various modes. Note that 
*   the BLE_1MBIT mode is required to let the mesh packetsbe on-air compatible
*   with other devices (The regular gateway interface will be compatible 
*   regardless).
*/
typedef enum
{
    RBC_MESH_RADIO_MODE_1MBIT,
    RBC_MESH_RADIO_MODE_2MBIT,
    RBC_MESH_RADIO_MODE_250KBIT,
    RBC_MESH_RADIO_MODE_BLE_1MBIT
} rbc_mesh_radio_mode_t;

/**
* @brief Enum for on-air packet interface format. The original packet interface
*   provide the original max packet length of 28 bytes, but is not compatible
*   with external devices on-air, as the payload of the advertisements does not
*   adhere to the specification. To be able to inject packets into the mesh 
*   without using a gateway interface, the adv_compatible version must be used. 
*   Note that this reduces the max packet length to 26 bytes.
*/
typedef enum
{
    RBC_MESH_PACKET_FORMAT_ORIGINAL,
    RBC_MESH_PACKET_FORMAT_ADV_COMPATIBLE
} rbc_mesh_packet_format_t;

/**
* @brief Initialization parameter struct for the rbc_mesh_init() function.
*
*
* @note The nRF51 Softdevice must be initialized by the application before
*    the mesh framework intialization is called, otherwise, the function will
*    return NRF_ERROR_SOFTDEVICE_NOT_ENABLED.
* 
* @param[in] access_addr The access address the mesh will work on. This must be the 
*    same for all nodes in the mesh. RBC_MESH_ACCESS_ADDRESS_BLE_ADV gives the mesh
*    the same access address as regular BLE advertisements, which makes the
*    traffic visible to external BLE devices (Note that other access addresses 
*    does not provide any data security, the traffic is merely ignored by 
*    regular BLE radios). Multiple meshes may in theory work concurrently in 
*    the same area with different access addresses, but will be prone to 
*    on-air collisions, and it is recommended to use separate channels for this
* @param[in] channel The BLE channel the mesh works on. It is strongly recommended 
*    to use one of the three adv channels 37, 38 or 39, as others may be prone
*    to on-air collisions with WiFi channels. Separate meshes may work 
*    concurrently without packet collision if they are assigned to different 
*    channels. Must be between 1 and 39.
* @param[in] handle_count The maximum number of handle-value pairs available to the
*    application. May not be higher than 155 due to BLE namespace requirements
* @param[in] adv_int_ms The minimum adv_interval for nodes in the network in 
*    millis. Must be between 5 and 60000.
* @param radio_mode The radio mode the mesh shall operate on. Must be the same
*    across all nodes in the mesh. NOT YET IMPLEMENTED
* @param packet_format The format the packets should follow. NOT YET IMPLEMENTED
*/
typedef struct
{
    uint32_t access_addr;
    uint8_t channel;
    uint8_t handle_count;
    uint32_t adv_int_ms;
    rbc_mesh_radio_mode_t radio_mode;
    rbc_mesh_packet_format_t packet_format;
} rbc_mesh_init_params_t;

/*****************************************************************************
     Interface Functions 
*****************************************************************************/

/**
* @brief Initialize Rebroadcast module, must be called before any other 
*   rebroadcast function. 
*
* @note The nRF51 Softdevice must be initialized by the application before
*    the mesh framework intialization is called, otherwise, the function will
*    return NRF_ERROR_SOFTDEVICE_NOT_ENABLED.
* 
* @return NRF_SUCCESS the initialization is successful 
* @return NRF_ERROR_INVALID_PARAM a parameter does not meet its requiremented range.
* @return NRF_ERROR_INVALID_STATE the framework has already been initialized.
* @return NRF_ERROR_SOFTDEVICE_NOT_ENABLED the Softdevice has not been enabled.
*/
uint32_t rbc_mesh_init(rbc_mesh_init_params_t init_params);

/**
* @brief Set the contents of the data array pointed to by the provided handle
* 
* @note If the indicated handle-value pair is in a disabled state, it will 
*   automatically be enabled.
*
* @param[in] handle The handle of the value we want to update. Is mesh-global.
* @param[in] data Databuffer to be copied into the value slot
* @param[in] len Length of the provided data. Must not exceed RBC_VALUE_MAX_LEN.
* 
* @return NRF_SUCCESS if the value has been successfully updated.
* @return NRF_ERROR_INVALID_STATE if the framework has not been initialized.
* @return NRF_ERROR_INVALID_ADDR if the handle is outside the range provided
*    in @ref rbc_mesh_init.
* @return NRF_ERROR_INVALID_LENGTH if len exceeds RBC_VALUE_MAX_LEN.
*/
uint32_t rbc_mesh_value_set(uint8_t handle, uint8_t* data, uint16_t len);

/**
* @brief Start broadcasting the handle-value pair. If the handle has not been 
*   assigned a value yet, it will start broadcasting a version 0 value, so 
*   that adjacent nodes may push an updated version of the handle-value pair.
*
* @note The value broadcast is sent asynchronously to the function call, and 
*   a response is likely to be received after several milliseconds (depending
*   on the adv_int_ms value set in @ref rbc_mesh_init
*
* @param[in] handle Handle to request a value for 
*
* @return NRF_SUCCESS A request was successfully scheduled for broadcast
* @return NRF_ERROR_INVALID_ADDR the handle is outside the range provided 
*   in @ref rbc_mesh_init.
* @return NRF_ERROR_INVALID_STATE The framework has not been initiated
*/
uint32_t rbc_mesh_value_enable(uint8_t handle);

/**
* @brief Stop rebroadcasting the indicated handle-value pair. 
*
* @note This will not stop the framework from updating the value in its local 
*   database upon external updates (and consequently give the application 
*   update events), but values will not be propagated to other nodes through 
*   this node before they are either written to or re-enabled.
* 
* @param[in] handle Handle to stop broadcasting
* 
* @return NRF_SUCCESS The handle was successfully taken off the broadcast list
* @return NRF_ERROR_INVALID_ADDR the handle is outside the range provided
*   @ref rbc_mesh_init.
* @return NRF_ERROR_INVALID_STATE The framework has not been initialized.
*/
uint32_t rbc_mesh_value_disable(uint8_t handle);

/**
 * @brief Get the contents of the data array pointed to by the provided handle
*
* @param[in] handle The handle of the value we want to update. Is mesh-global.
* @param[out] data Databuffer to be copied into the value slot. Must be at least
*    RBC_VALUE_MAX_LEN long
* @param[out] len Length of the copied data. Will not exceed RBC_VALUE_MAX_LEN.
* @param[out] origin_addr BLE GAP address of the node that first broadcasted 
*   the current version of this value. Set to NULL if the address is not of 
*   interest.
* 
* @return NRF_SUCCESS the value has been successfully fetched.
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized.
* @return NRF_ERROR_INVALID_ADDR the handle is outside the range provided
*    in @ref rbc_mesh_init.
*/
uint32_t rbc_mesh_value_get(uint8_t handle, 
    uint8_t* data, 
    uint16_t* len, 
    ble_gap_addr_t* origin_addr);

/**
* @brief Get current mesh access address
* 
* @param[out] access_addr Pointer location to put access address in
* 
* @return NRF_SUCCESS the value was fetched successfully
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized 
*/
uint32_t rbc_mesh_access_address_get(uint32_t* access_address);

/**
* @brief Get current mesh channel
* 
* @param[out] ch Pointer location to put mesh channel in 
*
* @return NRF_SUCCESS the value was fetched successfully
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized 
*/
uint32_t rbc_mesh_channel_get(uint8_t* ch);

/**
* @brief Get the amount of allocated handle-value pairs 
* 
* @param[out] handle_count Pointer location to put handle count in 
*
* @return NRF_SUCCESS the value was fetched successfully
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized 
*/
uint32_t rbc_mesh_handle_count_get(uint8_t* handle_count);

/**
* @brief Get the mesh minimum advertise interval in ms
*
* @param[out] adv_int_ms Pointer location to put adv int in
*
* @return NRF_SUCCESS the value was fetched successfully
 * @brief Get the contents of the data array pointed to by the provided handle
*
* @param[in] handle The handle of the value we want to update. Is mesh-global.
* @param[out] data Databuffer to be copied into the value slot. Must be at least
*    RBC_VALUE_MAX_LEN long
* @param[out] len Length of the copied data. Will not exceed RBC_VALUE_MAX_LEN.
* @param[out] origin_addr BLE GAP address of the node that first broadcasted 
*   the current version of this value. Set to NULL if the address is not of 
*   interest.
* 
* @return NRF_SUCCESS the value has been successfully fetched.
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized.
* @return NRF_ERROR_INVALID_ADDR the handle is outside the range provided
*    in @ref rbc_mesh_init.
*/
uint32_t rbc_mesh_value_get(uint8_t handle, 
    uint8_t* data, 
    uint16_t* len, 
    ble_gap_addr_t* origin_addr);

/**
* @brief Get current mesh access address
* 
* @param[out] access_addr Pointer location to put access address in
* 
* @return NRF_SUCCESS the value was fetched successfully
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized 
*/
uint32_t rbc_mesh_access_address_get(uint32_t* access_address);

/**
* @brief Get current mesh channel
* 
* @param[out] ch Pointer location to put mesh channel in 
*
* @return NRF_SUCCESS the value was fetched successfully
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized 
*/
uint32_t rbc_mesh_channel_get(uint8_t* ch);

/**
* @brief Get the amount of allocated handle-value pairs 
* 
* @param[out] handle_count Pointer location to put handle count in 
*
* @return NRF_SUCCESS the value was fetched successfully
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized 
*/
uint32_t rbc_mesh_handle_count_get(uint8_t* handle_count);

/**
* @brief Get the mesh minimum advertise interval in ms
*
* @param[out] adv_int_ms Pointer location to put adv int in
*
* @return NRF_SUCCESS the value was fetched successfully
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized
*/
uint32_t rbc_mesh_adv_int_get(uint32_t* adv_int_ms);

/**
* @brief Event handler to be called upon external BLE event arrival.
*   Only handles GATTS write events, all other types are ignored.
*   Has a similar effect as @ref rbc_mesh_value_set, by refreshing version
*   numbers and timing parameters related to the indicated characteristic.
*
* @note This event may be called regardless of if the indicated characteristic
*   belongs to the mesh or not, the framework will filter out uninteresting 
*   events and return NRF_SUCCESS. However, if the incoming event points at 
*   the mesh service, but the characteristic handle is out of range, the 
*   function returns NRF_ERROR_INVALID_ADDR. 
*
* @note This function will also trigger any update/new events in the application
*   space 
*   
* @param[in] evt BLE event received from softdevice.
*
* @return NRF_SUCCESS Event successfully handled.
* @return NRF_ERROR_INVALID_ADDR Handle is part of service, but does not belong
*   any valid characteristics.
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized.
*/
uint32_t rbc_mesh_ble_evt_handler(ble_evt_t* evt);

/**
* @brief Softdevice interrupt handler, checking if there are any 
*   incomming events related to the framework. 
*
* @note Should be called from the SD_IRQHandler function. Will poll the 
*   softdevice for new sd_evt.
*/
uint32_t rbc_mesh_sd_irq_handler(void);

/**
 * @brief Application space event handler. TO BE IMPLEMENTED IN APPLICATION 
*   SPACE.
*
* @note Does not have an implementation within the framework, but acts as a 
*   feedback channel for the framework to notify the application of any 
*   changes in values.
*
* @param evt Framework generated event presented to the application. 
*/
void rbc_mesh_event_handler(rbc_mesh_event_t* evt);

#endif /* _RBC_MESH_H__ */

