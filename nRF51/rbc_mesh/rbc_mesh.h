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
#include "nrf_sdm.h"
#include "ble.h"

#define RBC_MESH_ACCESS_ADDRESS_BLE_ADV             (0x8E89BED6) /**< BLE spec defined access address. */
#define RBC_MESH_INTERVAL_MIN_MIN_MS                (5) /**< Lowest min-interval allowed. */
#define RBC_MESH_INTERVAL_MIN_MAX_MS                (60000) /**< Highest min-interval allowed. */
#define RBC_MESH_VALUE_MAX_LEN                      (23) /**< Longest legal payload. */
#define RBC_MESH_INVALID_HANDLE                     (0xFFFF) /**< Designated "invalid" handle, may never be used */
#define RBC_MESH_APP_MAX_HANDLE                     (0xFFEF) /**< Upper limit to application defined handles. The last 16 handles are reserved for mesh-maintenance. */

#define RBC_MESH_GPREGRET_CODE_GO_TO_APP            (0x00) /**< Retention register code for immediately starting application when entering bootloader. The default behavior. */
#define RBC_MESH_GPREGRET_CODE_FORCED_REBOOT        (0x01) /**< Retention register code for telling the bootloader it's been started on purpose */
/*
   There are two caches in the framework:
   - The handle cache keeps track of the latest version number for each handle.
   - The data cache contains all handles currently being retransmitted by the device.
   If a handle falls out of the data cache, the device will stop broadcasting it.
   If a handle falls out of the handle cache, the device will not know whether updates
   to the handle are new or old.
*/

/** @brief Default value for the number of handle cache entries */
#ifndef RBC_MESH_HANDLE_CACHE_ENTRIES
    #define RBC_MESH_HANDLE_CACHE_ENTRIES           (100)
#endif

/** @brief Default value for the number of data cache entries */
#ifndef RBC_MESH_DATA_CACHE_ENTRIES
    #define RBC_MESH_DATA_CACHE_ENTRIES             (35)
#endif

/** @brief Length of app-event FIFO. Must be power of two. */
#ifndef RBC_MESH_APP_EVENT_QUEUE_LENGTH
    #define RBC_MESH_APP_EVENT_QUEUE_LENGTH         (16)
#endif

/** @brief Length of low level radio event FIFO. Must be power of two. */
#ifndef RBC_MESH_RADIO_QUEUE_LENGTH
    #define RBC_MESH_RADIO_QUEUE_LENGTH             (8)
#endif

/** @brief Length of internal async-event FIFO. Must be power of two. */
#ifndef RBC_MESH_INTERNAL_EVENT_QUEUE_LENGTH
    #define RBC_MESH_INTERNAL_EVENT_QUEUE_LENGTH    (8)
#endif

/** @brief Size of packet pool. Only accounts for one packet in the app-space at a time. */
#ifndef RBC_MESH_PACKET_POOL_SIZE
    #define RBC_MESH_PACKET_POOL_SIZE               (RBC_MESH_DATA_CACHE_ENTRIES +\
                                                     RBC_MESH_APP_EVENT_QUEUE_LENGTH + \
                                                     RBC_MESH_RADIO_QUEUE_LENGTH + \
                                                     RBC_MESH_INTERNAL_EVENT_QUEUE_LENGTH +\
                                                     3)
#endif

#if (RBC_MESH_HANDLE_CACHE_ENTRIES < RBC_MESH_DATA_CACHE_ENTRIES)
    #error "The number of handle cache entries cannot be lower than the number of data entries"
#endif

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
    RBC_MESH_EVENT_TYPE_INITIALIZED,     /** The framework has been initialized internally (most likely via serial interface) */
    RBC_MESH_EVENT_TYPE_TX,              /** The indicated handle was transmitted */
} rbc_mesh_event_type_t;

/** @brief Rebroadcast framework generated event. */
typedef struct
{
    rbc_mesh_event_type_t event_type;       /**< See @ref rbc_mesh_event_type_t */
    rbc_mesh_value_handle_t value_handle;   /**< Handle of the value the event is generated for */
    uint8_t* data;                          /**< Current data array contained at the event handle location */
    uint8_t data_len;                       /**< Length of data array */
    int8_t rssi;                            /**< RSSI of received data, in range of -100dBm to ~-40dBm */
    ble_gap_addr_t ble_adv_addr;            /**< Advertisement address of the device we got the update from. */
    uint16_t version_delta;                 /**< Version number increase since last update */
} rbc_mesh_event_t;

/**
* @brief Initialization parameter struct for the rbc_mesh_init() function.
*
* @param[in] access_addr The access address the mesh will work on. This must be the
*    same for all nodes in the mesh. RBC_MESH_ACCESS_ADDRESS_BLE_ADV gives the mesh
*    the same access address as regular BLE advertisements, which makes the
*    traffic visible to external BLE devices (Note that other access addresses
*    does not provide any data security, the traffic is merely ignored by
*    regular BLE radios). Multiple meshes may in theory work concurrently in
*    the same area with different access addresses, but will be prone to
*    on-air collisions, and it is recommended to use separate channels for this.
* @param[in] channel The BLE channel the mesh works on. It is strongly recommended
*    to use one of the three adv channels 37, 38 or 39, as others may be prone
*    to on-air collisions with WiFi channels. Separate meshes may work
*    concurrently without packet collision if they are assigned to different
*    channels. Must be between 1 and 39.
* @param[in] interval_min_ms The minimum tx interval for nodes in the network in
*    millis. Must be between 5 and 60000.
* @param[in] lfclksrc The LF-clock source parameter supplied to the softdevice_enable function.
*/
typedef struct
{
    uint32_t access_addr;
    uint8_t channel;
    uint32_t interval_min_ms;
    nrf_clock_lfclksrc_t lfclksrc;
} rbc_mesh_init_params_t;
/*****************************************************************************
     Interface Functions
*****************************************************************************/

/**
* @brief Initialize Rebroadcast module, must be called before any other
*   rebroadcast function.
*
* @note The nRF51 Softdevice must be initialized by the application before
*    the mesh framework intialization is called, or the function will
*    return NRF_ERROR_SOFTDEVICE_NOT_ENABLED.
*
* @return NRF_SUCCESS the initialization is successful
* @return NRF_ERROR_INVALID_PARAM a parameter does not meet its required range.
* @return NRF_ERROR_INVALID_STATE the framework has already been initialized.
* @return NRF_ERROR_SOFTDEVICE_NOT_ENABLED the Softdevice has not been enabled.
*/
uint32_t rbc_mesh_init(rbc_mesh_init_params_t init_params);

/**
* @brief Start mesh radio activity after stopping it.
*
* @details This function is called automatically in the @ref rbc_mesh_init
*   function, and only has to be explicitly called after a call to
*   @ref rbc_mesh_stop.
*
* @return NRF_SUCCESS the mesh successfully started radio operation
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized, or
*   the mesh is already running.
*/
uint32_t rbc_mesh_start(void);

/**
* @brief Stop mesh radio activity.
*
* @details Stop ordering timeslots from the Softdevice, effectively stopping
*   all radio activity. This allows the chip to enter long-term low power
*   operation.
*
* @note While other mesh calls will work locally, the device will not be able
*   to transmit or receive anything to/from other devices in the network before
*   being reactivated by a call to @ref rbc_mesh_start.
*
* @return NRF_SUCCESS the mesh successfully stopped all radio operation
* @return NRF_ERROR_INVALID_STATE te framework has not been initialized, or
*   the mesh operation is already stopped.
*/
uint32_t rbc_mesh_stop(void);

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
uint32_t rbc_mesh_value_set(rbc_mesh_value_handle_t handle, uint8_t* data, uint16_t len);

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
* @return NRF_SUCCESS A request was successfully scheduled for broadcast.
* @return NRF_ERROR_INVALID_ADDR the handle is invalid.
* @return NRF_ERROR_INVALID_STATE The framework has not been initiated.
*/
uint32_t rbc_mesh_value_enable(rbc_mesh_value_handle_t handle);

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
* @return NRF_ERROR_INVALID_ADDR the handle is invalid.
* @return NRF_ERROR_INVALID_STATE The framework has not been initialized.
*/
uint32_t rbc_mesh_value_disable(rbc_mesh_value_handle_t handle);

/**
* @brief Set whether the given handle should be persistent in the handle cache.
*
* @note While non-persistent values may be forgotten by the handle cache, a
*   persistent value will be retransmitted forever. Note that setting too many
*   persistent values in the cache will reduce the framework's ability to
*   retransmit non-persistent values, leading to more packet drops and poor
*   throughput. It is therefore recommended to be conservative about the
*   usage of this flag.
* @note If a device is known to be the lone maintainer of a particular
*   handle, it is recommended to let the handle be persistent in that device,
*   as a reset in version numbers may cause a disrupt in communication.
*
* @param[in] handle Handle to change the Persistent flag for.
* @param[in] persistent Whether or not to let the value be persistent in the
*   cache.
*
* @return NRF_SUCCESS the persistence configuration has been set successfully.
* @return NRF_ERROR_INVALID_ADDR the handle is invalid.
* @return NRF_ERROR_NO_MEM the number of persistent values in the cache exceeds
*   the cache size.
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized.
*/
uint32_t rbc_mesh_persistence_set(rbc_mesh_value_handle_t handle, bool persistent);

/**
* @brief Set whether the given handle should produce TX events for each time
*   the value is transmitted.
*
* @note In order to maintain high performance in the framework, it is
*   recommended that the amount of values that have this flag set is kept
*   as low as possible. The flag is set to 0 by default.
* @note The TX event data-field is set to NULL, and any need to access the
*   contents of the transmission must be done with the @ref
*   rbc_mesh_value_get() function.
*
* @param[in] handle Handle to change TX event flag for
* @param[in] do_tx_event The TX event configuration for the given value
*
* @return NRF_SUCCESS the TX event configuration has been set successfully
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized.
* @return NRF_ERROR_INVALID_ADDR the handle is invalid.
*/
uint32_t rbc_mesh_tx_event_set(rbc_mesh_value_handle_t handle, bool do_tx_event);

/**
* @brief Get the contents of the data array pointed to by the provided handle
*
* @param[in] handle The handle of the value we want to update. Is mesh-global.
* @param[out] data Databuffer to be copied into the value slot. Must be at least
*    RBC_VALUE_MAX_LEN long.
* @param[out] len Length of the copied data. Will not exceed RBC_VALUE_MAX_LEN.
*
* @return NRF_SUCCESS the value has been successfully fetched.
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized.
* @return NRF_ERROR_INVALID_ADDR the handle is invalid.
*/
uint32_t rbc_mesh_value_get(rbc_mesh_value_handle_t handle,
    uint8_t* data,
    uint16_t* len);

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
* @brief Get the mesh minimum transmit interval in ms
*
* @param[out] interval_min_ms Pointer location to put adv int in
*
* @return NRF_SUCCESS the value was fetched successfully
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized
*/
uint32_t rbc_mesh_interval_min_ms_get(uint32_t* interval_min_ms);

/**
* @brief get whether the given handle has its persistence flag set
*
* @param[in] handle The handle whose flag should be checked.
* @param[out] is_persistent a pointer to a boolean to which the flag status will
*   be copied.
*
* @return NRF_SUCCESS The flag status was successfully copied to the parameter.
* @return NRF_ERROR_INVALID_STATE The framework has not been initilalized.
* @return NRF_ERROR_NOT_FOUND The given handle is not present in the cache.
* @return NRF_ERROR_INVALID_ADDR The given handle is invalid.
*/
uint32_t rbc_mesh_persistence_get(rbc_mesh_value_handle_t handle, bool* is_persistent);

/**
* @brief get whether the given handle has its tx_event flag set
*
* @param[in] handle The handle whose flag should be checked.
* @param[out] is_doing_tx_event a pointer to a boolean to which the flag status will
*   be copied.
*
* @return NRF_SUCCESS The flag status was successfully copied to the parameter.
* @return NRF_ERROR_INVALID_STATE The framework has not been initilalized.
* @return NRF_ERROR_NOT_FOUND The given handle is not present in the cache.
* @return NRF_ERROR_INVALID_ADDR The given handle is invalid.
*/
uint32_t rbc_mesh_tx_event_flag_get(rbc_mesh_value_handle_t handle, bool* is_doing_tx_event);

/**
* @brief Event handler to be called upon Softdevice BLE event arrival.
*
* @details Event handler taking care of all mesh behavior related to BLE.
*   Typical events to trigger processing or local changes are
*   writes to the value characteristic or a change in connection status. The
*   framework will give events to the application if the function triggers a
*   write to a handle value or similar.
*
* @note This event may be called regardless of whether the event is relevant
*   to the mesh or not - the framework will filter out uninteresting
*   events and return NRF_SUCCESS.
*
* @param[in] p_evt BLE event received from softdevice.
*/
void rbc_mesh_ble_evt_handler(ble_evt_t* p_evt);

/**
* @brief Event handler to be called upon regular Softdevice event arrival.
*
* @details Event handler taking care of all mesh behavior related to the
*   Softdevice. Events are expected to be in the range
*
* @note This event may be called regardless of whether the event is relevant
*   to the mesh or not - the framework will filter out uninteresting
*   events and return NRF_SUCCESS.
*
* @param[in] evt event received from Softdevice through the sd_evt_get() call.
*/
void rbc_mesh_sd_evt_handler(uint32_t evt);

/**
* @brief Get an event from the mesh.
*
* @param[out] p_evt A pointer to the struct the event should be copied to.
*   May be set to NULL if the contents of the event isn't important.
*
* @return NRF_SUCCESS An event was successfully popped and copied into the
*   p_evt-parameter.
* @return NRF_ERROR_NOT_FOUND No events ready to be pulled.
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized.
*/
uint32_t rbc_mesh_event_get(rbc_mesh_event_t* p_evt);

/**
* @brief Get an event from the mesh, but don't remove it from the queue.
*
* @note This call has the same effect as the rbc_mesh_event_get call, except
*   it does not remove the event from the queue. Repeated calls to the peek
*   function will yield the same event.
*
* @param[out] p_evt A pointer to the struct the event should be copied to.
*
* @return NRF_SUCCESS An event was successfully popped and copied into the
*   p_evt-parameter.
* @return NRF_ERROR_NOT_FOUND No events ready to be pulled.
* @return NRF_ERROR_NULL The p_evt parameter is NULL.
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized.
*/
uint32_t rbc_mesh_event_peek(rbc_mesh_event_t* p_evt);

/**
* @brief Free the memory associated with the given mesh packet.
*
* @details: In order to reduce the amount of data copying going on for each
*   data packet, the data field of an rbc_mesh_event points directly to the
*   framework packet pool. This memory is managed by the mesh_packet module,
*   and must be explicitly released when the contents is no longer in use.
*   Failure to do so will result in a NO_MEM error when the framework runs out
*   of available packets in the packet pool.
*
* @param[in] p_data Pointer to a data array which originated from an event
*   in the framework functions rbc_mesh_event_get and rbc_mesh_event_peek.
*
* @return NRF_SUCCESS The memory associated with the given data pointer was
*   successfully freed.
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized.
*/
uint32_t rbc_mesh_packet_release(uint8_t* p_data);

#endif /* _RBC_MESH_H__ */

