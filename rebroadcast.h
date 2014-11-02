#ifndef _REBROADCAST_H__
#define _REBROADCAST_H__
#include <stdint.h>
#include <stdbool.h>
#include "nrf51.h"

/**
* @brief Compile time constant determining database size. 
*
* @note Should be set in "Preprocessor symbols" in Keil or with 
*   -D RBC_MAX_VALUE_COUNT=(X) in gcc
*/
#ifndef RBC_MAX_VALUE_COUNT
    #define RBC_MAX_VALUE_COUNT (8)
#endif


/** 
* @brief Rebroadcast value handle type 
*
* @detailed Handle type used to identify a value in the mesh, is consistent
*   throughout the network
*/
typedef uint16_t rbc_value_handle_t;
    
/** @brief Event type enum. Identifies framework generated events */
typedef enum
{
    RBC_EVENT_TYPE_UPDATE_VAL,      /** Another node has updated the value */
    RBC_EVENT_TYPE_CONFLICTING_VAL, /** Another node has a conflicting version of the value */
    RBC_EVENT_TYPE_NEW_VAL,         /** A previously unallocated value has been received and allocated */
    RBC_EVENT_TYPE_DISCARDED_VAL,   /** A previously unallocated value has been received but there was no room for it. */
    RBC_EVENT_TYPE_DELETE_VAL       /** The indicated value has been deleted from the database, but may be reallocated by user */
} rbc_event_type_t;

/** 
* @brief Rebroadcast framework generated event. Carries information about a
*   change to one database value.
*/
typedef struct
{
    rbc_event_type_t event_type;        /** See @ref rbc_event_type_t */
    rbc_value_handle_t value_handle;    /** Handle of the value the event is generated for */
    uint8_t* data;                      /** Current data array contained at the event handle location */
    uint8_t data_len;                   /** Length of data array */
} rbc_event_t;


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
* @param access_addr The access address the mesh will work on. This must be the 
*    same for all nodes in the mesh. RBC_ACCESS_ADDRESS_BLE_ADV gives the mesh
*    the same access address as regular BLE advertisements, which makes the
*    traffic visible to external BLE devices (Note that other access addresses 
*    does not provide any data security, the traffic is merely ignored by 
*    regular BLE radios). Multiple meshes may in theory work concurrently in 
*    the same area with different access addresses, but will be prone to 
*    on-air collisions, and it is recommended to use separate channels for this
* @param channel The BLE channel the mesh works on. It is strongly recommended 
*    to use one of the three adv channels 37, 38 or 39, as others may be prone
*    to on-air collisions with WiFi channels. Separate meshes may work 
*    concurrently without packet collision if they are assigned to different 
*    channels. Must be between 1 and 39.
* @param handle_count The maximum number of handle-value pairs available to the
*    application. May not be higher than 155 due to BLE namespace requirements
* @param adv_int_ms The minimum adv_interval for nodes in the network in 
*    millis. Must be between 5 and 60000.
* 
* @return NRF_SUCCESS the initialization is successful 
* @return NRF_ERROR_INVALID_PARAM a parameter does not meet its requiremented range.
* @return NRF_ERROR_INVALID_STATE the framework has already been initialized.
* @return NRF_ERROR_SOFTDEVICE_NOT_ENABLED the Softdevice has not been enabled.
*/
uint32_t rbc_init(uint32_t access_addr, uint8_t channel, uint8_t handle_count, uint8_t adv_int_ms);

/**
* @brief Set the contents of the data array pointed to by the provided handle
*
* @param[in] handle The handle of the value we want to update. Is mesh-global.
* @param[in] data Databuffer to be copied into the value slot
* @param[in] len Length of the provided data. Must not exceed RBC_VALUE_MAX_LEN.
* 
* @return NRF_SUCCESS if the value has been successfully updated.
* @return NRF_ERROR_INVALID_STATE if the framework has not been initialized.
* @return NRF_ERROR_INVALID_ADDR if the handle is outside the range provided
*    in rbc_init.
* @return NRF_ERROR_INVALID_LENGTH if len exceeds RBC_VALUE_MAX_LEN.
*/
uint32_t rbc_value_set(uint8_t handle, uint8_t* data, uint8_t len);

/**
 * @brief Get the contents of the data array pointed to by the provided handle
*
* @param[in] handle The handle of the value we want to update. Is mesh-global.
* @param[out] data Databuffer to be copied into the value slot. Must be at least
*    RBC_VALUE_MAX_LEN long
* @param[out] len Length of the copied data. Will not exceed RBC_VALUE_MAX_LEN.
* 
* @return NRF_SUCCESS the value has been successfully fetched.
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized.
* @return NRF_ERROR_INVALID_ADDR the handle is outside the range provided
*    in rbc_init.
*/
uint32_t rbc_value_get(uint8_t handle, uint8_t* data, uint8_t* len);

/**
* @brief Get the mesh minimum advertise interval in ms
*
* @param[out] adv_int_ms The current adv_int in milliseconds
*
* @return NRF_SUCCESS the value was fetched successfully
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized
*/
uint32_t rbc_adv_int_get(uint32_t adv_int_ms);

/**
* @brief Set the mesh minimum advertise interval in ms
*
* @param[in] adv_int_ms The wanted minimum adv interval in ms. Must be
*    between 5 and 60000.
*
* @return NRF_SUCCESS successfully updated adv_int
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized
* @return NRF_ERROR_INVALID_PARAM The adv int was outside the required range,
*    and has not been set.
*/
uint32_t rbc_adv_int_set(uint32_t adv_int_ms);

/**
 * @brief Softdevice interrupt handler, checking if there are any 
*   incomming events related to the framework. 
*
* @note Should be called from the SD_IRQHandler function. Will poll the 
*   softdevice for new sd_evt.
*/
void rbc_sd_irq_handler(void);

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
void rbc_event_handler(rbc_event_t* evt);

#endif /* _REBROADCAST_H__ */

