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


/** 
* @brief Function type for application space framework event handler. 
*   Passed to framework upon initialization.
*/
typedef void(*rbc_event_callback)(rbc_event_t* evt);



/**
* @brief Initialize Rebroadcast module, must be called before any other 
*   rebroadcast function. 
* 
* @param[in] event_callback_function Function pointer to user space event 
*   handler function. 
* 
* @return NRF_SUCCESS the initialization is successful 
* @return NRF_ERROR_INVALID_STATE the framework has already been initialized.
*/
uint32_t rbc_init(const rbc_event_callback event_callback_function);


/**
* @brief Allocate a new rebroadcast value in the database. 
*   The value is not shared with the network before any data is set. 
* 
* @warning If all value slots are full, the framework will overwrite an old value
*   that has not been marked as volatile. The user will be notified of this 
*   through a "DELETE_VAL" event in the callback. The callback will be invoked
*   at the calling interrupt level, before the allocate call concludes. If 
*   all value slots are occupied by volatile values, the function call will 
*   return NRF_ERROR_NO_MEM.
* 
* @param[in] value_handle Handle for the newly allocated value
*   in the database. Must be used as reference to all future
*   operations on this value. It is up to the application to ensure non-conflicting
*   handles throughout the network.
* @param[in] is_volatile Whether to allocate the value as a volatile, meaning
*   that it may not be deleted in this node in favor of new allocations
*
* @return NRF_SUCCESS the allocation is successful
* @return NRF_ERROR_NO_MEM there is no more non-volatile slots available
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized.
*/
uint32_t rbc_value_alloc(const rbc_value_handle_t value_handle, const bool is_volatile);


/**
* @brief Set contents of data buffer represented of handle. Will be broadcasted
*   to other nodes in the mesh. 
* 
* @note No data values will be broadcasted to the other notes before the first 
*   time this function is called. 
* @note If two different nodes update a value with this handle concurrently,
*   a "CONFLICTING_VAL" event will be sent to the event handler upon arrival 
*   of the external value. 
* 
* @param[in] value_handle Handle for the desired value, originating from 
*   rbc_value_alloc() or a "NEW_VAL" event 
* @param[in] data Byte array to be passed around the mesh as the handle value.
*   max length is 27 bytes.
* @param[in] len Length of the provided data array in number of byte elements.
*   max length is 27 bytes.
* 
* @return NRF_SUCCESS if the data value update is successful
* @return NRF_ERROR_INVALID_ADDR the given handle does not exist in the database
* @return NRF_ERROR_INVALID_LENGTH the len parameter exceeds 27 bytes.
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized.
*/
uint32_t rbc_value_data_set(const rbc_value_handle_t value_handle, 
    const uint8_t* data, 
    const uint8_t len);


/**
* @brief Get a copy of the data residing at the indicated handle slot
*
* @param[in] value_handle Handle to the wanted data set.
* @param[in,out] data Pointer to a buffer (at least 27 bytes long) where to 
*   store the value. Set to NULL to only obtain length of buffer
* @param[out] len Length of current data
*
* @return NRF_SUCCESS if data and len has been successfully filled with 
*   buffer information
* @return NRF_ERROR_INVALID_ADDR if the given handle does not exist in the database
* @return NRF_ERROR_INVALID_STATE if the framework has not been initialized.
*/
uint32_t rbc_value_data_get(const rbc_value_handle_t value_handle, 
    uint8_t* data, 
    uint8_t* len);
    
/**
* @brief Mark value handle for deletion. This frees the slot this value takes
*   in the database for new allocations. Overrides any volatile attributes on 
*   this value.
* 
* @param[in] value_handle Handle of the slot to be deleted
* 
* @return NRF_SUCCESS if the value has been successfully freed.
* @return NRF_ERROR_INVALID_ADDR the given handle does not exist in the database
* @return NRF_ERROR_INVALID_STATE the framework has not been initialized.
*/
uint32_t rbc_value_delete(const rbc_value_handle_t value_handle);




#endif /* _REBROADCAST_H__ */
