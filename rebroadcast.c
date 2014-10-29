#include "rebroadcast.h"
#include "trickle_common.h"
#include "rbc_database.h"

#include "nrf_error.h"

#include <string.h>


/*****************************************************************************
* Static globals
*****************************************************************************/

static rbc_event_callback g_event_callback;

static bool g_is_initialized = false;



/*****************************************************************************
* Static Functions
*****************************************************************************/

static void rbc_distribute_value_changed(



/*****************************************************************************
* Interface Functions
*****************************************************************************/

uint32_t rbc_init(const rbc_event_callback event_callback_function)
{
    if (g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    else
    {
        g_is_initialized = true;
    }
    
    g_event_callback = event_callback_function;
    
    db_init();
    
    return NRF_SUCCESS;
}


uint32_t rbc_value_alloc(const rbc_value_handle_t value_handle, const bool is_volatile)
{
    if (!g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    db_value_t* val = db_value_alloc();
    
    if (val == NULL)
    {
        return NRF_ERROR_NO_MEM;
    }
    
    val->identifier.broadcast.id = value_handle;
    
    val->flags |= (1 << DB_VALUE_FLAG_IS_BROADCAST_POS);
    
    if (is_volatile)
    {
        val->flags |= (1 << DB_VALUE_FLAG_VOLATILE_POS);
    }
    
    return NRF_SUCCESS;
}


uint32_t rbc_value_data_set(const rbc_value_handle_t value_handle, 
    const uint8_t* data, 
    const uint8_t len)
{
    if (!g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    if (len > DB_VARIATION_MAX_LENGTH)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    
    db_value_t* val = db_value_get(value_handle, 0);
    
    if (val == NULL)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    
    variation_t new_variation;
    
    memcpy(new_variation.data, data, len);
    
    new_variation.length = len;
    new_variation.version = val->variation.version + 1;
    
    db_value_update_variation(val, &new_variation);
    
    
    return NRF_SUCCESS;
}


uint32_t rbc_value_data_get(const rbc_value_handle_t value_handle, 
    uint8_t* data, 
    uint8_t* len)
{
    if (!g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    db_value_t* val = db_value_get(value_handle, 0);
    
    if (val == NULL)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    
    memcpy(data, val->variation.data, val->variation.length);
    *len = val->variation.length;
    
    return NRF_SUCCESS;
}


uint32_t rbc_value_delete(const rbc_value_handle_t value_handle)
{
    if (!g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    db_value_t* val = db_value_get(value_handle, 0);
    
    if (val == NULL)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    
    val->flags &= ~(1 << DB_VALUE_FLAG_ACTIVE_POS);
    
    return NRF_SUCCESS;
}


void rbc_sd_irq_handler(void)
{
    /* call lower layer event handler */
    broadcast_event_handler();
}
