#include "rebroadcast.h"
#include "trickle_common.h"
#include "rbc_database.h"

#include "nrf_error.h"

#include <string.h>


/*****************************************************************************
* Static globals
*****************************************************************************/


static bool g_is_initialized = false;
static uint32_t g_access_addr;
static uint8_t g_channel;
static uint8_t g_handle_count;
static uint8_t g_adv_int_ms;


/*****************************************************************************
* Static Functions
*****************************************************************************/





/*****************************************************************************
* Interface Functions
*****************************************************************************/

uint32_t rbc_init(uint32_t access_addr, 
        uint8_t channel, 
        uint8_t handle_count, 
        uint8_t adv_int_ms)
{
    uint8_t sd_is_enabled = 0;
    sd_softdevice_is_enabled(&sd_is_enabled);

    if (!sd_is_enabled)
    {
        return NRF_ERROR_SOFTDEVICE_NOT_ENABLED;
    }
        

    if (g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint32_t error_code;
    
    error_code = mesh_srv_init(handle_range, access_address, channel, adv_int_ms);

    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    g_access_addr = access_addr;
    g_channel = channel;
    g_handle_count = handle_count;
    g_adv_int_ms = adv_int_ms;

     g_is_initialized = true;
    
    return NRF_SUCCESS;
}

uint32_t rbc_value_set(uint8_t handle, uint8_t* data, uint8_t len)
{
    if (!g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return mesh_srv_char_val_set(handle, data, len);
}

uint32_t rbc_value_get(uint8_t handle, uint8_t* data, uint8_t* len)
{
    if (!g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    return mesh_srv_char_val_get(handle, data, len);
}

uint32_t rbc_adv_int_get(uint32_t* adv_int_ms)
{
    if (!g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    *adv_int_ms = g_adv_int_ms;

    return NRF_SUCCESS;
}

uint32_t rbc_adv_int_set(uint32_t adv_int_ms)
{
    if (!g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (adv_int_ms < RBC_ADV_INT_MIN ||
            adv_int_ms > RBC_ADV_INT_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    g_adv_int_ms = adv_int_ms;

    return NRF_SUCCESS;
}

void rbc_sd_irq_handler(void)
{
    /* call lower layer event handler */
    broadcast_event_handler();
}
