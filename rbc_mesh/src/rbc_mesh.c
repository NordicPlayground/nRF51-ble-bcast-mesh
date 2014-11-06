#include "rbc_mesh.h"
#include "rbc_mesh_common.h"
#include "mesh_srv.h"
#include "timeslot_handler.h"

#include "nrf_error.h"
#include "nrf_sdm.h"

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

uint32_t rbc_mesh_init(uint32_t access_addr, 
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
    
    error_code = mesh_srv_init(handle_count, access_addr, channel, adv_int_ms);

    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    timeslot_handler_init();
    
    g_access_addr = access_addr;
    g_channel = channel;
    g_handle_count = handle_count;
    g_adv_int_ms = adv_int_ms;

    g_is_initialized = true;
    
    return NRF_SUCCESS;
}

uint32_t rbc_mesh_value_req(uint8_t handle)
{
    return mesh_srv_char_val_init(handle);
}

/****** Getters and setters ******/

uint32_t rbc_mesh_value_set(uint8_t handle, uint8_t* data, uint16_t len)
{
    return mesh_srv_char_val_set(handle, data, len);
}

uint32_t rbc_mesh_value_get(uint8_t handle, uint8_t* data, uint16_t* len)
{
    return mesh_srv_char_val_get(handle, data, len);
}

uint32_t rbc_mesh_access_address_get(uint32_t* access_address)
{
    if (!g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    *access_address = g_access_addr;
    
    return NRF_SUCCESS;
}

uint32_t rbc_mesh_channel_get(uint8_t* ch)
{
    if (!g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    *ch = g_channel;
    
    return NRF_SUCCESS;
}
    
uint32_t rbc_mesh_handle_count_get(uint8_t* handle_count)
{
    if (!g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    *handle_count = g_handle_count;
    
    return NRF_SUCCESS;
}

uint32_t rbc_mesh_adv_int_get(uint32_t* adv_int_ms)
{
    if (!g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    *adv_int_ms = g_adv_int_ms;

    return NRF_SUCCESS;
}

uint32_t rbc_mesh_ble_evt_handler(ble_evt_t* evt)
{
    if (!g_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    
    /* may safely ignore all events that don't write to a value */
    if (evt->header.evt_id != BLE_GATTS_EVT_WRITE)
    {
        return NRF_SUCCESS;
    }
    ble_gatts_evt_write_t* write_evt = &evt->evt.gatts_evt.params.write;
    
    uint32_t error_code = mesh_srv_gatts_evt_write_handle(write_evt);
    
    if (error_code != NRF_SUCCESS && 
        error_code != NRF_ERROR_INVALID_ADDR)
    {
        if (error_code == NRF_ERROR_FORBIDDEN)
        {
            return NRF_SUCCESS; /* wrong service, just ignore */
        }
        else
        {
            return error_code;
        }
    }    
    
    return NRF_SUCCESS;
}

#if 0
uint32_t rbc_mesh_adv_int_set(uint32_t adv_int_ms)
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
#endif

/***** event handler ******/

void rbc_mesh_sd_irq_handler(void)
{
    /* call lower layer event handler */
    ts_sd_event_handler();
}
