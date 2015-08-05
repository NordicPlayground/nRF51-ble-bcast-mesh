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

#include "rbc_mesh.h"
#include "rbc_mesh_common.h"
#include "mesh_srv.h"
#include "timeslot_handler.h"
#include "event_handler.h"

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
static uint32_t g_adv_int_ms;


/*****************************************************************************
* Static Functions
*****************************************************************************/


/*****************************************************************************
* Interface Functions
*****************************************************************************/

uint32_t rbc_mesh_init(rbc_mesh_init_params_t init_params)
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

    error_code = mesh_srv_init(init_params.handle_count,
                                init_params.access_addr,
                                init_params.channel,
                                init_params.adv_int_ms);

    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    event_handler_init();
    timeslot_handler_init();

    g_access_addr = init_params.access_addr;
    g_channel = init_params.channel;
    g_handle_count = init_params.handle_count;
    g_adv_int_ms = init_params.adv_int_ms;

    g_is_initialized = true;

    return NRF_SUCCESS;
}

uint32_t rbc_mesh_value_enable(uint8_t handle)
{
    return mesh_srv_char_val_enable(handle);
}

uint32_t rbc_mesh_value_disable(uint8_t handle)
{
    return mesh_srv_char_val_disable(handle);
}


/****** Getters and setters ******/

uint32_t rbc_mesh_value_set(uint8_t handle, uint8_t* data, uint16_t len)
{
    return mesh_srv_char_val_set(handle, data, len, true);
}

uint32_t rbc_mesh_value_get(uint8_t handle, uint8_t* data, uint16_t* len, ble_gap_addr_t* origin_addr)
{
    return mesh_srv_char_val_get(handle, data, len, origin_addr);
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
    if (evt->header.evt_id == BLE_GAP_EVT_CONNECTED)
    {
        mesh_srv_conn_handle_update(evt->evt.gap_evt.conn_handle);
    }

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


/***** event handler ******/

uint32_t rbc_mesh_sd_irq_handler(void)
{
    /* call lower layer event handler */
    ts_sd_event_handler();

    return NRF_SUCCESS;
}
