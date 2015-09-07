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

#include "mesh_srv.h"
#include "rbc_mesh.h"
#include "timeslot_handler.h"
#include "trickle.h"
#include "rbc_mesh_common.h"
#include "mesh_aci.h"
#include "version_handler.h"

#include "nrf_soc.h"
#include "nrf_error.h"
#include "ble.h"
#include <stdlib.h>
#include <stdbool.h>
#include <string.h>

/* Packet related constants */
#define MESH_PACKET_HANDLE_LEN          (1)
#define MESH_PACKET_VERSION_LEN         (2)

#define MESH_PACKET_HANDLE_OFFSET       (0)
#define MESH_PACKET_VERSION_OFFSET      (MESH_PACKET_HANDLE_OFFSET + MESH_PACKET_HANDLE_LEN)
#define MESH_PACKET_DATA_OFFSET         (MESH_PACKET_VERSION_OFFSET + MESH_PACKET_VERSION_LEN)

#define MESH_CCCD_HANDLE_OFFSET         (1)

#define CONN_HANDLE_INVALID             (0xFFFF)

#ifndef RBC_MESH_GATTS_ATTR_TABLE_SIZE
    #define RBC_MESH_GATTS_ATTR_TABLE_SIZE (0x700)
#endif

#define MESH_CHANNEL_MAX                (39)
#define MESH_ADV_INT_MIN                (5)
#define MESH_ADV_INT_MAX                (60000)
/*****************************************************************************
* Local Type Definitions
*****************************************************************************/

typedef struct
{
    uint8_t value_count;
    uint16_t service_handle;
    ble_gatts_char_handles_t ble_md_char_handles;
} mesh_srv_t;



/*****************************************************************************
* Static globals
*****************************************************************************/
static mesh_srv_t g_mesh_service = {0, 0, {0}};

static const ble_uuid128_t mesh_base_uuid = {{0x1E, 0xCD, 0x00, 0x00,
                                            0x8C, 0xB9, 0xA8, 0x8B,
                                            0x82, 0xD8, 0x51, 0xFD,
                                            0xA1, 0x77, 0x1E, 0x2A}};
static uint8_t mesh_base_uuid_type;

static bool is_initialized = false;

static uint16_t g_active_conn_handle = CONN_HANDLE_INVALID;

/*****************************************************************************
* Static functions
*****************************************************************************/
/**
* @brief Add metadata GATT characteristic to the Mesh GATT service. Part of
*   the initialization procedure.
*/
static uint32_t mesh_md_char_add(mesh_metadata_char_t* metadata)
{
    if (metadata->mesh_channel > MESH_CHANNEL_MAX ||
        metadata->mesh_adv_int_ms < MESH_ADV_INT_MIN ||
        metadata->mesh_adv_int_ms > MESH_ADV_INT_MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    /* cccd for metadata char */
    ble_gatts_attr_md_t cccd_md;

    memset(&cccd_md, 0, sizeof(cccd_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);
    cccd_md.vloc = BLE_GATTS_VLOC_STACK;

    /* metadata char */
    ble_gatts_char_md_t ble_char_md;

    memset(&ble_char_md, 0, sizeof(ble_char_md));

    ble_char_md.char_props.read = 1;
    ble_char_md.char_props.notify = 1;

    ble_char_md.p_char_pf = NULL;
    ble_char_md.p_cccd_md = &cccd_md;
    ble_char_md.p_char_user_desc = NULL;
    ble_char_md.p_user_desc_md = NULL;

    /* ATT metadata */
    ble_gatts_attr_md_t ble_attr_md;

    memset(&ble_attr_md, 0, sizeof(ble_attr_md));

    ble_attr_md.read_perm.lv = 1;
    ble_attr_md.read_perm.sm = 1;
    ble_attr_md.write_perm.lv = 1;
    ble_attr_md.write_perm.sm = 1;

    ble_attr_md.rd_auth = 0;
    ble_attr_md.wr_auth = 0;
    ble_attr_md.vlen = 0;
    ble_attr_md.vloc = BLE_GATTS_VLOC_STACK;

    /* ble characteristic UUID */
    ble_uuid_t ble_uuid;

    ble_uuid.type = mesh_base_uuid_type;
    ble_uuid.uuid = MESH_MD_CHAR_UUID;

    /* metadata contents */
    uint8_t value_array[MESH_MD_CHAR_LEN];

    memcpy(&value_array[MESH_MD_CHAR_AA_OFFSET],
        &metadata->mesh_access_addr,
        sizeof(metadata->mesh_access_addr));

    memcpy(&value_array[MESH_MD_CHAR_ADV_INT_OFFSET],
        &metadata->mesh_adv_int_ms,
        sizeof(metadata->mesh_adv_int_ms));

    memcpy(&value_array[MESH_MD_CHAR_COUNT_OFFSET],
        &metadata->mesh_value_count,
        sizeof(metadata->mesh_value_count));

    memcpy(&value_array[MESH_MD_CHAR_CH_OFFSET],
        &metadata->mesh_channel,
        sizeof(metadata->mesh_channel));


    /* ble attribute */
    ble_gatts_attr_t ble_attr;

    memset(&ble_attr, 0, sizeof(ble_attr));

    ble_attr.init_len = MESH_MD_CHAR_LEN;
    ble_attr.init_offs = 0;
    ble_attr.max_len = MESH_MD_CHAR_LEN;
    ble_attr.p_uuid = &ble_uuid;
    ble_attr.p_value = value_array;
    ble_attr.p_attr_md = &ble_attr_md;

    /* add characteristic */
    uint32_t error_code = sd_ble_gatts_characteristic_add(
        g_mesh_service.service_handle,
        &ble_char_md,
        &ble_attr,
        &g_mesh_service.ble_md_char_handles);

    if (error_code != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }



    return NRF_SUCCESS;
}

/**
* @brief Add a mesh value GATT characterisitic to the mesh GATT service.
*   Part of the initialization procedure.
*/
static uint32_t mesh_value_char_add(uint8_t index)
{
    if (index >= MAX_VALUE_COUNT)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    /* metadata presentation format */
    ble_gatts_char_pf_t ble_char_pf;

    memset(&ble_char_pf, 0, sizeof(ble_char_pf));

    ble_char_pf.name_space = BLE_GATT_CPF_NAMESPACE_BTSIG;
    ble_char_pf.exponent = 0;
    ble_char_pf.format = BLE_GATT_CPF_FORMAT_UINT8;
    ble_char_pf.desc = index + 1; /* trickle instance id/handle */


    /* BLE GATT metadata */
    ble_gatts_char_md_t ble_char_md;

    memset(&ble_char_md, 0, sizeof(ble_char_md));

    ble_char_md.p_char_pf = &ble_char_pf;
    ble_char_md.char_props.read = 1;
    ble_char_md.char_props.write_wo_resp = 1;
    ble_char_md.char_props.notify = 1;

    ble_char_md.p_cccd_md = NULL;
    ble_char_md.p_sccd_md = NULL;
    ble_char_md.p_char_user_desc = NULL;
    ble_char_md.p_user_desc_md = NULL;

    /* ATT metadata */

    ble_gatts_attr_md_t ble_attr_md;

    memset(&ble_attr_md, 0, sizeof(ble_attr_md));

    /* No security is required whatsoever, needs to be changed when encryption
        is added */
    ble_attr_md.read_perm.lv = 1;
    ble_attr_md.read_perm.sm = 1;
    ble_attr_md.write_perm.lv = 1;
    ble_attr_md.write_perm.sm = 1;

    ble_attr_md.vloc = BLE_GATTS_VLOC_STACK;
    ble_attr_md.rd_auth = 0;
    ble_attr_md.wr_auth = 0;
    ble_attr_md.vlen = 1;

    /* ble characteristic UUID */
    ble_uuid_t ble_uuid;

    ble_uuid.type = mesh_base_uuid_type;
    ble_uuid.uuid = MESH_VALUE_CHAR_UUID;

    /* ble attribute */
    ble_gatts_attr_t ble_attr;
    uint8_t default_value = 0;

    memset(&ble_attr, 0, sizeof(ble_attr));

    ble_attr.init_len = 1;
    ble_attr.init_offs = 0;
    ble_attr.max_len = MAX_VALUE_LENGTH;
    ble_attr.p_attr_md = &ble_attr_md;
    ble_attr.p_uuid = &ble_uuid;
    ble_attr.p_value = &default_value;


    /* add to service */
    ble_gatts_char_handles_t ble_value_char_handles;

    uint32_t error_code = sd_ble_gatts_characteristic_add(
        g_mesh_service.service_handle,
        &ble_char_md,
        &ble_attr,
        &ble_value_char_handles);

    if (error_code != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    vh_set_gatts_handle(index + 1, ble_value_char_handles.value_handle);
    return NRF_SUCCESS;
}

/*****************************************************************************
* Interface functions
*****************************************************************************/

uint32_t mesh_srv_init(uint8_t mesh_value_count,
    uint32_t access_address, uint8_t channel, uint32_t adv_int_ms)
{
    if (mesh_value_count > MAX_VALUE_COUNT || mesh_value_count == 0)
    {
        return NRF_ERROR_INVALID_PARAM;
    }

    if (is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    is_initialized = true;

    ble_enable_params_t ble_enable_params;
    ble_enable_params.gatts_enable_params.attr_tab_size = RBC_MESH_GATTS_ATTR_TABLE_SIZE;
    ble_enable_params.gatts_enable_params.service_changed = 0;

    uint32_t error_code = sd_ble_enable(&ble_enable_params);
    if (error_code != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    g_mesh_service.value_count = mesh_value_count;

    /* add the mesh base UUID */
    ble_uuid_t ble_srv_uuid;
    mesh_base_uuid_type = BLE_UUID_TYPE_UNKNOWN;

    error_code = sd_ble_uuid_vs_add(&mesh_base_uuid, &mesh_base_uuid_type);
    if (error_code != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    ble_srv_uuid.type = mesh_base_uuid_type;
    ble_srv_uuid.uuid = MESH_SRV_UUID;

    error_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
        &ble_srv_uuid, &g_mesh_service.service_handle);

    if (error_code != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    /* Add metadata characteristic */
    mesh_metadata_char_t mesh_metadata;
    mesh_metadata.mesh_access_addr = access_address;
    mesh_metadata.mesh_adv_int_ms = adv_int_ms;
    mesh_metadata.mesh_channel = channel;
    mesh_metadata.mesh_value_count = mesh_value_count;

    error_code = mesh_md_char_add(&mesh_metadata);
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }
    
    /* add characteristics to mesh service */
    for (uint8_t i = 0; i < g_mesh_service.value_count; ++i)
    {
        error_code = mesh_value_char_add(i);

        if (error_code != NRF_SUCCESS)
        {
            return error_code;
        }
    }

    return NRF_SUCCESS;
}


uint32_t mesh_srv_char_val_set(uint8_t index, uint8_t* data, uint16_t len)
{
    if (!is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (index > g_mesh_service.value_count || index == 0)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    if (len > MAX_VALUE_LENGTH)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    uint32_t error_code = 0;

    uint8_t gatts_handle;
    vh_get_gatts_handle(index, &gatts_handle);

    ble_gatts_value_t cccd_val_struct;
    uint8_t cccd_val[2];
    cccd_val[0] = 0;
    cccd_val_struct.p_value = cccd_val;
    cccd_val_struct.len = 2;
    if (g_active_conn_handle != CONN_HANDLE_INVALID)
    {
        error_code = sd_ble_gatts_value_get(
                g_active_conn_handle, 
                gatts_handle + MESH_CCCD_HANDLE_OFFSET, 
                &cccd_val_struct);

        if (error_code != NRF_SUCCESS)
        {
            if (error_code == BLE_ERROR_INVALID_CONN_HANDLE)
            {
                g_active_conn_handle = CONN_HANDLE_INVALID;
            }
            else if (error_code == BLE_ERROR_GATTS_SYS_ATTR_MISSING)
            {
                /* cccd hasn't been initiated yet, can assume it's not active */
                cccd_val[0] = 0;
            }
            else
            {
                return NRF_ERROR_INTERNAL;
            }
        }
    }
    /* notify the connected central node, if any */
    if (g_active_conn_handle != CONN_HANDLE_INVALID && cccd_val[0] != 0)
    {
        ble_gatts_hvx_params_t notify_params;
        notify_params.handle = gatts_handle;
        notify_params.offset = 0;
        notify_params.p_data = data;
        notify_params.p_len = &len;
        notify_params.type = BLE_GATT_HVX_NOTIFICATION;
        error_code = sd_ble_gatts_hvx(g_active_conn_handle, &notify_params);
        if (error_code != NRF_SUCCESS)
        {
            if (error_code == BLE_ERROR_INVALID_CONN_HANDLE)
            {
                g_active_conn_handle = CONN_HANDLE_INVALID;
            }
            else if (error_code == BLE_GATTS_EVT_SYS_ATTR_MISSING)
            {
                uint32_t flags = BLE_GATTS_SYS_ATTR_FLAG_SYS_SRVCS |
                                 BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS;

                sd_ble_gatts_sys_attr_set(g_active_conn_handle, NULL, 0, flags);
            }
            else
            {
                return NRF_ERROR_INTERNAL;
            }
        }
    }
    else
    {
        ble_gatts_value_t gatts_value_set;
        gatts_value_set.len = len;
        gatts_value_set.offset = 0;
        gatts_value_set.p_value = data;
        error_code = sd_ble_gatts_value_set(
						BLE_CONN_HANDLE_INVALID,
                        gatts_handle,
            &gatts_value_set);

        if (error_code != NRF_SUCCESS)
        {
            return NRF_ERROR_INTERNAL;
        }
    }

    return NRF_SUCCESS;
}

uint32_t mesh_srv_char_val_get(uint8_t index, uint8_t* data, uint16_t* len)
{
    if (!is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (index > g_mesh_service.value_count || index == 0)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    *len = MAX_VALUE_LENGTH;
    uint8_t gatts_handle;
    vh_get_gatts_handle(index, &gatts_handle);

    ble_gatts_value_t ble_gatts_value_get;
    ble_gatts_value_get.len = *len;
    ble_gatts_value_get.offset = 0;
    ble_gatts_value_get.p_value = data;
    uint32_t error_code = sd_ble_gatts_value_get(
				BLE_CONN_HANDLE_INVALID,
        gatts_handle,
        &ble_gatts_value_get);
    
    *len = ble_gatts_value_get.len;

    if (error_code != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    return NRF_SUCCESS;
}

uint32_t mesh_srv_char_md_get(mesh_metadata_char_t* metadata)
{
    if (!is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint8_t data_array[MESH_MD_CHAR_LEN];
    uint16_t len = MESH_MD_CHAR_LEN;
    ble_gatts_value_t ble_gatts_value_get;
    ble_gatts_value_get.len = len;
    ble_gatts_value_get.offset = 0;
    ble_gatts_value_get.p_value = data_array;

    uint32_t error_code = sd_ble_gatts_value_get(
        BLE_CONN_HANDLE_INVALID,
				g_mesh_service.ble_md_char_handles.value_handle,
				&ble_gatts_value_get);

		len = ble_gatts_value_get.len;

    if (error_code != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    if (len != MESH_MD_CHAR_LEN)
    {
        return NRF_ERROR_INTERNAL;
    }

    memcpy(&metadata->mesh_access_addr,
        &data_array[MESH_MD_CHAR_AA_OFFSET],
        sizeof(metadata->mesh_access_addr));

    memcpy(&metadata->mesh_adv_int_ms,
        &data_array[MESH_MD_CHAR_ADV_INT_OFFSET],
        sizeof(metadata->mesh_adv_int_ms));

    memcpy(&metadata->mesh_channel,
        &data_array[MESH_MD_CHAR_CH_OFFSET],
        sizeof(metadata->mesh_channel));

    memcpy(&metadata->mesh_value_count,
        &data_array[MESH_MD_CHAR_COUNT_OFFSET],
        sizeof(metadata->mesh_value_count));

    return NRF_SUCCESS;
}

uint32_t mesh_srv_conn_handle_update(uint16_t conn_handle)
{
    g_active_conn_handle = conn_handle;

    return NRF_SUCCESS;
}

uint32_t mesh_srv_gatts_evt_write_handle(ble_gatts_evt_write_t* evt)
{
    if (!is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (evt->context.srvc_handle != g_mesh_service.service_handle)
    {
        return NRF_ERROR_FORBIDDEN;
    }

    for (uint8_t i = 0; i < g_mesh_service.value_count; ++i)
    {
        uint8_t gatts_handle;
        vh_get_gatts_handle(i + 1, &gatts_handle);

        if (gatts_handle == evt->handle)
        {
            vh_data_status_t update_status = vh_local_update(i + 1);
            if (update_status == VH_DATA_STATUS_UNKNOWN)
            {
                return NRF_ERROR_INTERNAL;
            }

            /* propagate to application */
            rbc_mesh_event_t update_evt;
            update_evt.event_type = ((update_status == VH_DATA_STATUS_NEW)?
                RBC_MESH_EVENT_TYPE_NEW_VAL :
                RBC_MESH_EVENT_TYPE_UPDATE_VAL);

            update_evt.data_len = evt->len;
            update_evt.value_handle = i + 1;
            update_evt.data = evt->data;

            rbc_mesh_event_handler(&update_evt);

#ifdef RBC_MESH_SERIAL
            mesh_aci_rbc_event_handler(&update_evt);
#endif
            return NRF_SUCCESS;
        }
    }
    return NRF_ERROR_INVALID_ADDR;
}

