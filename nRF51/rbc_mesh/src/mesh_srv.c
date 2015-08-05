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

#define CONN_HANDLE_INVALID             (0xFFFF)

#define MESH_VERSION_SCHEME_LOLLIPOP    (1)

#if MESH_VERSION_SCHEME_LOLLIPOP
    #define MESH_VALUE_LOLLIPOP_LIMIT       (200)
#endif    

#define RBC_MESH_GATTS_ATTR_TABLE_SIZE_DEFAULT (0x800)
/*****************************************************************************
* Local Type Definitions
*****************************************************************************/

typedef struct
{
    uint8_t value_count;
    mesh_char_metadata_t* char_metadata;
    uint16_t service_handle;
    ble_gatts_char_handles_t ble_md_char_handles;
} mesh_srv_t;



/*****************************************************************************
* Static globals
*****************************************************************************/
static mesh_srv_t g_mesh_service = {0, NULL, 0};

static const ble_uuid128_t mesh_base_uuid = {0x1E, 0xCD, 0x00, 0x00,
                                            0x8C, 0xB9, 0xA8, 0x8B,
                                            0x82, 0xD8, 0x51, 0xFD,
                                            0xA1, 0x77, 0x1E, 0x2A};
static uint8_t mesh_base_uuid_type;

static bool is_initialized = false;

static uint16_t g_active_conn_handle = CONN_HANDLE_INVALID;

/*****************************************************************************
* Static functions
*****************************************************************************/
static void version_increase(uint16_t* version)
{
#if MESH_VERSION_SCHEME_LOLLIPOP    
    if (*version == UINT16_MAX)
    {
        *version = MESH_VALUE_LOLLIPOP_LIMIT;
    }
    else
    {
        (*version)++;
    }
#else
    (*version)++;
#endif    
}
                                            
                                            
/**
* @brief Add metadata GATT characteristic to the Mesh GATT service. Part of
*   the initialization procedure.
*/
static uint32_t mesh_md_char_add(mesh_metadata_char_t* metadata)
{
    /**@TODO: put ranges in public #defines */
    if (metadata->mesh_channel > 39 ||
        metadata->mesh_adv_int_ms < 5 ||
        metadata->mesh_adv_int_ms > 60000)
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

    g_mesh_service.char_metadata[index].char_value_handle =  ble_value_char_handles.value_handle;

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
    ble_enable_params.gatts_enable_params.attr_tab_size = RBC_MESH_GATTS_ATTR_TABLE_SIZE_DEFAULT;
    ble_enable_params.gatts_enable_params.service_changed = 0;

    uint32_t error_code = sd_ble_enable(&ble_enable_params);
    if (error_code != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    g_mesh_service.value_count = mesh_value_count;

    ble_uuid_t ble_srv_uuid;
    mesh_base_uuid_type = BLE_UUID_TYPE_UNKNOWN;
    /* add the mesh base UUID */

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

    uint32_t md_len = sizeof(mesh_char_metadata_t) * g_mesh_service.value_count;

    /* allocate metadata array */
    g_mesh_service.char_metadata = (mesh_char_metadata_t*) malloc(md_len);
    memset(g_mesh_service.char_metadata, 0, md_len);

    /* add characteristics to mesh service */
    for (uint8_t i = 0; i < g_mesh_service.value_count; ++i)
    {
        error_code = mesh_value_char_add(i);

        if (error_code != NRF_SUCCESS)
        {
            return error_code;
        }
    }

    trickle_setup(1000 * adv_int_ms, 600, 3);

    return NRF_SUCCESS;
}


uint32_t mesh_srv_char_val_set(uint8_t index, uint8_t* data, uint16_t len, bool update_sender)
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

    mesh_char_metadata_t* ch_md = &g_mesh_service.char_metadata[index - 1];

    /* this is now a new version of this data, signal to the rest of the mesh */
    version_increase(&ch_md->version_number);

    bool first_time =
        (ch_md->flags &
        (1 << MESH_MD_FLAGS_USED_POS)) == 0;

    if (first_time)
    {
        ch_md->flags |=
            (1 << MESH_MD_FLAGS_INITIALIZED_POS) |
            (1 << MESH_MD_FLAGS_USED_POS);
        trickle_init(&ch_md->trickle);
    }
    else
    {
        trickle_rx_inconsistent(&ch_md->trickle);
    }

    if (update_sender || first_time)
    {
        ble_gap_addr_t my_addr;
        sd_ble_gap_address_get(&my_addr);
        memcpy(&ch_md->last_sender_addr, &my_addr, sizeof(ble_gap_addr_t));
        ch_md->flags |= (1 << MESH_MD_FLAGS_IS_ORIGIN_POS);
    }

    /* notify the connected central node, if any */
    if (g_active_conn_handle != CONN_HANDLE_INVALID)
    {
        ble_gatts_hvx_params_t notify_params;
        notify_params.handle = ch_md->char_value_handle;
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
                sd_ble_gatts_sys_attr_set(g_active_conn_handle, NULL, 0, BLE_GATTS_SYS_ATTR_FLAG_USR_SRVCS);
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
            ch_md->char_value_handle,
            &gatts_value_set);

        if (error_code != NRF_SUCCESS)
        {
            return NRF_ERROR_INTERNAL;
        }
    }

    return NRF_SUCCESS;
}

uint32_t mesh_srv_char_val_get(uint8_t index, uint8_t* data, uint16_t* len, ble_gap_addr_t* origin_addr)
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
    ble_gatts_value_t ble_gatts_value_get;
    ble_gatts_value_get.len = *(uint16_t*)len;
    ble_gatts_value_get.offset = 0;
    ble_gatts_value_get.p_value = data;
    uint32_t error_code = sd_ble_gatts_value_get(
				BLE_CONN_HANDLE_INVALID,
        g_mesh_service.char_metadata[index - 1].char_value_handle,
        &ble_gatts_value_get);

		*len = ble_gatts_value_get.len;

    if (error_code != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    if (origin_addr != NULL)
    {
        memcpy(origin_addr,
            &g_mesh_service.char_metadata[index - 1].last_sender_addr,
            sizeof(ble_gap_addr_t));
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

uint32_t mesh_srv_get_next_processing_time(uint64_t* time)
{
    if (!is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    bool anything_to_process = false;
    *time = UINT64_MAX;

    for (uint8_t i = 0; i < g_mesh_service.value_count; ++i)
    {
        if ((g_mesh_service.char_metadata[i].flags & (1 << MESH_MD_FLAGS_USED_POS)) == 0)
            continue;

        uint64_t temp_time = trickle_next_processing_get(&g_mesh_service.char_metadata[i].trickle);

        if (temp_time < *time)
        {
            anything_to_process = true;
            *time = temp_time;
        }
    }
    if (!anything_to_process)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    return NRF_SUCCESS;
}

uint32_t mesh_srv_packet_process(packet_t* packet)
{
    if (!is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    uint32_t error_code;

    uint8_t handle = packet->data[MESH_PACKET_HANDLE_OFFSET];
    uint16_t version = (packet->data[MESH_PACKET_VERSION_OFFSET] |
                    (((uint16_t) packet->data[MESH_PACKET_VERSION_OFFSET + 1]) << 8));
    uint8_t* data = &packet->data[MESH_PACKET_DATA_OFFSET];
    uint16_t data_len = packet->length - MESH_PACKET_DATA_OFFSET;

    if (data_len > MAX_VALUE_LENGTH)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    if (handle > g_mesh_service.value_count || handle == 0)
    {
        return NRF_ERROR_INVALID_ADDR;
    }


    mesh_char_metadata_t* ch_md = &g_mesh_service.char_metadata[handle - 1];

    bool uninitialized = !(ch_md->flags & (1 << MESH_MD_FLAGS_INITIALIZED_POS));

    if (uninitialized)
    {
        trickle_init(&ch_md->trickle);
    }

    if (ch_md->version_number != version)
    {
        trickle_rx_inconsistent(&ch_md->trickle);
    }

#if MESH_VERSION_SCHEME_LOLLIPOP    
    /* new version */
    uint16_t separation = (version >= ch_md->version_number)?
        (version - ch_md->version_number) :
        ((version - MESH_VALUE_LOLLIPOP_LIMIT) - (ch_md->version_number - MESH_VALUE_LOLLIPOP_LIMIT) - MESH_VALUE_LOLLIPOP_LIMIT);
#endif
    
    if (version == ch_md->version_number)
    {
        /* check for conflicting data */
        uint16_t old_len = MAX_VALUE_LENGTH;

        error_code = mesh_srv_char_val_get(handle, NULL, &old_len, NULL);
        if (error_code != NRF_SUCCESS)
        {
            return error_code;
        }

        volatile bool conflicting = false;

        if (packet->rx_crc != ch_md->crc &&
            !(ch_md->flags & (1 << MESH_MD_FLAGS_IS_ORIGIN_POS)))
        {
            conflicting = true;
        }
        else if (old_len != data_len)
        {
            conflicting = true;
        }


        if (conflicting)
        {
            TICK_PIN(7);
            rbc_mesh_event_t conflicting_evt;

            conflicting_evt.event_type = RBC_MESH_EVENT_TYPE_CONFLICTING_VAL;

            conflicting_evt.data_len = data_len;
            conflicting_evt.value_handle = handle;

            conflicting_evt.data = data;
            memcpy(&conflicting_evt.originator_address, &packet->sender, sizeof(ble_gap_addr_t));

            trickle_rx_inconsistent(&ch_md->trickle);

            rbc_mesh_event_handler(&conflicting_evt);
#ifdef RBC_MESH_SERIAL
            mesh_aci_rbc_event_handler(&conflicting_evt);
#endif
        }
        else
        {
            trickle_rx_consistent(&ch_md->trickle);
        }

    }
#if MESH_VERSION_SCHEME_LOLLIPOP
    else if ((ch_md->version_number < MESH_VALUE_LOLLIPOP_LIMIT && version > ch_md->version_number) ||
        (ch_md->version_number >= MESH_VALUE_LOLLIPOP_LIMIT && version >= MESH_VALUE_LOLLIPOP_LIMIT && separation < (UINT16_MAX - MESH_VALUE_LOLLIPOP_LIMIT)/2) ||
        uninitialized)
#else
    else if (ch_md->version_number < version)
#endif
    {
        /* update value */
        mesh_srv_char_val_set(handle, data, data_len, false);
        ch_md->flags |= (1 << MESH_MD_FLAGS_INITIALIZED_POS);
        ch_md->flags &= ~(1 << MESH_MD_FLAGS_IS_ORIGIN_POS);
        ch_md->version_number = version;

        /* Manually set originator address */
        memcpy(&ch_md->last_sender_addr, &packet->sender, sizeof(ble_gap_addr_t));

        rbc_mesh_event_t update_evt;
        update_evt.event_type = ((uninitialized)?
            RBC_MESH_EVENT_TYPE_NEW_VAL :
            RBC_MESH_EVENT_TYPE_UPDATE_VAL);
        update_evt.data_len = data_len;
        update_evt.value_handle = handle;

        update_evt.data = data;
        memcpy(&update_evt.originator_address, &packet->sender, sizeof(ble_gap_addr_t));

        rbc_mesh_event_handler(&update_evt);
#ifdef RBC_MESH_SERIAL
            mesh_aci_rbc_event_handler(&update_evt);
#endif
    }


    ch_md->crc = packet->rx_crc;

    return NRF_SUCCESS;
}

uint32_t mesh_srv_conn_handle_update(uint16_t conn_handle)
{
    g_active_conn_handle = conn_handle;

    return NRF_SUCCESS;
}

uint32_t mesh_srv_packet_assemble(packet_t* packet,
    uint16_t packet_max_len,
    bool* has_anything_to_send)
{
    *has_anything_to_send = false;
    if (!is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint32_t error_code;

    for (uint8_t i = 0; i < g_mesh_service.value_count; ++i)
    {
        mesh_char_metadata_t* md_ch = &g_mesh_service.char_metadata[i];

        if ((md_ch->flags & (1 << MESH_MD_FLAGS_USED_POS)) == 0)
            continue;

        bool do_trickle_tx = false;
        trickle_step(&md_ch->trickle, &do_trickle_tx);


        if (do_trickle_tx && !(*has_anything_to_send))
        {
            trickle_register_tx(&md_ch->trickle);
            uint8_t data[MAX_VALUE_LENGTH];
            uint16_t len = MAX_VALUE_LENGTH;

            error_code = mesh_srv_char_val_get(i + 1, data, &len, NULL);

            if (error_code != NRF_SUCCESS)
            {
                return error_code;
            }

            packet->data[MESH_PACKET_HANDLE_OFFSET] = i + 1;
            packet->data[MESH_PACKET_VERSION_OFFSET] =
                (md_ch->version_number & 0xFF);
            packet->data[MESH_PACKET_VERSION_OFFSET + 1] =
                ((md_ch->version_number >> 8) & 0xFF);

            memcpy(&packet->data[MESH_PACKET_DATA_OFFSET], data, len);
            packet->length = len + MESH_PACKET_DATA_OFFSET;

            memcpy(&packet->sender, &md_ch->last_sender_addr, sizeof(md_ch->last_sender_addr));

            /**@TODO: Add multiple trickle messages in one packet */

            *has_anything_to_send = true;
            //break;
        }
    }

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
        if (g_mesh_service.char_metadata[i].char_value_handle == evt->handle)
        {
            mesh_char_metadata_t* ch_md = &g_mesh_service.char_metadata[i];
            bool uninitialized = !(ch_md->flags & (1 << MESH_MD_FLAGS_INITIALIZED_POS));

            if (uninitialized)
            {
                trickle_init(&ch_md->trickle);
            }

            ch_md->flags |=
                (1 << MESH_MD_FLAGS_INITIALIZED_POS) |
                (1 << MESH_MD_FLAGS_USED_POS) |
                (1 << MESH_MD_FLAGS_IS_ORIGIN_POS);

            version_increase(&ch_md->version_number);
            
            trickle_rx_inconsistent(&ch_md->trickle);
            ble_gap_addr_t my_addr;
            sd_ble_gap_address_get(&my_addr);
            memcpy(&ch_md->last_sender_addr, &my_addr, sizeof(ble_gap_addr_t));

            rbc_mesh_event_t update_evt;
            update_evt.event_type = ((uninitialized)?
                RBC_MESH_EVENT_TYPE_NEW_VAL :
                RBC_MESH_EVENT_TYPE_UPDATE_VAL);

            update_evt.data_len = evt->len;
            update_evt.value_handle = i + 1;
            update_evt.data = evt->data;
            memcpy(&update_evt.originator_address, &my_addr, sizeof(ble_gap_addr_t));

            rbc_mesh_event_handler(&update_evt);
#ifdef RBC_MESH_SERIAL
            mesh_aci_rbc_event_handler(&update_evt);
#endif
            return NRF_SUCCESS;
        }
    }
    return NRF_ERROR_INVALID_ADDR;
}

uint32_t mesh_srv_char_val_enable(uint8_t index)
{
    if (!is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (index > g_mesh_service.value_count || index == 0)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    trickle_init(&g_mesh_service.char_metadata[index - 1].trickle);

    g_mesh_service.char_metadata[index - 1].flags |=
        (1 << MESH_MD_FLAGS_INITIALIZED_POS) |
        (1 << MESH_MD_FLAGS_USED_POS);


    return NRF_SUCCESS;
}

uint32_t mesh_srv_char_val_disable(uint8_t index)
{
    if (!is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (index > g_mesh_service.value_count || index == 0)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    g_mesh_service.char_metadata[index - 1].flags &=
        ~(1 << MESH_MD_FLAGS_USED_POS);

    return NRF_SUCCESS;
}
