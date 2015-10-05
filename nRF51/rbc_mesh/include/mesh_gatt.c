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
#include "mesh_gatt.h"

typedef struct
{
    uint16_t service_handle;
    ble_gatts_char_handles_t ble_md_char_handles;
    ble_gatts_char_handles_t ble_val_char_handles;
} mesh_srv_t;
/*****************************************************************************
* Static globals
*****************************************************************************/
static mesh_srv_t m_mesh_service = {0, 0, {0}};

static const ble_uuid128_t m_mesh_base_uuid = {{0x1E, 0xCD, 0x00, 0x00,
                                            0x8C, 0xB9, 0xA8, 0x8B,
                                            0x82, 0xD8, 0x51, 0xFD,
                                            0xA1, 0x77, 0x1E, 0x2A}};
static uint8_t m_mesh_base_uuid_type;

static bool m_is_initialized = false;

static uint16_t m_active_conn_handle = CONN_HANDLE_INVALID;

/*****************************************************************************
* Static functions
*****************************************************************************/
static uint32_t mesh_md_char_add(mesh_metadata_char_t* metadata)
{
    if (metadata->mesh_channel > MESH_CHANNEL_MAX ||
        metadata->mesh_interval_min_ms < MESH_INTERVAL_MIN_MAX ||
        metadata->mesh_interval_min_ms > MESH_INTERVAL_MIN_MAX)
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

    ble_uuid.type = BLE_UUID_TYPE_BLE;
    ble_uuid.uuid = MESH_MD_CHAR_UUID;

    /* metadata contents */
    uint8_t value_array[MESH_MD_CHAR_LEN];

    memcpy(&value_array[MESH_MD_CHAR_AA_OFFSET],
        &metadata->mesh_access_addr,
        sizeof(metadata->mesh_access_addr));

    memcpy(&value_array[MESH_MD_CHAR_ADV_INT_OFFSET],
        &metadata->mesh_interval_min_ms,
        sizeof(metadata->mesh_interval_min_ms));

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
        m_mesh_service.service_handle,
        &ble_char_md,
        &ble_attr,
        &m_mesh_service.ble_md_char_handles);

    if (error_code != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    return NRF_SUCCESS;
}

static uint32_t mesh_value_char_add(void)
{
    /* BLE GATT metadata */
    ble_gatts_char_md_t ble_char_md;

    memset(&ble_char_md, 0, sizeof(ble_char_md));

    ble_char_md.p_char_pf = &ble_char_pf;
    ble_char_md.char_props.write_wo_resp = 1;
    ble_char_md.char_props.notify = 1;

    ble_char_md.p_cccd_md = NULL;
    ble_char_md.p_sccd_md = NULL;
    ble_char_md.p_char_user_desc = NULL;
    ble_char_md.p_user_desc_md = NULL;

    /* ATT metadata */

    ble_gatts_attr_md_t ble_attr_md;

    memset(&ble_attr_md, 0, sizeof(ble_attr_md));

    /* No security is required */
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

    ble_uuid.type = m_mesh_base_uuid_type;
    ble_uuid.uuid = MESH_VALUE_CHAR_UUID;

    /* ble attribute */
    ble_gatts_attr_t ble_attr;
    uint8_t default_value = 0;

    memset(&ble_attr, 0, sizeof(ble_attr));

    ble_attr.init_len = 1;
    ble_attr.init_offs = 0;
    ble_attr.max_len = MESH_VALUE_CHAR_LEN;
    ble_attr.p_attr_md = &ble_attr_md;
    ble_attr.p_uuid = &ble_uuid;
    ble_attr.p_value = &default_value;

    /* add to service */
    uint32_t error_code = sd_ble_gatts_characteristic_add(
            m_mesh_service.service_handle,
            &ble_char_md,
            &ble_attr,
            &m_mesh_service.ble_val_char_handles);

    if (error_code != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }

    return NRF_SUCCESS;
}
/*****************************************************************************
* Interface functions
*****************************************************************************/
uint32_t mesh_gatt_init(uint32_t access_address, uint8_t channel, uint32_t interval_min_ms)
{
    uint32_t error_code;
    mesh_metadata_char_t md_char;
    md_char.mesh_access_addr = access_address;
    md_char.mesh_interval_min_ms = interval_min_ms;
    md_char.mesh_channel = channel;

    error_code = mesh_md_char_add(&md_char);
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    error_code = mesh_value_char_add();
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    m_is_initialized = true;

    return NRF_SUCCESS;
}

uint32_t mesh_gatt_value_set(rbc_mesh_value_handle_t handle, uint8_t* data, uint8_t length)
{
    return NRF_SUCCESS;
}

uint32_t mesh_gatt_conn_handle_update(uint16_t conn_handle)
{
    m_active_conn_handle = conn_handle;

    return NRF_SUCCESS;
}

uint32_t mesh_gatt_evt_write_handle(ble_gatts_evt_write_t* evt)
{
    return NRF_SUCCESS;
}



