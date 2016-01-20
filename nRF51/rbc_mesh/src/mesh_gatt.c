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
#include "mesh_gatt.h"

/* this module will automatically disabled if the framework is setup to don't use the SD */
#ifdef SOFTDEVICE_PRESENT


#include "rbc_mesh.h"
#include "version_handler.h"
#include "transport_control.h"
#include "app_error.h"

#include "ble_gatts.h"
#include "ble_err.h"
#include <string.h>

extern uint32_t rbc_mesh_event_push(rbc_mesh_event_t* p_event);

typedef struct
{
    uint16_t service_handle;
    bool notification_enabled;
    ble_gatts_char_handles_t ble_md_char_handles;
    ble_gatts_char_handles_t ble_val_char_handles;
} mesh_srv_t;
/*****************************************************************************
* Static globals
*****************************************************************************/
static mesh_srv_t m_mesh_service = {0, false, {0}, {0}};

static const ble_uuid128_t m_mesh_base_uuid = {{0x1E, 0xCD, 0x00, 0x00,
                                            0x8C, 0xB9, 0xA8, 0x8B,
                                            0x82, 0xD8, 0x51, 0xFD,
                                            0xA1, 0x77, 0x1E, 0x2A}};
static uint8_t m_mesh_base_uuid_type;

static uint16_t m_active_conn_handle = CONN_HANDLE_INVALID;

typedef enum
{
    MESH_GATT_EVT_OPCODE_DATA = 0x00,
    MESH_GATT_EVT_OPCODE_FLAG_SET = 0x01,
    MESH_GATT_EVT_OPCODE_FLAG_REQ = 0x02,
    MESH_GATT_EVT_OPCODE_CMD_RSP  = 0x11,
    MESH_GATT_EVT_OPCODE_FLAG_RSP = 0x12,
} mesh_gatt_evt_opcode_t;

typedef enum
{
    MESH_GATT_RESULT_SUCCESS = 0x80,
    MESH_GATT_RESULT_ERROR_BUSY = 0xF0,
    MESH_GATT_RESULT_ERROR_NOT_FOUND = 0xF1,
    MESH_GATT_RESULT_ERROR_INVALID_HANDLE = 0xF2,
    MESH_GATT_RESULT_ERROR_UNKNOWN_FLAG = 0xF3,
    MESH_GATT_RESULT_ERROR_INVALID_OPCODE = 0xF4,
} mesh_gatt_result_t;

typedef enum
{
    MESH_GATT_EVT_FLAG_PERSISTENT,
    MESH_GATT_EVT_FLAG_DO_TX
} mesh_gatt_evt_flag_t;

typedef __packed_armcc struct 
{
    rbc_mesh_value_handle_t handle;
    uint8_t flag;
    uint8_t value;
} __packed_gcc gatt_evt_flag_update_t;

typedef __packed_armcc struct 
{
    rbc_mesh_value_handle_t handle;
    uint8_t data_len;
    uint8_t data[RBC_MESH_VALUE_MAX_LEN];
} __packed_gcc gatt_evt_data_update_t;

typedef __packed_armcc struct 
{
    uint8_t opcode;
    uint8_t result;
} __packed_gcc gatt_evt_cmd_rsp_t;

typedef __packed_armcc struct
{
    uint8_t opcode;
    __packed_armcc union {
        gatt_evt_flag_update_t  flag_update;
        gatt_evt_data_update_t  data_update;
        gatt_evt_cmd_rsp_t      cmd_rsp;
    } __packed_gcc param;
} __packed_gcc mesh_gatt_evt_t;

/*****************************************************************************
* Static functions
*****************************************************************************/
static uint32_t mesh_gatt_evt_push(mesh_gatt_evt_t* p_gatt_evt)
{
    if (m_active_conn_handle == CONN_HANDLE_INVALID)
    {
        return BLE_ERROR_INVALID_CONN_HANDLE;
    }
    
    if (!m_mesh_service.notification_enabled)
    {
        return BLE_ERROR_NOT_ENABLED;
    }
    
    uint8_t count;
    if (sd_ble_tx_buffer_count_get(&count) != NRF_SUCCESS)
    {
        return NRF_ERROR_BUSY;
    }
    if (count == 0)
    {
        return BLE_ERROR_NO_TX_BUFFERS;  
    }

    ble_gatts_hvx_params_t hvx_params;
    hvx_params.handle = m_mesh_service.ble_val_char_handles.value_handle;
    hvx_params.type = BLE_GATT_HVX_NOTIFICATION;
    hvx_params.offset = 0;
    uint16_t hvx_len;
    switch (p_gatt_evt->opcode)
    {
        case MESH_GATT_EVT_OPCODE_DATA:
            hvx_len = p_gatt_evt->param.data_update.data_len + 4;
            break;
        case MESH_GATT_EVT_OPCODE_FLAG_SET:
        case MESH_GATT_EVT_OPCODE_FLAG_REQ:
        case MESH_GATT_EVT_OPCODE_FLAG_RSP:
            hvx_len = 5;
            break;
        case MESH_GATT_EVT_OPCODE_CMD_RSP:
            hvx_len = 3;
            break;
        default:
            hvx_len = 1;
    }
    hvx_params.p_len = &hvx_len;
    hvx_params.p_data = (uint8_t*) p_gatt_evt;

    return sd_ble_gatts_hvx(m_active_conn_handle, &hvx_params);
}

static uint32_t mesh_gatt_cmd_rsp_push(mesh_gatt_evt_opcode_t opcode, mesh_gatt_result_t result)
{
    mesh_gatt_evt_t rsp;
    rsp.opcode = MESH_GATT_EVT_OPCODE_CMD_RSP;
    rsp.param.cmd_rsp.opcode = opcode;
    rsp.param.cmd_rsp.result = result;
    return mesh_gatt_evt_push(&rsp);
}

static uint32_t mesh_md_char_add(mesh_metadata_char_t* metadata)
{
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

    ble_uuid.type = m_mesh_base_uuid_type;
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
    ble_attr.max_len = sizeof(mesh_gatt_evt_t);
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
    
    ble_uuid_t ble_srv_uuid;
    ble_srv_uuid.type = BLE_UUID_TYPE_BLE;
    ble_srv_uuid.uuid = MESH_SRV_UUID;

    error_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
        &ble_srv_uuid, &m_mesh_service.service_handle);
    if (error_code != NRF_SUCCESS)
    {
        return NRF_ERROR_INTERNAL;
    }
    
    error_code = sd_ble_uuid_vs_add(&m_mesh_base_uuid, &m_mesh_base_uuid_type);
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }
    
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

    return NRF_SUCCESS;
}

uint32_t mesh_gatt_value_set(rbc_mesh_value_handle_t handle, uint8_t* data, uint8_t length)
{
    if (length > RBC_MESH_VALUE_MAX_LEN)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    if (m_active_conn_handle != CONN_HANDLE_INVALID)
    {
        mesh_gatt_evt_t gatt_evt;
        gatt_evt.opcode = MESH_GATT_EVT_OPCODE_DATA;
        gatt_evt.param.data_update.handle = handle;
        gatt_evt.param.data_update.data_len = length;
        memcpy(gatt_evt.param.data_update.data, data, length);

        return mesh_gatt_evt_push(&gatt_evt);
    }
    else
    {
        return BLE_ERROR_INVALID_CONN_HANDLE;
    }
}

void mesh_gatt_sd_ble_event_handle(ble_evt_t* p_ble_evt)
{
    if (p_ble_evt->header.evt_id == BLE_GATTS_EVT_WRITE)
    {
        if (p_ble_evt->evt.gatts_evt.params.write.handle == m_mesh_service.ble_val_char_handles.value_handle)
        {
            mesh_gatt_evt_t* p_gatt_evt = (mesh_gatt_evt_t*) p_ble_evt->evt.gatts_evt.params.write.data;
            switch ((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode)
            {
                case MESH_GATT_EVT_OPCODE_DATA:
                    {
                        if (p_gatt_evt->param.data_update.handle == RBC_MESH_INVALID_HANDLE)
                        {
                            mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_ERROR_INVALID_HANDLE);
                            break;
                        }
                        vh_data_status_t vh_data_status = vh_local_update(
                                p_gatt_evt->param.data_update.handle,
                                p_gatt_evt->param.data_update.data,
                                p_gatt_evt->param.data_update.data_len);
                        
                        rbc_mesh_event_t mesh_evt;
                        mesh_evt.data = p_gatt_evt->param.data_update.data;
                        mesh_evt.data_len = p_gatt_evt->param.data_update.data_len;
                        mesh_evt.value_handle = p_gatt_evt->param.data_update.handle;
                        mesh_evt.version_delta = 1;
                        bool send_event = true;
                        switch (vh_data_status)
                        {
                            case VH_DATA_STATUS_CONFLICTING:
                                mesh_evt.event_type = RBC_MESH_EVENT_TYPE_CONFLICTING_VAL;
                                break;
                            case VH_DATA_STATUS_NEW:
                                mesh_evt.event_type = RBC_MESH_EVENT_TYPE_NEW_VAL;
                                break;
                            case VH_DATA_STATUS_UPDATED:
                                mesh_evt.event_type = RBC_MESH_EVENT_TYPE_UPDATE_VAL;
                                break;
                            default:
                                mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_ERROR_BUSY);
                                send_event = false;
                        }
                        if (send_event)
                        {
                            APP_ERROR_CHECK(rbc_mesh_event_push(&mesh_evt));
                            mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_SUCCESS);
                        }
                    }
                break;

                case MESH_GATT_EVT_OPCODE_FLAG_SET:
                    switch ((mesh_gatt_evt_flag_t) p_gatt_evt->param.flag_update.flag)
                    {
                        case MESH_GATT_EVT_FLAG_PERSISTENT:
                            if (vh_value_persistence_set(p_gatt_evt->param.flag_update.handle, 
                                        !!(p_gatt_evt->param.flag_update.value))
                                    != NRF_SUCCESS)
                            {
                                mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_ERROR_INVALID_HANDLE);
                                break;
                            }
                            mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_SUCCESS);
                            break;

                        case MESH_GATT_EVT_FLAG_DO_TX:
                            if (p_gatt_evt->param.flag_update.value)
                            {
                                if (vh_value_enable(p_gatt_evt->param.flag_update.handle)
                                        != NRF_SUCCESS)
                                {
                                    mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_ERROR_INVALID_HANDLE);
                                    break;
                                }
                                mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_SUCCESS);
                            }
                            else
                            {
                                if (vh_value_disable(p_gatt_evt->param.flag_update.handle)
                                        != NRF_SUCCESS)
                                {
                                    mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_ERROR_INVALID_HANDLE);
                                    break;
                                }
                                mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_SUCCESS);
                            }
                            break;

                        default:
                            mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_ERROR_UNKNOWN_FLAG);
                    }
                    break;

               case MESH_GATT_EVT_OPCODE_FLAG_REQ:
                    switch ((mesh_gatt_evt_flag_t) p_gatt_evt->param.flag_update.flag)
                    {
                        case MESH_GATT_EVT_FLAG_PERSISTENT:
                            {
                                if (p_gatt_evt->param.flag_update.handle == RBC_MESH_INVALID_HANDLE)
                                { 
                                    mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_ERROR_INVALID_HANDLE);
                                    break;
                                }

                                bool is_persistent = false;
                                if (vh_value_persistence_get(p_gatt_evt->param.flag_update.handle, &is_persistent) 
                                        != NRF_SUCCESS)
                                {
                                    mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_ERROR_NOT_FOUND);
                                    break;
                                }

                                mesh_gatt_evt_t rsp_evt;
                                rsp_evt.opcode = MESH_GATT_EVT_OPCODE_FLAG_RSP;
                                rsp_evt.param.flag_update.handle = p_gatt_evt->param.flag_update.handle;
                                rsp_evt.param.flag_update.flag = p_gatt_evt->param.flag_update.flag;
                                rsp_evt.param.flag_update.value = (uint8_t) is_persistent;
                                mesh_gatt_evt_push(&rsp_evt);
                            }
                            break;

                        case MESH_GATT_EVT_FLAG_DO_TX:
                            {
                                bool is_enabled;
                                if (vh_value_is_enabled(p_gatt_evt->param.flag_update.handle, &is_enabled)
                                        != NRF_SUCCESS)
                                {
                                    mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_ERROR_INVALID_HANDLE);
                                    break;
                                }

                                mesh_gatt_evt_t rsp_evt;
                                rsp_evt.opcode = MESH_GATT_EVT_OPCODE_FLAG_RSP;
                                rsp_evt.param.flag_update.handle = p_gatt_evt->param.flag_update.handle;
                                rsp_evt.param.flag_update.flag = p_gatt_evt->param.flag_update.flag;
                                rsp_evt.param.flag_update.value = (uint8_t) is_enabled;
                                mesh_gatt_evt_push(&rsp_evt);
                            }
                            break;

                        default:
                            mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_ERROR_UNKNOWN_FLAG);
                    }
                    break;

                default:
                    mesh_gatt_cmd_rsp_push((mesh_gatt_evt_opcode_t) p_gatt_evt->opcode, MESH_GATT_RESULT_ERROR_INVALID_OPCODE);
            }
        }
        else if (p_ble_evt->evt.gatts_evt.params.write.handle == m_mesh_service.ble_md_char_handles.value_handle)
        {
            mesh_metadata_char_t* p_md = (mesh_metadata_char_t*) p_ble_evt->evt.gatts_evt.params.write.data;
            tc_radio_params_set(p_md->mesh_access_addr, p_md->mesh_channel);
            vh_min_interval_set(p_md->mesh_interval_min_ms);
        }
        else if (p_ble_evt->evt.gatts_evt.params.write.handle == m_mesh_service.ble_val_char_handles.cccd_handle)
        {
            m_mesh_service.notification_enabled = (p_ble_evt->evt.gatts_evt.params.write.data[0] != 0);
        }
    }
    else if (p_ble_evt->header.evt_id == BLE_GAP_EVT_CONNECTED)
    {
        m_active_conn_handle = p_ble_evt->evt.gap_evt.conn_handle;
    }
    else if (p_ble_evt->header.evt_id == BLE_GAP_EVT_DISCONNECTED)
    {
        m_active_conn_handle = CONN_HANDLE_INVALID;
    }
}

#else /* SOFTDEVICE NOT PRESENT */

uint32_t mesh_gatt_init(uint32_t access_address, uint8_t channel, uint32_t interval_min_ms)
{
    return NRF_ERROR_SOFTDEVICE_NOT_ENABLED;
}

uint32_t mesh_gatt_value_set(rbc_mesh_value_handle_t handle, uint8_t* data, uint8_t length)
{
    return NRF_ERROR_SOFTDEVICE_NOT_ENABLED;
}

void mesh_gatt_sd_ble_event_handle(ble_evt_t* p_ble_evt)
{
    /* no actions, shouldn't even be called when the SD isn't present */
}

#endif

