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
#include "mesh_aci.h"
#include "rbc_mesh_common.h"

#include "serial_handler.h"
#include "version.h"

#include <string.h>



/*****************************************************************************
* Static globals
*****************************************************************************/



/*****************************************************************************
* Static functions
*****************************************************************************/

static aci_status_code_t error_code_translate(uint32_t nrf_error_code)
{
    switch (nrf_error_code)
    {
        case NRF_SUCCESS: 
            return ACI_STATUS_SUCCESS;
        case NRF_ERROR_INVALID_PARAM:
            return ACI_STATUS_ERROR_INVALID_PARAMETER;
        case NRF_ERROR_INVALID_STATE:
            return ACI_STATUS_ERROR_DEVICE_STATE_INVALID;
        case NRF_ERROR_SOFTDEVICE_NOT_ENABLED:
            return ACI_STATUS_ERROR_BUSY;
        case NRF_ERROR_INVALID_LENGTH:
            return ACI_STATUS_ERROR_INVALID_LENGTH;            
        default:
            return ACI_STATUS_ERROR_UNKNOWN;
    }
}
    
/**
* Handle events coming in on the serial line 
*/
static void serial_command_handler(serial_cmd_t* serial_cmd)
{
	serial_evt_t serial_evt;
    rbc_mesh_event_t app_evt;
	switch (serial_cmd->opcode)
	{
    case SERIAL_CMD_OPCODE_ECHO:
        serial_evt.opcode = SERIAL_EVT_OPCODE_ECHO_RSP;
        serial_evt.length = serial_cmd->length;
        if (serial_cmd->length > 1)
        {
            memcpy(serial_evt.params.echo.data, serial_cmd->params.echo.data, serial_cmd->length - 1);
        }
        
        serial_handler_event_send(&serial_evt);
        break;
        
    case SERIAL_CMD_OPCODE_INIT:
        TICK_PIN(4);
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 3;
        
        if (serial_cmd->length != sizeof(serial_cmd_params_init_t) + 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
        }
        else
        {
            rbc_mesh_init_params_t init_params;
            init_params.access_addr = serial_cmd->params.init.access_addr;
            init_params.channel = serial_cmd->params.init.channel;
            init_params.handle_count = serial_cmd->params.init.handle_count;
            init_params.adv_int_ms = serial_cmd->params.init.adv_int_min;
            init_params.packet_format = RBC_MESH_PACKET_FORMAT_ORIGINAL;
            init_params.radio_mode = RBC_MESH_RADIO_MODE_BLE_1MBIT;
                
            uint32_t error_code = rbc_mesh_init(init_params);
            
            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        }
        
        serial_handler_event_send(&serial_evt);
        
        /* notify application */
        memset(&app_evt, 0, sizeof(app_evt));
        app_evt.event_type = RBC_MESH_EVENT_TYPE_INITIALIZED;
        
        rbc_mesh_event_handler(&app_evt);
        
        break;
        
    case SERIAL_CMD_OPCODE_VALUE_SET:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 3;
        
        if (serial_cmd->length > sizeof(serial_cmd_params_value_set_t) + 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
        }
        else
        {
            uint32_t error_code = rbc_mesh_value_set(   serial_cmd->params.value_set.handle,
                                                        serial_cmd->params.value_set.value,
                                                        serial_cmd->length - 2);
                                                             
            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        }
        
        serial_handler_event_send(&serial_evt);
        
        /* notify application */
        memset(&app_evt, 0, sizeof(app_evt));
        app_evt.event_type = RBC_MESH_EVENT_TYPE_UPDATE_VAL;
        app_evt.data = serial_cmd->params.value_set.value;
        app_evt.data_len = serial_cmd->length - 2;
        sd_ble_gap_address_get(&app_evt.originator_address);
        app_evt.value_handle = serial_cmd->params.value_set.handle;
        
        rbc_mesh_event_handler(&app_evt);
        break;
        
    case SERIAL_CMD_OPCODE_VALUE_ENABLE:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 3;
        
        if (serial_cmd->length != sizeof(serial_cmd_params_value_enable_t) + 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
        }
        else
        {
            uint32_t error_code = rbc_mesh_value_enable(serial_cmd->params.value_enable.handle);
            
            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        }
        
        serial_handler_event_send(&serial_evt);
        break;
        
    case SERIAL_CMD_OPCODE_VALUE_DISABLE:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 3;
        
        if (serial_cmd->length != sizeof(serial_cmd_params_value_disable_t) + 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
        }
        else
        {
            uint32_t error_code = rbc_mesh_value_disable(serial_cmd->params.value_enable.handle);
            
            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        }
        
        serial_handler_event_send(&serial_evt);
        break;
        
    case SERIAL_CMD_OPCODE_VALUE_GET:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 3;
                    
        if (serial_cmd->length != sizeof(serial_cmd_params_value_get_t) + 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
        }
        else
        {
            ble_gap_addr_t origin_addr;
            uint32_t error_code = rbc_mesh_value_get(serial_cmd->params.value_get.handle, 
                                                        serial_evt.params.cmd_rsp.response.val_get.data,
                                                        (uint16_t*) &serial_evt.length,
                                                        &origin_addr);
            
            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
            
            memcpy(serial_evt.params.cmd_rsp.response.val_get.origin_addr, origin_addr.addr, BLE_GAP_ADDR_LEN);
            serial_evt.params.cmd_rsp.response.val_get.addr_type = ADDR_TYPE_BLE_GAP_ADV_ADDR;
            serial_evt.params.cmd_rsp.response.val_get.handle = serial_cmd->params.value_get.handle;
            serial_evt.length += 3 + 1 + 1 + 6; /* opcode + command + status + handle + addr_type + addr */
        }
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        
        serial_handler_event_send(&serial_evt);
        break;
        
    case SERIAL_CMD_OPCODE_BUILD_VERSION_GET:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 6;
                    
        if (serial_cmd->length != 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
        }
        else
        {
            serial_evt.params.cmd_rsp.response.build_version.major = VERSION_MAJOR;
            serial_evt.params.cmd_rsp.response.build_version.minor_1 = VERSION_MINOR1;
            serial_evt.params.cmd_rsp.response.build_version.minor_2 = VERSION_MINOR2;
            serial_evt.params.cmd_rsp.status = ACI_STATUS_SUCCESS;
        }
        
        serial_handler_event_send(&serial_evt);
        break;
        
    case SERIAL_CMD_OPCODE_ACCESS_ADDR_GET:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 7;
                    
        if (serial_cmd->length != 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
        }
        else
        {
            uint32_t access_addr;
            
            uint32_t error_code = rbc_mesh_access_address_get(&access_addr);
            serial_evt.params.cmd_rsp.response.access_addr.access_addr = access_addr;
            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        }
        
        serial_handler_event_send(&serial_evt);
        break;
        
    case SERIAL_CMD_OPCODE_CHANNEL_GET:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 4;
                    
        if (serial_cmd->length != 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
        }
        else
        {
            uint8_t channel;
            uint32_t error_code = rbc_mesh_channel_get(&channel);
            serial_evt.params.cmd_rsp.response.channel.channel = channel;
            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        }
        
        serial_handler_event_send(&serial_evt);
        
        break;
        
    case SERIAL_CMD_OPCODE_HANDLE_COUNT_GET:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 4;
                    
        if (serial_cmd->length != 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
        }
        else
        {
            uint8_t handle_count;
            uint32_t error_code = rbc_mesh_handle_count_get(&handle_count);
            serial_evt.params.cmd_rsp.response.handle_count.handle_count = handle_count;
            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        }
        
        serial_handler_event_send(&serial_evt);
        break;
        
    case SERIAL_CMD_OPCODE_ADV_INT_GET:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 7;
                    
        if (serial_cmd->length != 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
        }
        else
        {
            uint32_t adv_int;
            uint32_t error_code = rbc_mesh_adv_int_get(&adv_int);
            serial_evt.params.cmd_rsp.response.adv_int.adv_int = adv_int;
            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        }
      
        serial_handler_event_send(&serial_evt);
        break;			
            
    default:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 3;
        serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_CMD_UNKNOWN;
        serial_handler_event_send(&serial_evt);
	}    
}


/*****************************************************************************
* Interface functions
*****************************************************************************/

void mesh_aci_init(void)
{
	serial_handler_init();
}

void mesh_aci_command_check(void)
{
    serial_cmd_t serial_cmd;
    /* poll queue */
    while (serial_handler_command_get(&serial_cmd))
    {
        TICK_PIN(3);
        serial_command_handler(&serial_cmd);
    }
}


void mesh_aci_rbc_event_handler(rbc_mesh_event_t* evt)
{
    serial_evt_t serial_evt;
    switch (evt->event_type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
            serial_evt.opcode = SERIAL_EVT_OPCODE_EVENT_CONFLICTING;
            break;
        
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
            serial_evt.opcode = SERIAL_EVT_OPCODE_EVENT_NEW;
            break;
        
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
            serial_evt.opcode = SERIAL_EVT_OPCODE_EVENT_UPDATE;
            break;
    }
    
    /* opcode + handle + addr type + addr = 9 */
    serial_evt.length = 9 + evt->data_len;
    
    /* all event parameter types are the same, just use event_update for all */
    serial_evt.params.event_update.addr_type = ADDR_TYPE_BLE_GAP_ADV_ADDR;
    memcpy(serial_evt.params.event_update.origin_addr, evt->originator_address.addr, BLE_GAP_ADDR_LEN);
    serial_evt.params.event_update.handle = evt->value_handle;
    memcpy(serial_evt.params.event_update.data, evt->data, evt->data_len);
    
    serial_handler_event_send(&serial_evt);
}

