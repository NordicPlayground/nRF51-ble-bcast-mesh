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
#include <string.h>
#include "mesh_aci.h"
#include "rbc_mesh_common.h"

#include "serial_handler.h"
#include "event_handler.h"
#include "timeslot_handler.h"
#include "version.h"
#include "mesh_packet.h"

#ifdef BOOTLOADER
#include "bootloader_mesh.h"
#endif


/* event push isn't present in the API header file. */
extern uint32_t rbc_mesh_event_push(rbc_mesh_event_t* p_evt);

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
        case NRF_ERROR_NOT_FOUND:
            return ACI_STATUS_ERROR_PIPE_INVALID;
        case NRF_ERROR_BUSY:
            return ACI_STATUS_ERROR_BUSY;
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
    uint32_t error_code;
    rbc_mesh_event_t app_evt;
    (void) app_evt;
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

    case SERIAL_CMD_OPCODE_RADIO_RESET:
        /* Host gets out of sync if we cut off in the middle of a TX */
        serial_wait_for_completion();
        
        /* kill ourself :) */
#ifdef SOFTDEVICE_PRESENT
        sd_power_reset_reason_clr(0x0F000F);
        sd_power_gpregret_set(RBC_MESH_GPREGRET_CODE_FORCED_REBOOT);
        sd_nvic_SystemReset();
#else
        NRF_POWER->RESETREAS = 0x0F000F; /* erase reset-reason to avoid wrongful state-readout on reboot */
        NRF_POWER->GPREGRET = RBC_MESH_GPREGRET_CODE_FORCED_REBOOT;
        NVIC_SystemReset();
#endif    
        break;
#ifndef BOOTLOADER
    case SERIAL_CMD_OPCODE_INIT:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 3;

        if (serial_cmd->length != sizeof(serial_cmd_params_init_t) + 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
            error_code = NRF_ERROR_INVALID_LENGTH;
        }
        else
        {
            rbc_mesh_init_params_t init_params;
            init_params.access_addr = serial_cmd->params.init.access_addr;
            init_params.channel = serial_cmd->params.init.channel;
            init_params.interval_min_ms = serial_cmd->params.init.interval_min;
            init_params.lfclksrc = NRF_CLOCK_LFCLKSRC_XTAL_500_PPM; /* choose worst clock, just to be safe */

            error_code = rbc_mesh_init(init_params);

            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        }

        if (error_code == NRF_SUCCESS)
        {
            /* notify application */
            memset(&app_evt, 0, sizeof(app_evt));
            app_evt.event_type = RBC_MESH_EVENT_TYPE_INITIALIZED;

            serial_evt.params.cmd_rsp.status = error_code_translate(rbc_mesh_event_push(&app_evt));
        }

        serial_handler_event_send(&serial_evt);
        break;

    case SERIAL_CMD_OPCODE_START:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 3;

        error_code = rbc_mesh_start();
        serial_evt.params.cmd_rsp.status = error_code_translate(error_code);

        serial_handler_event_send(&serial_evt);
        break;

    case SERIAL_CMD_OPCODE_STOP:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 3;

        error_code = rbc_mesh_stop();
        serial_evt.params.cmd_rsp.status = error_code_translate(error_code);

        serial_handler_event_send(&serial_evt);
        break;

    case SERIAL_CMD_OPCODE_VALUE_SET:
    {
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 3;

        mesh_packet_t* p_packet;
        if (mesh_packet_acquire(&p_packet))
        {
            const uint8_t data_len = serial_cmd->length - 1 - sizeof(rbc_mesh_value_handle_t);
            
            if (serial_cmd->length > sizeof(serial_cmd_params_value_set_t) + 1)
            {
                serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
                error_code = NRF_ERROR_INVALID_LENGTH;
            }
            else
            {
                error_code = rbc_mesh_value_set(serial_cmd->params.value_set.handle,
                                                serial_cmd->params.value_set.value,
                                                data_len);

                serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
            }

            /* notify application */
            if (error_code == NRF_SUCCESS)
            {
                memcpy(p_packet->payload, serial_cmd->params.value_set.value, data_len);
                memset(&app_evt, 0, sizeof(app_evt));
                app_evt.event_type = RBC_MESH_EVENT_TYPE_UPDATE_VAL;
                app_evt.data = serial_cmd->params.value_set.value;
                app_evt.data_len = data_len;
                app_evt.value_handle = serial_cmd->params.value_set.handle;

                error_code = rbc_mesh_event_push(&app_evt);
                mesh_packet_ref_count_dec(p_packet);
                
                serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
            }
        }
        else
        {
            serial_evt.params.cmd_rsp.status = error_code_translate(NRF_ERROR_BUSY);
        }

        serial_handler_event_send(&serial_evt);
        break;
    }

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
            error_code = rbc_mesh_value_enable(serial_cmd->params.value_enable.handle);

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
            error_code = rbc_mesh_value_disable(serial_cmd->params.value_enable.handle);

            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        }

        serial_handler_event_send(&serial_evt);
        break;

    case SERIAL_CMD_OPCODE_FLAG_SET:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 3;

        if (serial_cmd->length != sizeof(serial_cmd_params_flag_set_t) + 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
        }
        else
        {
            switch ((aci_flag_t) serial_cmd->params.flag_set.flag)
            {
                case ACI_FLAG_PERSISTENT:
                    error_code = rbc_mesh_persistence_set(serial_cmd->params.flag_set.handle,
                            serial_cmd->params.flag_set.value);

                    break;

                case ACI_FLAG_TX_EVENT:
                    error_code = rbc_mesh_tx_event_set(serial_cmd->params.flag_set.handle,
                            serial_cmd->params.flag_set.value);
                    break;
                default:
                    error_code = NRF_ERROR_INVALID_PARAM;
            }
            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        }
        break;

    case SERIAL_CMD_OPCODE_FLAG_GET:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 7;

        if (serial_cmd->length != sizeof(serial_cmd_params_flag_get_t) + 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
        }
        else
        {
            uint32_t error_code;
            bool flag_status = false;
            switch ((aci_flag_t) serial_cmd->params.flag_set.flag)
            {
                case ACI_FLAG_PERSISTENT:
                    error_code = rbc_mesh_persistence_get(serial_cmd->params.flag_get.handle,
                            &flag_status);

                    break;

                case ACI_FLAG_TX_EVENT:
                    error_code = rbc_mesh_tx_event_flag_get(serial_cmd->params.flag_get.handle,
                            &flag_status);
                    break;
                default:
                    error_code = NRF_ERROR_INVALID_PARAM;
                    serial_evt.length = 3;
            }
            serial_evt.params.cmd_rsp.response.flag.handle = serial_cmd->params.flag_get.handle;
            serial_evt.params.cmd_rsp.response.flag.flag = serial_cmd->params.flag_get.flag;
            serial_evt.params.cmd_rsp.response.flag.value = flag_status;
            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        }
        break;

    case SERIAL_CMD_OPCODE_VALUE_GET:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = RBC_MESH_VALUE_MAX_LEN; /* signal to the framework that we can fit the entire payload in our buffer */

        if (serial_cmd->length != sizeof(serial_cmd_params_value_get_t) + 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
        }
        else
        {
            uint32_t error_code = rbc_mesh_value_get(serial_cmd->params.value_get.handle,
                                                        serial_evt.params.cmd_rsp.response.val_get.data,
                                                        (uint16_t*) &serial_evt.length);

            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);

            serial_evt.params.cmd_rsp.response.val_get.handle = serial_cmd->params.value_get.handle;
            serial_evt.length += 1 + 1 + 1 + 2 ; /* opcode + command + status + handle */
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

            error_code = rbc_mesh_access_address_get(&access_addr);
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

    case SERIAL_CMD_OPCODE_INTERVAL_GET:
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 7;

        if (serial_cmd->length != 1)
        {
            serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
        }
        else
        {
            uint32_t interval_min_ms;
            error_code = rbc_mesh_interval_min_ms_get(&interval_min_ms);
            serial_evt.params.cmd_rsp.response.int_min.int_min = interval_min_ms;
            serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        }

        serial_handler_event_send(&serial_evt);
        break;

#endif /* BOOTLOADER */

    case SERIAL_CMD_OPCODE_DFU:
    {
        serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
        serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
        serial_evt.length = 5;

        /* do not attempt to police the length for this event type, as additional data might be added in future versions */

#ifdef BOOTLOADER
        /* attempt to allocate a packet for the buffer before entering the handler, allowing to catch
           out-of-memory issues right away. */
        mesh_packet_t* p_packet = NULL;
        if (mesh_packet_acquire(&p_packet))
        {
            error_code = NRF_SUCCESS;
            /* verify length */
            switch (serial_cmd->params.dfu.packet.packet_type)
            {
                case DFU_PACKET_TYPE_DATA:
                case DFU_PACKET_TYPE_DATA_RSP:
                    if (serial_cmd->length < 10)
                    {
                        error_code = NRF_ERROR_INVALID_LENGTH;
                    }
                    break;
                case DFU_PACKET_TYPE_DATA_REQ:
                    if (serial_cmd->length != DFU_PACKET_LEN_DATA_REQ + 1)
                    {
                        error_code = NRF_ERROR_INVALID_LENGTH;
                    }
                    break;
                case DFU_PACKET_TYPE_FWID:
                    if (serial_cmd->length != DFU_PACKET_LEN_FWID + 1)
                    {
                        error_code = NRF_ERROR_INVALID_LENGTH;
                    }
                    break;
                case DFU_PACKET_TYPE_STATE:
                    if (serial_cmd->params.dfu.packet.payload.state.dfu_type == DFU_TYPE_APP)
                    {
                        if (serial_cmd->length != DFU_PACKET_LEN_STATE_APP + 1)
                        {
                            error_code = NRF_ERROR_INVALID_LENGTH;
                        }
                    }
                    else if (serial_cmd->params.dfu.packet.payload.state.dfu_type == DFU_TYPE_SD)
                    {
                        if (serial_cmd->length != DFU_PACKET_LEN_STATE_SD + 1)
                        {
                            error_code = NRF_ERROR_INVALID_LENGTH;
                        }
                    }
                    else
                    {
                        if (serial_cmd->length != DFU_PACKET_LEN_STATE_BL + 1)
                        {
                            error_code = NRF_ERROR_INVALID_LENGTH;
                        }
                    }
                    break;
                
            }
        }
        else
        {
            error_code = NRF_ERROR_BUSY;
        }

        /* send ack */
        serial_evt.params.cmd_rsp.response.dfu.packet_type = serial_cmd->params.dfu.packet.packet_type;
        serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        serial_handler_event_send(&serial_evt);

        /* propagate to handler */
        if (error_code == NRF_SUCCESS)
        {
            bootloader_packet_set_local_fields(p_packet, serial_cmd->length - SERIAL_PACKET_OVERHEAD);
            memcpy(&p_packet->payload[4], &serial_cmd->params.dfu.packet, serial_cmd->length - SERIAL_PACKET_OVERHEAD);

            error_code = bootloader_rx((dfu_packet_t*) &p_packet->payload[4], serial_cmd->length - SERIAL_PACKET_OVERHEAD, true);
            mesh_packet_ref_count_dec(p_packet);
        }
#else
        error_code = NRF_ERROR_INVALID_STATE;
        serial_evt.params.cmd_rsp.response.dfu.packet_type = serial_cmd->params.dfu.packet.packet_type;
        serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
        serial_handler_event_send(&serial_evt);

#endif
    }
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
#ifdef BOOTLOADER
    NVIC_SetPriority(SWI1_IRQn, 3);
    NVIC_EnableIRQ(SWI1_IRQn);
#else
    event_handler_init();
#endif
	serial_handler_init();
}

void mesh_aci_command_check(void)
{
    serial_cmd_t serial_cmd;
    /* poll queue */
    while (serial_handler_command_get(&serial_cmd))
    {
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

        case RBC_MESH_EVENT_TYPE_TX:
            serial_evt.opcode = SERIAL_EVT_OPCODE_EVENT_TX;
            break;

        default:
            break;
    }

    /* serial overhead: opcode + handle = 3 */
    serial_evt.length = 3 + evt->data_len;

    /* all event parameter types are the same, just use event_update for all */
    serial_evt.params.event_update.handle = evt->value_handle;
    memcpy(serial_evt.params.event_update.data, evt->data, evt->data_len);

    serial_handler_event_send(&serial_evt);
}

