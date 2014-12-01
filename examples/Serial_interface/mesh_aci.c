#include "mesh_aci.h"

#include "serial_handler.h"

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
    

static void serial_command_handler(serial_cmd_t* serial_cmd)
{
	serial_evt_t serial_evt;
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
			serial_evt.opcode = SERIAL_EVT_OPCODE_CMD_RSP;
			serial_evt.params.cmd_rsp.command_opcode = serial_cmd->opcode;
			serial_evt.length = 3;
            
			if (serial_cmd->length != sizeof(serial_cmd_params_init_t) + 1)
			{
				serial_evt.params.cmd_rsp.status = ACI_STATUS_ERROR_INVALID_LENGTH;
			}
			else
			{
				uint32_t error_code = rbc_mesh_init(serial_cmd->params.init.access_addr, 
													serial_cmd->params.init.channel,
													serial_cmd->params.init.handle_count,
													serial_cmd->params.init.adv_int_min);
				
                serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
			}
			
			serial_handler_event_send(&serial_evt);
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
                uint32_t error_code = rbc_mesh_value_get(serial_cmd->params.value_get.handle, 
                                                            serial_evt.params.cmd_rsp.response.val_get.data,
                                                            (uint16_t*) &serial_evt.length);
                               
                serial_evt.params.cmd_rsp.status = error_code_translate(error_code);
                
                serial_evt.length += 3 + 1 + 6;
            }
            
            serial_handler_event_send(&serial_evt);
			break;
			
		case SERIAL_CMD_OPCODE_BUILD_VERSION_GET:
			break;
			
		case SERIAL_CMD_OPCODE_ADV_ADDR_GET:
			break;
			
		case SERIAL_CMD_OPCODE_CHANNEL_GET:
			break;
			
		case SERIAL_CMD_OPCODE_HANDLE_COUNT_GET:
			break;
			
		case SERIAL_CMD_OPCODE_ADV_INT_GET:
			break;			
	}
}


/*****************************************************************************
* Interface functions
*****************************************************************************/

void mesh_aci_init(void)
{
	serial_handler_init(serial_command_handler);
}


