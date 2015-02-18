#ifndef _SERIAL_COMMAND_H__
#define _SERIAL_COMMAND_H__

#include "serial_handler.h"
#include "rbc_mesh.h"
#include <stdint.h>

typedef __packed enum
{
    SERIAL_CMD_OPCODE_ECHO                  = 0x02,
    SERIAL_CMD_OPCODE_INIT                  = 0x70,
    SERIAL_CMD_OPCODE_VALUE_SET             = 0x71,
    SERIAL_CMD_OPCODE_VALUE_ENABLE          = 0x72,
    SERIAL_CMD_OPCODE_VALUE_DISABLE         = 0x73,
    
    SERIAL_CMD_OPCODE_VALUE_GET             = 0x7A,
    SERIAL_CMD_OPCODE_BUILD_VERSION_GET     = 0x7B,
    SERIAL_CMD_OPCODE_ADV_ADDR_GET          = 0x7C,
    SERIAL_CMD_OPCODE_CHANNEL_GET           = 0x7D,
    SERIAL_CMD_OPCODE_HANDLE_COUNT_GET      = 0x7E,
    SERIAL_CMD_OPCODE_ADV_INT_GET           = 0x7F
} serial_cmd_opcode_t;


/****** CMD PARAMS ******/
typedef __packed struct
{
	uint8_t data[29];
} serial_cmd_params_echo_t;

typedef __packed struct
{
    uint32_t access_addr;
    uint8_t channel;
    uint8_t handle_count;
    uint32_t adv_int_min;
} serial_cmd_params_init_t;

typedef __packed struct
{
    uint8_t handle;
    uint8_t value[RBC_MESH_VALUE_MAX_LEN];
} serial_cmd_params_value_set_t;

typedef __packed struct
{
    uint8_t handle;
} serial_cmd_params_value_enable_t;

typedef __packed struct
{
    uint8_t handle;
} serial_cmd_params_value_disable_t;


typedef __packed struct
{
    uint8_t handle;
} serial_cmd_params_value_get_t;







typedef __packed struct
{
	uint8_t length;
	serial_cmd_opcode_t opcode;
    __packed union
    {
        serial_cmd_params_echo_t            echo;
        serial_cmd_params_init_t            init;
        serial_cmd_params_value_set_t       value_set;
        serial_cmd_params_value_enable_t   value_enable;
        serial_cmd_params_value_disable_t   value_disable;
        serial_cmd_params_value_get_t       value_get;
    } params;
} serial_cmd_t;


#endif /* _SERIAL_COMMAND_H__ */
