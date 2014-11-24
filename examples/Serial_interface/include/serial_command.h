#ifndef _SERIAL_COMMAND_H__
#define _SERIAL_COMMAND_H__

#include "serial_handler.h"
#include "rbc_mesh.h"
#include <stdint.h>

#define SERIAL_CMD_OPCODE_ECHO                      (0x02)
#define SERIAL_CMD_OPCODE_INIT	    				(0x70)
#define SERIAL_CMD_OPCODE_VALUE_SET					(0x71)
#define SERIAL_CMD_OPCODE_VALUE_GET					(0x72)
#define SERIAL_CMD_OPCODE_VALUE_REQUEST				(0x73)
#define SERIAL_CMD_OPCODE_VALUE_DISABLE				(0x74)
#define SERIAL_CMD_OPCODE_BUILD_VERSION_REQUEST		(0x75)

typedef __packed struct
{
	uint8_t data[SERIAL_DATA_MAX_LEN];
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
    uint8_t value[RBC_MESH_VALUE_MAX_LEN];
} serial_cmd_params_value_get_t;

typedef __packed struct
{
    uint8_t handle;
} serial_cmd_params_value_request_t;

typedef __packed struct
{
    uint8_t handle;
} serial_cmd_params_value_disable_t;




typedef __packed struct
{
	uint8_t length;
	uint8_t opcode;
    __packed union
    {
        serial_cmd_params_echo_t            echo;
        serial_cmd_params_init_t            init;
        serial_cmd_params_value_set_t       value_set;
        serial_cmd_params_value_get_t       value_get;
        serial_cmd_params_value_request_t   value_request;
        serial_cmd_params_value_disable_t   value_disable;
    } params;
} serial_cmd_t;


#endif /* _SERIAL_COMMAND_H__ */
