/* Copyright (c) 2014, Nordic Semiconductor ASA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#ifndef _SERIAL_COMMAND_H__
#define _SERIAL_COMMAND_H__

#include "serial_internal.h"
#include <stdint.h>


typedef enum
{
    SERIAL_CMD_OPCODE_ECHO                  = 0x02,
    SERIAL_CMD_OPCODE_RADIO_RESET           = 0x0E,
    
    SERIAL_CMD_OPCODE_INIT                  = 0x70,
    SERIAL_CMD_OPCODE_VALUE_SET             = 0x71,
    SERIAL_CMD_OPCODE_VALUE_ENABLE          = 0x72,
    SERIAL_CMD_OPCODE_VALUE_DISABLE         = 0x73,
    SERIAL_CMD_OPCODE_START                 = 0x74,
    SERIAL_CMD_OPCODE_STOP                  = 0x75,
    SERIAL_CMD_OPCODE_FLAG_SET              = 0x76,
    SERIAL_CMD_OPCODE_FLAG_GET              = 0x77,

    SERIAL_CMD_OPCODE_VALUE_GET             = 0x7A,
    SERIAL_CMD_OPCODE_BUILD_VERSION_GET     = 0x7B,
    SERIAL_CMD_OPCODE_ACCESS_ADDR_GET       = 0x7C,
    SERIAL_CMD_OPCODE_CHANNEL_GET           = 0x7D,
    SERIAL_CMD_OPCODE_INTERVAL_GET          = 0x7F,
} __packed serial_cmd_opcode_t;


/****** CMD PARAMS ******/
typedef struct 
{
	uint8_t data[29];
} __packed serial_cmd_params_echo_t;

typedef struct 
{
    uint32_t access_addr;
    uint32_t int_min;
    uint8_t channel;
} __packed serial_cmd_params_init_t;

typedef struct 
{
    uint16_t handle;
    uint8_t flag; 
    uint8_t value;
} __packed serial_cmd_params_flag_set_t;

typedef struct 
{
    uint16_t handle;
    uint8_t flag; 
} __packed serial_cmd_params_flag_get_t;

typedef struct 
{
    uint16_t handle;
    uint8_t value[RBC_MESH_VALUE_MAX_LEN];
} __packed serial_cmd_params_value_set_t;

typedef struct 
{
    uint16_t handle;
} __packed serial_cmd_params_value_enable_t;

typedef struct 
{
    uint16_t handle;
} __packed serial_cmd_params_value_disable_t;

typedef struct 
{
    uint16_t handle;
} __packed serial_cmd_params_value_get_t;


typedef struct 
{
    uint8_t length;
    serial_cmd_opcode_t opcode;
    union 
    {
        serial_cmd_params_echo_t            echo;
        serial_cmd_params_init_t            init;
        serial_cmd_params_value_set_t       value_set;
        serial_cmd_params_flag_set_t        flag_set;
        serial_cmd_params_flag_get_t        flag_get;
        serial_cmd_params_value_enable_t    value_enable;
        serial_cmd_params_value_disable_t   value_disable;
        serial_cmd_params_value_get_t       value_get;
    } __packed params;
} __packed  serial_cmd_t;


#endif /* _SERIAL_COMMAND_H__ */
