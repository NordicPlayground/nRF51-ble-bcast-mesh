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
#ifndef _SERIAL_COMMAND_H__
#define _SERIAL_COMMAND_H__

#include <stdint.h>
#include "rbc_mesh.h"
#include "toolchain.h"
#include "dfu_types_mesh.h"


typedef __packed_armcc enum
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
    SERIAL_CMD_OPCODE_DFU                   = 0x78,

    SERIAL_CMD_OPCODE_VALUE_GET             = 0x7A,
    SERIAL_CMD_OPCODE_BUILD_VERSION_GET     = 0x7B,
    SERIAL_CMD_OPCODE_ACCESS_ADDR_GET       = 0x7C,
    SERIAL_CMD_OPCODE_CHANNEL_GET           = 0x7D,
    SERIAL_CMD_OPCODE_INTERVAL_GET          = 0x7F,    
} __packed_gcc serial_cmd_opcode_t;


/****** CMD PARAMS ******/
typedef __packed_armcc struct 
{
	uint8_t data[29];
} __packed_gcc serial_cmd_params_echo_t;

typedef __packed_armcc struct 
{
    uint32_t access_addr;
    uint32_t interval_min;
    uint8_t channel;
} __packed_gcc serial_cmd_params_init_t;

typedef __packed_armcc struct 
{
    rbc_mesh_value_handle_t handle;
    uint8_t flag; 
    uint8_t value;
} __packed_gcc serial_cmd_params_flag_set_t;

typedef __packed_armcc struct 
{
    rbc_mesh_value_handle_t handle;
    uint8_t flag; 
} __packed_gcc serial_cmd_params_flag_get_t;

typedef __packed_armcc struct 
{
    rbc_mesh_value_handle_t handle;
    uint8_t value[RBC_MESH_VALUE_MAX_LEN];
} __packed_gcc serial_cmd_params_value_set_t;

typedef __packed_armcc struct 
{
    rbc_mesh_value_handle_t handle;
} __packed_gcc serial_cmd_params_value_enable_t;

typedef __packed_armcc struct 
{
    rbc_mesh_value_handle_t handle;
} __packed_gcc serial_cmd_params_value_disable_t;

typedef __packed_armcc struct 
{
    rbc_mesh_value_handle_t handle;
} __packed_gcc serial_cmd_params_value_get_t;

typedef __packed_armcc struct 
{
    dfu_packet_t packet;
} __packed_gcc serial_cmd_params_dfu_t;




typedef __packed_armcc struct 
{
    uint8_t length;
    serial_cmd_opcode_t opcode;
    __packed_armcc union 
    {
        serial_cmd_params_echo_t            echo;
        serial_cmd_params_init_t            init;
        serial_cmd_params_value_set_t       value_set;
        serial_cmd_params_flag_set_t        flag_set;
        serial_cmd_params_flag_get_t        flag_get;
        serial_cmd_params_value_enable_t    value_enable;
        serial_cmd_params_value_disable_t   value_disable;
        serial_cmd_params_value_get_t       value_get;
        serial_cmd_params_dfu_t             dfu;
    } __packed_gcc params;
} __packed_gcc  serial_cmd_t;


#endif /* _SERIAL_COMMAND_H__ */
