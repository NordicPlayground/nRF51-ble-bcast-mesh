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
#ifndef _SERIAL_EVENT_H__
#define _SERIAL_EVENT_H__

#include "serial_handler.h"

#include "mesh_aci.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    SERIAL_EVT_OPCODE_DEVICE_STARTED        = 0x81,
    SERIAL_EVT_OPCODE_ECHO_RSP              = 0x82,
    SERIAL_EVT_OPCODE_CMD_RSP               = 0x84,
    SERIAL_EVT_OPCODE_EVENT_NEW             = 0xB3,
    SERIAL_EVT_OPCODE_EVENT_UPDATE          = 0xB4,
    SERIAL_EVT_OPCODE_EVENT_CONFLICTING     = 0xB5
} __packed serial_evt_opcode_t;


typedef enum
{
    ADDR_TYPE_BLE_GAP_ADV_ADDR,
    ADDR_TYPE_6LOWPAN
} __packed addr_type_t;

typedef enum
{
    OPERATING_MODE_TEST,
    OPERATING_MODE_SETUP,
    OPERATING_MODE_STANDBY
} __packed operating_mode_t;


/****** CMD RSP EVT PARAMS ******/
typedef __packed struct
{
    uint8_t major;
    uint8_t minor_1;
    uint8_t minor_2;
} serial_evt_cmd_rsp_params_build_version_t;

typedef __packed struct
{
    uint32_t access_addr;
} serial_evt_cmd_rsp_params_access_addr_t;

typedef __packed struct
{
    uint8_t channel;
} serial_evt_cmd_rsp_params_channel_t;

typedef __packed struct
{
    uint8_t handle_count;
} serial_evt_cmd_rsp_params_handle_count_t;

typedef __packed struct
{
    uint32_t adv_int;
} serial_evt_cmd_rsp_params_adv_int_t;

typedef __packed struct
{
    uint8_t handle;
    addr_type_t addr_type;
    uint8_t origin_addr[6];
    uint8_t data[SERIAL_DATA_MAX_LEN];
} serial_evt_cmd_rsp_params_val_get_t;


/****** EVT PARAMS ******/
typedef __packed struct
{
    uint8_t data[29];
} serial_evt_params_echo_t;

typedef __packed struct
{
    uint8_t command_opcode;
    aci_status_code_t status;
    __packed union
    {
        serial_evt_cmd_rsp_params_build_version_t build_version;
        serial_evt_cmd_rsp_params_access_addr_t access_addr;
        serial_evt_cmd_rsp_params_channel_t channel;
        serial_evt_cmd_rsp_params_handle_count_t handle_count;
        serial_evt_cmd_rsp_params_adv_int_t adv_int;
        serial_evt_cmd_rsp_params_val_get_t val_get;
    } response;        
} serial_evt_params_cmd_rsp_t;

typedef __packed struct
{
    uint8_t handle;
    addr_type_t addr_type;
    uint8_t origin_addr[6];
    uint8_t data[SERIAL_DATA_MAX_LEN];
} serial_evt_params_event_new_t;

typedef __packed struct
{
    uint8_t handle;
    addr_type_t addr_type;
    uint8_t origin_addr[6];
    uint8_t data[SERIAL_DATA_MAX_LEN];
} serial_evt_params_event_update_t;

typedef __packed struct
{
    uint8_t handle;
    addr_type_t addr_type;
    uint8_t origin_addr[6];
    uint8_t data[SERIAL_DATA_MAX_LEN];
} serial_evt_params_event_conflicting_t;

typedef __packed struct
{
    operating_mode_t operating_mode;
    uint8_t hw_error;
    uint8_t data_credit_available;
} serial_evt_params_event_device_started_t;

typedef __packed struct
{
    uint8_t length;
	uint8_t opcode;
	__packed union
	{
		serial_evt_params_echo_t echo;
        serial_evt_params_cmd_rsp_t cmd_rsp;
        serial_evt_params_event_new_t event_new;
        serial_evt_params_event_update_t event_update;
        serial_evt_params_event_conflicting_t event_conflicting;
        serial_evt_params_event_device_started_t device_started;
	} params;
} serial_evt_t;


#endif /* _SERIAL_EVENT_H__ */
