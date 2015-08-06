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

#ifndef _SERIAL_EVENT_H__
#define _SERIAL_EVENT_H__

#include "serial_internal.h"
#include "serial_command.h"
#include <stdint.h>
#include <stdbool.h>


typedef enum
{
    SERIAL_EVT_OPCODE_DEVICE_STARTED        = 0x81,
    SERIAL_EVT_OPCODE_ECHO_RSP              = 0x82,
    SERIAL_EVT_OPCODE_CMD_RSP               = 0x84,
    SERIAL_EVT_OPCODE_EVENT_NEW             = 0XB3,
    SERIAL_EVT_OPCODE_EVENT_UPDATE          = 0XB4,
    SERIAL_EVT_OPCODE_EVENT_CONFLICTING     = 0XB5
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
typedef struct
{
    uint8_t major;
    uint8_t minor_1;
    uint8_t minor_2;
} __packed serial_evt_cmd_rsp_params_build_version_t;

typedef struct
{
    uint32_t access_addr;
} __packed serial_evt_cmd_rsp_params_access_addr_t;

typedef struct
{
    uint8_t channel;
} __packed serial_evt_cmd_rsp_params_channel_t;

typedef struct
{
    uint8_t handle_count;
} __packed serial_evt_cmd_rsp_params_handle_count_t;

typedef struct
{
    uint32_t adv_int;
} __packed serial_evt_cmd_rsp_params_adv_int_t;

typedef struct
{
    uint8_t handle;
    addr_type_t addr_type;
    uint8_t origin_addr[6];
    uint8_t data[RBC_MESH_VALUE_MAX_LEN];
} __packed serial_evt_cmd_rsp_params_val_get_t;


/****** EVT PARAMS ******/
typedef struct
{
    uint8_t data[29];
} __packed serial_evt_params_echo_t;

typedef struct
{
    uint8_t command_opcode;
    aci_status_code_t status;
    union
    {
        serial_evt_cmd_rsp_params_build_version_t build_version;
        serial_evt_cmd_rsp_params_access_addr_t access_addr;
        serial_evt_cmd_rsp_params_channel_t channel;
        serial_evt_cmd_rsp_params_handle_count_t handle_count;
        serial_evt_cmd_rsp_params_adv_int_t adv_int;
        serial_evt_cmd_rsp_params_val_get_t val_get;
    } __packed response;        
} __packed serial_evt_params_cmd_rsp_t;

typedef struct
{
    uint8_t handle;
    addr_type_t addr_type;
    uint8_t origin_addr[6];
    uint8_t data[RBC_MESH_VALUE_MAX_LEN];
} __packed serial_evt_params_event_new_t;

typedef struct
{
    uint8_t handle;
    addr_type_t addr_type;
    uint8_t origin_addr[6];
    uint8_t data[RBC_MESH_VALUE_MAX_LEN];
} __packed serial_evt_params_event_update_t;

typedef struct
{
    uint8_t handle;
    addr_type_t addr_type;
    uint8_t origin_addr[6];
    uint8_t data[RBC_MESH_VALUE_MAX_LEN];
} __packed serial_evt_params_event_conflicting_t;

typedef struct
{
    operating_mode_t operating_mode;
    uint8_t hw_error;
    uint8_t data_credit_available;
} __packed serial_evt_params_event_device_started_t;

typedef struct
{
    uint8_t length;
	uint8_t opcode;
	union
	{
		serial_evt_params_echo_t echo;
        serial_evt_params_cmd_rsp_t cmd_rsp;
        serial_evt_params_event_new_t event_new;
        serial_evt_params_event_update_t event_update;
        serial_evt_params_event_conflicting_t event_conflicting;
	} __packed params;
} __packed serial_evt_t;


#endif /* _SERIAL_EVENT_H__ */
