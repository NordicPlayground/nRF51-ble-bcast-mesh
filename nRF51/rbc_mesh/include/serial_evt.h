#ifndef _SERIAL_EVENT_H__
#define _SERIAL_EVENT_H__

#include "serial_handler.h"

#include "mesh_aci.h"
#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    SERIAL_EVT_OPCODE_ECHO_RSP              = 0x82,
    SERIAL_EVT_OPCODE_CMD_RSP               = 0x84,
    SERIAL_EVT_OPCODE_EVENT_NEW             = 0XB3,
    SERIAL_EVT_OPCODE_EVENT_UPDATE          = 0XB4,
    SERIAL_EVT_OPCODE_EVENT_CONFLICTING     = 0XB5
} serial_evt_opcode_t;


typedef enum
{
    ADDR_TYPE_BLE_GAP_ADV_ADDR,
    ADDR_TYPE_6LOWPAN
} __packed addr_type_t;


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
    uint8_t length;
	uint8_t opcode;
	__packed union
	{
		serial_evt_params_echo_t echo;
        serial_evt_params_cmd_rsp_t cmd_rsp;
        serial_evt_params_event_new_t event_new;
        serial_evt_params_event_update_t event_update;
        serial_evt_params_event_conflicting_t event_conflicting;
	} params;
} serial_evt_t;


#endif /* _SERIAL_EVENT_H__ */
