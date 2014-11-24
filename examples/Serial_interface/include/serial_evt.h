#ifndef _SERIAL_EVENT_H__
#define _SERIAL_EVENT_H__

#include "serial_handler.h"
#include <stdint.h>
#include <stdbool.h>

#define SERIAL_EVT_OPCODE_ECHO_RSP              (0xB0)
#define SERIAL_EVT_OPCODE_CMD_RSP               (0xB1)
#define SERIAL_EVT_OPCODE_EVENT_NEW             (0xB2)
#define SERIAL_EVT_OPCODE_EVENT_UPDATE          (0xB3)
#define SERIAL_EVT_OPCODE_EVENT_CONFLICTING     (0xB4)
#define SERIAL_EVT_OPCODE_BUILD_VERSION_RSP     (0xB5)

typedef __packed struct
{
    uint8_t data[SERIAL_DATA_MAX_LEN];
} serial_evt_params_echo_t;

typedef __packed struct
{
    uint8_t command_opcode;
    uint32_t response;
} serial_evt_params_cmd_rsp_t;

typedef __packed struct
{
    uint8_t handle;
    uint8_t origin_addr[6];
    uint8_t data;
} serial_evt_params_event_new_t;

typedef __packed struct
{
    uint8_t handle;
    uint8_t origin_addr[6];
    uint8_t data;
} serial_evt_params_event_update_t;

typedef __packed struct
{
    uint8_t handle;
    uint8_t origin_addr[6];
    uint8_t data;
} serial_evt_params_event_conflicting_t;

typedef __packed struct
{
    uint8_t major;
    uint8_t minor_1;
    uint8_t minor_2;
} serial_evt_params_build_version_rsp_t;



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
        serial_evt_params_build_version_rsp_t build_version_rsp;
	} params;
} serial_evt_t;


#endif /* _SERIAL_EVENT_H__ */
