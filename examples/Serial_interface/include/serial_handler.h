#ifndef _SERIAL_HANDLER_H__
#define _SERIAL_HANDLER_H__

#define SERIAL_DATA_MAX_LEN  (36)

#include "serial_evt.h"
#include "serial_command.h"

#include <stdint.h>
#include <stdbool.h>


typedef __packed struct 
{
	uint8_t status_byte;
	uint8_t buffer[SERIAL_DATA_MAX_LEN + 2];
} serial_data_t;


void serial_handler_init(void);

bool serial_handler_event_send(serial_evt_t* evt);

bool serial_handler_command_get(serial_cmd_t* evt);




#endif /* _SERIAL_HANDLER_H__ */
