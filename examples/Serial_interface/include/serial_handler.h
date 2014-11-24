#ifndef _SERIAL_HANDLER_H__
#define _SERIAL_HANDLER_H__

#define SERIAL_DATA_MAX_LEN  (36)

#include "serial_evt.h"
#include "serial_command.h"

#include <stdint.h>


typedef __packed struct 
{
	uint8_t status_byte;
	uint8_t buffer[SERIAL_DATA_MAX_LEN + 2];
} serial_data_t;



typedef void (*serial_cmd_handler)(serial_cmd_t*);

void serial_handler_init(serial_cmd_handler cmd_handler);

void serial_handler_event_send(serial_evt_t* evt);




#endif /* _SERIAL_HANDLER_H__ */
