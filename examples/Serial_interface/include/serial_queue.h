#ifndef _SERIAL_QUEUE_H__
#define _SERIAL_QUEUE_H__

#include "serial_handler.h"
#include <stdint.h>
#include <stdbool.h>

#define SERIAL_QUEUE_SIZE 4

typedef struct
{
	serial_data_t		serial_data[SERIAL_QUEUE_SIZE];
	uint8_t					head;
	uint8_t 				tail;
} serial_queue_t;

void serial_queue_init(serial_queue_t* queue);

bool serial_queue_dequeue(serial_queue_t* queue, serial_data_t* data);

bool serial_queue_enqueue(serial_queue_t* queue, serial_data_t* data);

bool serial_queue_is_empty(serial_queue_t* queue);

bool serial_queue_is_full(serial_queue_t* queue);

bool serial_queue_peek(serial_queue_t* queue, serial_data_t* data);

#endif /* _SERIAL_QUEUE_H__ */
