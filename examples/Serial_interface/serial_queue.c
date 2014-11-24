#include "serial_queue.h"

#include "app_error.h"
#include <string.h>


void serial_queue_init(serial_queue_t *queue)
{
	uint8_t loop;

	APP_ERROR_CHECK_BOOL(NULL != queue);

	queue->head = 0;
	queue->tail = 0;
	for(loop=0; loop<SERIAL_QUEUE_SIZE; loop++)
	{
		queue->serial_data[loop].buffer[0] = 0x00;
		queue->serial_data[loop].buffer[1] = 0x00;
	}
}

bool serial_queue_dequeue(serial_queue_t *queue, serial_data_t *p_data)
{
	APP_ERROR_CHECK_BOOL(NULL != queue);
	APP_ERROR_CHECK_BOOL(NULL != p_data);

	if (serial_queue_is_empty(queue))
	{
		return false;
	}

	memcpy((uint8_t *)p_data, (uint8_t *)&(queue->serial_data[queue->head % SERIAL_QUEUE_SIZE]), sizeof(serial_data_t));
	++queue->head;

	return true;
}

bool serial_queue_enqueue(serial_queue_t *queue, serial_data_t *p_data)
{
	const uint8_t length = p_data->buffer[0];

	APP_ERROR_CHECK_BOOL(NULL != queue);
	APP_ERROR_CHECK_BOOL(NULL != p_data);

	if (serial_queue_is_full(queue))
	{
		return false;
	}

	queue->serial_data[queue->tail % SERIAL_QUEUE_SIZE].status_byte = 0;
	memcpy((uint8_t *)&(queue->serial_data[queue->tail % SERIAL_QUEUE_SIZE].buffer[0]), (uint8_t *)&p_data->buffer[0], length + 1);
	++queue->tail;

	return true;
}

bool serial_queue_is_empty(serial_queue_t *queue)
{
	bool state = false;

	APP_ERROR_CHECK_BOOL(NULL != queue);

	if (queue->head == queue->tail)
	{
		state = true;
	}

	return state;
}

bool serial_queue_is_full(serial_queue_t *queue)
{
	bool state;

	APP_ERROR_CHECK_BOOL(NULL != queue);

	state = (queue->tail == queue->head + SERIAL_QUEUE_SIZE);

	return state;
}

bool serial_queue_is_full_from_isr(serial_queue_t *queue)
{
	APP_ERROR_CHECK_BOOL(NULL != queue);

	return (queue->tail == queue->head + SERIAL_QUEUE_SIZE);
}

bool serial_queue_peek(serial_queue_t *queue, serial_data_t *p_data)
{
	APP_ERROR_CHECK_BOOL(NULL != queue);
	APP_ERROR_CHECK_BOOL(NULL != p_data);

	if (serial_queue_is_empty(queue))
	{
		return false;
	}

	memcpy((uint8_t *)p_data, (uint8_t *)&(queue->serial_data[queue->head % SERIAL_QUEUE_SIZE]), sizeof(serial_data_t));

	return true;
}
