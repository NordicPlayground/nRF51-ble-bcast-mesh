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
	uint8_t length = p_data->buffer[0];
    
    /* secure that buffer length isn't violated */
    if (length > SERIAL_DATA_MAX_LEN + 2)
    {
        length = SERIAL_DATA_MAX_LEN + 2;
    }
    

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
