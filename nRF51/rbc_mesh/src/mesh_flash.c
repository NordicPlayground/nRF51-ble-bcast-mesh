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
#include <string.h>

#include "mesh_flash.h"
#include "event_handler.h"
#include "fifo.h"
#include "nrf_flash.h"
#include "nrf51.h"

/*****************************************************************************
* Local defines
*****************************************************************************/
#define FLASH_OP_QUEUE_LEN					(8)

#define FLASH_OP_POST_PROCESS_TIME_US		(300)

#ifdef NRF51
#define FLASH_TIME_TO_ERASE_PAGE_US         (22050)
#define FLASH_TIME_TO_WRITE_ONE_WORD_US     (48)
#endif

#ifdef NRF52
#define FLASH_TIME_TO_ERASE_PAGE_US         (89700)
#define FLASH_TIME_TO_WRITE_ONE_WORD_US     (338)
#endif
/*****************************************************************************
* Local typedefs
*****************************************************************************/
typedef struct
{
    flash_op_type_t type;
    flash_op_t operation;
} operation_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
static fifo_t				m_flash_op_fifo;
static operation_t			m_flash_op_fifo_queue[FLASH_OP_QUEUE_LEN];
static mesh_flash_op_cb_t	mp_cb;

/*****************************************************************************
* Static functions
*****************************************************************************/

timestamp_t operation_time(operation_t* p_op)
{
    if (p_op->type == FLASH_OP_TYPE_WRITE)
    {
        return (p_op->operation.write.length / 4) * FLASH_TIME_TO_WRITE_ONE_WORD_US;
    }
    if (p_op->type == FLASH_OP_TYPE_ERASE)
    {
        return (p_op->operation.erase.length / PAGE_SIZE) * FLASH_TIME_TO_ERASE_PAGE_US;
    }
    return 0xFFFFFFFF;
}

void operation_execute(operation_t* p_op)
{
    NRF_GPIO->OUTSET = (1 << 0);
    if (p_op->type == FLASH_OP_TYPE_ERASE)
    {
        nrf_flash_erase((uint32_t*) p_op->operation.erase.start_addr, p_op->operation.erase.length);
    }
    else
    {
        nrf_flash_store((uint32_t*) p_op->operation.write.start_addr,
                        (uint8_t*) p_op->operation.write.p_data,
                        p_op->operation.write.length,
                        0);
    }
    NRF_GPIO->OUTCLR = (1 << 0);
}

#ifdef ASYNC_END_EVT
static void write_operation_ended(void* p_source)
{
    m_cb(FLASH_OP_TYPE_WRITE, p_location);
}

static void erase_operation_ended(void* p_source)
{
    m_cb(FLASH_OP_TYPE_ERASE, p_source);
}
#endif
/*****************************************************************************
* Interface functions
*****************************************************************************/

void mesh_flash_init(mesh_flash_op_cb_t cb)
{
    mp_cb = cb;
    m_flash_op_fifo.elem_array = m_flash_op_fifo_queue;
    m_flash_op_fifo.elem_size = sizeof(operation_t);
    m_flash_op_fifo.array_len = FLASH_OP_QUEUE_LEN;
    fifo_init(&m_flash_op_fifo);
}

uint32_t mesh_flash_op_push(flash_op_type_t type, const flash_op_t* p_op)
{
    operation_t op;
    op.type = type;
    memcpy(&op.operation, p_op, sizeof(flash_op_t));
    return fifo_push(&m_flash_op_fifo, &op);
}

uint32_t mesh_flash_op_available_slots(void)
{
    return FLASH_OP_QUEUE_LEN - fifo_get_len(&m_flash_op_fifo);
}

void mesh_flash_op_execute(timestamp_t available_time)
{
    operation_t op;
    while (fifo_peek(&m_flash_op_fifo, &op) == NRF_SUCCESS)
    {
        timestamp_t required_time = operation_time(&op);
        if (required_time == 0xFFFFFFFF ||
                available_time < (required_time + FLASH_OP_POST_PROCESS_TIME_US))
        {
            break;
        }

#ifdef ASYNC_END_EVT
        /* The event push is the only thing that can actually fail, let's make sure it works. */
        async_evt_t end_evt;
        end_evt.type = EVENT_TYPE_GENERIC;
        if (op.type == FLASH_OP_TYPE_ERASE)
        {
            end_evt.callback.generic.cb = erase_operation_ended;
            end_evt.callback.generic.p_context = (void*) op.operation.erase.start_addr;
        }
        else
        {
            end_evt.callback.generic.cb = write_operation_ended;
            end_evt.callback.generic.p_context = (void*) op.operation.write.start_addr;
        }
        if (event_handler_push(&end_evt) == NRF_SUCCESS)
        {
            /* get down to business */
            operation_execute(&op);
            available_time -= (required_time + FLASH_OP_POST_PROCESS_TIME_US);
            fifo_pop(&m_flash_op_fifo, NULL);
        }
        else
        {
            break;
        }
#else
        operation_execute(&op);
        available_time -= (required_time + FLASH_OP_POST_PROCESS_TIME_US);
        fifo_pop(&m_flash_op_fifo, NULL);
        if (op.type == FLASH_OP_TYPE_ERASE)
        {
            mp_cb(op.type, (void*) op.operation.erase.start_addr);
        }
        else
        {
            mp_cb(op.type, op.operation.write.p_data);
        }
#endif
    }
}

