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
#ifdef NRF51
#include "nrf51.h"
#else
#include "nrf.h"
#endif
#include "nrf_error.h"
#include "toolchain.h"
#include "rtt_log.h"
#include "app_error.h"

/*****************************************************************************
* Local defines
*****************************************************************************/

#define FLASH_OP_QUEUE_LEN					(8)

#define FLASH_OP_POST_PROCESS_TIME_US		(500)
#define FLASH_OP_MAX_TIME_US                (50000)

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
static operation_t          m_curr_op;
static uint32_t             m_op_addr;
static bool                 m_suspended;
static uint32_t             m_operation_count;
static uint32_t             m_operations_reported;

/*****************************************************************************
* Static functions
*****************************************************************************/

static timestamp_t operation_time(operation_t* p_op)
{
    if (p_op->type == FLASH_OP_TYPE_WRITE)
    {
        return ((p_op->operation.write.length + 3)/ 4) * FLASH_TIME_TO_WRITE_ONE_WORD_US;
    }
    if (p_op->type == FLASH_OP_TYPE_ERASE)
    {
        return ((p_op->operation.erase.length + PAGE_SIZE - 1) / PAGE_SIZE) * FLASH_TIME_TO_ERASE_PAGE_US;
    }
    if (p_op->type == FLASH_OP_TYPE_NONE)
    {
        return 0;
    }
    return 0xFFFFFFFF;
}

static void operation_execute(operation_t* p_op)
{
    NRF_GPIO->OUTSET = (1 << 0);
    if (p_op->type == FLASH_OP_TYPE_ERASE)
    {
        nrf_flash_erase((uint32_t*) p_op->operation.erase.start_addr, p_op->operation.erase.length);
        while (p_op->operation.erase.length == 0);
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

static void write_as_much_as_possible(flash_op_t* p_write_op, timestamp_t* p_available_time, uint32_t* p_bytes_written)
{
    const uint32_t max_time = ((*p_available_time < FLASH_OP_MAX_TIME_US) ? *p_available_time : FLASH_OP_MAX_TIME_US);
    uint32_t bytes_to_write = 4 * ((max_time - FLASH_OP_POST_PROCESS_TIME_US) / FLASH_TIME_TO_WRITE_ONE_WORD_US);
    if (bytes_to_write > p_write_op->write.length)
    {
        bytes_to_write = p_write_op->write.length;
    }

    *p_bytes_written = bytes_to_write;

    if (bytes_to_write == 0|| max_time < FLASH_OP_POST_PROCESS_TIME_US)
    {
        return;
    }

    operation_t temp_op;
    temp_op.type = FLASH_OP_TYPE_WRITE;
    temp_op.operation.write.start_addr = p_write_op->write.start_addr;
    temp_op.operation.write.p_data = p_write_op->write.p_data;
    temp_op.operation.write.length = bytes_to_write;
    operation_execute(&temp_op);
    p_write_op->write.length -= bytes_to_write;
    p_write_op->write.p_data += bytes_to_write;
    p_write_op->write.start_addr += bytes_to_write;
    *p_available_time -= operation_time(&temp_op);
}

static void erase_as_much_as_possible(flash_op_t* p_erase_op, timestamp_t* p_available_time, uint32_t* p_bytes_erased)
{
    const uint32_t max_time = ((*p_available_time < FLASH_OP_MAX_TIME_US) ? *p_available_time : FLASH_OP_MAX_TIME_US);
    uint32_t bytes_to_erase = PAGE_SIZE * ((max_time - FLASH_OP_POST_PROCESS_TIME_US) / FLASH_TIME_TO_ERASE_PAGE_US);
    if (bytes_to_erase > p_erase_op->erase.length)
    {
        bytes_to_erase = p_erase_op->erase.length;
    }

    *p_bytes_erased = bytes_to_erase;

    if (bytes_to_erase == 0 || max_time < FLASH_OP_POST_PROCESS_TIME_US)
    {
        return;
    }

    operation_t temp_op;
    temp_op.type = FLASH_OP_TYPE_ERASE;
    temp_op.operation.erase.start_addr = p_erase_op->erase.start_addr;
    temp_op.operation.erase.length = bytes_to_erase;
    operation_execute(&temp_op);
    p_erase_op->erase.length -= bytes_to_erase;
    p_erase_op->erase.start_addr += bytes_to_erase;
    *p_available_time -= operation_time(&temp_op);
}

static void write_operation_ended(void* p_location)
{
    mp_cb(FLASH_OP_TYPE_WRITE, p_location);
    ++m_operations_reported;
    if (fifo_is_empty(&m_flash_op_fifo) && (m_operations_reported == m_operation_count))
    {
        mp_cb(FLASH_OP_TYPE_ALL, NULL);
    }
}

static void erase_operation_ended(void* p_source)
{
    mp_cb(FLASH_OP_TYPE_ERASE, p_source);
    ++m_operations_reported;
    if (fifo_is_empty(&m_flash_op_fifo) && (m_operations_reported == m_operation_count))
    {
        mp_cb(FLASH_OP_TYPE_ALL, NULL);
    }
}

static bool send_end_evt(void)
{
    if (m_operation_count == 0 || m_curr_op.type == FLASH_OP_TYPE_NONE)
    {
        return true; /* no events to send */
    }
    async_event_t end_evt;
    end_evt.type = EVENT_TYPE_GENERIC;
    if (m_curr_op.type == FLASH_OP_TYPE_ERASE)
    {
        end_evt.callback.generic.cb = erase_operation_ended;
        end_evt.callback.generic.p_context = (void*) m_op_addr;
    }
    else
    {
        end_evt.callback.generic.cb = write_operation_ended;
        end_evt.callback.generic.p_context = (void*) m_op_addr;
    }
    if (event_handler_push(&end_evt) != NRF_SUCCESS)
    {
        NRF_GPIO->OUTCLR = (1 << 13);
        return false;
    }

#if 0
    if (fifo_is_empty(&m_flash_op_fifo))
    {
        async_event_t idle_evt;
        idle_evt.type = EVENT_TYPE_GENERIC;
        idle_evt.callback.generic.cb = all_operations_ended;
        idle_evt.callback.generic.p_context = NULL;
        if (event_handler_push(&idle_evt) == NRF_SUCCESS)
        {
            __LOG(RTT_CTRL_TEXT_CYAN "\tPushed idle!\n");
        }
        else
        {
            __LOG(RTT_CTRL_TEXT_RED "NOOOOOOOOOOOO\n");
            NRF_GPIO->OUTCLR = (1 << 13);
            return;
        }
    }
#endif
    return true;
}

static bool get_next_operation(void)
{
    /* Get next operation */
    if (fifo_pop(&m_flash_op_fifo, &m_curr_op) != NRF_SUCCESS)
    {
        m_curr_op.type = FLASH_OP_TYPE_NONE;
        NRF_GPIO->OUTCLR = (1 << 13);
        return false;
    }
    APP_ERROR_CHECK_BOOL(m_curr_op.type != FLASH_OP_TYPE_NONE);

    m_operation_count++;
    /* Save initial start address for the end-event */
    if (m_curr_op.type == FLASH_OP_TYPE_WRITE)
    {
        m_op_addr = (uint32_t) m_curr_op.operation.write.p_data;
    }
    else
    {
        m_op_addr = m_curr_op.operation.erase.start_addr;
    }
    return true;
}
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
    m_curr_op.type = FLASH_OP_TYPE_NONE;
}

uint32_t mesh_flash_op_push(flash_op_type_t type, const flash_op_t* p_op)
{
    APP_ERROR_CHECK_BOOL(type != FLASH_OP_TYPE_NONE);
    APP_ERROR_CHECK_BOOL(p_op != NULL);
    operation_t op;
    op.type = type;
    memcpy(&op.operation, p_op, sizeof(flash_op_t));
    return fifo_push(&m_flash_op_fifo, &op);
}

uint32_t mesh_flash_op_available_slots(void)
{
    return FLASH_OP_QUEUE_LEN - fifo_get_len(&m_flash_op_fifo);
}

bool mesh_flash_in_progress(void)
{
    return (!fifo_is_empty(&m_flash_op_fifo) || m_curr_op.type != FLASH_OP_TYPE_NONE);
}

void mesh_flash_op_execute(timestamp_t available_time)
{
    NRF_GPIO->OUTSET = (1 << 13);
    while (true)
    {
        if (m_suspended)
        {
            NRF_GPIO->OUTCLR = (1 << 13);
            return;
        }
        if (operation_time(&m_curr_op) == 0) /* done with previous event */
        {
            if (!send_end_evt())
            {
                NRF_GPIO->OUTCLR = (1 << 13);
                return;
            }
            if (!get_next_operation())
            {
                NRF_GPIO->OUTCLR = (1 << 13);
                return;
            }
        }

        uint32_t byte_count = 0;
        if (m_curr_op.type == FLASH_OP_TYPE_ERASE)
        {
            erase_as_much_as_possible(&m_curr_op.operation, &available_time, &byte_count);
        }
        else if (m_curr_op.type == FLASH_OP_TYPE_WRITE)
        {
            write_as_much_as_possible(&m_curr_op.operation, &available_time, &byte_count);
        }

        if (available_time <= FLASH_OP_POST_PROCESS_TIME_US || byte_count == 0)
        {
            NRF_GPIO->OUTCLR = (1 << 13);
            return;
        }

        available_time -= FLASH_OP_POST_PROCESS_TIME_US;
    }
}

void mesh_flash_set_suspended(bool suspend)
{
    static uint32_t suspend_count = 0;
    uint32_t was_masked;
    _DISABLE_IRQS(was_masked);
#if 0
    if (suspend)
    {
        __LOG(RTT_CTRL_TEXT_YELLOW "<<<<SUSPEND\n");
    }
    else
    {
        __LOG(RTT_CTRL_TEXT_YELLOW ">>>>RELEASE\n");
    }
#endif
    suspend_count += (2 * suspend - 1);
    m_suspended = (suspend_count > 0);
    _ENABLE_IRQS(was_masked);
}
