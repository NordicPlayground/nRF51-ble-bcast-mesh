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

#include "nrf.h"

#include "event_handler.h"
#include "fifo.h"
#include "nrf_flash.h"
#include "nrf_error.h"
#include "toolchain.h"
#include "app_error.h"
#include "dfu_util.h"

/*****************************************************************************
* Local defines
*****************************************************************************/

/** Number of flash operations that can be queued at once. */
#define FLASH_OP_QUEUE_LEN					(8)

/** Maximum time spent after a flash operation for cleanup. */
#define FLASH_OP_POST_PROCESS_TIME_US		(500)
/** Longest time spent on a single flash operation. Longer operations will be
 * broken up. */
#define FLASH_OP_MAX_TIME_US                (100000)

#if defined(NRF51)
/** Timer to erase a single flash page. */
#define FLASH_TIME_TO_ERASE_PAGE_US         (22050)
/** Timer to write a single flash word. */
#define FLASH_TIME_TO_WRITE_ONE_WORD_US     (48)
#elif defined(NRF52)
/** Timer to erase a single flash page. */
#define FLASH_TIME_TO_ERASE_PAGE_US         (89700)
/** Timer to write a single flash word. */
#define FLASH_TIME_TO_WRITE_ONE_WORD_US     (338)
#else
/** Timer to erase a single flash page. */
#define FLASH_TIME_TO_ERASE_PAGE_US         (20000)
/** Timer to write a single flash word. */
#define FLASH_TIME_TO_WRITE_ONE_WORD_US     (50)
#endif
/*****************************************************************************
* Local typedefs
*****************************************************************************/
/** Single flash operation. */
typedef struct
{
    flash_op_type_t type;     /**< Type of flash operation. */
    flash_op_t operation;     /**< Operation parameters. */
} operation_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
static fifo_t				m_flash_op_fifo;                           /**< FIFO structure for the flash operations. */
static operation_t			m_flash_op_fifo_queue[FLASH_OP_QUEUE_LEN]; /**< FIFO buffer for flash operations. */
static mesh_flash_op_cb_t	mp_cb;                                     /**< Flash operation end callback pointer. Called when a flash operation ended. */
static operation_t          m_curr_op;                                 /**< Current flash operation. */
static uint32_t             m_op_addr;                                 /**< Start address of current operation. */
static bool                 m_suspended;                               /**< Suspend flag, preventing flash operations while set. */

/* In order to check that all flash events have been reported to the user, the
 * module need to keep track of how many events have been popped from the operation queue.
 * There might be multiple operation reports queued to the bearer event, without this module
 * being able to tell. The @c m_operation_count and @c m_operations_reported variables are
 * responsible for keeping track of this.
 */
static uint32_t             m_operation_count;                         /**< Number of flash operations executed since bootup. */
static uint32_t             m_operations_reported;                     /**< Number of flash operations reported to app as ended since bootup. */
/*****************************************************************************
* Static functions
*****************************************************************************/

static timestamp_t operation_time(const operation_t* p_op)
{
    switch (p_op->type)
    {
        case FLASH_OP_TYPE_WRITE:
            return ((p_op->operation.write.length + WORD_SIZE - 1) / WORD_SIZE) * FLASH_TIME_TO_WRITE_ONE_WORD_US;
        case FLASH_OP_TYPE_ERASE:
            return ((p_op->operation.erase.length + PAGE_SIZE - 1) / PAGE_SIZE) * FLASH_TIME_TO_ERASE_PAGE_US;
        case FLASH_OP_TYPE_NONE:
            return 0;
        default:
            return 0xFFFFFFFF;
    }
}

static void operation_execute(operation_t* p_op)
{
    switch (p_op->type)
    {
        case FLASH_OP_TYPE_ERASE:
            nrf_flash_erase((uint32_t*) p_op->operation.erase.start_addr, p_op->operation.erase.length);
            break;
        case FLASH_OP_TYPE_WRITE:
            nrf_flash_store((uint32_t*) p_op->operation.write.start_addr,
                            p_op->operation.write.p_data,
                            p_op->operation.write.length, 0);
            break;
        default:
            APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA);
    }
}

static void write_as_much_as_possible(flash_op_t* p_write_op, timestamp_t* p_available_time, uint32_t* p_bytes_written)
{
    const uint32_t max_time = ((*p_available_time < FLASH_OP_MAX_TIME_US) ? *p_available_time : FLASH_OP_MAX_TIME_US);
    uint32_t bytes_to_write = WORD_SIZE * ((max_time - FLASH_OP_POST_PROCESS_TIME_US) / FLASH_TIME_TO_WRITE_ONE_WORD_US);
    if (bytes_to_write > p_write_op->write.length)
    {
        bytes_to_write = p_write_op->write.length;
    }

    if (bytes_to_write == 0 || max_time < FLASH_OP_POST_PROCESS_TIME_US)
    {
        *p_bytes_written = 0;
        return;
    }

    *p_bytes_written = bytes_to_write;

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

    if (bytes_to_erase == 0 || max_time < FLASH_OP_POST_PROCESS_TIME_US)
    {
        *p_bytes_erased = 0;
        return;
    }

    *p_bytes_erased = bytes_to_erase;

    operation_t temp_op;
    temp_op.type = FLASH_OP_TYPE_ERASE;
    temp_op.operation.erase.start_addr = p_erase_op->erase.start_addr;
    temp_op.operation.erase.length = bytes_to_erase;
    operation_execute(&temp_op);
    p_erase_op->erase.length -= bytes_to_erase;
    p_erase_op->erase.start_addr += bytes_to_erase;
    *p_available_time -= operation_time(&temp_op);
}

static inline bool all_operations_ended(void)
{
    return (fifo_is_empty(&m_flash_op_fifo) && (m_operations_reported == m_operation_count));
}

static void write_operation_ended(void* p_location)
{
    mp_cb(FLASH_OP_TYPE_WRITE, p_location);
    ++m_operations_reported;
    if (all_operations_ended())
    {
        mp_cb(FLASH_OP_TYPE_ALL, NULL);
    }
}

static void erase_operation_ended(void* p_source)
{
    mp_cb(FLASH_OP_TYPE_ERASE, p_source);
    ++m_operations_reported;
    if (all_operations_ended())
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
        return false;
    }

    return true;
}

static bool next_operation_get(void)
{
    /* Get next operation */
    if (fifo_pop(&m_flash_op_fifo, &m_curr_op) != NRF_SUCCESS)
    {
        m_curr_op.type = FLASH_OP_TYPE_NONE;
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

uint32_t mesh_flash_init(mesh_flash_op_cb_t cb)
{
    if (cb == NULL)
    {
        return NRF_ERROR_NULL;
    }
    m_flash_op_fifo.elem_array = m_flash_op_fifo_queue;
    m_flash_op_fifo.elem_size = sizeof(operation_t);
    m_flash_op_fifo.array_len = FLASH_OP_QUEUE_LEN;
    fifo_init(&m_flash_op_fifo);
    mp_cb = cb;
    m_curr_op.type = FLASH_OP_TYPE_NONE;

    return NRF_SUCCESS;
}

uint32_t mesh_flash_op_push(flash_op_type_t type, const flash_op_t* p_op)
{
    if (mp_cb == NULL)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    if (p_op == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (type == FLASH_OP_TYPE_WRITE)
    {
        if (!IS_WORD_ALIGNED(p_op->write.start_addr) ||
            !IS_WORD_ALIGNED(p_op->write.p_data))
        {
            return NRF_ERROR_INVALID_ADDR;
        }
        if (!IS_WORD_ALIGNED(p_op->write.length) ||
            p_op->write.length == 0)
        {
            return NRF_ERROR_INVALID_LENGTH;
        }
    }
    else if (type == FLASH_OP_TYPE_ERASE)
    {
        if (!IS_PAGE_ALIGNED(p_op->erase.start_addr))
        {
            return NRF_ERROR_INVALID_ADDR;
        }
        if (p_op->erase.length == 0)
        {
            return NRF_ERROR_INVALID_LENGTH;
        }
    }
    else
    {
        return NRF_ERROR_INVALID_PARAM;
    }

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
    while (true)
    {
        if (m_suspended)
        {
            return;
        }
        if (operation_time(&m_curr_op) == 0) /* done with previous event */
        {
            if (!send_end_evt())
            {
                return;
            }
            if (!next_operation_get())
            {
                return;
            }
        }

        uint32_t byte_count = 0;
        switch (m_curr_op.type)
        {
            case FLASH_OP_TYPE_ERASE:
                erase_as_much_as_possible(&m_curr_op.operation, &available_time, &byte_count);
                break;
            case FLASH_OP_TYPE_WRITE:
                write_as_much_as_possible(&m_curr_op.operation, &available_time, &byte_count);
                break;
            default:
                APP_ERROR_CHECK_BOOL(false);
        }

        if (available_time <= FLASH_OP_POST_PROCESS_TIME_US || byte_count == 0)
        {
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
    APP_ERROR_CHECK_BOOL(suspend || suspend_count > 0);
    suspend_count += (2 * suspend - 1);
    m_suspended = (suspend_count > 0);
    _ENABLE_IRQS(was_masked);
}
