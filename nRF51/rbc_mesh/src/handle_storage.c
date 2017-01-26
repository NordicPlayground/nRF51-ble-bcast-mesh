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
#include <stddef.h>
#include <string.h>
#include <stdlib.h>

#include "handle_storage.h"
#include "trickle.h"
#include "event_handler.h"
#include "fifo.h"
#include "rbc_mesh_common.h"
#include "timer.h"
#include "app_error.h"

#define MESH_TRICKLE_I_MAX              (2048)
#define MESH_TRICKLE_K                  (3)


#define HANDLE_CACHE_ENTRY_INVALID      (RBC_MESH_HANDLE_CACHE_ENTRIES)
#define DATA_CACHE_ENTRY_INVALID        (RBC_MESH_DATA_CACHE_ENTRIES)

#define CACHE_TASK_FIFO_SIZE            (8)

#define HANDLE_CACHE_ITERATE(index)     do { index = m_handle_cache[index].index_next; } while (0)
#define HANDLE_CACHE_ITERATE_BACK(index)     do { index = m_handle_cache[index].index_prev; } while (0)

/*****************************************************************************
* Local Typedefs
*****************************************************************************/
typedef struct
{
    rbc_mesh_value_handle_t handle;             /** data handle */
    uint16_t                version;            /** last received handle version */
    uint16_t                index_next : 15;    /** linked list index next */
    uint16_t                tx_event   : 1;     /** TX event flag */
    uint16_t                index_prev : 15;    /** linked list index prev */
    uint16_t                persistent : 1;     /** Persistent flag */
    uint16_t                data_entry;         /** index of the associated data entry */
} handle_entry_t;

typedef struct
{
    trickle_t trickle;
    mesh_packet_t* p_packet;
} data_entry_t;

/******************************************************************************
* Static globals
******************************************************************************/
static handle_entry_t   m_handle_cache[RBC_MESH_HANDLE_CACHE_ENTRIES];
static data_entry_t     m_data_cache[RBC_MESH_DATA_CACHE_ENTRIES];
static uint32_t         m_handle_cache_head;
static uint32_t         m_handle_cache_tail;

/*****************************************************************************
* Static Functions
*****************************************************************************/
static void version_increment(uint16_t* version)
{
    if (*version == UINT16_MAX)
    {
        *version = MESH_VALUE_LOLLIPOP_LIMIT;
    }
    else
    {
        (*version)++;
    }
}

static void data_entry_free(data_entry_t* p_data_entry)
{
    if (p_data_entry == NULL)
        return;

    if (p_data_entry->p_packet != NULL)
    {
        mesh_packet_ref_count_dec(p_data_entry->p_packet); /* data cache ref remove */
        p_data_entry->p_packet = NULL;
    }
    /* reset trickle params */
    trickle_enable(&p_data_entry->trickle);
}

/** Allocate a new data entry. Will take the least recently updated entry if all are allocated.
  Returns the index of the resulting entry. */
static uint16_t data_entry_allocate(void)
{
    for (uint32_t i = 0; i < RBC_MESH_DATA_CACHE_ENTRIES; ++i)
    {
        if (m_data_cache[i].p_packet == NULL)
        {
            trickle_timer_reset(&m_data_cache[i].trickle, 0);
            return i;
        }
    }

    /* no unused entries, take the least recently updated (and disregard persistent handles) */
    uint32_t handle_index = m_handle_cache_tail;
    while (m_handle_cache[handle_index].data_entry == DATA_CACHE_ENTRY_INVALID ||
           m_handle_cache[handle_index].persistent)
    {
        HANDLE_CACHE_ITERATE_BACK(handle_index);

        if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
        {
            return DATA_CACHE_ENTRY_INVALID;
        }
    }

    uint32_t data_index = m_handle_cache[handle_index].data_entry;
    APP_ERROR_CHECK_BOOL(data_index < RBC_MESH_DATA_CACHE_ENTRIES);

    /* cleanup */
    m_handle_cache[handle_index].data_entry = DATA_CACHE_ENTRY_INVALID;

    data_entry_free(&m_data_cache[data_index]);
    trickle_timer_reset(&m_data_cache[data_index].trickle, 0);
    return data_index;
}

/** Get the index of the handle entry representing the given handle.
  Returns HANDLE_CACHE_ENTRY_INVALID if not found */
static uint16_t handle_entry_get(rbc_mesh_value_handle_t handle, bool shortcut)
{
    static uint16_t prev_index = HANDLE_CACHE_ENTRY_INVALID;
    static uint16_t prev_handle = RBC_MESH_INVALID_HANDLE;

    event_handler_critical_section_begin();

    if (shortcut)
    {
        /* shortcut when accessing the same element in succession. */
        if (prev_handle == handle &&
            prev_index < RBC_MESH_HANDLE_CACHE_ENTRIES &&
            m_handle_cache[prev_index].handle == handle)
        {
            event_handler_critical_section_end();
            return prev_index;
        }
    }

    uint16_t i = m_handle_cache_head;

    while (m_handle_cache[i].handle != handle)
    {
        if (m_handle_cache_tail == i)
        {
            event_handler_critical_section_end();
            return HANDLE_CACHE_ENTRY_INVALID; /* checked all entries */
        }
        HANDLE_CACHE_ITERATE(i);
    }

    if (shortcut)
    {
        prev_handle = handle;
        prev_index = i;
    }

    event_handler_critical_section_end();
    return i;
}

/** Moves the given handle to the head of the handle cache.
  If it doesn't exist, it allocates the tail, and moves it to head.
  Returns the index in the cache, or HANDLE_CACHE_ENTRY_INVALID if the cache
  is full of persistent handles */
static uint16_t handle_entry_to_head(rbc_mesh_value_handle_t handle)
{
    uint16_t i = handle_entry_get(handle, true);
    if (i == HANDLE_CACHE_ENTRY_INVALID)
    {
        i = m_handle_cache_tail;
        while (m_handle_cache[i].persistent)
        {
            HANDLE_CACHE_ITERATE_BACK(i);
            if (i == HANDLE_CACHE_ENTRY_INVALID)
            {
                return i; /* reached the head without hitting a non-persistent handle */
            }
        }
        /* clean up old data */
        m_handle_cache[i].handle = handle;
        m_handle_cache[i].tx_event = 0;
        m_handle_cache[i].version = 0;
        if (m_handle_cache[i].data_entry != DATA_CACHE_ENTRY_INVALID)
        {
            data_entry_free(&m_data_cache[m_handle_cache[i].data_entry]);
            m_handle_cache[i].data_entry = DATA_CACHE_ENTRY_INVALID;
        }
    }
    /* detach and move to head */
    if (i != m_handle_cache_tail)
    {
        if (i != m_handle_cache_head)
        {
            m_handle_cache[m_handle_cache[i].index_next].index_prev = m_handle_cache[i].index_prev;
        }
    }
    else
    {
        m_handle_cache_tail = m_handle_cache[i].index_prev;
        m_handle_cache[i].index_next = HANDLE_CACHE_ENTRY_INVALID;
    }

    if (i != m_handle_cache_head)
    {
        if (m_handle_cache[i].index_prev != HANDLE_CACHE_ENTRY_INVALID)
        {
            m_handle_cache[m_handle_cache[i].index_prev].index_next = m_handle_cache[i].index_next;
        }

        m_handle_cache[i].index_prev = HANDLE_CACHE_ENTRY_INVALID;
        m_handle_cache[i].index_next = m_handle_cache_head;
        m_handle_cache[m_handle_cache_head].index_prev = i;
        m_handle_cache_head = i;
    }

    m_handle_cache[m_handle_cache_head].index_prev = HANDLE_CACHE_ENTRY_INVALID;
    m_handle_cache[m_handle_cache_tail].index_next = HANDLE_CACHE_ENTRY_INVALID;

    return i;
}

void local_packet_push(void* p_context)
{
    mesh_packet_t* p_packet = (mesh_packet_t*) p_context;
    mesh_adv_data_t* p_adv = mesh_packet_adv_data_get(p_packet);
    if (p_adv != NULL)
    {
        handle_info_t info =
        {
            .version = p_adv->version,
            .p_packet = p_packet
        };
        uint16_t handle_index = handle_entry_get(p_adv->handle, true);
        if (handle_index != HANDLE_CACHE_ENTRY_INVALID)
        {
            info.version = m_handle_cache[handle_index].version;
            version_increment(&info.version);
        }
        p_adv->version = info.version;

        handle_storage_info_set(p_adv->handle, &info);
    }
    mesh_packet_ref_count_dec(p_packet); /* for the event queue */
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/
uint32_t handle_storage_init(uint32_t min_interval_us)
{
    event_handler_critical_section_begin();
    uint32_t error_code = handle_storage_min_interval_set(min_interval_us);
    if (error_code != NRF_SUCCESS)
    {
        event_handler_critical_section_end();
        return error_code;
    }

    for (uint32_t i = 0; i < RBC_MESH_DATA_CACHE_ENTRIES; ++i)
    {
        m_data_cache[i].p_packet = NULL;
    }

    for (uint32_t i = 0; i < RBC_MESH_HANDLE_CACHE_ENTRIES; ++i)
    {
        m_handle_cache[i].handle = RBC_MESH_INVALID_HANDLE;
        m_handle_cache[i].version = 0;
        m_handle_cache[i].persistent = 0;
        m_handle_cache[i].tx_event = 0;
        m_handle_cache[i].data_entry = DATA_CACHE_ENTRY_INVALID;
        m_handle_cache[i].index_prev = i - 1;
        m_handle_cache[i].index_next = i + 1;
    }

    m_handle_cache_head = 0;
    m_handle_cache_tail = RBC_MESH_HANDLE_CACHE_ENTRIES - 1;
    m_handle_cache[m_handle_cache_head].index_prev = HANDLE_CACHE_ENTRY_INVALID;
    m_handle_cache[m_handle_cache_tail].index_next = HANDLE_CACHE_ENTRY_INVALID;

    event_handler_critical_section_end();
    return NRF_SUCCESS;
}

uint32_t handle_storage_min_interval_set(uint32_t min_interval_us)
{
    if (min_interval_us < RBC_MESH_INTERVAL_MIN_MIN_MS * 1000 ||
        min_interval_us > RBC_MESH_INTERVAL_MIN_MAX_MS * 1000)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    trickle_setup(min_interval_us, MESH_TRICKLE_I_MAX, MESH_TRICKLE_K);

    return NRF_SUCCESS;
}

uint32_t handle_storage_info_get(uint16_t handle, handle_info_t* p_info)
{
    if (p_info == NULL)
    {
        return NRF_ERROR_NULL;
    }
    memset(p_info, 0, sizeof(handle_info_t));
    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    event_handler_critical_section_begin();

    uint16_t handle_index = handle_entry_get(handle, false);
    if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
    {
        event_handler_critical_section_end();
        return NRF_ERROR_NOT_FOUND;
    }

    p_info->version = m_handle_cache[handle_index].version;
    if (m_handle_cache[handle_index].data_entry != DATA_CACHE_ENTRY_INVALID)
    {
        if (mesh_packet_ref_count_inc(m_data_cache[m_handle_cache[handle_index].data_entry].p_packet))
        {
            p_info->p_packet = m_data_cache[m_handle_cache[handle_index].data_entry].p_packet;
        }
    }

    event_handler_critical_section_end();
    return NRF_SUCCESS;
}

uint32_t handle_storage_info_set(uint16_t handle, handle_info_t* p_info)
{
    if (p_info == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    uint16_t handle_index = handle_entry_get(handle, true);
    if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
    {
        /* couldn't find an existing entry, allocate one */
        handle_index = handle_entry_to_head(handle);
        if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
        {
            return NRF_ERROR_NO_MEM;
        }
    }

    uint16_t data_index = m_handle_cache[handle_index].data_entry;

    if (data_index == DATA_CACHE_ENTRY_INVALID)
    {
        data_index = data_entry_allocate();
        if (data_index == DATA_CACHE_ENTRY_INVALID)
        {
            return NRF_ERROR_NO_MEM;
        }
        m_handle_cache[handle_index].data_entry = data_index;
    }
    trickle_timer_reset(&m_data_cache[data_index].trickle, timer_now());

    m_handle_cache[handle_index].version = p_info->version;
    if (m_data_cache[data_index].p_packet != NULL)
    {
        mesh_packet_ref_count_dec(m_data_cache[data_index].p_packet);
    }

    /* reference for the cache */
    mesh_packet_ref_count_inc(p_info->p_packet);
    m_data_cache[m_handle_cache[handle_index].data_entry].p_packet = p_info->p_packet;
    return NRF_SUCCESS;
}

uint32_t handle_storage_local_packet_push(mesh_packet_t* p_packet)
{
    if (p_packet == NULL)
    {
        return NRF_ERROR_NULL;
    }
    async_event_t evt;
    evt.type = EVENT_TYPE_GENERIC;
    evt.callback.generic.p_context = p_packet;
    evt.callback.generic.cb = local_packet_push;

    mesh_packet_ref_count_inc(p_packet);
    uint32_t error_code = event_handler_push(&evt);
    if (error_code != NRF_SUCCESS)
    {
        mesh_packet_ref_count_dec(p_packet);
    }
    return error_code;
}

uint32_t handle_storage_flag_set(uint16_t handle, handle_flag_t flag, bool value)
{
    if (flag >= HANDLE_FLAG__MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    uint16_t handle_index = handle_entry_get(handle, true);

    switch (flag)
    {
        case HANDLE_FLAG_PERSISTENT:
            if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
            {
                handle_index = handle_entry_to_head(handle);

                if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
                {
                    return NRF_ERROR_NO_MEM;
                }
            }
            m_handle_cache[handle_index].persistent = value;
            break;

        case HANDLE_FLAG_TX_EVENT:
            if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
            {
                handle_index = handle_entry_to_head(handle);

                if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
                {
                    return NRF_ERROR_NO_MEM;
                }
            }
            m_handle_cache[handle_index].tx_event = value;
            break;

        case HANDLE_FLAG_DISABLED:
            if (value)
            {
                if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
                {
                    return NRF_SUCCESS; /* the value is already disabled */
                }
                if (m_handle_cache[handle_index].data_entry == DATA_CACHE_ENTRY_INVALID)
                {
                    return NRF_SUCCESS; /* the value is already disabled */
                }
                trickle_disable(&m_data_cache[m_handle_cache[handle_index].data_entry].trickle);
            }
            else
            {
                uint32_t error_code;
                mesh_packet_t* p_packet = NULL;
                if (!mesh_packet_acquire(&p_packet))
                {
                    return NRF_ERROR_NO_MEM;
                }

                error_code = mesh_packet_build(p_packet,
                        handle,
                        0,
                        NULL,
                        0);
                if (error_code != NRF_SUCCESS)
                {
                    mesh_packet_ref_count_dec(p_packet);
                    return error_code;
                }

                if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
                {
                    /* may safely run this function inline, as we're already in event handler */
                    local_packet_push(p_packet);
                    return NRF_SUCCESS;
                }
                else
                {
                    if (m_handle_cache[handle_index].data_entry == DATA_CACHE_ENTRY_INVALID)
                    {
                        m_handle_cache[handle_index].data_entry = data_entry_allocate();
                        if (m_handle_cache[handle_index].data_entry == DATA_CACHE_ENTRY_INVALID)
                        {
                            return NRF_ERROR_NO_MEM;
                        }
                    }
                    if (m_data_cache[m_handle_cache[handle_index].data_entry].p_packet != NULL)
                    {
                        /* someone set the packet already, let's not overwrite it. */
                        mesh_packet_ref_count_dec(p_packet);
                    }
                    else
                    {
                        m_data_cache[m_handle_cache[handle_index].data_entry].p_packet = p_packet;
                    }
                    trickle_enable(&m_data_cache[m_handle_cache[handle_index].data_entry].trickle);
                }
            }
            break;
        default:
            return NRF_ERROR_INVALID_PARAM;
    }

    return NRF_SUCCESS;
}

uint32_t handle_storage_flag_set_async(uint16_t handle, handle_flag_t flag, bool value)
{
    async_event_t evt;
    evt.type = EVENT_TYPE_SET_FLAG;
    evt.callback.set_flag.handle = handle;
    evt.callback.set_flag.flag = flag;
    evt.callback.set_flag.value = value;
    return event_handler_push(&evt);
}

uint32_t handle_storage_flag_get(uint16_t handle, handle_flag_t flag, bool* p_value)
{
    if (flag >= HANDLE_FLAG__MAX)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    if (p_value == NULL)
    {
        return NRF_ERROR_NULL;
    }
    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    event_handler_critical_section_begin();

    uint16_t handle_index = handle_entry_get(handle, false);
    if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
    {
        event_handler_critical_section_end();
        return NRF_ERROR_NOT_FOUND;
    }

    switch (flag)
    {
        case HANDLE_FLAG_PERSISTENT:
            *p_value = m_handle_cache[handle_index].persistent;
            break;
        case HANDLE_FLAG_TX_EVENT:
            *p_value = m_handle_cache[handle_index].tx_event;
            break;
        default:
            event_handler_critical_section_end();
            return NRF_ERROR_INVALID_PARAM;
    }

    event_handler_critical_section_end();
    return NRF_SUCCESS;
}

uint32_t handle_storage_rx_consistent(uint16_t handle, uint32_t timestamp)
{
    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    uint16_t handle_index = handle_entry_get(handle, true);
    if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    uint16_t data_index = m_handle_cache[handle_index].data_entry;

    if (data_index == DATA_CACHE_ENTRY_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    trickle_rx_consistent(&m_data_cache[data_index].trickle, timestamp);

    return NRF_SUCCESS;
}

uint32_t handle_storage_rx_inconsistent(uint16_t handle, uint32_t timestamp)
{
    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    uint16_t handle_index = handle_entry_get(handle, true);
    if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    uint16_t data_index = m_handle_cache[handle_index].data_entry;

    if (data_index == DATA_CACHE_ENTRY_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    trickle_rx_inconsistent(&m_data_cache[data_index].trickle, timestamp);

    return NRF_SUCCESS;
}
uint32_t handle_storage_next_timeout_get(bool* p_found_value)
{
    uint32_t time_earliest = 0;
    *p_found_value = false;
    for (uint32_t i = 0; i < RBC_MESH_DATA_CACHE_ENTRIES; ++i)
    {
        if (trickle_is_enabled(&m_data_cache[i].trickle) &&
            m_data_cache[i].p_packet != NULL &&
            (!(*p_found_value) || TIMER_OLDER_THAN(m_data_cache[i].trickle.t, time_earliest)))
        {
            *p_found_value = true;
            time_earliest = m_data_cache[i].trickle.t;
        }
    }
    return time_earliest;
}

uint32_t handle_storage_tx_packets_get(uint32_t time_now, mesh_packet_t** pp_packets, uint32_t* p_count)
{
    /* Continue where we left off */
    static uint16_t data_index = 0;

    uint32_t count = 0;

    for (uint32_t i = 0; i < RBC_MESH_DATA_CACHE_ENTRIES; ++i)
    {
        while (data_index >= RBC_MESH_DATA_CACHE_ENTRIES)
            data_index -= RBC_MESH_DATA_CACHE_ENTRIES;

        if ((m_data_cache[data_index].p_packet != NULL) &&
            (trickle_is_enabled(&m_data_cache[data_index].trickle)) &&
            !TIMER_OLDER_THAN(time_now, m_data_cache[data_index].trickle.t))
        {
            bool do_tx = false;
            trickle_tx_timeout(&m_data_cache[data_index].trickle, &do_tx, time_now);
            if (do_tx)
            {
                mesh_packet_ref_count_inc(m_data_cache[data_index].p_packet); /* return the packet with an additional reference */
                pp_packets[count++] = m_data_cache[data_index].p_packet;
                if (count == *p_count)
                {
                    /* packet array is full */
                    data_index++;
                    return NRF_SUCCESS;
                }
            }
        }
        data_index++;
    }
    *p_count = count;

    return NRF_SUCCESS;
}

uint32_t handle_storage_transmitted(uint16_t handle, uint32_t timestamp)
{
    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    uint16_t handle_index = handle_entry_get(handle, true);
    if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    uint16_t data_index = m_handle_cache[handle_index].data_entry;
    if (data_index == DATA_CACHE_ENTRY_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    trickle_tx_register(&m_data_cache[data_index].trickle, timestamp);

    return NRF_SUCCESS;
}
