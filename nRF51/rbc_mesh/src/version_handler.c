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

#include "version_handler.h"

#include "transport_control.h"
#include "timeslot_handler.h"
#include "timer_control.h"
#include "event_handler.h"
#include "rbc_mesh_common.h"
#include "toolchain.h"
#include "trickle.h"
#include "rbc_mesh.h"
#include "mesh_packet.h"
#include "mesh_aci.h"

#include "app_error.h"
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#define MESH_TRICKLE_I_MAX              (2000)
#define MESH_TRICKLE_K                  (3)

#define MESH_VALUE_LOLLIPOP_LIMIT       (200)

#define TIMESLOT_STARTUP_DELAY_US       (100)

#define HANDLE_CACHE_ENTRY_INVALID      (RBC_MESH_HANDLE_CACHE_ENTRIES)
#define DATA_CACHE_ENTRY_INVALID        (RBC_MESH_DATA_CACHE_ENTRIES)
/******************************************************************************
* Local typedefs
******************************************************************************/
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
static handle_entry_t m_handle_cache[RBC_MESH_HANDLE_CACHE_ENTRIES];
static data_entry_t m_data_cache[RBC_MESH_DATA_CACHE_ENTRIES];
static uint32_t m_handle_cache_head;
static uint32_t m_handle_cache_tail;
static bool g_is_initialized = false;

/******************************************************************************
* Static functions
******************************************************************************/
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

/** Allocate a new data entry. Will take the least recently updated entry if all are allocated.
  Returns the index of the resulting entry. */
static uint16_t data_entry_allocate(void)
{
    /** @TODO: Can this bit be removed when all entries are allocated?*/
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
        handle_index = m_handle_cache[handle_index].index_prev;
        if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
        {
            return DATA_CACHE_ENTRY_INVALID;
        }
    }

    uint32_t data_index = m_handle_cache[handle_index].data_entry;

    /* cleanup */
    m_handle_cache[handle_index].data_entry = DATA_CACHE_ENTRY_INVALID;

    trickle_enable(&m_data_cache[data_index].trickle);
    trickle_timer_reset(&m_data_cache[data_index].trickle, 0);
    if (m_data_cache[data_index].p_packet != NULL)
    {
        mesh_packet_free(m_data_cache[data_index].p_packet); /** @TODO: !!!! IS THIS SAFE? */
        m_data_cache[data_index].p_packet = NULL;
    }

    return data_index;
}

/** Get the index of the handle entry representing the given handle.
  Returns HANDLE_CACHE_ENTRY_INVALID if not found */
static uint16_t handle_entry_get(rbc_mesh_value_handle_t handle)
{
    uint16_t i = m_handle_cache_head;
    
    while (m_handle_cache[i].handle != handle)
    {
        if (m_handle_cache_tail == i)
        {
            return HANDLE_CACHE_ENTRY_INVALID; /* checked all entries */
        }
        i = m_handle_cache[i].index_next; /* iterate */
    }
    
    return i;
}

/** Moves the given handle to the head of the handle cache.
  If it doesn't exist, it allocates the tail, and moves it to head. 
  Returns the index in the cache, or HANDLE_CACHE_ENTRY_INVALID if the cache 
  is full of persistent handles */
static uint16_t handle_entry_to_head(rbc_mesh_value_handle_t handle)
{
    /* this may be executed in main context, and we don't want an event to 
       come in and change the linked list while we're working on it */
    event_handler_critical_section_begin();
    
    uint16_t i = handle_entry_get(handle);
    if (i == HANDLE_CACHE_ENTRY_INVALID)
    {
        i = m_handle_cache_tail;
        while (m_handle_cache[i].persistent)
        {
            i = m_handle_cache[i].index_prev;
            if (i == HANDLE_CACHE_ENTRY_INVALID)
            {
                event_handler_critical_section_end();
                return i; /* reached the head without hitting a non-persistent handle */
            }
        }
        /* clean up old data */
        m_handle_cache[i].handle = handle;
        m_handle_cache[i].tx_event = 0;
        m_handle_cache[i].version = 0;
        if (m_handle_cache[i].data_entry != DATA_CACHE_ENTRY_INVALID)
        {
             data_entry_t* p_data_entry = &m_data_cache[m_handle_cache[i].data_entry];
             if (p_data_entry->p_packet != NULL)
             {
                 mesh_packet_free(p_data_entry->p_packet); /** !!!! @TODO: IS THIS SAFE? */
                 p_data_entry->p_packet = NULL;
             }
             trickle_timer_reset(&p_data_entry->trickle, 0);
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
    
    event_handler_critical_section_end();
    
    return i;
}

/** Find or allocate the given handle, and give it a value. 
  Returns the index of the handle in the cache */
static uint16_t handle_version_set(rbc_mesh_value_handle_t handle, 
        uint16_t version, 
        bool* cache_hit,
        int16_t* delta)
{
    uint16_t i = handle_entry_to_head(handle);
    if (i == HANDLE_CACHE_ENTRY_INVALID)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }

    *delta = vh_get_version_delta(m_handle_cache[i].version, version);
    *cache_hit = (m_handle_cache[i].version > 0);

    if (*delta > 0)
    {
        m_handle_cache[i].version = version;
    }

    return i;
}

/** Find or allocate the given handle in the cache, and increment the version.
  Returns the index of the handle in the cache. */
static uint16_t handle_version_increment(rbc_mesh_value_handle_t handle)
{
    uint16_t i = handle_entry_to_head(handle);
    if (i == HANDLE_CACHE_ENTRY_INVALID)
    {
        APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
    }

    version_increment(&m_handle_cache[i].version);

    return i;
}

static void transmit_all_instances(uint64_t timestamp);

static void order_next_transmission(uint64_t time_now)
{
    uint64_t ts_begin_time = timeslot_get_global_time();
    uint64_t ts_end_time = timeslot_get_end_time();
    uint64_t earliest = UINT64_MAX;
    uint16_t handle_index = m_handle_cache_head;
    
    do
    {
        uint16_t data_index = m_handle_cache[handle_index].data_entry;
        if (data_index != DATA_CACHE_ENTRY_INVALID &&
            m_data_cache[data_index].p_packet != NULL && 
            m_data_cache[data_index].trickle.t < earliest
        )
        {
            earliest = m_data_cache[data_index].trickle.t;
        }

        handle_index = m_handle_cache[handle_index].index_next;
        
    } while (handle_index != m_handle_cache_tail);
    
    if (earliest < ts_end_time)
    {
        timer_order_cb(TIMER_INDEX_VH, earliest - ts_begin_time, transmit_all_instances);
    }
}

static void transmit_all_instances(uint64_t timestamp)
{
    /* the framework continues where it left off */
    static uint16_t data_index = 0;
    
    uint64_t ts_begin_time = timeslot_get_global_time();

    for (uint32_t i = 0; i < RBC_MESH_DATA_CACHE_ENTRIES; ++i)
    {
        while (data_index >= RBC_MESH_DATA_CACHE_ENTRIES)
            data_index -= RBC_MESH_DATA_CACHE_ENTRIES;

        if ((m_data_cache[data_index].p_packet != NULL) && 
            (m_data_cache[data_index].trickle.t <= timestamp + ts_begin_time))
        {
            bool do_tx = false;
            trickle_tx_timeout(&m_data_cache[data_index].trickle, &do_tx, timestamp + ts_begin_time);
            if (do_tx)
            {
                if (tc_tx(m_data_cache[data_index].p_packet) != NRF_SUCCESS)
                {
                    /* the radio queue is full, tc will notify us when it's available again */
                    TICK_PIN(PIN_TC_QUEUE_FULL);
                    break;
                }
                else
                {
                    /* the handle is queued for transmission */
                    trickle_tx_register(&m_data_cache[data_index].trickle);

                    uint16_t handle = mesh_packet_handle_get(m_data_cache[data_index].p_packet);
                    uint16_t handle_index = handle_entry_get(handle);
                    if (handle_index != HANDLE_CACHE_ENTRY_INVALID &&
                            m_handle_cache[handle_index].tx_event)
                    {
                        rbc_mesh_event_t tx_event;
                        mesh_adv_data_t* p_adv_data = mesh_packet_adv_data_get(m_data_cache[data_index].p_packet);
                        tx_event.event_type = RBC_MESH_EVENT_TYPE_TX;
                        tx_event.value_handle = handle;
                        tx_event.data = p_adv_data->data;
                        tx_event.data_len = p_adv_data->adv_data_length - MESH_PACKET_ADV_OVERHEAD;
                        tx_event.version_delta = 0;

                        rbc_mesh_event_handler(&tx_event);

                        /* report to serial as well */
#if RBC_MESH_SERIAL
                        mesh_aci_rbc_event_handler(&tx_event);
#endif
                    }
                }
            }
        }
        data_index++;
    }

    order_next_transmission(timestamp);
}

/** compare payloads, assuming version number is equal */
static bool payload_has_conflict(mesh_adv_data_t* p_old_adv, mesh_adv_data_t* p_new_adv)
{
    if (p_old_adv == NULL ||
        p_new_adv == NULL ||
        (p_old_adv->adv_data_length != 
        p_new_adv->adv_data_length))
    {
        return true;
    }

    return (memcmp(p_old_adv->data, p_new_adv->data, p_old_adv->adv_data_length - MESH_PACKET_ADV_OVERHEAD) != 0);
}

/******************************************************************************
* Interface functions
******************************************************************************/
uint32_t vh_init(uint32_t min_interval_us)
{
    uint32_t error_code = vh_min_interval_set(min_interval_us);
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }
    
    for (uint32_t i = 0; i < RBC_MESH_DATA_CACHE_ENTRIES; ++i)
    {
        trickle_timer_reset(&m_data_cache[i].trickle, 0);
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

    g_is_initialized = true;
    return NRF_SUCCESS;
}

uint32_t vh_min_interval_set(uint32_t min_interval_us)
{
    if (min_interval_us < RBC_MESH_INTERVAL_MIN_MIN_MS * 1000 ||
        min_interval_us > RBC_MESH_INTERVAL_MIN_MAX_MS * 1000)
    {
        return NRF_ERROR_INVALID_PARAM;
    }
    trickle_setup(min_interval_us, MESH_TRICKLE_I_MAX, MESH_TRICKLE_K);

    return NRF_SUCCESS;
}

vh_data_status_t vh_rx_register(mesh_packet_t* p_packet, uint64_t timestamp)
{
    if (!g_is_initialized)
        return VH_DATA_STATUS_UNKNOWN;

    mesh_adv_data_t* p_adv_data = mesh_packet_adv_data_get(p_packet);
    if (p_adv_data == NULL)
    {
        return VH_DATA_STATUS_UNKNOWN;
    }

    bool cache_hit = false;
    int16_t delta = 0;
    uint16_t handle_index = handle_version_set(p_adv_data->handle, p_adv_data->version, &cache_hit, &delta);
    uint16_t data_index = m_handle_cache[handle_index].data_entry;
    uint64_t ts_start_time = timeslot_get_global_time();

    if (delta < 0)
    {
        /* the incoming packet is old */
        if (data_index != DATA_CACHE_ENTRY_INVALID)
        {
            trickle_rx_inconsistent(&m_data_cache[data_index].trickle, ts_start_time + timestamp);
            vh_order_update(0);
        }
        return VH_DATA_STATUS_OLD;
    }
    else if (delta == 0)
    {
        /* the incoming packet has the same version as the current */
        if (data_index == DATA_CACHE_ENTRY_INVALID)
        {
            /* no entry, no way of proving conflict */
            return VH_DATA_STATUS_SAME;
        }

        if (p_adv_data->version > 0 &&
            cache_hit && 
            payload_has_conflict(mesh_packet_adv_data_get(m_data_cache[data_index].p_packet), p_adv_data))
        {
            trickle_rx_inconsistent(&m_data_cache[data_index].trickle, ts_start_time + timestamp);
            vh_order_update(0);
            return VH_DATA_STATUS_CONFLICTING;
        }
        else
        {
            trickle_rx_consistent(&m_data_cache[data_index].trickle, ts_start_time + timestamp); 
            return VH_DATA_STATUS_SAME;
        }
    }
    else
    {
        /* didn't know this value before, should start retransmitting it */
        if (data_index == DATA_CACHE_ENTRY_INVALID)
        {
            data_index = data_entry_allocate();

            /* associate with handle */
            m_handle_cache[handle_index].data_entry = data_index;
        }
        else
        {
            /* free old packet */
            mesh_packet_free(m_data_cache[data_index].p_packet); /** @TODO: !!!! IS THIS SAFE? */
        }

        mesh_packet_take_ownership(p_packet);
        m_data_cache[data_index].p_packet = p_packet;
        trickle_rx_inconsistent(&m_data_cache[data_index].trickle, ts_start_time + timestamp);
        vh_order_update(0);

        if (cache_hit)
        {
            return VH_DATA_STATUS_UPDATED;
        }
        else
        {
            return VH_DATA_STATUS_NEW;
        }
    }
}

vh_data_status_t vh_local_update(rbc_mesh_value_handle_t handle, uint8_t* data, uint8_t length)
{
    if (!g_is_initialized)
        return VH_DATA_STATUS_UNKNOWN;

    vh_data_status_t status;
    uint32_t error_code;
    const uint64_t ts_time = timer_get_timestamp();
    const uint64_t time_now = ts_time + timeslot_get_global_time();
    const uint16_t handle_index = handle_version_increment(handle);
    uint16_t data_index = m_handle_cache[handle_index].data_entry;

    if (data_index == DATA_CACHE_ENTRY_INVALID)
    {
        data_index = data_entry_allocate();
        if (!mesh_packet_acquire(&m_data_cache[data_index].p_packet))
        {
            return VH_DATA_STATUS_UNKNOWN;
        }
        m_handle_cache[handle_index].data_entry = data_index;
    }

    error_code = mesh_packet_build(m_data_cache[data_index].p_packet,
            handle,
            m_handle_cache[handle_index].version,
            data, 
            length);
    if (error_code != NRF_SUCCESS)
    {
        return VH_DATA_STATUS_UNKNOWN;
    }

    trickle_enable(&m_data_cache[data_index].trickle);

    if (m_handle_cache[handle_index].version == 1) /* first update */
    {
        status = VH_DATA_STATUS_NEW;
        trickle_timer_reset(&m_data_cache[data_index].trickle, time_now);
    }
    else
    {
        status = VH_DATA_STATUS_UPDATED;
        trickle_rx_inconsistent(&m_data_cache[data_index].trickle, time_now);
    }

    vh_order_update(ts_time);

    return status;
}

uint32_t vh_on_timeslot_begin(void)
{
    return vh_order_update(TIMESLOT_STARTUP_DELAY_US);
}

uint32_t vh_order_update(uint64_t time_now)
{
    /* handle expired trickle timers */
    async_event_t tx_event;
    tx_event.type = EVENT_TYPE_TIMER;
    tx_event.callback.timer.cb = transmit_all_instances;
    tx_event.callback.timer.timestamp = time_now;
    return event_handler_push(&tx_event);
}

uint32_t vh_value_get(rbc_mesh_value_handle_t handle, uint8_t* data, uint16_t* length)
{
    if (!g_is_initialized)
        return NRF_ERROR_INVALID_STATE;

    uint16_t handle_index = handle_entry_get(handle);
    if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    uint16_t data_index = m_handle_cache[handle_index].data_entry;
    if (data_index == DATA_CACHE_ENTRY_INVALID || m_data_cache[data_index].p_packet == NULL)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    mesh_adv_data_t* p_adv_data = mesh_packet_adv_data_get(m_data_cache[data_index].p_packet);

    /* don't exceed the supplied length, as this will overflow the buffer */
    if (*length < p_adv_data->adv_data_length - MESH_PACKET_ADV_OVERHEAD)
    {
        *length = p_adv_data->adv_data_length - MESH_PACKET_ADV_OVERHEAD;
    }

    /* make copy */
    memcpy(data, p_adv_data->data, *length);

    return NRF_SUCCESS;
}

uint32_t vh_tx_event_set(rbc_mesh_value_handle_t handle, bool do_tx_event)
{
    if (!g_is_initialized)
        return NRF_ERROR_INVALID_STATE;

    uint16_t handle_index = handle_entry_get(handle);

    if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    m_handle_cache[handle_index].tx_event = do_tx_event;

    return NRF_SUCCESS;
}

uint32_t vh_tx_event_flag_get(rbc_mesh_value_handle_t handle, bool* is_doing_tx_event)
{
    if (!g_is_initialized)
        return NRF_ERROR_INVALID_STATE;

    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    uint16_t handle_index = handle_entry_to_head(handle);
    if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    *is_doing_tx_event = m_handle_cache[handle_index].tx_event;

    return NRF_SUCCESS;
}

int16_t vh_get_version_delta(uint16_t old_version, uint16_t new_version)
{
    if (old_version < MESH_VALUE_LOLLIPOP_LIMIT)
    {
        return (((int32_t)old_version + INT16_MAX) > ((int32_t)new_version)) ?
            (new_version - old_version) :
            (INT16_MAX);
    }
    else if (new_version < MESH_VALUE_LOLLIPOP_LIMIT)
    {
        return (((int32_t)new_version + INT16_MAX) > ((int32_t)old_version)) ?
            (new_version - old_version) :
            (INT16_MIN);
    }
    else
    {
        const uint32_t separation = (new_version >= old_version)?
            (new_version - old_version) :
            (old_version - new_version);

        if (separation > INT16_MAX)
        {
            if (old_version > new_version)
            {
                old_version += MESH_VALUE_LOLLIPOP_LIMIT;
            }
            else
            {
                new_version += MESH_VALUE_LOLLIPOP_LIMIT;
            }
        }

        return (new_version - old_version);
    }
}

uint32_t vh_value_enable(rbc_mesh_value_handle_t handle)
{
    if (!g_is_initialized)
        return NRF_ERROR_INVALID_STATE;
    
    const uint64_t ts_time = timer_get_timestamp();
    const uint64_t time_now = ts_time + timeslot_get_global_time();
    uint16_t handle_index = handle_entry_to_head(handle);
    uint16_t data_index = m_handle_cache[handle_index].data_entry;

    if (data_index == DATA_CACHE_ENTRY_INVALID)
    {
        data_index = data_entry_allocate();
        if (!mesh_packet_acquire(&m_data_cache[data_index].p_packet))
        {
            return NRF_ERROR_NO_MEM;
        }

        /* build a dummy packet to request data, as we don't know the contents of the packet */
        uint32_t error_code = mesh_packet_build(m_data_cache[data_index].p_packet,
                handle,
                0,
                NULL,
                0);
        if (error_code != NRF_SUCCESS)
        {
            mesh_packet_free(m_data_cache[data_index].p_packet);
            return NRF_ERROR_INTERNAL;
        }

        m_handle_cache[handle_index].data_entry = data_index;
        trickle_timer_reset(&m_data_cache[data_index].trickle, time_now);
    }
    else
    {
        trickle_enable(&m_data_cache[data_index].trickle);
    }

    vh_order_update(ts_time);

    return NRF_SUCCESS;
}

uint32_t vh_value_disable(rbc_mesh_value_handle_t handle)
{
    if (!g_is_initialized)
        return NRF_ERROR_INVALID_STATE;

    uint16_t handle_index = handle_entry_get(handle);
    if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    uint16_t data_index = m_handle_cache[handle_index].data_entry;

    if (data_index != DATA_CACHE_ENTRY_INVALID)
    {
        trickle_disable(&m_data_cache[data_index].trickle);
    }

    return NRF_SUCCESS;
}

uint32_t vh_value_is_enabled(rbc_mesh_value_handle_t handle, bool* p_is_enabled)
{
    if (!g_is_initialized)
        return NRF_ERROR_INVALID_STATE;
    
    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    
    uint16_t handle_index = handle_entry_to_head(handle);
    if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
    {
        *p_is_enabled = false;
        return NRF_SUCCESS;
    }

    uint16_t data_index = m_handle_cache[handle_index].data_entry;
    if (data_index == DATA_CACHE_ENTRY_INVALID)
    {
        *p_is_enabled = false;
        return NRF_SUCCESS;
    }

    *p_is_enabled = trickle_is_enabled(&m_data_cache[data_index].trickle);
    return NRF_SUCCESS;
}

uint32_t vh_value_persistence_set(rbc_mesh_value_handle_t handle, bool persistent)
{
    if (!g_is_initialized)
        return NRF_ERROR_INVALID_STATE;
    
    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    
    uint16_t handle_index = handle_entry_to_head(handle);
    if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    
    m_handle_cache[handle_index].persistent = persistent;
    
    return NRF_SUCCESS;
}

uint32_t vh_value_persistence_get(rbc_mesh_value_handle_t handle, bool* p_persistent)
{
    if (!g_is_initialized)
        return NRF_ERROR_INVALID_STATE;
    
    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    uint16_t handle_index = handle_entry_to_head(handle);
    if (handle_index == HANDLE_CACHE_ENTRY_INVALID)
    {
        return NRF_ERROR_NOT_FOUND;
    }
    
    *p_persistent = m_handle_cache[handle_index].persistent;
    
    return NRF_SUCCESS;
}

