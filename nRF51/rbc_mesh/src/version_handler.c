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

#include "version_handler.h"

#include "handle_storage.h"
#include "transport_control.h"
#include "timer.h"
#include "timer_scheduler.h"
#include "event_handler.h"
#include "rbc_mesh_common.h"
#include "toolchain.h"
#include "trickle.h"
#include "rbc_mesh.h"
#include "mesh_packet.h"
#include "mesh_gatt.h"
#include "mesh_aci.h"

#include "nrf_error.h"
#include "app_error.h"
#include "fifo.h"
#include <stdlib.h>
#include <stddef.h>
#include <string.h>


#define TIMESLOT_STARTUP_DELAY_US       (100)

/* event push isn't present in the API header file. */
extern uint32_t rbc_mesh_event_push(rbc_mesh_event_t* p_evt);


/******************************************************************************
* Static globals
******************************************************************************/
static bool             m_is_initialized = false;
static timer_event_t    m_tx_timer_evt;
static tc_tx_config_t   m_tx_config;
/******************************************************************************
* Static functions
******************************************************************************/
int16_t version_delta(uint16_t old_version, uint16_t new_version)
{
    /* utility function */
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


static void transmit_all_instances(uint32_t timestamp, void* p_context);

static void order_next_transmission(uint32_t time_now)
{
    bool found_value;
    uint32_t timeout = handle_storage_next_timeout_get(&found_value);
    if (!found_value)
    {
        return;
    }
    if (timeout < time_now + 1000)
    {
        vh_order_update(timeout);
    }
    else
    {
        if (timer_sch_reschedule(&m_tx_timer_evt, timeout) != NRF_SUCCESS)
        {
            vh_order_update(timeout);
        }
    }
}

static void transmit_all_instances(uint32_t timestamp, void* p_context)
{
    SET_PIN(8);
    mesh_packet_t* pp_tx_packets[RBC_MESH_RADIO_QUEUE_LENGTH - 1];
    uint32_t count = RBC_MESH_RADIO_QUEUE_LENGTH - 1;

    uint32_t error_code = handle_storage_tx_packets_get(timestamp, pp_tx_packets, &count);
    if (error_code == NRF_SUCCESS)
    {
        for (uint32_t i = 0; i < count; ++i)
        {
            error_code = tc_tx(pp_tx_packets[i], &m_tx_config);
            if (error_code == NRF_SUCCESS)
            {
                mesh_adv_data_t* p_adv = mesh_packet_adv_data_get(pp_tx_packets[i]);
                if (p_adv)
                {
                    PIN_OUT(p_adv->handle, 8);
                    APP_ERROR_CHECK(handle_storage_transmitted(p_adv->handle, timestamp));
                }
                else
                {
                    APP_ERROR_CHECK(NRF_ERROR_INVALID_DATA);
                }
            }
            mesh_packet_ref_count_dec(pp_tx_packets[i]);
        }
    }
    CLEAR_PIN(8);
    order_next_transmission(timestamp);
}

/******************************************************************************
* Interface functions
******************************************************************************/
uint32_t vh_init(uint32_t min_interval_us,
                 uint32_t access_address,
                 uint8_t channel,
                 rbc_mesh_txpower_t tx_power)
{
    uint32_t error_code = handle_storage_init(min_interval_us);
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    m_tx_timer_evt.p_next = NULL;
    m_tx_timer_evt.cb = transmit_all_instances;
    m_tx_timer_evt.interval = 0;
    m_tx_timer_evt.p_context = NULL;

    m_tx_config.alt_access_address = (access_address != RBC_MESH_ACCESS_ADDRESS_BLE_ADV);
    m_tx_config.first_channel = channel;
    m_tx_config.channel_map = 1; /* Only the first channel */
    m_tx_config.tx_power = tx_power;

    m_is_initialized = true;
    return NRF_SUCCESS;
}

uint32_t vh_min_interval_set(uint32_t min_interval_us)
{
    return handle_storage_min_interval_set(min_interval_us);
}

void vh_tx_power_set(rbc_mesh_txpower_t tx_power)
{
    m_tx_config.tx_power = tx_power;
}

uint32_t vh_rx(mesh_packet_t* p_packet, uint32_t timestamp, uint8_t rssi)
{
    mesh_adv_data_t* p_adv_data = mesh_packet_adv_data_get(p_packet);
    if (p_adv_data == NULL)
    {
        return NRF_ERROR_INVALID_DATA;
    }

    handle_info_t info;
    uint32_t error_code = handle_storage_info_get(p_adv_data->handle, &info);

    int16_t delta = version_delta(info.version, p_adv_data->version);

    /* prepare app event */
    rbc_mesh_event_t evt;
    evt.params.rx.version_delta = delta;
    evt.params.rx.ble_adv_addr.addr_type = p_packet->header.addr_type;
    memcpy(evt.params.rx.ble_adv_addr.addr, p_packet->addr, BLE_GAP_ADDR_LEN);
    evt.params.rx.rssi = -((int8_t) rssi);
    evt.params.rx.p_data = p_adv_data->data;
    evt.params.rx.data_len = p_adv_data->adv_data_length - MESH_PACKET_ADV_OVERHEAD;
    evt.params.rx.value_handle = p_adv_data->handle;
    evt.params.rx.timestamp_us = timestamp;

    if (error_code == NRF_ERROR_NOT_FOUND)
    {
        /* couldn't find the handle in the handle storage */
        evt.type = RBC_MESH_EVENT_TYPE_NEW_VAL;
        evt.params.rx.version_delta = delta;

        /* First allocate an element in the storage to ensure that we're not out of memory. */
        error_code = handle_storage_info_set(p_adv_data->handle, &info);
        if (error_code != NRF_SUCCESS)
        {
            mesh_packet_ref_count_dec(info.p_packet);
            return error_code;
        }

        mesh_packet_take_ownership(p_packet);
        handle_info_t new_info =
        {
            .p_packet = p_packet,
            .version = p_adv_data->version
        };

        if (rbc_mesh_event_push(&evt) == NRF_SUCCESS)
        {
            /* assert if this doesn't work. The empty allocation above should have prevented any errors this time. */
            APP_ERROR_CHECK(handle_storage_info_set(p_adv_data->handle, &new_info));

            mesh_gatt_value_set(p_adv_data->handle,
                p_adv_data->data,
                p_adv_data->adv_data_length - MESH_PACKET_ADV_OVERHEAD);
        }

        vh_order_update(timestamp);

#ifdef RBC_MESH_SERIAL
        mesh_aci_rbc_event_handler(&evt);
#endif
    }
    else if (delta < 0)
    {
        handle_storage_rx_inconsistent(p_adv_data->handle, timestamp);
        vh_order_update(timestamp);
    }
    else if (delta == 0)
    {
        mesh_adv_data_t* p_stored_adv_data = NULL;
        if (info.p_packet)
        {
            p_stored_adv_data = mesh_packet_adv_data_get(info.p_packet);
        }

        if (p_stored_adv_data &&
            payload_has_conflict(p_stored_adv_data, p_adv_data))
        {
            evt.type = RBC_MESH_EVENT_TYPE_CONFLICTING_VAL;
            rbc_mesh_event_push(&evt); /* not really important whether this succeeds. */

#ifdef RBC_MESH_SERIAL
            mesh_aci_rbc_event_handler(&evt);
#endif
        }

        handle_storage_rx_consistent(p_adv_data->handle, timestamp);
    }
    else /* delta > 0 */
    {
        evt.type = RBC_MESH_EVENT_TYPE_UPDATE_VAL;
        evt.params.rx.version_delta = delta;

        /* First allocate an element in the storage to ensure that we're not out of memory. */
        error_code = handle_storage_info_set(p_adv_data->handle, &info);
        if (error_code != NRF_SUCCESS)
        {
            mesh_packet_ref_count_dec(info.p_packet);
            return error_code;
        }

        mesh_packet_take_ownership(p_packet);
        handle_info_t new_info =
        {
            .p_packet = p_packet,
            .version = p_adv_data->version
        };

        if (rbc_mesh_event_push(&evt) == NRF_SUCCESS)
        {
            /* assert if this doesn't work. The empty allocation above should have prevented any errors this time. */
            APP_ERROR_CHECK(handle_storage_info_set(p_adv_data->handle, &new_info));

            mesh_gatt_value_set(p_adv_data->handle,
                p_adv_data->data,
                p_adv_data->adv_data_length - MESH_PACKET_ADV_OVERHEAD);
        }

        vh_order_update(timestamp);

#ifdef RBC_MESH_SERIAL
        mesh_aci_rbc_event_handler(&evt);
#endif
    }

    mesh_packet_ref_count_dec(info.p_packet);
    return NRF_SUCCESS;
}

uint32_t vh_local_update(rbc_mesh_value_handle_t handle, uint8_t* data, uint8_t length)
{
    if (!m_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    uint32_t error_code;
    mesh_packet_t* p_packet = NULL;

    if (!mesh_packet_acquire(&p_packet))
    {
        return NRF_ERROR_NO_MEM;
    }

    error_code = mesh_packet_build(p_packet,
            handle,
            1, /* Will be overwritten if handle storage knows the current version */
            data,
            length);
    if (error_code != NRF_SUCCESS)
    {
        mesh_packet_ref_count_dec(p_packet);
        return error_code;
    }

    error_code = handle_storage_local_packet_push(p_packet);
    if (error_code == NRF_SUCCESS)
    {
        vh_order_update(timer_now()); /* will be executed after the packet push */
    }

    mesh_packet_ref_count_dec(p_packet);
    return error_code;
}

uint32_t vh_on_timeslot_begin(void)
{
    return vh_order_update(timer_now());
}

uint32_t vh_order_update(uint32_t time_now)
{
    /* handle expired trickle timers */
    async_event_t tx_event;
    tx_event.type = EVENT_TYPE_TIMER_SCH;
    tx_event.callback.timer_sch.cb = transmit_all_instances;
    tx_event.callback.timer_sch.timestamp = time_now;
    tx_event.callback.timer_sch.p_context = NULL;
    return event_handler_push(&tx_event);
}

uint32_t vh_value_get(rbc_mesh_value_handle_t handle, uint8_t* data, uint16_t* length)
{
    if (!m_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    handle_info_t info;

    uint32_t error_code = handle_storage_info_get(handle, &info);
    if (error_code != NRF_SUCCESS)
    {
        return error_code;
    }

    if (info.p_packet == NULL)
    {
        return NRF_ERROR_NOT_FOUND;
    }

    mesh_adv_data_t* p_adv_data = mesh_packet_adv_data_get(info.p_packet);
    if (p_adv_data == NULL)
    {
        mesh_packet_ref_count_dec(info.p_packet);
        return NRF_ERROR_NOT_FOUND;
    }

    uint16_t data_length = p_adv_data->adv_data_length - MESH_PACKET_ADV_OVERHEAD;
    if (data_length > *length)
    {
        mesh_packet_ref_count_dec(info.p_packet);
        return NRF_ERROR_INVALID_LENGTH;
    }
    memcpy(data, p_adv_data->data, data_length);
    *length = data_length;

    mesh_packet_ref_count_dec(info.p_packet);
    return NRF_SUCCESS;
}

uint32_t vh_tx_event_set(rbc_mesh_value_handle_t handle, bool do_tx_event)
{
    if (!m_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    event_handler_critical_section_begin();

    uint32_t error_code = handle_storage_flag_set_async(handle, HANDLE_FLAG_TX_EVENT, do_tx_event);

    event_handler_critical_section_end();
    return error_code;
}

uint32_t vh_tx_event_flag_get(rbc_mesh_value_handle_t handle, bool* p_is_doing_tx_event)
{
    if (!m_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    event_handler_critical_section_begin();

    uint32_t error_code = handle_storage_flag_get(handle, HANDLE_FLAG_TX_EVENT, p_is_doing_tx_event);

    event_handler_critical_section_end();
    return error_code;
}


uint32_t vh_value_enable(rbc_mesh_value_handle_t handle)
{
    if (!m_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    uint32_t error_code = handle_storage_flag_set_async(handle, HANDLE_FLAG_DISABLED, false);
    return error_code;
}

uint32_t vh_value_disable(rbc_mesh_value_handle_t handle)
{
    if (!m_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    event_handler_critical_section_begin();

    uint32_t error_code = handle_storage_flag_set_async(handle, HANDLE_FLAG_DISABLED, true);

    event_handler_critical_section_end();
    return error_code;
}

uint32_t vh_value_is_enabled(rbc_mesh_value_handle_t handle, bool* p_is_enabled)
{
    if (!m_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    event_handler_critical_section_begin();

    uint32_t error_code = handle_storage_flag_get(handle, HANDLE_FLAG_DISABLED, p_is_enabled);
    *p_is_enabled = !(*p_is_enabled);

    event_handler_critical_section_end();
    return error_code;
}

uint32_t vh_value_persistence_set(rbc_mesh_value_handle_t handle, bool persistent)
{
    if (!m_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    uint32_t error_code = handle_storage_flag_set_async(handle, HANDLE_FLAG_PERSISTENT, persistent);
    return error_code;
}

uint32_t vh_value_persistence_get(rbc_mesh_value_handle_t handle, bool* p_persistent)
{
    if (!m_is_initialized)
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (handle == RBC_MESH_INVALID_HANDLE)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    event_handler_critical_section_begin();

    uint32_t error_code = handle_storage_flag_get(handle, HANDLE_FLAG_PERSISTENT, p_persistent);

    event_handler_critical_section_end();
    return error_code;
}
