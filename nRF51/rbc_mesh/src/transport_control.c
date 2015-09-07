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

#include "transport_control.h"

#include "radio_control.h"
#include "mesh_packet.h"
#include "timer_control.h"
#include "mesh_srv.h"
#include "rbc_mesh.h"
#include "event_handler.h"
#include "timeslot_handler.h"
#include "rbc_mesh_common.h"
#include "version_handler.h"
#include "mesh_aci.h"

#include <string.h>

/******************************************************************************
* Local typedefs
******************************************************************************/
typedef struct
{
    uint32_t access_address;
    uint8_t channel;
    bool queue_saturation; /* flag indicating a full processing queue */
} tc_state_t;

/******************************************************************************
* Static globals
******************************************************************************/
static tc_state_t g_state;
/******************************************************************************
* Static functions
******************************************************************************/
static void rx_cb(uint8_t* data, bool success, uint32_t crc);
static void tx_cb(uint8_t* data);


static void order_search(void)
{
    radio_event_t evt;
    evt.event_type = RADIO_EVENT_TYPE_RX_PREEMPTABLE;
    evt.start_time = 0;
    if (!mesh_packet_acquire((mesh_packet_t**) &evt.packet_ptr))
    {
        return; /* something is hogging all the packets */
    }
    evt.access_address = g_state.access_address;
    evt.channel = g_state.channel;
    evt.callback.rx = rx_cb;
    if (!radio_order(&evt))
    {
        mesh_packet_free((mesh_packet_t*) evt.packet_ptr);
    }
}


static void prepare_event(rbc_mesh_event_t* evt, mesh_packet_t* p_packet)
{
    evt->value_handle = p_packet->payload.handle;
    evt->data = &p_packet->payload.data[0];
    evt->data_len = p_packet->header.length - MESH_PACKET_OVERHEAD;
}

/* radio callback, executed in STACK_LOW */
static void rx_cb(uint8_t* data, bool success, uint32_t crc)
{
    if (success)
    {
        async_event_t evt;
        evt.type = EVENT_TYPE_PACKET;
        evt.callback.packet.payload = data;
        evt.callback.packet.crc = crc;
        evt.callback.packet.timestamp = timer_get_timestamp();
        if (event_handler_push(&evt) != NRF_SUCCESS)
        {
            /* packet will not be freed by the packet processing event */
            mesh_packet_free((mesh_packet_t*) data);
            g_state.queue_saturation = true;
        }
    }
    else
    {
        /* packet will not be freed by the packet processing event*/
        mesh_packet_free((mesh_packet_t*) data);
    }
}

/* radio callback, executed in STACK_LOW */
static void tx_cb(uint8_t* data)
{
    mesh_packet_free((mesh_packet_t*) data);
    vh_order_update(timer_get_timestamp()); /* tell the vh, so that it can push more updates */
}


static void radio_idle_callback(void)
{
    /* If the processor is unable to keep up, we should back down, and give it time */
    if (!g_state.queue_saturation)
        order_search();
}


/******************************************************************************
* Interface functions
******************************************************************************/
void tc_init(uint32_t access_address, uint8_t channel)
{
    g_state.access_address = access_address;
    g_state.channel = channel;
}

void tc_on_ts_begin(void)
{
    
    radio_init(g_state.access_address, radio_idle_callback);
}

uint32_t tc_tx(uint8_t handle, uint16_t version)
{
    uint32_t error_code;
    mesh_packet_t* p_packet = NULL;
    if (!mesh_packet_acquire(&p_packet))
    {
        return NRF_ERROR_NO_MEM;
    }

    uint16_t length = MAX_VALUE_LENGTH;
    error_code = mesh_srv_char_val_get(handle, &p_packet->payload.data[0], &length);
    if (error_code != NRF_SUCCESS)
    {
        mesh_packet_free(p_packet);
        return error_code;
    }

    if (length == 0)
    {
        mesh_packet_free(p_packet);
        return NRF_ERROR_INTERNAL;
    }
    
    ble_gap_addr_t my_addr;
    sd_ble_gap_address_get(&my_addr);

    p_packet->header.length = MESH_PACKET_OVERHEAD + length;
    p_packet->header.addr_type = my_addr.addr_type;
    memcpy(&p_packet->addr, &my_addr.addr, BLE_GAP_ADDR_LEN);
    p_packet->header.type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
    p_packet->payload.handle = handle;
    p_packet->payload.version = version;

    TICK_PIN(PIN_MESH_TX);
    /* queue the packet for transmission */
    radio_event_t event;
    event.start_time = 0;
    event.packet_ptr = (uint8_t*) p_packet;
    event.access_address = g_state.access_address;
    event.channel = g_state.channel;
    event.callback.tx = tx_cb;
    if (!radio_order(&event))
    {
        mesh_packet_free(p_packet);
        return NRF_ERROR_NO_MEM;
    }
    
    return NRF_SUCCESS;
}

/* packet processing, executed in APP_LOW */
void tc_packet_handler(uint8_t* data, uint32_t crc, uint64_t timestamp)
{
    SET_PIN(PIN_RX);
    mesh_packet_t* p_packet = (mesh_packet_t*) data;

    if (p_packet->header.length > MESH_PACKET_OVERHEAD + MAX_VALUE_LENGTH || 
            p_packet->header.type != BLE_PACKET_TYPE_ADV_NONCONN_IND)
    {
        /* invalid packet, ignore */
        mesh_packet_free(p_packet);
        CLEAR_PIN(PIN_RX);
        return;
    }
    
    ble_gap_addr_t addr;
    memcpy(addr.addr, p_packet->addr, BLE_GAP_ADDR_LEN);
    addr.addr_type = p_packet->header.addr_type;

    vh_data_status_t data_status = vh_compare_metadata(
            p_packet->payload.handle, 
            p_packet->payload.version,
            p_packet->payload.data,
            p_packet->header.length - MESH_PACKET_OVERHEAD
    );
    
    uint16_t delta = vh_get_version_delta(p_packet->payload.handle, p_packet->payload.version);
    uint32_t error_code = NRF_SUCCESS;

    if (data_status != VH_DATA_STATUS_UNKNOWN)
    {
        error_code = 
            vh_rx_register(
                data_status, 
                p_packet->payload.handle, 
                p_packet->payload.version,
                timestamp
        );
        if (error_code != NRF_SUCCESS)
        {
            CLEAR_PIN(PIN_RX);
            mesh_packet_free(p_packet);
            return;
        }
    }

    /* prepare app event */
    rbc_mesh_event_t evt;
    evt.version_delta = delta;

    switch (data_status)
    {
        case VH_DATA_STATUS_NEW:
            mesh_srv_char_val_set(
                    p_packet->payload.handle, 
                    &p_packet->payload.data[0], 
                    p_packet->header.length - MESH_PACKET_OVERHEAD);

            /* notify application */
            prepare_event(&evt, p_packet);
            evt.event_type = RBC_MESH_EVENT_TYPE_NEW_VAL;
            rbc_mesh_event_handler(&evt);
#ifdef RBC_MESH_SERIAL
            mesh_aci_rbc_event_handler(&evt);
#endif
            break;

        case VH_DATA_STATUS_UPDATED:
            mesh_srv_char_val_set(
                    p_packet->payload.handle, 
                    &p_packet->payload.data[0], 
                    p_packet->header.length - MESH_PACKET_OVERHEAD);

            /* notify application */
            prepare_event(&evt, p_packet);
            evt.event_type = RBC_MESH_EVENT_TYPE_UPDATE_VAL;
            rbc_mesh_event_handler(&evt);
#ifdef RBC_MESH_SERIAL
            mesh_aci_rbc_event_handler(&evt);
#endif
            break;

        case VH_DATA_STATUS_OLD:
            /* do nothing */
            break;
            
        case VH_DATA_STATUS_SAME:
            /* do nothing */
            break;

        case VH_DATA_STATUS_CONFLICTING:

            prepare_event(&evt, p_packet);
            evt.event_type = RBC_MESH_EVENT_TYPE_CONFLICTING_VAL;
            rbc_mesh_event_handler(&evt);
#ifdef RBC_MESH_SERIAL
            mesh_aci_rbc_event_handler(&evt);
#endif
            break;

        case VH_DATA_STATUS_UNKNOWN:
            
            break;
    }

    mesh_packet_free(p_packet);
    
    if (g_state.queue_saturation)
    {
        order_search();
        g_state.queue_saturation = false;
    }
    
    CLEAR_PIN(PIN_RX);
}
