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
#include "transport_control.h"
#include "radio_control.h"
#include "mesh_gatt.h"
#include "mesh_packet.h"
#include "timer.h"
#include "rbc_mesh.h"
#include "event_handler.h"
#include "timeslot.h"
#include "timer_scheduler.h"
#include "rbc_mesh_common.h"
#include "version_handler.h"
#include "mesh_aci.h"
#include "app_error.h"

#if defined(WITH_ACK_MASTER) || defined (WITHOUT_ACK_MASTER)|| defined (WITH_ACK_SLAVE)

#include "SEGGER_RTT.h"
#include "nrf_gpio.h"
#include "boards.h"
#include <stdio.h>

#endif

#if defined(WITH_ACK_SLAVE)
#include <handle.h>
#endif



#ifdef MESH_DFU
#include "dfu_types_mesh.h"
#include "dfu_app.h"
#endif
/* event push isn't present in the API header file. */
extern uint32_t rbc_mesh_event_push(rbc_mesh_event_t* p_evt);

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
static tc_state_t m_state;
static rbc_mesh_packet_peek_cb_t mp_packet_peek_cb;

/* STATS */
#ifdef PACKET_STATS


#if defined (WITH_ACK_MASTER) 
static struct
{
    uint32_t queue_drop;
    uint32_t queue_ok;
    uint32_t crc_fail;
} m_packet_stats __attribute__((at(0x20003030)))  = {0};


#elif defined(WITHOUT_ACK_MASTER) 
static struct
{
    uint32_t queue_drop;
    uint32_t queue_ok;
    uint32_t crc_fail;
} m_packet_stats __attribute__((at(0x2000273C))) = {0};



#elif defined(WITH_ACK_SLAVE)
static struct
{
    uint32_t queue_drop;
    uint32_t queue_ok;
    uint32_t crc_fail;
} m_packet_stats __attribute__((at(0x20002750))) = {0};

#elif defined(WITHOUT_ACK_SLAVE)
static struct
{
    uint32_t queue_drop;
    uint32_t queue_ok;
    uint32_t crc_fail;
} m_packet_stats __attribute__((at(0x20002738))) = {0};

#else
static struct
{
    uint32_t queue_drop;
    uint32_t queue_ok;
    uint32_t crc_fail;
} m_packet_stats = {0};

#endif

#endif
/******************************************************************************
* Static functions
******************************************************************************/
static void rx_cb(uint8_t* p_data, bool success, uint32_t crc, uint8_t rssi);
static void tx_cb(uint8_t* p_data);

static void order_search(void)
{
    radio_event_t evt;

    evt.event_type = RADIO_EVENT_TYPE_RX_PREEMPTABLE;
    evt.channel = m_state.channel;

    if (!mesh_packet_acquire((mesh_packet_t**) &evt.packet_ptr))
    {
        return; /* something is hogging all the packets */
    }

    if (radio_order(&evt) != NRF_SUCCESS)
    {
        /* couldn't queue the packet for reception, immediately free its only ref */
        mesh_packet_ref_count_dec((mesh_packet_t*) evt.packet_ptr);
    }
}


/* immediate radio callback, executed in STACK_LOW */
static void rx_cb(uint8_t* p_data, bool success, uint32_t crc, uint8_t rssi)
{
    if (success && ((mesh_packet_t*) p_data)->header.length <= MESH_PACKET_BLE_OVERHEAD + BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH)
    {
        async_event_t evt;
        evt.type = EVENT_TYPE_PACKET;
        evt.callback.packet.payload = p_data;
        evt.callback.packet.crc = crc;
        evt.callback.packet.timestamp = timer_now();
        evt.callback.packet.rssi = rssi;
        mesh_packet_ref_count_inc((mesh_packet_t*) p_data); /* event handler has a ref */
        
        if (event_handler_push(&evt) != NRF_SUCCESS)
        {
            mesh_packet_ref_count_dec((mesh_packet_t*) p_data);
            m_state.queue_saturation = true;
#ifdef PACKET_STATS
            m_packet_stats.queue_drop++;
#endif
        }
        else
        {
#ifdef PACKET_STATS
            m_packet_stats.queue_ok++;
#endif
        }
    }
    else if (crc < 0x1000000) /* don't want to trigger on artifical crc values */
    {
#ifdef PACKET_STATS
        m_packet_stats.crc_fail++;
#endif
    }

    /* no longer needed in this context */
    mesh_packet_ref_count_dec((mesh_packet_t*) p_data);
}






/* radio tx cb, executed in APP_LOW */
static void async_tx_cb(void* p_context)
{
    mesh_packet_t* p_packet = (mesh_packet_t*) p_context;
    rbc_mesh_event_t tx_event;
    mesh_adv_data_t* p_adv_data = mesh_packet_adv_data_get(p_packet);
    bool doing_tx_event = false;
    if (p_adv_data != NULL &&
        vh_tx_event_flag_get(p_adv_data->handle, &doing_tx_event) == NRF_SUCCESS
        && doing_tx_event)
    {
        tx_event.type = RBC_MESH_EVENT_TYPE_TX;
        tx_event.params.tx.value_handle  = p_adv_data->handle;
        tx_event.params.tx.p_data        = p_adv_data->data;
        tx_event.params.tx.data_len      = p_adv_data->adv_data_length - MESH_PACKET_ADV_OVERHEAD;
        tx_event.params.tx.timestamp_us  = timer_now();

        rbc_mesh_event_push(&tx_event); /* will take care of the reference counting itself. */
#ifdef RBC_MESH_SERIAL
        mesh_aci_rbc_event_handler(&tx_event);
#endif
    }
    mesh_packet_ref_count_dec(p_packet); /* event-handler reference popped. */
}

/* radio callback, executed in STACK_LOW */
static void tx_cb(uint8_t* p_data)
{
    /* have to defer tx-event handling to async context to avoid race
       conditions in the handle_storage */
    async_event_t tx_cb_evt =
    {
        .type = EVENT_TYPE_GENERIC,
        .callback.generic =
        {
            .cb = async_tx_cb,
            .p_context = p_data
        }
    };
    if (event_handler_push(&tx_cb_evt) != NRF_SUCCESS)
    {
        mesh_packet_ref_count_dec((mesh_packet_t*) p_data); /* radio ref removed (pushed in tc_tx) */
    }
}

static void radio_idle_callback(void)
{
    /* If the processor is unable to keep up, we should back down, and give it time */
    if (!m_state.queue_saturation)
        order_search();
}

static void mesh_framework_packet_handle(mesh_adv_data_t* p_adv_data, uint32_t timestamp)
{
#ifdef MESH_DFU
    mesh_dfu_adv_data_t* p_dfu = (mesh_dfu_adv_data_t*) p_adv_data;
    /* Tell the shared BL about the packet */
    bl_cmd_t rx_cmd;
    rx_cmd.type = BL_CMD_TYPE_RX;
    rx_cmd.params.rx.length = p_dfu->adv_data_length - DFU_PACKET_ADV_OVERHEAD;
    rx_cmd.params.rx.p_dfu_packet = &p_dfu->dfu_packet;
    dfu_cmd_send(&rx_cmd);
#endif
}

/******************************************************************************
* Interface functions
******************************************************************************/
void tc_init(uint32_t access_address, uint8_t channel)
{
    mp_packet_peek_cb = NULL;
    tc_radio_params_set(access_address, channel);
}

void tc_radio_params_set(uint32_t access_address, uint8_t channel)
{
    if (channel < 40)
    {
        m_state.access_address = access_address;
        m_state.channel = channel;
        radio_alt_aa_set(access_address);
        timeslot_restart();
    }
}

void tc_on_ts_begin(void)
{
    radio_init(radio_idle_callback, rx_cb, tx_cb);
}

uint32_t tc_tx(mesh_packet_t* p_packet, const tc_tx_config_t* p_config)
{
    TICK_PIN(PIN_MESH_TX);
    /* queue the packet for transmission */
    radio_event_t event;
    memset(&event, 0, sizeof(radio_event_t));

    /* clean packet header before sending */
    p_packet->header._rfu1 = 0;
    p_packet->header._rfu2 = 0;
    p_packet->header._rfu3 = 0;

    event.packet_ptr = (uint8_t*) p_packet;
    event.access_address = p_config->alt_access_address;
    event.channel = p_config->first_channel;
    event.event_type = RADIO_EVENT_TYPE_TX;
    event.tx_power = (uint8_t) p_config->tx_power;

    /* send packet on each channel in the channel map */
    for (uint32_t i = 0; i < 32; ++i)
    {
        if (p_config->channel_map & (1 << i))
        {
            mesh_packet_ref_count_inc(p_packet); /* queue will have a reference until tx_cb */
            if (radio_order(&event) != NRF_SUCCESS)
            {
                mesh_packet_ref_count_dec(p_packet); /* queue couldn't hold the ref */
                return NRF_ERROR_NO_MEM;
            }
        }
        else if ((p_config->channel_map >> i) == 0) /* all channels hit */
        {
            break;
        }

        event.channel++;
    }

    return NRF_SUCCESS;
}

/* packet processing, executed in APP_LOW */
void tc_packet_handler(uint8_t* data, uint32_t crc, uint32_t timestamp, uint8_t rssi)
{
    APP_ERROR_CHECK_BOOL(data != NULL);
    SET_PIN(PIN_RX);
    mesh_packet_t* p_packet = (mesh_packet_t*) data;

    if (p_packet->header.length > BLE_GAP_ADDR_LEN + BLE_ADV_PACKET_PAYLOAD_MAX_LENGTH)
    {
        /* invalid packet, ignore */
        CLEAR_PIN(PIN_RX);
        mesh_packet_ref_count_dec(p_packet); /* from rx_cb */

        return;
    }

    /* Pass packet to packet peek function */
    if (mp_packet_peek_cb)
    {
        rbc_mesh_packet_peek_params_t peek_params;
        peek_params.rssi = rssi;
        peek_params.p_payload = p_packet->payload;
        peek_params.payload_len = p_packet->header.length - MESH_PACKET_BLE_OVERHEAD;
        memcpy(peek_params.adv_addr.addr, p_packet->addr, BLE_GAP_ADDR_LEN);
        peek_params.adv_addr.addr_type = p_packet->header.addr_type;
        peek_params.crc = crc;
        peek_params.packet_type = (ble_packet_type_t) p_packet->header.type;
        peek_params.timestamp = timestamp;
        mp_packet_peek_cb(&peek_params);
    }

    ble_gap_addr_t addr;
    memcpy(addr.addr, p_packet->addr, BLE_GAP_ADDR_LEN);
    addr.addr_type = p_packet->header.addr_type;

    mesh_adv_data_t* p_mesh_adv_data = mesh_packet_adv_data_get(p_packet);

    if (p_mesh_adv_data != NULL)
    {
        /* filter mesh packets on handle range */
        if (p_mesh_adv_data->handle <= RBC_MESH_APP_MAX_HANDLE)
        {
            vh_rx(p_packet, timestamp, rssi);
        }
        else
        {
            mesh_framework_packet_handle(p_mesh_adv_data, timestamp);
        }
    }

    /* this packet is no longer needed in this context */
    mesh_packet_ref_count_dec(p_packet); /* from rx_cb */

    if (m_state.queue_saturation)
    {
        order_search();
        m_state.queue_saturation = false;
    }

    CLEAR_PIN(PIN_RX);
}

void tc_packet_peek_cb_set(rbc_mesh_packet_peek_cb_t packet_peek_cb)
{
    mp_packet_peek_cb = packet_peek_cb;
}
