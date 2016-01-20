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

#include "mesh_dfu.h"
#include "rand.h"
#include "transport.h"
#include <string.h>

#define SEGMENT_CACHE_SIZE      (32)
#define RETRANSMIT_ENTRY_COUNT  (8)
#define DFU_CHANNELS            ((1ULL << 37ULL) | (1ULL << 38ULL) | (1ULL << 39ULL))

#define TX_INTERVAL_FWID        (100000)
#define TX_INTERVAL_STATE       (100000)
#define TX_INTERVAL_DATA        ( 50000)
#define TX_INTERVAL_DATA_RSP    ( 50000)

#define TX_REPEATS_DEFAULT      (4)

typedef struct
{
    dfu_type_t type;
    fwid_union_t fwid;
    uint32_t transaction_id;
    enum
    {
        DFU_ROLE_NONE,
        DFU_ROLE_SOURCE_CANDIDATE,
        DFU_ROLE_SOURCE,
        DFU_ROLE_POTENTIAL_RELAY,   /* A DFU is in progress in the neighborhood, awaiting potential relay role */
        DFU_ROLE_RELAY,
    } role;
    uint8_t authority;
    uint8_t tree_parent_addr[BLE_GAP_ADDR_LEN];
    bool parent_is_target;
} dfu_transfer_t;

typedef struct
{
    mesh_packet_t* p_packet;
    uint32_t next_tx_offset;
    uint64_t receive_time;
} retransmit_entry_t;

static retransmit_entry_t m_retransmit_entries[RETRANSMIT_ENTRY_COUNT];
static uint16_t m_retransmit_index;
static fwid_t m_fwid;
static bool m_can_source_dfu;
static mesh_packet_t* mp_fwid_beacon;
static mesh_packet_t* mp_state_beacon;
static dfu_transfer_t m_current_transfer;
static uint8_t m_authority;
static prng_t m_rand;
static uint16_t m_segment_cache[SEGMENT_CACHE_SIZE];
static uint32_t m_segment_cache_index;
static tx_config_t m_fwid_config;
static tx_config_t m_state_config;
static tx_config_t m_data_config;
static tx_config_t m_data_rsp_config;

extern uint32_t rbc_mesh_event_push(rbc_mesh_event_t*);

static uint32_t dfu_packet_len_get(uint16_t packet_type, dfu_type_t dfu_type)
{
    switch (packet_type)
    {
        case DFU_PACKET_TYPE_DATA_RSP:
            return DFU_PACKET_LEN_DATA_RSP;
        case DFU_PACKET_TYPE_DATA_REQ:
            return DFU_PACKET_LEN_DATA_REQ;
        case DFU_PACKET_TYPE_DATA:
            return DFU_PACKET_LEN_DATA;
        case DFU_PACKET_TYPE_STATE:
            switch (dfu_type)
            {
                case DFU_TYPE_SD:
                    return DFU_PACKET_LEN_STATE_SD;
                case DFU_TYPE_BOOTLOADER:
                    return DFU_PACKET_LEN_STATE_BL;
                case DFU_TYPE_APP:
                    return DFU_PACKET_LEN_STATE_APP;
                default:
                    return 0;
            }
        case DFU_PACKET_TYPE_FWID:
            return DFU_PACKET_LEN_FWID;
        default:
            return 0;
    }
}

static void dfu_packet_build(mesh_packet_t* p_packet,
                             dfu_packet_t* p_dfu_packet)
{
    mesh_packet_build(p_packet,
            p_dfu_packet->packet_type,
            p_dfu_packet->payload.raw.version,
            p_dfu_packet->payload.raw.data,
            dfu_packet_len_get(p_dfu_packet->packet_type,
                               p_dfu_packet->payload.state.dfu_type));
}

static bool notify_app_of_source(void)
{
        rbc_mesh_event_t event;
        memset(&event, 0, sizeof(rbc_mesh_event_t));
        event.event_type = RBC_MESH_EVENT_TYPE_DFU_SOURCE;
        event.value_handle = RBC_MESH_INVALID_HANDLE;
        return (rbc_mesh_event_push(&event) == NRF_SUCCESS);
}


static void packet_handle_fwid(dfu_packet_t* p_dfu_packet)
{
    if (p_dfu_packet->payload.fwid.bootloader > m_fwid.bootloader ||
            (
             p_dfu_packet->payload.fwid.app.company_id == m_fwid.app.company_id &&
             p_dfu_packet->payload.fwid.app.app_id == m_fwid.app.app_id &&
             p_dfu_packet->payload.fwid.app.app_version > m_fwid.app.app_version
            )
       )
    {
        /* there is newer firmware available */
        rbc_mesh_event_t event;
        memset(&event, 0, sizeof(rbc_mesh_event_t));
        event.value_handle = RBC_MESH_INVALID_HANDLE;
        event.event_type = RBC_MESH_EVENT_TYPE_DFU_AVAILABLE;
        rbc_mesh_event_push(&event);
    }
}

static void dfu_respond(dfu_packet_t* p_dfu_packet)
{
    p_dfu_packet->payload.state.authority = m_authority;
    p_dfu_packet->payload.state.transaction_id = rand_prng_get(&m_rand);
    p_dfu_packet->payload.state.flood = 1;
    p_dfu_packet->payload.state.is_target = false;
}

static void packet_handle_state(dfu_packet_t* p_dfu_packet)
{
    fwid_union_t* p_packet_fwid = &p_dfu_packet->payload.state.fwid;
    bool i_have_requested_fw = false;
    mesh_packet_t* p_mesh_packet = mesh_packet_get_aligned(p_dfu_packet);

    switch (p_dfu_packet->payload.state.dfu_type)
    {
        case DFU_TYPE_APP:
            if (p_packet_fwid->app.company_id == m_fwid.app.company_id &&
                p_packet_fwid->app.app_id == m_fwid.app.app_id &&
                p_packet_fwid->app.app_version == m_fwid.app.app_version)
            {
                i_have_requested_fw = true;
            }
            break;

        case DFU_TYPE_SD:
            if (p_packet_fwid->sd == m_fwid.sd)
            {
                i_have_requested_fw = true;
            }
            break;
        case DFU_TYPE_BOOTLOADER:
            if (p_packet_fwid->bootloader == m_fwid.bootloader)
            {
                i_have_requested_fw = true;
            }
            break;
        default:
            break;
    }

    bool update_packet = false;

    if (i_have_requested_fw && m_authority > p_dfu_packet->payload.state.authority)
    {
        /* update data in packet */
        dfu_respond(p_dfu_packet);
        m_current_transfer.role = DFU_ROLE_SOURCE_CANDIDATE;
        update_packet = true;
    }
    else
    {
        bool packet_authority_is_higher =
            p_dfu_packet->payload.state.authority > m_current_transfer.authority ||
            (
             p_dfu_packet->payload.state.authority == m_current_transfer.authority &&
             p_dfu_packet->payload.state.transaction_id > m_current_transfer.transaction_id
            );
        bool update_parent = false;

        switch (m_current_transfer.role)
        {
            case DFU_ROLE_NONE:
                m_current_transfer.role = DFU_ROLE_POTENTIAL_RELAY;
                update_packet = true;
                break;

            case DFU_ROLE_SOURCE_CANDIDATE:
                if (packet_authority_is_higher)
                {
                    m_current_transfer.role == DFU_ROLE_POTENTIAL_RELAY; /* someone is a better candidate */
                    update_packet = true;
                }
                break;

            case DFU_ROLE_POTENTIAL_RELAY:
            case DFU_ROLE_RELAY:
                if (packet_authority_is_higher)
                {
                    update_packet = true;
                }
                break;
            default:
                break;
        }

        if (update_parent)
        {
            m_current_transfer.parent_is_target = p_dfu_packet->payload.state.is_target;
            if (p_mesh_packet)
            {
                memcpy(m_current_transfer.tree_parent_addr, p_mesh_packet->addr, BLE_GAP_ADDR_LEN);
            }
            else
            {
                memset(m_current_transfer.tree_parent_addr, 0, BLE_GAP_ADDR_LEN);
            }
        }
    }

    if (update_packet)
    {
        /* TODO */
        /* TODO: set fwid */
        if (mp_state_beacon)
        {
            transport_tx_abort(mp_state_beacon);
            mesh_packet_ref_count_dec(mp_state_beacon);
        }
        if (mesh_packet_acquire(&mp_state_beacon))
        {
            m_current_transfer.authority        = p_dfu_packet->payload.state.authority;
            m_current_transfer.type             = p_dfu_packet->payload.state.dfu_type;
            m_current_transfer.transaction_id   = p_dfu_packet->payload.state.transaction_id;
            memcpy(&m_current_transfer.fwid,
                   &p_dfu_packet->payload.state.fwid,
                   sizeof(fwid_union_t));

            /* adopt the packet */
            dfu_packet_build(mp_state_beacon,
                             p_dfu_packet);
            transport_tx(mp_state_beacon, &m_state_config);
        }
    }
}

static void packet_handle_data(dfu_packet_t* p_dfu_packet, uint64_t timestamp)
{
    bool retransmit = false;
    switch (m_current_transfer.role)
    {
        case DFU_ROLE_NONE:
        case DFU_ROLE_POTENTIAL_RELAY:
        case DFU_ROLE_SOURCE:
        case DFU_ROLE_SOURCE_CANDIDATE:
            m_current_transfer.role = DFU_ROLE_NONE;
            break;

        case DFU_ROLE_RELAY:
            for (uint32_t i = 0; i < SEGMENT_CACHE_SIZE && i < m_segment_cache_index; ++i)
            {
                if (m_segment_cache[i] == p_dfu_packet->payload.data.segment)
                {
                    return;
                }
            }

            retransmit = true;
            break;
    }

    if (retransmit)
    {
        if (
        m_segment_cache[(m_segment_cache_index++) & (SEGMENT_CACHE_SIZE - 1)]
            = p_dfu_packet->payload.data.segment;
    }
}

void mesh_dfu_init(void)
{
    //TODO: get FWID
    //TODO: start FWID beacon
    rand_prng_seed(&m_rand);
    memset(m_segment_cache, 0, 2 * SEGMENT_CACHE_SIZE);
    m_segment_cache_index = 0;
    memset(m_retransmit_entries, 0, sizeof(retransmit_entry_t) * RETRANSMIT_ENTRY_COUNT);
    m_retransmit_index = 0;

    m_fwid_config.tx_callback = NULL;
    m_fwid_config.channel_map = DFU_CHANNELS;
    m_fwid_config.interval_us = TX_INTERVAL_FWID;
    m_fwid_config.interval_type = TX_INTERVAL_TYPE_REGULAR;
    m_fwid_config.repeats = TX_REPEATS_INF;

    m_state_config.tx_callback = NULL;
    m_state_config.channel_map = DFU_CHANNELS;
    m_state_config.interval_us = TX_INTERVAL_STATE;
    m_state_config.interval_type = TX_INTERVAL_TYPE_REGULAR;
    m_state_config.repeats = TX_REPEATS_INF;

    m_data_config.tx_callback = NULL;
    m_data_config.channel_map = DFU_CHANNELS;
    m_data_config.interval_us = TX_INTERVAL_DATA;
    m_data_config.interval_type = TX_INTERVAL_TYPE_EXPONENTIAL;
    m_data_config.repeats = TX_REPEATS_DEFAULT;

    m_data_rsp_config.tx_callback = NULL;
    m_data_rsp_config.channel_map = DFU_CHANNELS;
    m_data_rsp_config.interval_us = TX_INTERVAL_DATA_RSP;
    m_data_rsp_config.interval_type = TX_INTERVAL_TYPE_EXPONENTIAL;
    m_data_rsp_config.repeats = TX_REPEATS_DEFAULT;
}

void mesh_dfu_packet_handle(dfu_packet_t* p_dfu_packet, uint32_t timestamp)
{
    switch (p_dfu_packet->packet_type)
    {
        case DFU_PACKET_TYPE_FWID:
            packet_handle_fwid(p_dfu_packet);
            break;

        case DFU_PACKET_TYPE_STATE:
            packet_handle_state(p_dfu_packet);
            break;

        case DFU_PACKET_TYPE_DATA:
            packet_handle_data(p_dfu_packet, timestamp);
            break;

    }
}

uint32_t mesh_dfu_enter_bootloader(dfu_type_t dfu_type)
{

    return NRF_ERROR_NOT_SUPPORTED;
}

void mesh_dfu_source_begin(dfu_type_t dfu_type, uint8_t authority)
{

}

