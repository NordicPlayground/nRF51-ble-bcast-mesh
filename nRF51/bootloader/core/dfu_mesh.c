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
#include "dfu_mesh.h"
#include "sha256.h"
#include "dfu_transfer_mesh.h"
#include "dfu_types_mesh.h"
#include "uECC.h"
#include "bootloader_info.h"
#include "bootloader_app_bridge.h"
#include "nrf_mbr.h"

/*****************************************************************************
* Local defines
*****************************************************************************/
#define TX_REPEATS_DEFAULT          (3)
#define TX_REPEATS_FWID             (TX_REPEATS_INF)
#define TX_REPEATS_DFU_REQ          (TX_REPEATS_INF)
#define TX_REPEATS_READY            (TX_REPEATS_INF)
#define TX_REPEATS_DATA             (TX_REPEATS_DEFAULT)
#define TX_REPEATS_RSP              (TX_REPEATS_DEFAULT)
#define TX_REPEATS_REQ              (TX_REPEATS_DEFAULT)
#define TX_REPEATS_START            (TX_REPEATS_DEFAULT);

#define TX_INTERVAL_TYPE_FWID       (BL_RADIO_INTERVAL_TYPE_REGULAR)
#define TX_INTERVAL_TYPE_DFU_REQ    (BL_RADIO_INTERVAL_TYPE_REGULAR)
#define TX_INTERVAL_TYPE_READY      (BL_RADIO_INTERVAL_TYPE_REGULAR)
#define TX_INTERVAL_TYPE_DATA       (BL_RADIO_INTERVAL_TYPE_EXPONENTIAL)
#define TX_INTERVAL_TYPE_RSP        (BL_RADIO_INTERVAL_TYPE_EXPONENTIAL)
#define TX_INTERVAL_TYPE_REQ        (BL_RADIO_INTERVAL_TYPE_REGULAR)

#define STATE_TIMEOUT_FIND_FWID     ( 5000000) /*  5.0s */
#define STATE_TIMEOUT_REQ           ( 1000000) /*  1.0s */
#define STATE_TIMEOUT_READY         ( 3000000) /*  3.0s */
#define STATE_TIMEOUT_TARGET        ( 5000000) /*  5.0s */
#define STATE_TIMEOUT_RAMPDOWN      ( 1000000) /*  1.0s */
#define STATE_TIMEOUT_RELAY         (10000000) /* 10.0s */

#define TX_SLOT_BEACON              (0)    

#define START_ADDRESS_UNKNOWN       (0xFFFFFFFF)

#define PACKET_CACHE_SIZE           (16)

#define REQ_CACHE_SIZE              (4)
#define TRANSACTION_ID_CACHE_SIZE   (8)
#define REQ_RX_COUNT_RETRY          (8)

/*****************************************************************************
* Local typedefs
*****************************************************************************/
typedef enum
{
    BEACON_TYPE_FWID,
    BEACON_TYPE_READY_APP,
    BEACON_TYPE_READY_SD,
    BEACON_TYPE_READY_BL
} beacon_type_t;

typedef struct
{
    uint32_t        transaction_id;
    uint8_t         authority;
    dfu_type_t      type;
    uint32_t*       p_start_addr; /* duplicate */
    uint32_t*       p_bank_addr; /* duplicate */
    uint32_t*       p_indicated_start_addr;
    uint32_t*       p_last_requested_entry;
    uint32_t        length; /* duplicate */
    uint32_t        signature_length; /* redundant */
    uint8_t         signature[DFU_SIGNATURE_LEN];
    uint8_t         signature_bitmap;
    uint16_t        segments_remaining; /* redundant? */
    uint16_t        segment_count; /* redundant */
    fwid_union_t    target_fwid_union;
    bool            segment_is_valid_after_transfer;
    bool            flood;
} transaction_t;

typedef struct
{
    bl_info_version_t*      p_fwid;
    bl_info_segment_t*      p_segment_sd;
    bl_info_segment_t*      p_segment_bl;
    bl_info_segment_t*      p_segment_app;
    bl_info_flags_t*        p_flags;
    uint8_t*                p_ecdsa_public_key;
    uint8_t*                p_signature_sd;
    uint8_t*                p_signature_bl;
    uint8_t*                p_signature_app;
    uint8_t*                p_signature_bl_info;
} bl_info_pointers_t;

typedef struct
{
    uint16_t segment;
    uint16_t rx_count;
} req_cache_entry_t;

typedef struct
{
    dfu_packet_type_t   type;
    uint16_t            segment;
} packet_cache_entry_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
static transaction_t            m_transaction;
static dfu_state_t              m_state = DFU_STATE_FIND_FWID;
static bl_info_pointers_t       m_bl_info_pointers;
static req_cache_entry_t        m_req_cache[REQ_CACHE_SIZE];
static uint8_t                  m_req_index;
static uint32_t                 m_tid_cache[TRANSACTION_ID_CACHE_SIZE];
static uint8_t                  m_tid_index;
static packet_cache_entry_t     m_packet_cache[PACKET_CACHE_SIZE];
static uint32_t                 m_packet_cache_index;
static uint8_t                  m_tx_slots;
/*****************************************************************************
* Static Functions
*****************************************************************************/
static beacon_type_t state_beacon_type(dfu_type_t transfer_type)
{
    switch (transfer_type)
    {
        case DFU_TYPE_APP:
            return BEACON_TYPE_READY_APP;
        case DFU_TYPE_BOOTLOADER:
            return BEACON_TYPE_READY_BL;
        case DFU_TYPE_SD:
            return BEACON_TYPE_READY_SD;
        default:
            return BEACON_TYPE_FWID;
    }
}

static void keep_alive(void)
{
    bl_evt_t keep_alive_evt;
    keep_alive_evt.type = BL_EVT_TYPE_KEEP_ALIVE;
    bootloader_evt_send(&keep_alive_evt);
}

static void packet_tx_dynamic(dfu_packet_t* p_packet, 
    uint32_t length, 
    bl_radio_interval_type_t interval_type, 
    uint8_t repeats)
{
    static uint8_t tx_slot = 1;
    bl_evt_t tx_evt;
    tx_evt.type = BL_EVT_TYPE_TX_RADIO;
    tx_evt.params.tx.radio.p_dfu_packet = p_packet;
    tx_evt.params.tx.radio.length = length;
    tx_evt.params.tx.radio.interval_type = interval_type;
    tx_evt.params.tx.radio.tx_count = repeats;
    tx_evt.params.tx.radio.tx_slot = tx_slot;
    bootloader_evt_send(&tx_evt);
    
    if (++tx_slot >= m_tx_slots)
    {
        tx_slot = 1;
    }
}

static bool packet_is_from_serial(void* p_packet)
{
#ifdef SERIAL
    /* serial packets will have the device's own address as source */

    return (memcmp(mesh_packet_get_aligned(p_packet)->addr, (const uint8_t*) &NRF_FICR->DEVICEADDR[0], BLE_GAP_ADDR_LEN) == 0);
#else
    return false;
#endif
}

static bool fwid_union_cmp(fwid_union_t* p_a, fwid_union_t* p_b, dfu_type_t dfu_type)
{
    switch (dfu_type)
    {
        case DFU_TYPE_APP:
            return memcmp(&p_a->app,
                          &p_b->app,
                          sizeof(app_id_t)) == 0;
        case DFU_TYPE_SD:
            return (p_a->sd == p_b->sd);
        case DFU_TYPE_BOOTLOADER:
            return memcmp(&p_a->bootloader,
                          &p_b->bootloader,
                          sizeof(bl_id_t)) == 0;
        default: return false;
    }
}

static bool signature_check(void)
{
    /* if we don't have a public key we will accept all firmware upgrades. */
    if (m_bl_info_pointers.p_ecdsa_public_key == NULL)
    {
        return true;
    }

    /* if we have a key, but the transfer isn't signed, we will fail */
    if (m_transaction.signature_length == 0)
    {
        return false;
    }

    uint8_t hash[uECC_BYTES];
    sha256_context_t hash_context;
    sha256_init(&hash_context);
    sha256_update(&hash_context, (uint8_t*) &m_transaction.type, 1);
    sha256_update(&hash_context, (uint8_t*) &m_transaction.p_indicated_start_addr, 4);
    sha256_update(&hash_context, (uint8_t*) &m_transaction.length, 4);
    uint8_t padding = 0;
    sha256_update(&hash_context, &padding, 1);

    switch (m_transaction.type)
    {
        case DFU_TYPE_APP:
            sha256_update(&hash_context, (uint8_t*) &m_transaction.target_fwid_union, DFU_FWID_LEN_APP);
            break;
        case DFU_TYPE_SD:
            sha256_update(&hash_context, (uint8_t*) &m_transaction.target_fwid_union, DFU_FWID_LEN_SD);
            break;
        case DFU_TYPE_BOOTLOADER:
            sha256_update(&hash_context, (uint8_t*) &m_transaction.target_fwid_union, DFU_FWID_LEN_BL);
            break;
        default:
            break;
    }

    dfu_transfer_sha256(&hash_context);

    sha256_final(&hash_context, hash);
    return (bool) (uECC_verify(m_bl_info_pointers.p_ecdsa_public_key, hash, m_transaction.signature));
}

static bool ready_packet_is_upgrade(dfu_packet_t* p_packet)
{
    /* check that we haven't failed to connect to this TID before */
    for (uint32_t i = 0; i < TRANSACTION_ID_CACHE_SIZE; ++i)
    {
        if (m_tid_cache[i] == p_packet->payload.state.transaction_id)
        {
            return false;
        }
    }

    switch (p_packet->payload.state.dfu_type)
    {
        case DFU_TYPE_APP:
            return (p_packet->payload.state.fwid.app.app_id == m_bl_info_pointers.p_fwid->app.app_id &&
                    p_packet->payload.state.fwid.app.company_id== m_bl_info_pointers.p_fwid->app.company_id &&
                    p_packet->payload.state.fwid.app.app_version > m_bl_info_pointers.p_fwid->app.app_version);

        case DFU_TYPE_BOOTLOADER:
            return (p_packet->payload.state.fwid.bootloader.id ==
                    m_bl_info_pointers.p_fwid->bootloader.id &&
                    p_packet->payload.state.fwid.bootloader.ver >
                    m_bl_info_pointers.p_fwid->bootloader.ver);

        case DFU_TYPE_SD:
            return (p_packet->payload.state.fwid.sd ==
                    m_bl_info_pointers.p_fwid->sd);
        default:
            return false;
    }
}

static bool ready_packet_matches_our_req(dfu_packet_t* p_packet)
{
    if (p_packet->payload.state.dfu_type != m_transaction.type && m_transaction.type != DFU_TYPE_NONE)
    {
        return false;
    }

    /* check that we haven't failed to connect to this TID before */
    for (uint32_t i = 0; i < TRANSACTION_ID_CACHE_SIZE; ++i)
    {
        if (m_tid_cache[i] == p_packet->payload.state.transaction_id)
        {
            return false;
        }
    }
    return fwid_union_cmp(&p_packet->payload.state.fwid,
                   &m_transaction.target_fwid_union,
                   m_transaction.type);
}

static void beacon_set(beacon_type_t type)
{
    dfu_packet_t dfu_packet;
    memset(&dfu_packet, 0, sizeof(dfu_packet_t));

    uint8_t repeats = TX_REPEATS_FWID;
    bl_radio_interval_type_t interval_type = TX_INTERVAL_TYPE_FWID;
    uint8_t length = 0;

    if (type >= BEACON_TYPE_READY_APP &&
        type <= BEACON_TYPE_READY_BL)
    {
        dfu_packet.packet_type = DFU_PACKET_TYPE_STATE;
        dfu_packet.payload.state.authority = m_transaction.authority;
        dfu_packet.payload.state.flood = m_transaction.flood;
        dfu_packet.payload.state.transaction_id = m_transaction.transaction_id;
        dfu_packet.payload.state.relay_node = (m_state == DFU_STATE_RELAY_CANDIDATE);
        dfu_packet.payload.state.fwid = m_transaction.target_fwid_union;
        repeats = TX_REPEATS_READY;
        interval_type = TX_INTERVAL_TYPE_READY;
    }

    switch (type)
    {
        case BEACON_TYPE_FWID:
            length = DFU_PACKET_LEN_FWID;
            dfu_packet.packet_type = DFU_PACKET_TYPE_FWID;
            dfu_packet.payload.fwid = *m_bl_info_pointers.p_fwid;
            break;

        case BEACON_TYPE_READY_APP:
            length = DFU_PACKET_LEN_STATE_APP;
            dfu_packet.payload.state.dfu_type = DFU_TYPE_APP;
            break;

        case BEACON_TYPE_READY_SD:
            length = DFU_PACKET_LEN_STATE_SD;
            dfu_packet.payload.state.dfu_type = DFU_TYPE_SD;
            break;

        case BEACON_TYPE_READY_BL:
            length = DFU_PACKET_LEN_STATE_BL;
            dfu_packet.payload.state.dfu_type = DFU_TYPE_BOOTLOADER;
            break;
    }
    
    bl_evt_t tx_evt;
    tx_evt.type = BL_EVT_TYPE_TX_RADIO;
    tx_evt.params.tx.radio.p_dfu_packet = &dfu_packet;
    tx_evt.params.tx.radio.length = length;
    tx_evt.params.tx.radio.interval_type = interval_type;
    tx_evt.params.tx.radio.tx_count = repeats;
    tx_evt.params.tx.radio.tx_slot = TX_SLOT_BEACON;
    
    bootloader_evt_send(&tx_evt);
    
#ifdef SERIAL
    tx_evt.type = BL_EVT_TYPE_TX_SERIAL;
    tx_evt.params.tx.serial.p_dfu_packet = &dfu_packet;
    tx_evt.params.tx.serial.length = length;
    bootloader_evt_send(&tx_evt);
#endif
}

static inline uint32_t* addr_from_seg(uint16_t segment)
{
    if (segment == 1)
    {
        return m_transaction.p_start_addr;
    }
    else
    {
        return (uint32_t*) (((segment - 1) << 4) + ((uint32_t) m_transaction.p_start_addr & 0xFFFFFFF0));
    }
}

static bool app_is_newer(app_id_t* p_app_id)
{
    return (p_app_id->app_id     == m_bl_info_pointers.p_fwid->app.app_id &&
            p_app_id->company_id == m_bl_info_pointers.p_fwid->app.company_id &&
            p_app_id->app_version > m_bl_info_pointers.p_fwid->app.app_version);
}

static bool bootloader_is_newer(bl_id_t bl_id)
{
    return (bl_id.id == m_bl_info_pointers.p_fwid->bootloader.id &&
            bl_id.ver > m_bl_info_pointers.p_fwid->bootloader.ver);
}

/********** STATE MACHINE ENTRY POINTS ***********/
static void start_find_fwid(void)
{
    beacon_set(BEACON_TYPE_FWID);
    m_state = DFU_STATE_FIND_FWID;
    memset(&m_transaction, 0, sizeof(transaction_t));
}

static void start_req(dfu_type_t type, bool timeout)
{
    m_transaction.authority = 0;
    m_transaction.length = 0;
    m_transaction.p_bank_addr = NULL;
    m_transaction.p_start_addr = NULL;
    m_transaction.segments_remaining = 0xFFFF;
    m_transaction.segment_count = 0;
    m_transaction.segment_is_valid_after_transfer = false;
    m_transaction.signature_length = 0;
    m_transaction.transaction_id = 0;
    m_transaction.type = type;
    m_state = DFU_STATE_DFU_REQ;

    beacon_set(state_beacon_type(type));
}

static void start_ready(dfu_packet_t* p_ready_packet)
{
    if (p_ready_packet->packet_type != DFU_PACKET_TYPE_STATE ||
        p_ready_packet->payload.state.authority == 0 ||
        p_ready_packet->payload.state.dfu_type != m_transaction.type)
    {
        APP_ERROR_CHECK(NRF_ERROR_INVALID_PARAM);
    }
    m_transaction.transaction_id = p_ready_packet->payload.state.transaction_id;
    m_transaction.authority = p_ready_packet->payload.state.authority;
    m_transaction.flood = p_ready_packet->payload.state.flood;
    m_state = DFU_STATE_DFU_READY;

    beacon_set(state_beacon_type(m_transaction.type));
}

static void start_target(void)
{
    m_state = DFU_STATE_DFU_TARGET;

    uint32_t segment_size = 0;
    bl_info_entry_t flags_entry;
    memset(&flags_entry, 0xFF, (BL_INFO_LEN_FLAGS + 3) & ~0x03UL);

    switch (m_transaction.type)
    {
        case DFU_TYPE_SD:
            segment_size = m_bl_info_pointers.p_segment_sd->length;
            flags_entry.flags.sd_intact = false;
            break;
        case DFU_TYPE_APP:
            segment_size = m_bl_info_pointers.p_segment_app->length;
            flags_entry.flags.app_intact = false;
            break;
        case DFU_TYPE_BOOTLOADER:
            segment_size = m_bl_info_pointers.p_segment_bl->length;
            flags_entry.flags.bl_intact = false;

            /* check whether we're destroying the application: */
            for (uint32_t* p_addr = m_transaction.p_bank_addr;
                 p_addr < (uint32_t*) (m_bl_info_pointers.p_segment_app->start + m_bl_info_pointers.p_segment_app->length);
                 p_addr++)
            {
                if (*p_addr != 0xFFFFFFFF)
                {
                    flags_entry.flags.app_intact = false;
                }
            }
            break;
        default:
            segment_size = 0;
    }

    /* Tag the transfer as incomplete in device page if we're about to overwrite it. */
    if (m_transaction.p_start_addr == m_transaction.p_bank_addr)
    {
        if (m_bl_info_pointers.p_flags == NULL)
        {
            m_bl_info_pointers.p_flags = &bootloader_info_entry_put(BL_INFO_TYPE_FLAGS, &flags_entry, BL_INFO_LEN_FLAGS)->flags;
        }
        else
        {
            /* update inline */
            if (flash_write(
                    (uint32_t*) m_bl_info_pointers.p_flags,
                    (uint8_t*) &flags_entry,
                    (BL_INFO_LEN_FLAGS + 3) & ~0x03UL) != NRF_SUCCESS)
            {
                start_req(m_transaction.type, true);
            }
        }
    }

    if (dfu_transfer_start(
            m_transaction.p_start_addr,
            m_transaction.p_bank_addr,
            m_transaction.length,
            segment_size,
            m_transaction.segment_is_valid_after_transfer) != NRF_SUCCESS)
    {
        start_req(m_transaction.type, true);
    }
    else
    {
        bl_evt_t send_abort_evt_evt;
        send_abort_evt_evt.type = BL_EVT_TYPE_TX_ABORT;
        send_abort_evt_evt.params.tx.abort.tx_slot = TX_SLOT_BEACON;
        bootloader_evt_send(&send_abort_evt_evt);
    }
}

static void start_rampdown(void)
{
    bl_evt_t timer_evt;
    timer_evt.type = BL_EVT_TYPE_TIMER_SET;
    timer_evt.params.timer.set.delay_us = STATE_TIMEOUT_RAMPDOWN;
    timer_evt.params.timer.set.index = 0;
    bootloader_evt_send(&timer_evt);
    
    m_state = DFU_STATE_VALIDATE;
}

static void start_relay_candidate(dfu_packet_t* p_packet)
{
    m_transaction.authority = p_packet->payload.state.authority;
    m_transaction.transaction_id = p_packet->payload.state.transaction_id;
    m_transaction.target_fwid_union = p_packet->payload.state.fwid;
    m_state = DFU_STATE_RELAY_CANDIDATE;

    beacon_set(state_beacon_type(m_transaction.type));
}


static bool packet_in_cache(dfu_packet_t* p_packet)
{
    for (uint32_t i = 0; i < PACKET_CACHE_SIZE; ++i)
    {
        if (m_packet_cache[i].type    == p_packet->packet_type &&
            m_packet_cache[i].segment == p_packet->payload.data.segment)
        {
            return true;
        }
    }
    return false;
}

static void packet_cache_put(dfu_packet_t* p_packet)
{
    m_packet_cache[(m_packet_cache_index) & (PACKET_CACHE_SIZE - 1)].type = (dfu_packet_type_t) p_packet->packet_type;
    m_packet_cache[(m_packet_cache_index) & (PACKET_CACHE_SIZE - 1)].segment = p_packet->payload.data.segment;
    m_packet_cache_index++;
}

static void relay_packet(dfu_packet_t* p_packet, uint16_t length)
{
    mesh_packet_t* p_mesh_packet = mesh_packet_get_aligned(p_packet);
    if (!p_mesh_packet)
    {
        if (!mesh_packet_acquire(&p_mesh_packet))
        {
            APP_ERROR_CHECK(NRF_ERROR_NO_MEM);
        }
        mesh_packet_build(
            p_mesh_packet,
            p_packet->packet_type,
            p_packet->payload.data.segment,
            (uint8_t*) &p_packet->payload.data.transaction_id,
            length - 4);
    }
    else
    {
        mesh_packet_ref_count_inc(p_mesh_packet);
    }

    packet_tx_dynamic(p_packet, length, TX_INTERVAL_TYPE_DATA, TX_REPEATS_DATA);
    packet_cache_put(p_packet);
    mesh_packet_ref_count_dec(p_mesh_packet);
}

static void handle_data_packet(dfu_packet_t* p_packet, uint16_t length)
{
    bool do_relay = false;
    if (p_packet->payload.data.transaction_id == m_transaction.transaction_id)
    {
        /* check and add to cache */
        if (packet_in_cache(p_packet))
        {
            return;
        }

        if (m_state == DFU_STATE_DFU_READY)
        {
            if (p_packet->payload.start.segment == 0)
            {
                bl_info_segment_t* p_segment = NULL;
                switch (m_transaction.type)
                {
                    case DFU_TYPE_APP:
                        p_segment = m_bl_info_pointers.p_segment_app;
                        break;
                    case DFU_TYPE_SD:
                        p_segment = m_bl_info_pointers.p_segment_sd;
                        break;
                    case DFU_TYPE_BOOTLOADER:
                        p_segment = m_bl_info_pointers.p_segment_bl;
                        break;
                    default:
                        APP_ERROR_CHECK(NRF_ERROR_NOT_SUPPORTED);
                }

                m_transaction.p_indicated_start_addr = (uint32_t*) p_packet->payload.start.start_address;
                uint32_t start_address = p_packet->payload.start.start_address;
                /* if the host doesn't know the start address, we use start of segment: */
                if (start_address == START_ADDRESS_UNKNOWN)
                {
                    start_address = p_segment->start;
                }

                uint32_t segment_count = ((p_packet->payload.start.length * 4) + (start_address & 0x0F) - 1) / 16 + 1;

                if (p_packet->payload.start.signature_length != 0)
                {
                    segment_count += p_packet->payload.start.signature_length / SEGMENT_LENGTH;
                }
                if (segment_count > 0xFFFF)
                {
                    /* can't have more than 65536 segments in a transmission */
                    segment_count = 0xFFFF;
                }

                m_transaction.segments_remaining                = segment_count;
                m_transaction.segment_count                     = segment_count;
                m_transaction.p_start_addr                      = (uint32_t*) start_address;
                m_transaction.length                            = p_packet->payload.start.length * 4;
                m_transaction.signature_length                  = p_packet->payload.start.signature_length;
                m_transaction.segment_is_valid_after_transfer   = p_packet->payload.start.last;
                m_transaction.p_last_requested_entry            = NULL;
                m_transaction.signature_bitmap                  = 0;


                if (m_transaction.type == DFU_TYPE_BOOTLOADER)
                {
                    m_transaction.p_bank_addr = (uint32_t*) (
                        (m_bl_info_pointers.p_segment_app->start) +
                        (m_bl_info_pointers.p_segment_app->length) -
                        (m_transaction.length & ((uint32_t) ~(PAGE_SIZE - 1))) -
                        (PAGE_SIZE)
                    );

                }
                else
                {
                    m_transaction.p_bank_addr = m_transaction.p_start_addr;
                }

                if ((uint32_t) m_transaction.p_start_addr >= p_segment->start &&
                    (uint32_t) m_transaction.p_start_addr + m_transaction.length <= p_segment->start + p_segment->length)
                {
                    start_target();
                    do_relay = true;
                }
            }
            else
            {
                m_tid_cache[(m_tid_index++) & (TRANSACTION_ID_CACHE_SIZE - 1)] = m_transaction.transaction_id;
                start_req(m_transaction.type, true); /* go back to req, we've missed packet 0 */
            }
        }
        else if (m_state == DFU_STATE_DFU_TARGET)
        {
            if (p_packet->payload.data.segment > 0 &&
                p_packet->payload.data.segment <= m_transaction.segment_count)
            {
                uint32_t* p_addr = NULL;
                uint32_t error_code = NRF_ERROR_NULL;

                if (p_packet->payload.data.segment <=
                    m_transaction.segment_count - m_transaction.signature_length / SEGMENT_LENGTH)
                {
                    p_addr = addr_from_seg(p_packet->payload.data.segment);
                    error_code  = dfu_transfer_data((uint32_t) p_addr,
                                               p_packet->payload.data.data,
                                               length - (DFU_PACKET_LEN_DATA - SEGMENT_LENGTH));
                }
                else /* treat signature packets at the end */
                {
                    uint32_t index = p_packet->payload.data.segment - (m_transaction.segment_count - m_transaction.signature_length / SEGMENT_LENGTH) - 1;
                    if (index >= m_transaction.signature_length / SEGMENT_LENGTH ||
                        m_transaction.signature_bitmap & (1 << index))
                    {
                        error_code = NRF_ERROR_INVALID_STATE;
                    }
                    else
                    {
                        memcpy(&m_transaction.signature[index * SEGMENT_LENGTH],
                               p_packet->payload.data.data,
                               length - (DFU_PACKET_LEN_DATA - SEGMENT_LENGTH));

                        m_transaction.signature_bitmap |= (1 << index);
                        error_code = NRF_SUCCESS;
                    }
                }

                if (error_code == NRF_SUCCESS)
                {
                    keep_alive();
                    m_transaction.segments_remaining--;
                    do_relay = true;
                    /* check whether we've lost any entries, and request them */
                    uint32_t* p_req_entry = NULL;
                    uint32_t req_entry_len = 0;
                    
                    if (dfu_transfer_get_oldest_missing_entry(
                            m_transaction.p_last_requested_entry,
                            &p_req_entry,
                            &req_entry_len) &&
                        (
                         /* don't request the previous packet yet */
                         ADDR_SEGMENT(p_req_entry, m_transaction.p_start_addr) < p_packet->payload.data.segment - 1 ||
                         m_transaction.segment_count == p_packet->payload.data.segment
                        )
                       )
                    {
                        dfu_packet_t req_packet;
                        req_packet.packet_type = DFU_PACKET_TYPE_DATA_REQ;
                        req_packet.payload.req_data.segment = ADDR_SEGMENT(p_req_entry, m_transaction.p_start_addr);
                        req_packet.payload.req_data.transaction_id = m_transaction.transaction_id;
                        
                        packet_tx_dynamic(&req_packet, DFU_PACKET_LEN_DATA_REQ, TX_INTERVAL_TYPE_REQ, TX_REPEATS_REQ);
                        m_transaction.p_last_requested_entry = (uint32_t*) p_req_entry;
                    }
                }
            }

            /* ending the DFU */
            if (m_transaction.segments_remaining == 0)
            {
                dfu_transfer_end();
                start_rampdown();
            }
        }
        else if (m_state == DFU_STATE_RELAY_CANDIDATE ||
                 m_state == DFU_STATE_RELAY)
        {
            m_state = DFU_STATE_RELAY;
            tx_abort(TX_SLOT_BEACON);
            keep_alive();
            do_relay = true;
        }
    }
    
    packet_cache_put(p_packet);
    
    if (do_relay)
    {
        relay_packet(p_packet, length);
    }
}

static void handle_state_packet(dfu_packet_t* p_packet)
{
    switch (m_state)
    {
        case DFU_STATE_FIND_FWID:
            if (p_packet->payload.state.authority > 0)
            {
                m_transaction.type = (dfu_type_t) p_packet->payload.state.dfu_type;
                m_transaction.target_fwid_union = p_packet->payload.state.fwid;

                if (ready_packet_is_upgrade(p_packet))
                {
                    bl_evt_t fw_evt;
                    fw_evt.type = BL_EVT_TYPE_NEW_FW;
                    fw_evt.params.new_fw.fw_type = p_packet->payload.state.dfu_type;
                    memcpy(&fw_evt.params.new_fw.fwid, &p_packet->payload.state.fwid, sizeof(fwid_union_t));
                    bootloader_evt_send(&fw_evt);
                }
                else
                {
                    //TODO
                    start_relay_candidate(p_packet);
                }
            }
            else
            {
                //TODO
                //handle relaying of requests
            }
            break;
        case DFU_STATE_DFU_REQ:
            if (p_packet->payload.state.authority > 0)
            {
                if (ready_packet_matches_our_req(p_packet) ||
                    ready_packet_is_upgrade(p_packet))
                {
                    /* assume that the other device knows what to upgrade */
                    m_transaction.type = (dfu_type_t) p_packet->payload.state.dfu_type;
                    m_transaction.target_fwid_union = p_packet->payload.state.fwid;
                    
                    start_ready(p_packet);
                }
                else if (packet_is_from_serial(p_packet))
                {
                    start_relay_candidate(p_packet);
                }
            }
            break;
        case DFU_STATE_DFU_READY:
            if (ready_packet_matches_our_req(p_packet))
            {
                if (p_packet->payload.state.authority > m_transaction.authority ||
                    (
                     p_packet->payload.state.authority == m_transaction.authority &&
                     p_packet->payload.state.transaction_id > m_transaction.transaction_id
                    )
                   )
                {
                    m_transaction.authority = p_packet->payload.state.authority;
                    m_transaction.transaction_id = p_packet->payload.state.transaction_id;
                    beacon_set(state_beacon_type(m_transaction.type));
                }
            }
            break;
        case DFU_STATE_RELAY_CANDIDATE:
            if (m_transaction.type == p_packet->payload.state.dfu_type &&
                fwid_union_cmp(&m_transaction.target_fwid_union, &p_packet->payload.state.fwid, m_transaction.type))
            {
                if (m_transaction.authority < p_packet->payload.state.authority ||
                    (m_transaction.authority      == p_packet->payload.state.authority &&
                     m_transaction.transaction_id <  p_packet->payload.state.transaction_id))
                {
                    m_transaction.transaction_id = p_packet->payload.state.transaction_id;
                    m_transaction.authority      = p_packet->payload.state.authority;
                    beacon_set(state_beacon_type(m_transaction.type));
                }
            }
            break;
        default:
            break;
    }

}

static void handle_fwid_packet(dfu_packet_t* p_packet)
{
    if (m_state == DFU_STATE_FIND_FWID)
    {
        /* always upgrade bootloader first */
        if (bootloader_is_newer(p_packet->payload.fwid.bootloader))
        {
            bl_evt_t fwid_evt;
            fwid_evt.type = BL_EVT_TYPE_NEW_FW;
            fwid_evt.params.new_fw.fw_type = DFU_TYPE_BOOTLOADER;
            fwid_evt.params.new_fw.fwid.bootloader.id  = p_packet->payload.fwid.bootloader.id;
            fwid_evt.params.new_fw.fwid.bootloader.ver = p_packet->payload.fwid.bootloader.ver;
            bootloader_evt_send(&fwid_evt);
        }
        else if (app_is_newer(&p_packet->payload.fwid.app))
        {
            /* SD shall only be upgraded if a newer version of our app requires a different SD */
            if (p_packet->payload.fwid.sd != 0xFFFE && p_packet->payload.fwid.sd != m_bl_info_pointers.p_fwid->sd)
            {
                bl_evt_t fwid_evt;
                fwid_evt.type = BL_EVT_TYPE_NEW_FW;
                fwid_evt.params.new_fw.fw_type = DFU_TYPE_SD;
                fwid_evt.params.new_fw.fwid.sd = p_packet->payload.fwid.sd;
                bootloader_evt_send(&fwid_evt);
            }
            else
            {
                bl_evt_t fwid_evt;
                fwid_evt.type = BL_EVT_TYPE_NEW_FW;
                fwid_evt.params.new_fw.fw_type = DFU_TYPE_APP;
                memcpy(&fwid_evt.params.new_fw.fwid.app, &p_packet->payload.fwid.app, sizeof(app_id_t));
                bootloader_evt_send(&fwid_evt);
            }
        }
    }
}

static void handle_data_req_packet(dfu_packet_t* p_packet)
{
    if (p_packet->payload.data.transaction_id == m_transaction.transaction_id)
    {
        if (m_state == DFU_STATE_RELAY)
        {
            /* only relay new packets, look for it in cache */
            if (!packet_in_cache(p_packet))
            {
                keep_alive();
                relay_packet(p_packet, 8);
            }
        }
        else
        {
            req_cache_entry_t* p_req_entry = NULL;
            /* check that we haven't served this request recently. */
            for (uint32_t i = 0; i < REQ_CACHE_SIZE; ++i)
            {
                if (m_req_cache[i].segment == p_packet->payload.req_data.segment)
                {
                    if (m_req_cache[i].rx_count++ < REQ_RX_COUNT_RETRY)
                    {
                        return;
                    }
                    p_req_entry = &m_req_cache[i];
                    break;
                }
            }
            /* serve request */
            dfu_packet_t dfu_rsp;
            if (
                dfu_transfer_has_entry(
                    (uint32_t*) SEGMENT_ADDR(p_packet->payload.req_data.segment, m_transaction.p_start_addr),
                    dfu_rsp.payload.rsp_data.data, SEGMENT_LENGTH)
               )
            {
                dfu_rsp.packet_type = DFU_PACKET_TYPE_DATA_RSP;
                dfu_rsp.payload.rsp_data.segment = p_packet->payload.req_data.segment;
                dfu_rsp.payload.rsp_data.transaction_id = p_packet->payload.req_data.transaction_id;

                packet_tx_dynamic(&dfu_rsp, DFU_PACKET_LEN_DATA_RSP, TX_INTERVAL_TYPE_RSP, TX_REPEATS_RSP);

                /* reset time to allow for more updates */
                keep_alive();
            }

            /* log our attempt at responding */
            if (!p_req_entry)
            {
                p_req_entry = &m_req_cache[(m_req_index++) & (REQ_CACHE_SIZE - 1)];
                p_req_entry->segment = p_packet->payload.req_data.segment;
            }
            p_req_entry->rx_count = 0;
        }
    }
}

static void handle_data_rsp_packet(dfu_packet_t* p_packet, uint16_t length)
{
    if (m_state == DFU_STATE_RELAY)
    {
        /* only relay new packets, look for it in cache */
        if (!packet_in_cache(p_packet))
        {
            keep_alive();
            relay_packet(p_packet, length);
        }
    }
    else
    {
        handle_data_packet(p_packet, length);
    }
}

static bool fw_is_verified(void)
{
    bl_info_entry_t* p_flag_entry = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_FLAGS);
    if (p_flag_entry)
    {
        return (p_flag_entry->flags.sd_intact &&
                p_flag_entry->flags.app_intact &&
                p_flag_entry->flags.bl_intact);
    }

    return true;
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/
void dfu_mesh_init(uint8_t tx_slots)
{
    m_state = DFU_STATE_FIND_FWID;
    m_transaction.transaction_id = 0;
    m_transaction.type = DFU_TYPE_NONE;
    memset(m_req_cache, 0, REQ_CACHE_SIZE * sizeof(m_req_cache[0]));
    memset(m_tid_cache, 0, TRANSACTION_ID_CACHE_SIZE);
    memset(m_packet_cache, 0, PACKET_CACHE_SIZE * sizeof(packet_cache_entry_t));
    m_tid_index = 0;
    m_packet_cache_index = 0;
    m_req_index = 0;
    m_tx_slots = tx_slots;

    /* fetch persistent entries */
    m_bl_info_pointers.p_flags              = &bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_FLAGS)->flags;
    m_bl_info_pointers.p_fwid               = &bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_VERSION)->version;
    m_bl_info_pointers.p_segment_app        = &bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_SEGMENT_APP)->segment;
    m_bl_info_pointers.p_segment_bl         = &bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_SEGMENT_BL)->segment;
    m_bl_info_pointers.p_segment_sd         = &bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_SEGMENT_SD)->segment;
    m_bl_info_pointers.p_ecdsa_public_key   = &bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_ECDSA_PUBLIC_KEY)->public_key[0];

    if (
        ((uint32_t) m_bl_info_pointers.p_flags              < BOOTLOADER_INFO_ADDRESS) ||
        ((uint32_t) m_bl_info_pointers.p_fwid               < BOOTLOADER_INFO_ADDRESS) ||
        ((uint32_t) m_bl_info_pointers.p_segment_app        < BOOTLOADER_INFO_ADDRESS) ||
        ((uint32_t) m_bl_info_pointers.p_segment_sd         < BOOTLOADER_INFO_ADDRESS) ||
        ((uint32_t) m_bl_info_pointers.p_segment_bl         < BOOTLOADER_INFO_ADDRESS)
       )
    {
        send_abort_evt(BL_END_ERROR_INVALID_PERSISTENT_STORAGE);
    }
}

void dfu_mesh_start(void)
{
#if 0
    if (!m_bl_info_pointers.p_flags->sd_intact ||
         m_bl_info_pointers.p_fwid->sd == SD_VERSION_INVALID)
    {
        m_transaction.target_fwid_union.sd = 0;
        start_req(DFU_TYPE_SD, false);
    }
    else if (!dfu_mesh_app_is_valid())
    {
        memcpy(&m_transaction.target_fwid_union.app, &m_bl_info_pointers.p_fwid->app, sizeof(app_id_t));
        start_req(DFU_TYPE_APP, false);
    }
    else
    {
        start_find_fwid();
    }
#else
    const bl_info_type_t bank_types[] = {BL_INFO_TYPE_BANK_BL, BL_INFO_TYPE_BANK_SD, BL_INFO_TYPE_BANK_APP}; 
    const dfu_type_t dfu_types[] = {DFU_TYPE_BOOTLOADER, DFU_TYPE_SD, DFU_TYPE_APP};
    const void* p_curr_fwid[] = {&m_bl_info_pointers.p_fwid->bootloader, &m_bl_info_pointers.p_fwid->sd, &m_bl_info_pointers.p_fwid->app};

    for (uint32_t i = 0; i < 3; ++i)
    {
        bl_info_entry_t* p_bank_entry = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, bank_types[i]);
        if (p_bank_entry)
        {
            if (fwid_union_cmp(&p_bank_entry->bank.fwid, (fwid_union_t*) p_curr_fwid[i], dfu_types[i]))
            {
                bl_evt_t bank_evt;
                bank_evt.type = BL_EVT_TYPE_BANK_AVAILABLE;
                bank_evt.params.bank_available.bank_dfu_type = dfu_types[i];
                memcpy(&bank_evt.params.bank_available.bank_fwid, &p_bank_entry->bank.fwid, sizeof(fwid_union_t));
                memcpy(&bank_evt.params.bank_available.current_fwid, (fwid_union_t*) p_curr_fwid[i], sizeof(fwid_union_t));
                bank_evt.params.bank_available.p_bank_addr = p_bank_entry->bank.p_bank_addr;
                bank_evt.params.bank_available.bank_length = p_bank_entry->bank.length;
                bootloader_evt_send(&bank_evt);
            }
        }
    }
    
    start_find_fwid();
#endif
}

uint32_t dfu_mesh_rx(dfu_packet_t* p_packet, uint16_t length, bool from_serial)
{
#ifdef DEBUG_LEDS
    static bool led = false;
    if (led)
    {
        NRF_GPIO->OUTCLR = (1 << 22);
    }
    else
    {
        NRF_GPIO->OUTSET = (1 << 22);
    }
    led = !led;
    NRF_GPIO->OUTSET = (1 << 1);
#endif

    switch (p_packet->packet_type)
    {
        case DFU_PACKET_TYPE_FWID:
            handle_fwid_packet(p_packet);
            break;

        case DFU_PACKET_TYPE_STATE:
            handle_state_packet(p_packet);
            break;

        case DFU_PACKET_TYPE_DATA:
            handle_data_packet(p_packet, length);
            break;

        case DFU_PACKET_TYPE_DATA_REQ:
            handle_data_req_packet(p_packet);
            break;

        case DFU_PACKET_TYPE_DATA_RSP:
            handle_data_rsp_packet(p_packet, length);
            break;

        default:
            /* don't care */
            break;
    }
#ifdef DEBUG_LEDS
    NRF_GPIO->OUTCLR = (1 << 1);
#endif
    return NRF_SUCCESS;
}

void dfu_mesh_timeout(void)
{
    switch (m_state)
    {
        case DFU_STATE_FIND_FWID:
            send_abort_evt(BL_END_FWID_VALID);
            break;

        case DFU_STATE_DFU_REQ:
        case DFU_STATE_DFU_READY:
            send_abort_evt(BL_END_ERROR_NO_START);
            break;

        case DFU_STATE_DFU_TARGET:
            start_req(m_transaction.type, true);
            break;

        case DFU_STATE_VALIDATE:
            if (signature_check())
            {
                bl_evt_t end_evt;
                end_evt.type = BL_EVT_TYPE_END_TARGET;
                end_evt.params.end.dfu_type = m_transaction.type;
                memcpy(&end_evt.params.end.fwid, &m_transaction.target_fwid_union, sizeof(fwid_union_t));
                bootloader_evt_send(&end_evt);
            }
            else
            {
                /* someone gave us unauthorized firmware, and we're broken.
                   need to reboot and try to request a new transfer */
                send_abort_evt(BL_END_ERROR_UNAUTHORIZED);
            }
            break;
        case DFU_STATE_RELAY:
        case DFU_STATE_RELAY_CANDIDATE:
            send_abort_evt(BL_END_SUCCESS);
            break;
        default:
            break;
    }
}

bool dfu_mesh_app_is_valid(void)
{
    bl_info_entry_t* p_fwid_entry    = bootloader_info_entry_get((uint32_t*) BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_FLAGS);

    return (p_fwid_entry != NULL &&
            (uint32_t*) m_bl_info_pointers.p_segment_app->start != (uint32_t*) 0xFFFFFFFF &&
            *((uint32_t*) m_bl_info_pointers.p_segment_app->start) != 0xFFFFFFFF &&
            p_fwid_entry->version.app.app_version != APP_VERSION_INVALID &&
            fw_is_verified());
}

uint32_t dfu_mesh_finalize(void)
{
    /* write new version in bl info: */
    bl_info_entry_t new_version_entry;
    memcpy(&new_version_entry.version, m_bl_info_pointers.p_fwid, sizeof(fwid_t));
    bl_info_type_t sign_info_type = BL_INFO_TYPE_INVALID;

    if (m_transaction.p_bank_addr != m_transaction.p_start_addr) /* Dual bank! */
    {
        bl_info_entry_t bank_entry;
        memcpy(&bank_entry.bank.fwid, &m_transaction.target_fwid_union, sizeof(fwid_union_t));
        bank_entry.bank.length = m_transaction.length;
        bank_entry.bank.p_bank_addr = m_transaction.p_bank_addr;
        uint32_t entry_length = BL_INFO_LEN_BANK;
        
        if (m_transaction.signature_length > 0)
        {
            memcpy(bank_entry.bank.signature, m_transaction.signature, BL_INFO_LEN_SIGNATURE);
            entry_length = BL_INFO_LEN_BANK_SIGNED;
        }
        
        switch (m_transaction.type)
        {
            case DFU_TYPE_APP:    
                bootloader_info_entry_put(BL_INFO_TYPE_BANK_APP, &bank_entry, entry_length);
                break;
            case DFU_TYPE_BOOTLOADER:
                bootloader_info_entry_put(BL_INFO_TYPE_BANK_BL, &bank_entry, entry_length);
                break;
            case DFU_TYPE_SD:    
                bootloader_info_entry_put(BL_INFO_TYPE_BANK_SD, &bank_entry, entry_length);
                break;
            default:
                return NRF_ERROR_INVALID_DATA;
        }
        
    }
    else
    {
        if (m_bl_info_pointers.p_flags == NULL)
        {
            APP_ERROR_CHECK(NRF_ERROR_NULL);
        }
        
        /* copy flags, then mark the type we just verified as intact before reflashing it. */
        bl_info_entry_t flags_entry;
        memcpy(&flags_entry, m_bl_info_pointers.p_flags, ((BL_INFO_LEN_FLAGS + 3) & ~0x03UL));

        switch (m_transaction.type)
        {
            case DFU_TYPE_APP:
                memcpy((void*) &new_version_entry.version.app, (void*) &m_transaction.target_fwid_union.app, DFU_FWID_LEN_APP);
                sign_info_type = BL_INFO_TYPE_SIGNATURE_APP;
                flags_entry.flags.app_intact = true;
                break;
            case DFU_TYPE_SD:
                memcpy((void*) &new_version_entry.version.sd, (void*) &m_transaction.target_fwid_union.sd, DFU_FWID_LEN_SD);
                sign_info_type = BL_INFO_TYPE_SIGNATURE_SD;
                flags_entry.flags.sd_intact = true;
                break;
            case DFU_TYPE_BOOTLOADER:
                memcpy((void*) &new_version_entry.version.bootloader, (void*) &m_transaction.target_fwid_union.bootloader, DFU_FWID_LEN_BL);
                sign_info_type = BL_INFO_TYPE_SIGNATURE_BL;
                flags_entry.flags.bl_intact = true;
                break;
            default:
                break;
        }
        m_bl_info_pointers.p_fwid  = &bootloader_info_entry_put(BL_INFO_TYPE_VERSION, &new_version_entry, BL_INFO_LEN_FWID)->version;
        m_bl_info_pointers.p_flags = &bootloader_info_entry_put(BL_INFO_TYPE_FLAGS, &flags_entry, BL_INFO_LEN_FLAGS)->flags;

        /* add signature to bl info, if applicable: */
        if (m_transaction.signature_length != 0)
        {
            bootloader_info_entry_put(sign_info_type, (bl_info_entry_t*) m_transaction.signature, DFU_SIGNATURE_LEN);
        }
    }
    

}

void dfu_mesh_restart(void)
{
    start_find_fwid();
}

uint32_t dfu_mesh_flash_bank(dfu_type_t type, uint32_t* p_bank_addr, uint32_t bank_len)
{
    uint32_t error_code;            
    switch (type)
    {
        case DFU_TYPE_BOOTLOADER:
        {
            /* move the bank with MBR. NRF_UICR->BOOTLOADERADDR must have been set. */
            sd_mbr_command_t sd_mbr_cmd;

            sd_mbr_cmd.command               = SD_MBR_COMMAND_COPY_BL;
            sd_mbr_cmd.params.copy_bl.bl_src = p_bank_addr;
            sd_mbr_cmd.params.copy_bl.bl_len = bank_len / sizeof(uint32_t);
            error_code = sd_mbr_command(&sd_mbr_cmd);
        }
            break;
        case DFU_TYPE_SD:
        {
            /* move the bank with MBR. */
            sd_mbr_command_t sd_mbr_cmd;

            sd_mbr_cmd.command               = SD_MBR_COMMAND_COPY_SD;
            sd_mbr_cmd.params.copy_sd.src    = p_bank_addr;
            sd_mbr_cmd.params.copy_sd.len    = bank_len / sizeof(uint32_t);
            sd_mbr_cmd.params.copy_sd.dst    = (uint32_t*) m_bl_info_pointers.p_segment_sd->start;
            error_code = sd_mbr_command(&sd_mbr_cmd);
        }
            break;
        default:
            /* This nukes the call stack. */
            dfu_transfer_flash_bank();
            
            ///@TODO: Mark the target as intact.
            
            /* Let's just kill ourself! */
            NVIC_SystemReset();        
    }
    return error_code;
}