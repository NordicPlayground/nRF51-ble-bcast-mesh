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
#include "bootloader_mesh.h"
#include "sha256.h"
#include "dfu_mesh.h"
#include "dfu_types_mesh.h"
#include "mesh_packet.h"
#include "uECC.h"
#include "transport.h"
#include "bootloader_util.h"
#include "bootloader_rtc.h"
#include "bootloader_info.h"
#include "app_error.h"
#include "serial_handler.h"
#include "serial_evt.h"
#include "nrf_flash.h"
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

#define TX_INTERVAL_TYPE_FWID       (TX_INTERVAL_TYPE_REGULAR)
#define TX_INTERVAL_TYPE_DFU_REQ    (TX_INTERVAL_TYPE_REGULAR)
#define TX_INTERVAL_TYPE_READY      (TX_INTERVAL_TYPE_REGULAR)
#define TX_INTERVAL_TYPE_DATA       (TX_INTERVAL_TYPE_EXPONENTIAL)
#define TX_INTERVAL_TYPE_RSP        (TX_INTERVAL_TYPE_EXPONENTIAL)
#define TX_INTERVAL_TYPE_REQ        (TX_INTERVAL_TYPE_REGULAR)

#define STATE_TIMEOUT_FIND_FWID     (US_TO_RTC_TICKS( 5000000)) /* 2.0s */
#define STATE_TIMEOUT_REQ           (US_TO_RTC_TICKS( 1000000)) /* 1.0s */
#define STATE_TIMEOUT_READY         (US_TO_RTC_TICKS( 3000000)) /* 3.0s */
#define STATE_TIMEOUT_TARGET        (US_TO_RTC_TICKS( 5000000)) /* 5.0s */
#define STATE_TIMEOUT_RAMPDOWN      (US_TO_RTC_TICKS( 1000000)) /* 1.0s */
#define STATE_TIMEOUT_RELAY         (US_TO_RTC_TICKS(10000000)) /* 8.0s */

#define IRQ_ENABLED            0x01     /**< Field that identifies if an interrupt is enabled. */
#define MAX_NUMBER_INTERRUPTS  32       /**< Maximum number of interrupts available. */

#define START_ADDRESS_UNKNOWN       (0xFFFFFFFF)

#define DATA_CACHE_SIZE             (16)
/* important that req-cache isn't too big - might lead to starvation in req-device */
#define REQ_CACHE_SIZE              (4)
#define TRANSACTION_ID_CACHE_SIZE   (8)
#define REQ_RX_COUNT_RETRY          (8)

#define SENT_PACKET_COUNT           (32)

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
    uint32_t*       p_start_addr;
    uint32_t*       p_bank_addr;
    uint32_t*       p_indicated_start_addr;
    uint32_t*       p_last_requested_entry;
    uint32_t        length;
    uint32_t        signature_length;
    uint8_t         signature[DFU_SIGNATURE_LEN];
    uint8_t         signature_bitmap;
    uint16_t        segments_remaining;
    uint16_t        segment_count;
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

/*****************************************************************************
* Static globals
*****************************************************************************/
static transaction_t        m_transaction;
static bl_state_t           m_state = BL_STATE_FIND_FWID;
static bl_info_pointers_t   m_bl_info_pointers;
static req_cache_entry_t    m_req_cache[REQ_CACHE_SIZE];
static uint16_t             m_data_cache[DATA_CACHE_SIZE];
static uint8_t              m_req_index;
static uint8_t              m_data_index;
static uint32_t             m_tid_cache[TRANSACTION_ID_CACHE_SIZE];
static uint8_t              m_tid_index;
static volatile uint32_t    m_key_len = uECC_BYTES * 2;
static mesh_packet_t*       mp_sent_packets[SENT_PACKET_COUNT];
static uint32_t             m_sent_packet_index;
static mesh_packet_t*       mp_beacon;

/*****************************************************************************
* Static Functions
*****************************************************************************/
static bool fw_is_verified(void)
{
    if (m_bl_info_pointers.p_flags)
    {
        return (m_bl_info_pointers.p_flags->sd_intact &&
                m_bl_info_pointers.p_flags->app_intact &&
                m_bl_info_pointers.p_flags->bl_intact);
    }

    return true;
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

static void packet_release_callback(mesh_packet_t* p_packet)
{
    for (uint32_t i = 0; i < SENT_PACKET_COUNT; ++i)
    {
        if (p_packet == mp_sent_packets[i])
        {
            mp_sent_packets[i] = NULL;
            break;
        }
    }
}

static void interrupts_disable(void)
{
    uint32_t interrupt_setting_mask;
    uint32_t irq;

    /* Fetch the current interrupt settings. */
    interrupt_setting_mask = NVIC->ISER[0];

    /* Loop from interrupt 0 for disabling of all interrupts. */
    for (irq = 0; irq < MAX_NUMBER_INTERRUPTS; irq++)
    {
        if (interrupt_setting_mask & (IRQ_ENABLED << irq))
        {
            /* The interrupt was enabled, hence disable it. */
            NVIC_DisableIRQ((IRQn_Type)irq);
        }
    }
}

static bool app_is_valid(uint32_t* app_start, uint32_t app_length)
{
    return (m_bl_info_pointers.p_segment_app != NULL &&
            *((uint32_t*) m_bl_info_pointers.p_segment_app->start) != 0xFFFFFFFF &&
            m_bl_info_pointers.p_fwid->app.app_version != APP_VERSION_INVALID &&
            fw_is_verified());
}

static void serial_tx(dfu_packet_t* p_packet, uint16_t len)
{
#ifdef SERIAL
    serial_evt_t evt;
    if (len > 29)
    {
        APP_ERROR_CHECK(NRF_ERROR_INVALID_LENGTH);
    }
    evt.opcode = SERIAL_EVT_OPCODE_DFU;
    evt.length = SERIAL_PACKET_OVERHEAD + len;
    memcpy(&evt.params.dfu.packet, p_packet, len);
    serial_handler_event_send(&evt);
#endif
}

static void set_timeout(uint32_t time)
{
#ifndef NO_TIMEOUTS    
    NRF_RTC0->EVENTS_COMPARE[RTC_BL_STATE_CH] = 0;
    NRF_RTC0->CC[RTC_BL_STATE_CH] = (NRF_RTC0->COUNTER + time) & RTC_MASK;
    NRF_RTC0->INTENSET = (1 << (RTC_BL_STATE_CH + RTC_INTENSET_COMPARE0_Pos));
#endif    
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

    dfu_sha256(&hash_context);

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

static dfu_packet_t* beacon_set(beacon_type_t type)
{
    if (mp_beacon)
    {
        transport_tx_abort(mp_beacon);
        if (!mesh_packet_ref_count_dec(mp_beacon))
        {
#ifdef DEBUG_LEDS            
            NRF_GPIO->OUTCLR = (1 << 24);
#endif
        }
    }
    if (!mesh_packet_acquire(&mp_beacon))
    {
        bootloader_abort(BL_END_ERROR_NO_MEM);
    }
    dfu_packet_t* p_dfu = (dfu_packet_t*) &(((ble_ad_t*) mp_beacon->payload)->data[2]);
    memset(p_dfu, 0, sizeof(dfu_packet_t));
    
    uint8_t repeats = TX_REPEATS_FWID;
    tx_interval_type_t interval_type = TX_INTERVAL_TYPE_FWID;
    
    if (type >= BEACON_TYPE_READY_APP &&
        type <= BEACON_TYPE_READY_BL)
    {
        p_dfu->packet_type = DFU_PACKET_TYPE_STATE;
        p_dfu->payload.state.authority = m_transaction.authority;
        p_dfu->payload.state.flood = m_transaction.flood;
        p_dfu->payload.state.transaction_id = m_transaction.transaction_id;
        p_dfu->payload.state.relay_node = (m_state == BL_STATE_RELAY_CANDIDATE);
        p_dfu->payload.state.fwid = m_transaction.target_fwid_union;
        repeats = TX_REPEATS_READY;
        interval_type = TX_INTERVAL_TYPE_READY;
    }

    switch (type)
    {
        case BEACON_TYPE_FWID:
            bootloader_packet_set_local_fields(mp_beacon, DFU_PACKET_LEN_FWID);
            p_dfu->packet_type = DFU_PACKET_TYPE_FWID;
            p_dfu->payload.fwid = *m_bl_info_pointers.p_fwid;
            break;

        case BEACON_TYPE_READY_APP:
            bootloader_packet_set_local_fields(mp_beacon, DFU_PACKET_LEN_STATE_APP);
            p_dfu->payload.state.dfu_type = DFU_TYPE_APP;
            break;

        case BEACON_TYPE_READY_SD:
            bootloader_packet_set_local_fields(mp_beacon, DFU_PACKET_LEN_STATE_SD);
            p_dfu->payload.state.dfu_type = DFU_TYPE_SD;
            break;

        case BEACON_TYPE_READY_BL:
            bootloader_packet_set_local_fields(mp_beacon, DFU_PACKET_LEN_STATE_BL);
            p_dfu->payload.state.dfu_type = DFU_TYPE_BOOTLOADER;
            break;
    }
    transport_tx(mp_beacon, repeats, interval_type, NULL);

    return p_dfu;
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

static void update_state_beacon(void)
{
    dfu_packet_t* p_beacon = NULL;
    uint16_t beacon_len = 0;
    switch (m_transaction.type)
    {
        case DFU_TYPE_APP:
            p_beacon = beacon_set(BEACON_TYPE_READY_APP);
            beacon_len = DFU_PACKET_LEN_STATE_APP;
            break;

        case DFU_TYPE_SD:
            p_beacon = beacon_set(BEACON_TYPE_READY_SD);
            beacon_len = DFU_PACKET_LEN_STATE_SD;
            break;

        case DFU_TYPE_BOOTLOADER:
            p_beacon = beacon_set(BEACON_TYPE_READY_BL);
            beacon_len = DFU_PACKET_LEN_STATE_BL;
            break;
        default:
            APP_ERROR_CHECK(NRF_ERROR_NOT_SUPPORTED);
    }

    if (p_beacon)
    {
        serial_tx(p_beacon, beacon_len);
    }
}

/********** STATE MACHINE ENTRY POINTS ***********/
static void start_find_fwid(void)
{
    dfu_packet_t* p_fwid = beacon_set(BEACON_TYPE_FWID);
    set_timeout(STATE_TIMEOUT_FIND_FWID);
    m_state = BL_STATE_FIND_FWID;
    memset(&m_transaction, 0, sizeof(transaction_t));

    serial_tx(p_fwid, DFU_PACKET_LEN_FWID);
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
    if (timeout)
    {
        set_timeout(STATE_TIMEOUT_REQ);
    }
    m_state = BL_STATE_DFU_REQ;

    update_state_beacon();
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
    set_timeout(STATE_TIMEOUT_READY);
    m_state = BL_STATE_DFU_READY;

    update_state_beacon();
}

static void start_target(void)
{
    set_timeout(STATE_TIMEOUT_TARGET);
    m_state = BL_STATE_DFU_TARGET;

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
            nrf_flash_store((uint32_t*) m_bl_info_pointers.p_flags, (uint8_t*) &flags_entry, (BL_INFO_LEN_FLAGS + 3) & ~0x03UL, 0);
        }
    }

    if (dfu_start(
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
        transport_tx_abort(mp_beacon); /* stop beaconing */
    }
}

static void start_rampdown(void)
{
    /* the final timeout should be setup regardless of NO_TIMEOUTS flag. */
    NRF_RTC0->EVENTS_COMPARE[RTC_BL_STATE_CH] = 0;
    NRF_RTC0->CC[RTC_BL_STATE_CH] = (NRF_RTC0->COUNTER + STATE_TIMEOUT_RAMPDOWN) & RTC_MASK;
    NRF_RTC0->INTENSET = (1 << (RTC_BL_STATE_CH + RTC_INTENSET_COMPARE0_Pos));
    
    m_state = BL_STATE_VALIDATE;
}

static void start_relay_candidate(dfu_packet_t* p_packet)
{
    set_timeout(STATE_TIMEOUT_RELAY);
    
    m_transaction.authority = p_packet->payload.state.authority;
    m_transaction.transaction_id = p_packet->payload.state.transaction_id;
    m_transaction.target_fwid_union = p_packet->payload.state.fwid;
    m_state = BL_STATE_RELAY_CANDIDATE;
    
    update_state_beacon();
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

    mesh_packet_set_local_addr(p_mesh_packet);

    if (transport_tx(p_mesh_packet, TX_REPEATS_DATA, TX_INTERVAL_TYPE_DATA, packet_release_callback))
    {
        mp_sent_packets[(m_sent_packet_index++) & (SENT_PACKET_COUNT - 1)] = p_mesh_packet;
    }
    mesh_packet_ref_count_dec(p_mesh_packet);
}

static mesh_packet_t* packet_cache_entry_get(dfu_packet_t* p_packet)
{ 
    NVIC_DisableIRQ(RTC0_IRQn);
    mesh_packet_t* p_cache_packet = NULL;
    for (uint32_t i = 0; i < SENT_PACKET_COUNT; ++i)
    {
        if (mp_sent_packets[i] && 
            ((mesh_dfu_adv_data_t*) mp_sent_packets[i]->payload)->dfu_packet.packet_type == p_packet->packet_type &&
            ((mesh_dfu_adv_data_t*) mp_sent_packets[i]->payload)->dfu_packet.payload.data.segment == 
            p_packet->payload.data.segment)
        {
            p_cache_packet = mp_sent_packets[i];
            break;
        }
    }
    NVIC_EnableIRQ(RTC0_IRQn);   
    return p_cache_packet;
}

static bool data_packet_in_cache(dfu_packet_t* p_packet)
{
    for (uint32_t i = 0; i < DATA_CACHE_SIZE; ++i)
    {
        if (m_data_cache[i] == p_packet->payload.data.segment)
        {
            return true;
        }
    }
    return false;
}

static void handle_data_packet(dfu_packet_t* p_packet, uint16_t length)
{
    mesh_packet_t* p_cache_packet = packet_cache_entry_get(p_packet);
    if (p_cache_packet)
    {
        transport_tx_skip(p_cache_packet);
    }
    
    bool do_relay = false;
    if (p_packet->payload.data.transaction_id == m_transaction.transaction_id)
    {
        /* check and add to cache */
        if (data_packet_in_cache(p_packet))
        {
            return;
        }
        m_data_cache[(m_data_index++) & (DATA_CACHE_SIZE - 1)] = p_packet->payload.data.segment;
        
        
        if (m_state == BL_STATE_DFU_READY)
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
        else if (m_state == BL_STATE_DFU_TARGET)
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
                    error_code  = dfu_data((uint32_t) p_addr,
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
                    set_timeout(STATE_TIMEOUT_TARGET);
                    m_transaction.segments_remaining--;
                    do_relay = true;
                    /* check whether we've lost any entries, and request them */
                    uint32_t* p_req_entry = NULL;
                    uint32_t req_entry_len = 0;
                    mesh_packet_t* p_req_packet;
                    
                    if (dfu_get_oldest_missing_entry(
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
                        if(!mesh_packet_acquire(&p_req_packet))
                        {
                            return;
                        }
                        if (mesh_packet_build(p_req_packet,
                                DFU_PACKET_TYPE_DATA_REQ,
                                ADDR_SEGMENT(p_req_entry, m_transaction.p_start_addr),
                                (uint8_t*) &m_transaction.transaction_id,
                                4) == NRF_SUCCESS &&
                            transport_tx(p_req_packet, TX_REPEATS_REQ, TX_INTERVAL_TYPE_REQ, NULL))
                        {
                            m_transaction.p_last_requested_entry = (uint32_t*) p_req_entry;
                        }
                        mesh_packet_ref_count_dec(p_req_packet);
                    }
                }
            }

            /* ending the DFU */
            if (m_transaction.segments_remaining == 0)
            {
                dfu_end();
                start_rampdown();
            }
        }
        else if (m_state == BL_STATE_RELAY_CANDIDATE || 
                 m_state == BL_STATE_RELAY)
        {
            m_state = BL_STATE_RELAY;
            transport_tx_abort(mp_beacon);
            set_timeout(STATE_TIMEOUT_RELAY);
            do_relay = true;
        }
    }

    if (do_relay)
    {
        relay_packet(p_packet, length);
    }
}

static void handle_state_packet(dfu_packet_t* p_packet)
{
    switch (m_state)
    {
        case BL_STATE_FIND_FWID:
            if (p_packet->payload.state.authority > 0)
            {
                m_transaction.type = (dfu_type_t) p_packet->payload.state.dfu_type;
                m_transaction.target_fwid_union = p_packet->payload.state.fwid;
                
                if (ready_packet_is_upgrade(p_packet))
                {
                    start_ready(p_packet);
                }
                else
                {
                    start_relay_candidate(p_packet);
                }
            }
            break;
        case BL_STATE_DFU_REQ:
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
        case BL_STATE_DFU_READY:
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
                    update_state_beacon();
                }
            }
            break;
        case BL_STATE_RELAY_CANDIDATE:
            if (m_transaction.type == p_packet->payload.state.dfu_type &&
                fwid_union_cmp(&m_transaction.target_fwid_union, &p_packet->payload.state.fwid, m_transaction.type))
            {
                if (m_transaction.authority < p_packet->payload.state.authority ||
                    (m_transaction.authority      == p_packet->payload.state.authority &&
                     m_transaction.transaction_id <  p_packet->payload.state.transaction_id))
                {
                    m_transaction.transaction_id = p_packet->payload.state.transaction_id;
                    m_transaction.authority      = p_packet->payload.state.authority;
                    update_state_beacon();
                }
            }
            break;
        default:
            break;
    }

}

static void handle_fwid_packet(dfu_packet_t* p_packet)
{
    if (m_state == BL_STATE_FIND_FWID)
    {
        /* always upgrade bootloader first */
        if (bootloader_is_newer(p_packet->payload.fwid.bootloader))
        {
            NRF_RTC0->INTENCLR = (1 << (RTC_BL_STATE_CH + RTC_INTENCLR_COMPARE0_Pos));
            m_transaction.target_fwid_union.bootloader = p_packet->payload.fwid.bootloader;
            start_req(DFU_TYPE_BOOTLOADER, true);
        }
        else if (app_is_newer(&p_packet->payload.fwid.app))
        {
            NRF_RTC0->INTENCLR = (1 << (RTC_BL_STATE_CH + RTC_INTENCLR_COMPARE0_Pos));
            /* SD shall only be upgraded if a newer version of our app requires a different SD */
            if (p_packet->payload.fwid.sd != 0xFFFE && p_packet->payload.fwid.sd != m_bl_info_pointers.p_fwid->sd)
            {
                m_transaction.target_fwid_union.sd = p_packet->payload.fwid.sd;
                start_req(DFU_TYPE_SD, true);
            }
            else
            {
                memcpy(&m_transaction.target_fwid_union.app, &p_packet->payload.fwid.app, sizeof(app_id_t));
                start_req(DFU_TYPE_APP, true);
            }
        }
    }
}

static void handle_data_req_packet(dfu_packet_t* p_packet)
{
    if (p_packet->payload.data.transaction_id == m_transaction.transaction_id)
    {
        if (m_state == BL_STATE_RELAY)
        {
            /* only relay new packets, look for it in cache */
            if (packet_cache_entry_get(p_packet) == NULL)
            {
                set_timeout(STATE_TIMEOUT_RELAY);
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
            mesh_packet_t* p_rsp;
            if (mesh_packet_acquire(&p_rsp))
            {
                dfu_packet_t* p_dfu_rsp = (dfu_packet_t*) &((ble_ad_t*) p_rsp->payload)->data[2];
                /* serve request */
                if (
                    dfu_has_entry(
                        (uint32_t*) SEGMENT_ADDR(p_packet->payload.req_data.segment, m_transaction.p_start_addr),
                        p_dfu_rsp->payload.rsp_data.data, SEGMENT_LENGTH)
                   )
                {
                    p_dfu_rsp->packet_type = DFU_PACKET_TYPE_DATA_RSP;
                    p_dfu_rsp->payload.rsp_data.segment = p_packet->payload.req_data.segment;
                    p_dfu_rsp->payload.rsp_data.transaction_id = p_packet->payload.req_data.transaction_id;
                    
                    bootloader_packet_set_local_fields(p_rsp, DFU_PACKET_LEN_DATA_RSP);
                    transport_tx(p_rsp, TX_REPEATS_RSP, TX_INTERVAL_TYPE_RSP, NULL);
                    serial_tx(p_dfu_rsp, DFU_PACKET_LEN_DATA_RSP);
                }
                mesh_packet_ref_count_dec(p_rsp);

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
}

static void handle_data_rsp_packet(dfu_packet_t* p_packet, uint16_t length)
{
    if (m_state == BL_STATE_RELAY)
    {
        /* only relay new packets, look for it in cache */
        if (packet_cache_entry_get(p_packet) == NULL)
        {
            set_timeout(STATE_TIMEOUT_RELAY);
            relay_packet(p_packet, length);
        }
    }
    else
    {
        handle_data_packet(p_packet, length);
    }
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/
void bootloader_init(void)
{
    mp_beacon = NULL;
    m_state = BL_STATE_FIND_FWID;
    m_transaction.transaction_id = 0;
    m_transaction.type = DFU_TYPE_NONE;
    memset(m_data_cache, 0xFF, DATA_CACHE_SIZE * sizeof(uint16_t));
    memset(m_req_cache, 0, REQ_CACHE_SIZE * sizeof(m_req_cache[0]));
    memset(m_tid_cache, 0, TRANSACTION_ID_CACHE_SIZE);
    m_tid_index = 0;
    m_data_index = 0;
    m_req_index = 0;

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
        bootloader_abort(BL_END_ERROR_INVALID_PERSISTENT_STORAGE);
    }
}

void bootloader_start(void)
{
    if (!m_bl_info_pointers.p_flags->sd_intact ||
         m_bl_info_pointers.p_fwid->sd == SD_VERSION_INVALID)
    {
        m_transaction.target_fwid_union.sd = 0;
        start_req(DFU_TYPE_SD, false);
    }
    else if (!app_is_valid((uint32_t*) m_bl_info_pointers.p_segment_app->start,
                           m_bl_info_pointers.p_segment_app->length))
    {
        memcpy(&m_transaction.target_fwid_union.app, &m_bl_info_pointers.p_fwid->app, sizeof(app_id_t));
        start_req(DFU_TYPE_APP, false);
    }
    else
    {
        start_find_fwid();
    }
    
}

uint32_t bootloader_rx(dfu_packet_t* p_packet, uint16_t length, bool from_serial)
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

void bootloader_abort(bl_end_t end_reason)
{
    uint32_t app_length = m_bl_info_pointers.p_segment_app->length;
    if (m_transaction.transaction_id != 0)
    {
        app_length = m_transaction.length;
    }
    switch (end_reason)
    {
        case BL_END_SUCCESS:
        case BL_END_ERROR_TIMEOUT:
        case BL_END_FWID_VALID:
            if (app_is_valid((uint32_t*) m_bl_info_pointers.p_segment_app->start, app_length))
            {
                interrupts_disable();
#ifdef DEBUG_LEDS                
                NRF_GPIO->OUTCLR = (1 << 22);
#endif                
                
                sd_mbr_command_t com = {SD_MBR_COMMAND_INIT_SD, };
        
                volatile uint32_t err_code = sd_mbr_command(&com);
                APP_ERROR_CHECK(err_code);
                
                err_code = sd_softdevice_vector_table_base_set(m_bl_info_pointers.p_segment_app->start);
                APP_ERROR_CHECK(err_code);
#ifdef DEBUG_LEDS
                NRF_GPIO->OUTSET = (1 << 21);
                NRF_GPIO->OUTSET = (1 << 22);
#endif
                bootloader_util_app_start(m_bl_info_pointers.p_segment_app->start);
            }
            break;
        case BL_END_ERROR_INVALID_PERSISTENT_STORAGE:
            APP_ERROR_CHECK_BOOL(false);
        default:
            NVIC_SystemReset();
            break;
    }
}

void bootloader_rtc_irq_handler(void)
{
    NRF_RTC0->INTENCLR = (1 << (RTC_BL_STATE_CH + RTC_INTENCLR_COMPARE0_Pos));
    switch (m_state)
    {
        case BL_STATE_FIND_FWID:
            bootloader_abort(BL_END_FWID_VALID);
            break;

        case BL_STATE_DFU_REQ:
        case BL_STATE_DFU_READY:
            bootloader_abort(BL_END_ERROR_NO_START);
            break;

        case BL_STATE_DFU_TARGET:
            start_req(m_transaction.type, true);
            break;

        case BL_STATE_VALIDATE:
            if (signature_check())
            {
                /* Don't want any interrupts disturbing this final stage */
                uint32_t was_masked;
                _DISABLE_IRQS(was_masked);

                /* write new version in bl info: */
                bl_info_entry_t new_version_entry;
                memcpy(&new_version_entry.version, m_bl_info_pointers.p_fwid, sizeof(fwid_t));
                bl_info_type_t sign_info_type = BL_INFO_TYPE_INVALID;

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
                m_bl_info_pointers.p_fwid = &bootloader_info_entry_put(BL_INFO_TYPE_VERSION, &new_version_entry, BL_INFO_LEN_FWID)->version;
                m_bl_info_pointers.p_flags = &bootloader_info_entry_put(BL_INFO_TYPE_FLAGS, &flags_entry, BL_INFO_LEN_FLAGS)->flags;

                /* add signature to bl info, if applicable: */
                if (m_transaction.signature_length != 0)
                {
                    bootloader_info_entry_put(sign_info_type, (bl_info_entry_t*) m_transaction.signature, DFU_SIGNATURE_LEN);
                }

                _ENABLE_IRQS(was_masked);

                bootloader_abort(BL_END_SUCCESS);
            }
            else
            {
                /* someone gave us anauthorized firmware, and we're broken.
                   need to reboot and try to request a new transfer */
                bootloader_abort(BL_END_ERROR_UNAUTHORIZED);
            }
            break;
        case BL_STATE_RELAY:
        case BL_STATE_RELAY_CANDIDATE:
            bootloader_abort(BL_END_SUCCESS);
            break;
        default:
            break;
    }
}

void bootloader_packet_set_local_fields(mesh_packet_t* p_packet, uint8_t dfu_packet_len)
{
    mesh_packet_set_local_addr(p_packet);
    p_packet->header.type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
    p_packet->header.length = DFU_PACKET_OVERHEAD + dfu_packet_len;
    ((ble_ad_t*) p_packet->payload)->adv_data_type = MESH_ADV_DATA_TYPE;
    ((ble_ad_t*) p_packet->payload)->data[0] = (MESH_UUID & 0xFF);
    ((ble_ad_t*) p_packet->payload)->data[1] = (MESH_UUID >> 8) & 0xFF;
    ((ble_ad_t*) p_packet->payload)->adv_data_length = DFU_PACKET_ADV_OVERHEAD + dfu_packet_len;
}
