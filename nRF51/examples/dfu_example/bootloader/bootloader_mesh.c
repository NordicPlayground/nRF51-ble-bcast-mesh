#include <string.h>
#include "bootloader_mesh.h"
#include "sha256.h"
#include "dfu_mesh.h"
#include "dfu_types_mesh.h"
#include "uECC.h"
#include "transport.h"
#include "bootloader_util.h"
#include "bootloader_rtc.h"
#include "bootloader_info.h"
#include "app_error.h"

#define TX_REPEATS_DEFAULT          (5)
#define TX_REPEATS_FWID             (TX_REPEATS_INF)
#define TX_REPEATS_DFU_REQ          (TX_REPEATS_INF)
#define TX_REPEATS_READY            (TX_REPEATS_INF)
#define TX_REPEATS_DATA             (TX_REPEATS_DEFAULT)
#define TX_REPEATS_RSP              (TX_REPEATS_DEFAULT)
#define TX_REPEATS_REQ              (TX_REPEATS_DEFAULT)

#define TX_INTERVAL_TYPE_FWID       (TX_INTERVAL_TYPE_REGULAR)
#define TX_INTERVAL_TYPE_DFU_REQ    (TX_INTERVAL_TYPE_REGULAR)
#define TX_INTERVAL_TYPE_READY      (TX_INTERVAL_TYPE_REGULAR)
#define TX_INTERVAL_TYPE_DATA       (TX_INTERVAL_TYPE_REGULAR)
#define TX_INTERVAL_TYPE_RSP        (TX_INTERVAL_TYPE_REGULAR)
#define TX_INTERVAL_TYPE_REQ        (TX_INTERVAL_TYPE_REGULAR)

#define STATE_TIMEOUT_FIND_FWID     (US_TO_RTC_TICKS(500000)) /* 0.5s */

typedef enum
{
    BEACON_TYPE_FWID,
    BEACON_TYPE_DFU_REQ_APP,
    BEACON_TYPE_DFU_REQ_SD,
    BEACON_TYPE_DFU_REQ_BL,
    BEACON_TYPE_READY_APP,
    BEACON_TYPE_READY_SD,
    BEACON_TYPE_READY_BL
} beacon_type_t;

typedef struct
{
    uint32_t transaction_id;
    uint8_t authority;
    dfu_type_t type;
    uint32_t* p_start_addr;
    uint16_t segments_remaining;
    uint16_t segment_count;
    id_t target_fwid; /* union */
    uint32_t ready_mic;
} transaction_t;

typedef struct
{
    bl_info_version_t*  p_fwid;
    bl_info_segment_t*  p_segment_sd;
    bl_info_segment_t*  p_segment_bl;
    bl_info_segment_t*  p_segment_app;
    bl_info_flags_t*    p_flags; 
} bl_info_pointers_t;


static transaction_t        m_transaction;
static bl_state_t           m_state = BL_STATE_FIND_FWID;
static bl_info_pointers_t   m_bl_info_pointers;

static mesh_packet_t*       mp_beacon;

static struct
{
    uint8_t signature[64];
    bool valid[4];
} m_signkeys;
static uint32_t m_start_addr;

static bool signature_check(void)
{
    uint8_t hash[32];
    dfu_sha256(hash);
    //TODO: check signature
    return true;
}

static uint32_t upper_addr_limit_get(dfu_type_t type)
{
    switch (type)
    {
        case DFU_TYPE_APP:
            return m_bl_info_pointers.p_segment_app->start + m_bl_info_pointers.p_segment_app->length;
        case DFU_TYPE_SD:
            return m_bl_info_pointers.p_segment_sd->start + m_bl_info_pointers.p_segment_sd->length;
        case DFU_TYPE_BOOTLOADER: 
            return m_bl_info_pointers.p_segment_bl->start + m_bl_info_pointers.p_segment_bl->length;
    }
    return 0;
}

static bool ready_packet_matches_our_req(dfu_packet_t* p_packet)
{
    if (p_packet->payload.state.dfu_type != m_transaction.type)
    {
        return false;
    }
    switch (m_transaction.type)
    {
        case DFU_TYPE_APP:
            return (memcmp(&p_packet->payload.state.params.ready.id.app, 
                &m_transaction.target_fwid.app, sizeof(app_id_t)) == 0);
        case DFU_TYPE_BOOTLOADER:
            return (p_packet->payload.state.params.ready.id.bootloader == 
                    m_transaction.target_fwid.bootloader);
        case DFU_TYPE_SD:
            return (p_packet->payload.state.params.ready.id.sd ==
                    m_transaction.target_fwid.sd);
        default:
            return false;
    }
}

static void packet_set_local_fields(mesh_packet_t* p_packet, uint8_t dfu_packet_len)
{
    mesh_packet_set_local_addr(p_packet);
    p_packet->header.type = BLE_PACKET_TYPE_ADV_NONCONN_IND;
    p_packet->header.length = DFU_PACKET_OVERHEAD + dfu_packet_len;
    ((ble_ad_t*) p_packet->payload)->adv_data_type = MESH_ADV_DATA_TYPE;
    ((ble_ad_t*) p_packet->payload)->data[0] = (MESH_UUID & 0xFF);
    ((ble_ad_t*) p_packet->payload)->data[1] = (MESH_UUID >> 8) & 0xFF;
    ((ble_ad_t*) p_packet->payload)->adv_data_length = DFU_PACKET_ADV_OVERHEAD + dfu_packet_len;
}

static void beacon_set(beacon_type_t type)
{
    if (mp_beacon)
    {
        transport_tx_abort(mp_beacon);
        mesh_packet_ref_count_dec(mp_beacon);
    }
    if (!mesh_packet_acquire(&mp_beacon))
    {
        bootloader_abort(BL_END_ERROR_NO_MEM);
    }
    dfu_packet_t* p_dfu = (dfu_packet_t*) &(((ble_ad_t*) mp_beacon->payload)->data[2]);
    
    switch (type)
    {
        case BEACON_TYPE_FWID:
            packet_set_local_fields(mp_beacon, DFU_PACKET_LEN_FWID);
            p_dfu->packet_type = DFU_PACKET_TYPE_FWID;
            p_dfu->payload.fwid.app.app_id      = m_bl_info_pointers.p_fwid->app.app_id;
            p_dfu->payload.fwid.app.app_version = m_bl_info_pointers.p_fwid->app.app_version;
            p_dfu->payload.fwid.app.company_id  = m_bl_info_pointers.p_fwid->app.company_id;
            p_dfu->payload.fwid.bootloader      = m_bl_info_pointers.p_fwid->bootloader;
            p_dfu->payload.fwid.sd              = m_bl_info_pointers.p_fwid->sd;
            transport_tx(mp_beacon, TX_REPEATS_FWID, TX_INTERVAL_TYPE_FWID);
            break;
        
        case BEACON_TYPE_DFU_REQ_APP:
            packet_set_local_fields(mp_beacon, DFU_PACKET_LEN_REQ_APP);
            p_dfu->packet_type = DFU_PACKET_TYPE_STATE;
            p_dfu->payload.state.dfu_type = DFU_TYPE_APP;
            p_dfu->payload.state.authority = m_transaction.authority;
            memcpy(&p_dfu->payload.state.params.req.id.app, &m_transaction.target_fwid.app, sizeof(app_id_t));
            transport_tx(mp_beacon, TX_REPEATS_REQ, TX_INTERVAL_TYPE_REQ);
            break;
        
        case BEACON_TYPE_DFU_REQ_SD:
            packet_set_local_fields(mp_beacon, DFU_PACKET_LEN_REQ_SD);
            p_dfu->packet_type = DFU_PACKET_TYPE_STATE;
            p_dfu->payload.state.dfu_type = DFU_TYPE_SD;
            p_dfu->payload.state.authority = m_transaction.authority;
            p_dfu->payload.state.params.req.id.sd = m_transaction.target_fwid.sd;
            transport_tx(mp_beacon, TX_REPEATS_REQ, TX_INTERVAL_TYPE_REQ);
            break;
        
        case BEACON_TYPE_DFU_REQ_BL:
            packet_set_local_fields(mp_beacon, DFU_PACKET_LEN_REQ_BL);
            p_dfu->packet_type = DFU_PACKET_TYPE_STATE;
            p_dfu->payload.state.dfu_type = DFU_TYPE_BOOTLOADER;
            p_dfu->payload.state.authority = m_transaction.authority;
            p_dfu->payload.state.params.req.id.bootloader = m_transaction.target_fwid.bootloader;
            transport_tx(mp_beacon, TX_REPEATS_REQ, TX_INTERVAL_TYPE_REQ);
            break;
        
        case BEACON_TYPE_READY_APP:
            packet_set_local_fields(mp_beacon, DFU_PACKET_LEN_READY_APP);
            p_dfu->packet_type = DFU_PACKET_TYPE_STATE;
            p_dfu->payload.state.dfu_type = DFU_TYPE_APP;
            p_dfu->payload.state.authority = m_transaction.authority;
            p_dfu->payload.state.params.ready.transaction_id = m_transaction.transaction_id;
            p_dfu->payload.state.params.ready.MIC = m_transaction.ready_mic;
            memcpy(&p_dfu->payload.state.params.ready.id.app, &m_transaction.target_fwid.app, sizeof(app_id_t));
            transport_tx(mp_beacon, TX_REPEATS_READY, TX_INTERVAL_TYPE_READY);
            break;
        
        case BEACON_TYPE_READY_SD:
            packet_set_local_fields(mp_beacon, DFU_PACKET_LEN_READY_SD);
            p_dfu->packet_type = DFU_PACKET_TYPE_STATE;
            p_dfu->payload.state.dfu_type = DFU_TYPE_SD;
            p_dfu->payload.state.authority = m_transaction.authority;
            p_dfu->payload.state.params.ready.transaction_id = m_transaction.transaction_id;
            p_dfu->payload.state.params.ready.MIC = m_transaction.ready_mic;
            p_dfu->payload.state.params.ready.id.app = m_transaction.target_fwid.app;
            transport_tx(mp_beacon, TX_REPEATS_READY, TX_INTERVAL_TYPE_READY);
            break;
        
        case BEACON_TYPE_READY_BL:
            packet_set_local_fields(mp_beacon, DFU_PACKET_LEN_READY_BL);
            p_dfu->packet_type = DFU_PACKET_TYPE_STATE;
            p_dfu->payload.state.dfu_type = DFU_TYPE_BOOTLOADER;
            p_dfu->payload.state.authority = m_transaction.authority;
            p_dfu->payload.state.params.ready.transaction_id = m_transaction.transaction_id;
            p_dfu->payload.state.params.ready.MIC = m_transaction.ready_mic;
            p_dfu->payload.state.params.ready.id.bootloader = m_transaction.target_fwid.bootloader;
            transport_tx(mp_beacon, TX_REPEATS_READY, TX_INTERVAL_TYPE_READY);
            break;
    }
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

static bool bootloader_is_newer(uint16_t bl_id)
{
    return (bl_id > m_bl_info_pointers.p_fwid->bootloader);
}

static void start_find_fwid(void)
{
    beacon_set(BEACON_TYPE_FWID);
    m_state = BL_STATE_FIND_FWID;
    
    NRF_RTC0->CC[RTC_BL_STATE_CH] = (NRF_RTC0->COUNTER + STATE_TIMEOUT_FIND_FWID) & RTC_MASK;
    NRF_RTC0->INTENSET = (1 << (RTC_BL_STATE_CH + RTC_INTENSET_COMPARE0_Pos));
}

static void start_req(dfu_type_t type)
{
    m_transaction.type = type;
    m_state = BL_STATE_DFU_REQ;

    switch (type)
    {
        case DFU_TYPE_APP:
            beacon_set(BEACON_TYPE_DFU_REQ_APP);
            break;
        case DFU_TYPE_SD:
            beacon_set(BEACON_TYPE_DFU_REQ_SD);
            break;
        case DFU_TYPE_BOOTLOADER:
            beacon_set(BEACON_TYPE_DFU_REQ_BL);
            break;
    }
}

static void start_ready(dfu_packet_t* p_ready_packet)
{
    if (p_ready_packet->packet_type != DFU_PACKET_TYPE_STATE || 
        p_ready_packet->payload.state.authority == 0 || 
        p_ready_packet->payload.state.dfu_type != m_transaction.type)
    {
        APP_ERROR_CHECK(NRF_ERROR_INVALID_PARAM);
    }
    m_transaction.transaction_id = p_ready_packet->payload.state.params.ready.transaction_id;
    m_transaction.authority = p_ready_packet->payload.state.authority;
    m_transaction.ready_mic = p_ready_packet->payload.state.params.ready.MIC;
    m_state = BL_STATE_DFU_READY;
    memset(&m_signkeys, 0, sizeof(m_signkeys));
    
    switch (m_transaction.type)
    {
        case DFU_TYPE_APP:
            beacon_set(BEACON_TYPE_READY_APP);
            break;
        case DFU_TYPE_SD:
            beacon_set(BEACON_TYPE_READY_SD);
            break;
        case DFU_TYPE_BOOTLOADER:
            beacon_set(BEACON_TYPE_READY_BL);
            break;
    }
}

static void handle_data_packet(dfu_packet_t* p_packet)
{
    if (p_packet->payload.data.transaction_id == m_transaction.transaction_id)
    {
        if (m_state == BL_STATE_DFU_READY)
        {
            if (p_packet->payload.start.segment == 0)
            {
                //TODO: add boundary checks to this
                //TODO: Journal changes
                m_transaction.segments_remaining = p_packet->payload.start.segment_count;
                m_transaction.segment_count      = p_packet->payload.start.segment_count;
                m_start_addr = p_packet->payload.start.start_address;
                if ((uint32_t) addr_from_seg(p_packet->payload.start.segment_count) < upper_addr_limit_get(m_transaction.type))
                {
                    dfu_start(
                            (uint32_t*) p_packet->payload.start.start_address,
                            (uint32_t*) p_packet->payload.start.start_address,
                            p_packet->payload.start.segment_count,
                            p_packet->payload.start.last);
                    m_state = BL_STATE_DFU_TARGET;
                }
            }
            else
            {
                start_req(m_transaction.type); /* go back to req, we've missed packet 0 */
            }
        }
        else if (m_state == BL_STATE_DFU_TARGET)
        {
            switch (p_packet->payload.start.segment)
            {
                case 0:
                    /* ignore a repeated start message */
                    break;

                default:
                    m_transaction.segments_remaining--;
                    if (p_packet->payload.data.segment < m_transaction.segment_count)
                    {
                        uint32_t* p_addr = addr_from_seg(p_packet->payload.data.segment);
                        dfu_data((uint32_t) p_addr,
                                p_packet->payload.data.data, 0x10 - ((uint32_t) p_addr) & 0x0F);
                    }
            }
        }

        if (m_transaction.segments_remaining == 0 &&
                m_signkeys.valid[0] &&
                m_signkeys.valid[1] &&
                m_signkeys.valid[2] &&
                m_signkeys.valid[3] &&
                signature_check())
        {
            dfu_end();
            bootloader_abort(BL_END_SUCCESS);
        }
    }
}

static void handle_state_packet(dfu_packet_t* p_packet)
{
    switch (m_state)
    {
        case BL_STATE_DFU_REQ:
            if (p_packet->payload.state.authority > 0 &&
                ready_packet_matches_our_req(p_packet))
            {
                start_ready(p_packet);
            }
            break;
        case BL_STATE_DFU_READY:
            if (ready_packet_matches_our_req(p_packet))
            {
                if (p_packet->payload.state.authority > m_transaction.authority)
                {
                    m_transaction.authority = p_packet->payload.state.authority;
                    m_transaction.transaction_id = p_packet->payload.state.params.ready.transaction_id;
                }
                else if (p_packet->payload.state.authority == m_transaction.authority && 
                         p_packet->payload.state.params.ready.transaction_id > m_transaction.transaction_id)
                {
                    m_transaction.authority = p_packet->payload.state.authority;
                    m_transaction.transaction_id = p_packet->payload.state.params.ready.transaction_id;
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
        if (app_is_newer(&p_packet->payload.fwid.app))
        {
            /* SD shall only be upgraded if a newer version of our app requires a different SD */
            if (p_packet->payload.fwid.sd != m_bl_info_pointers.p_fwid->sd)
            {
                m_transaction.target_fwid.sd = p_packet->payload.fwid.sd;
                NRF_RTC0->INTENCLR = (1 << RTC_BL_STATE_CH);
                start_req(DFU_TYPE_SD);
            }
            else
            {
                memcpy(&m_transaction.target_fwid.app, &p_packet->payload.fwid.app, sizeof(app_id_t));
                NRF_RTC0->INTENCLR = (1 << RTC_BL_STATE_CH);
                start_req(DFU_TYPE_APP);
            }
        }
        
        else if (bootloader_is_newer(p_packet->payload.fwid.bootloader))
        {
            m_transaction.target_fwid.bootloader = p_packet->payload.fwid.bootloader;
            NRF_RTC0->INTENCLR = (1 << RTC_BL_STATE_CH);
            start_req(DFU_TYPE_BOOTLOADER);
        }
        
    }
}

static void handle_data_req_packet(dfu_packet_t* p_packet)
{
    if (p_packet->payload.data.transaction_id == m_transaction.transaction_id)
    {
        mesh_packet_t* p_rsp;
        if (mesh_packet_acquire(&p_rsp))
        {            
            if (
                dfu_has_entry((uint32_t*) SEGMENT_ADDR(p_packet->payload.req_data.segment, m_start_addr), 
                    ((dfu_packet_t*) p_rsp->payload)->payload.rsp_data.data, SEGMENT_LENGTH)
               )
            {
                packet_set_local_fields(p_rsp, DFU_PACKET_LEN_RSP_DATA);
                transport_tx(p_rsp, TX_REPEATS_RSP, TX_INTERVAL_TYPE_RSP); 
            }
            mesh_packet_ref_count_dec(p_rsp);
        }
    }
}

static void handle_data_rsp_packet(dfu_packet_t* p_packet)
{
    if (p_packet->payload.rsp_data.transaction_id == m_transaction.transaction_id)
    {
        dfu_data(SEGMENT_ADDR(p_packet->payload.rsp_data.segment, m_start_addr), 
            p_packet->payload.rsp_data.data, SEGMENT_LENGTH);
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
    
    /* fetch persistent entries */
    m_bl_info_pointers.p_flags          = &bootloader_info_entry_get(BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_FLAGS)->flags;
    m_bl_info_pointers.p_fwid           = &bootloader_info_entry_get(BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_VERSION)->version;
    m_bl_info_pointers.p_segment_app    = &bootloader_info_entry_get(BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_SEGMENT_APP)->segment;
    m_bl_info_pointers.p_segment_bl     = &bootloader_info_entry_get(BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_SEGMENT_BL)->segment;
    m_bl_info_pointers.p_segment_sd     = &bootloader_info_entry_get(BOOTLOADER_INFO_ADDRESS, BL_INFO_TYPE_SEGMENT_SD)->segment;
    
    if (
        ((uint32_t) m_bl_info_pointers.p_flags          < BOOTLOADER_INFO_ADDRESS) ||
        ((uint32_t) m_bl_info_pointers.p_fwid           < BOOTLOADER_INFO_ADDRESS) ||
        ((uint32_t) m_bl_info_pointers.p_segment_app    < BOOTLOADER_INFO_ADDRESS) ||
        ((uint32_t) m_bl_info_pointers.p_segment_sd     < BOOTLOADER_INFO_ADDRESS) ||
        ((uint32_t) m_bl_info_pointers.p_segment_bl     < BOOTLOADER_INFO_ADDRESS)
       )
    {
        bootloader_abort(BL_END_ERROR_INVALID_PERSISTANT_STORAGE);
    }

    if (!m_bl_info_pointers.p_flags->sd_intact || 
        m_bl_info_pointers.p_fwid->sd == SD_VERSION_INVALID)
    {
        start_req(DFU_TYPE_SD);
    }
    else if (!m_bl_info_pointers.p_flags->app_intact ||
        m_bl_info_pointers.p_fwid->app.app_version == APP_VERSION_INVALID)
    {
        start_req(DFU_TYPE_APP);
    }
    else
    {
        start_find_fwid();
    }
}

void bootloader_rx(dfu_packet_t* p_packet)
{
    switch (p_packet->packet_type)
    {
        case DFU_PACKET_TYPE_FWID:
            handle_fwid_packet(p_packet);
            break;

        case DFU_PACKET_TYPE_STATE:
            handle_state_packet(p_packet);
            break;

        case DFU_PACKET_TYPE_DATA:
            handle_data_packet(p_packet);
            break;

        case DFU_PACKET_TYPE_DATA_REQ:
            handle_data_req_packet(p_packet);
            break;

        case DFU_PACKET_TYPE_DATA_RSP:
            handle_data_rsp_packet(p_packet);
            break;

        default:
            /* don't care */
            break;
    }
}

void bootloader_abort(bl_end_t end_reason)
{
    bootloader_util_app_start(m_bl_info_pointers.p_segment_app->start);
}

void bootloader_rtc_irq_handler(void)
{
    NRF_RTC0->INTENCLR = (1 << RTC_BL_STATE_CH);
    switch (m_state)
    {
        case BL_STATE_FIND_FWID:
            bootloader_abort(BL_END_FWID_VALID);
            break;
        default:
            break;
    }
}
