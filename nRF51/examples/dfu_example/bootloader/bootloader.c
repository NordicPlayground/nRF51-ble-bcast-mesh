#include "bootloader.h"
#include "sha256.h"
#include <string.h>

/* TODO: move to a common header for bl and app */
typedef struct
{
    uint16_t* p_sd_version;
    uint16_t bootloader_version;
    app_id_t* p_app_id;
} fwid_t;

static bl_state_t m_state = BL_STATE_FIND_FWID;
static uint32_t m_transaction_id;
static dfu_type_t m_type;
static fwid_t m_fwid;
static uint32_t* mp_start_addr;
static uint16_t m_segments_remaining;
static uint16_t m_last_seq;
static struct
{
    uint8_t signature[64];
    bool valid[4];
} m_signkeys;

/** log the given target as invalid. */
static void mark_invalid(dfu_type_t fw_type)
{
    //TODO
}

static bool signature_check(void)
{
    uint8_t hash[32];
    dfu_sha256(hash);
    //TODO: check signature
}

static inline uint32_t* addr_from_seg(uint16_t segment)
{
    if (segment == 1)
    {
        return mp_start_addr;
    }
    else
    {
        return (uint32_t*) (((segment - 1) << 4) + ((uint32_t)mp_start_addr & 0xFFFFFFF0));
    }
}

static bool sd_is_newer(uint16_t sd_id)
{
    return (sd_id > *(m_fwid.p_sd_version)); /* TODO: That's not really how this works... */
}

static bool app_is_newer(app_id_t* p_app_id)
{
    return (p_app_id->app_id == m_fwid.p_app_id->app_id &&
            p_app_id->company_id == m_fwid.p_app_id->company_id &&
            p_app_id->app_version > m_fwid.p_app_id->app_version);
}

static bool bootloader_is_newer(uint16_t bl_id)
{
    return (bl_id > m_fwid.bootloader_version);
}

static void start_req(dfu_type_t type)
{
    m_type = type;
    m_state = BL_STATE_DFU_REQ;

    transport_tx(&m_req_beacon, REPEAT_INF);
}

static void start_ready(uint32_t transaction_id, uint32_t mic)
{
    m_transaction_id = transaction_id;
    m_state = BL_STATE_DFU_READY;
    memset(&m_signkeys, 0, sizeof(m_signkeys));
}

static void handle_data_packet(dfu_packet_t* p_packet)
{
    if (p_packet->payload.data.transaction_id == m_transaction_id)
    {
        if (m_state == BL_STATE_DFU_READY)
        {
            if (p_packet->payload.start.seq == 0)
            {
                if (m_state != p_packet->payload.start.dfu_type)
                {
                    break;
                }
                //TODO: add boundary checks to this
                mark_invalid(m_type);
                m_segments_remaining = p_packet->payload.start.segment_count;
                (uint32_t*) p_packet->payload.start.start_address,
                    dfu_start(
                            (uint32_t*) p_packet->payload.start.end_address,
                            p_packet->payload.start.dfu_type,
                            p_packet->payload.start.segment_count,
                            p_packet->payload.start.last);
                m_state = BL_STATE_DFU_TARGET;
            }
            else
            {
                start_req(m_type); /* go back to req, we've missed packet 0 */
            }
        }
        else if (m_state == BL_STATE_DFU_TARGET)
        {
            switch (p_packet->payload.start.seq)
            {
                case 0:
                    /* ignore a misplaced start message */
                    break;
                case DATA_SEQ_END:
                    m_signkeys.valid[p_packet->payload.end.offset >> 2] = true;
                    memcpy(m_signkeys.signature, p_packet->payload.end.signchunk, SIGNCHUNK_LENGTH);
                    break;

                default:
                    m_segments_remaining--;
                    for (uint32_t seq = m_last_seq + 1; seq < p_packet->payload.data.seq; ++i)
                    {
                        seq_lost(seq);
                    }

                    uint32_t* p_addr = addr_from_seg(p_packet->payload.data.segment_index);
                    dfu_data(p_addr,
                            p_packet->payload.data.data, 0x10 - ((uint32_t) p_addr) & 0x0F);
            }
        }
        m_last_seq = p_packet->payload.start.seq;

        if (m_segments_remaining == 0 &&
                seq_finished() && /* redundant? */
                m_signkeys.valid[0] &&
                m_signkeys.valid[1] &&
                m_signkeys.valid[2] &&
                m_signkeys.valid[3] &&
                signature_check())
        {
            dfu_end();
            bootloader_abort(BL_SUCCESS);
        }
    }
}

static void handle_state_packet(dfu_packet_t* p_packet)
{
    if (m_state == BL_STATE_DFU_REQ)
    {
        if (p_packet->payload.state.ready &&
            p_packet->payload.state.dfu_type == m_type)
        {
            uint32_t mic;
            switch (m_type)
            {
                case DFU_TYPE_APP:
                    mic = p_packet->payload.state.params.ready.id.app.MIC;
                    break;
                case DFU_TYPE_SD:
                    mic = p_packet->payload.state.params.ready.id.sd.MIC;
                    break;
                case DFU_TYPE_BOOTLOADER:
                    mic = p_packet->payload.state.params.ready.id.bootloader.MIC;
                    break;
            }
            start_ready(p_packet->payload.state.params.ready.transaction_id, mic);
        }
    }

}

static void handle_fwid_packet(dfu_packet_t* p_packet)
{
    if (m_state == BL_STATE_FIND_FWID)
    {
        if (sd_is_newer(p_packet->payload.fwid.sd_version))
        {
            start_req(DFU_TYPE_SD);
        }
        else if (bootloader_is_newer(p_packet->payload.fwid.bootloader_version))
        {
            start_req(DFU_TYPE_BOOTLOADER);
        }
        else if (app_is_newer(&p_packet->payload.fwid.app_id))
        {
            start_req(DFU_TYPE_APP);
        }
    }
}

void bootloader_init(void)
{
    m_state = BL_STATE_FIND_FWID;
    m_transaction_id = 0;
    m_fwid.p_app_id = (app_id_t*) BOOTLOADER_DATA_POS_APP_ID;
    m_fwid.p_sd_version = (uint16_t*) BOOTLOADER_DATA_POS_SD_ID;

    if (*m_fwid.p_sd_version == SD_VERSION_INVALID)
    {
        start_req(DFU_TYPE_SD);
    }
    else if (m_fwid.p_app_id->app_version == APP_VERSION_INVALID)
    {
        start_req(DFU_TYPE_APP);
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
            break;

        case DFU_PACKET_TYPE_DATA_RSP:
            break;

        default:
            /* don't care */
            break;
    }

}

void bootloader_abort(void)
{

}

