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
#include "dfu_transfer_mesh.h"
#include "dfu_types_mesh.h"
#include "dfu_mesh.h"
#include "nrf.h"
#include "bootloader_app_bridge.h"
#include "sha256.h"
#include "nrf_mbr.h"
#include "rtt_log.h"
#include "dfu_util.h"

/*****************************************************************************
* Local defines
*****************************************************************************/
#define INVALID_SEGMENT_INDEX   (0xFFFF)
#define MISSING_BITFIELD_WIDTH  (64ULL)

/*****************************************************************************
* Local typedefs
*****************************************************************************/

typedef uint64_t bitfield_t;

typedef struct
{
    uint32_t*       p_start_addr;
    uint32_t*       p_bank_addr;
    uint16_t        segment_count;
    bool            final_transfer;
    uint32_t        size;
    uint32_t*       p_write_pointer;
    bitfield_t      missing_segments;
    uint8_t         write_buffer[SEGMENT_LENGTH];
    uint16_t        segment_max;
    uint16_t        segment_prev;
} dfu_transfer_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
static dfu_transfer_t m_transfer;

/*****************************************************************************
* Static functions
*****************************************************************************/

static void transfer_abort(dfu_end_t end_reason)
{
    m_transfer.segment_max = INVALID_SEGMENT_INDEX;
    send_end_evt(end_reason);
}

static bool segment_is_missing(uint16_t segment)
{
    if (segment > m_transfer.segment_max)
    {
        return true;
    }
    if (m_transfer.segment_max - segment > MISSING_BITFIELD_WIDTH)
    {
        return false;
    }
    return !!((1ULL << (m_transfer.segment_max - segment)) & m_transfer.missing_segments);
}

/*****************************************************************************
* Interface functions
*****************************************************************************/
void dfu_transfer_init(void)
{
    memset(&m_transfer, 0, sizeof(dfu_transfer_t));
    m_transfer.segment_max = INVALID_SEGMENT_INDEX;
}

uint32_t dfu_transfer_start(
        uint32_t* p_start_addr,
        uint32_t* p_bank_addr,
        uint32_t size,
        bool final_transfer)
{
    dfu_transfer_init();
    uint16_t segment_count = (((size + (uint32_t) p_start_addr) & 0xFFFFFFF0) - ((uint32_t) p_start_addr & 0xFFFFFFF0)) / 16;

    if (PAGE_OFFSET(p_start_addr) != 0 ||
        PAGE_OFFSET(p_bank_addr) != 0)
    {
        return NRF_ERROR_INVALID_ADDR;
    }

    if (p_bank_addr == NULL)
    {
        m_transfer.p_bank_addr = p_start_addr;
    }
    else
    {
        m_transfer.p_bank_addr = p_bank_addr;
    }

    /* erase all affected pages. */
    flash_erase((uint32_t*) PAGE_ALIGN(m_transfer.p_bank_addr),
                            PAGE_ALIGN(size + PAGE_SIZE - 1));

    m_transfer.p_start_addr = p_start_addr;
    m_transfer.segment_count = segment_count;
    m_transfer.final_transfer = final_transfer;
    m_transfer.p_write_pointer = m_transfer.p_start_addr;
    m_transfer.size = size;
    m_transfer.missing_segments = 0;
    m_transfer.segment_prev = INVALID_SEGMENT_INDEX;
    m_transfer.segment_max = 0;
    return NRF_SUCCESS;
}

uint32_t dfu_transfer_data(uint32_t p_addr, uint8_t* p_data, uint16_t length)
{
    if (m_transfer.segment_max == INVALID_SEGMENT_INDEX)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    /* Data block must be segment-aligned. */
    if ((p_addr & (SEGMENT_LENGTH - 1)) != 0 ||
            p_addr          < (uint32_t) m_transfer.p_start_addr ||
            p_addr + length > (uint32_t) m_transfer.p_start_addr + m_transfer.size)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    if (length > SEGMENT_LENGTH || (length & 0x03))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    if (m_transfer.segment_prev != INVALID_SEGMENT_INDEX)
    {
        return NRF_ERROR_BUSY;
    }

    uint16_t segment = ADDR_SEGMENT(p_addr, m_transfer.p_start_addr);

    if (!segment_is_missing(segment))
    {
        return NRF_ERROR_INVALID_STATE;
    }

    if (segment > m_transfer.segment_max)
    {
        /* Ensure that we don't shift out any set bits: Mask the shift on the
         * MSB side (ie only look at the part that will overflow), and check
         * whether there are any bits set in that section. */
        uint16_t segment_offset = segment - m_transfer.segment_max;
        bitfield_t shift_mask = (1ULL << segment_offset) - 1ULL;
        if (m_transfer.missing_segments & (shift_mask << (MISSING_BITFIELD_WIDTH - segment_offset)))
        {
            transfer_abort(DFU_END_ERROR_PACKET_LOSS);
            return NRF_ERROR_NOT_FOUND;
        }
        /* Offset the bitfield to match the new segment_max, and set all bits
         * that we skipped to 1, as they're considered missing. */
        m_transfer.missing_segments = (m_transfer.missing_segments << segment_offset) | shift_mask;
        m_transfer.missing_segments &= ~1ULL; /* The newest segment isn't missing. */
        m_transfer.segment_max = segment;
    }

    m_transfer.segment_prev = segment;
    memcpy(m_transfer.write_buffer, p_data, length);
    if (flash_write(
            (void*) ((uint32_t) m_transfer.p_bank_addr + (p_addr - (uint32_t) m_transfer.p_start_addr)),
            m_transfer.write_buffer,
            length) != NRF_SUCCESS)
    {
        transfer_abort(DFU_END_ERROR_NO_MEM);
        return NRF_ERROR_INTERNAL;
    }
    return NRF_SUCCESS;
}

bool dfu_transfer_has_entry(uint32_t* p_addr, uint8_t* p_out_buffer, uint16_t len)
{
    if (m_transfer.segment_max == INVALID_SEGMENT_INDEX)
    {
        return false;
    }
    uint16_t segment = ADDR_SEGMENT(p_addr, m_transfer.p_start_addr);
    if (segment > m_transfer.segment_max)
    {
        return false;
    }
    if (!segment_is_missing(segment))
    {
        if (p_out_buffer && len)
        {
            uint32_t* p_storage_addr = (uint32_t*) SEGMENT_ADDR(segment, m_transfer.p_bank_addr);
            memcpy(p_out_buffer, p_storage_addr, len);
        }
        return true;
    }
    return false;
}

bool dfu_transfer_get_oldest_missing_entry(
        uint32_t* p_start_addr,
        uint32_t** pp_entry,
        uint32_t* p_len)
{
    if (m_transfer.segment_max == INVALID_SEGMENT_INDEX)
    {
        return false;
    }
    for (int32_t i = 63; i >= 0; i--)
    {
        if (m_transfer.missing_segments & (1ULL << i))
        {
            uint16_t segment = m_transfer.segment_max - i;
            uint32_t addr = SEGMENT_ADDR(segment, m_transfer.p_start_addr);
            if (addr >= (uint32_t) p_start_addr)
            {
                *pp_entry = (uint32_t*) addr;
                *p_len = SEGMENT_LENGTH;
                return true;
            }
        }
    }
    return false;
}

uint32_t dfu_transfer_sha256(sha256_context_t* p_hash_context)
{
    if (m_transfer.segment_max == INVALID_SEGMENT_INDEX)
    {
        return NRF_ERROR_INVALID_STATE;
    }
    return sha256_update(p_hash_context,
                  (uint8_t*) m_transfer.p_bank_addr,
                  m_transfer.size);
}

void dfu_transfer_end(void)
{
    memset(&m_transfer, 0, sizeof(m_transfer));
    m_transfer.segment_max = INVALID_SEGMENT_INDEX;
}

void dfu_transfer_flash_write_complete(uint8_t* p_write_src)
{
    if (p_write_src == m_transfer.write_buffer)
    {
        uint32_t offset = m_transfer.segment_max - m_transfer.segment_prev;
        m_transfer.missing_segments &= ~(1 << offset);
        m_transfer.segment_prev = INVALID_SEGMENT_INDEX;
    }
}

