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
#include <stdio.h>
#include "dfu_transfer_mesh.h"
#include "dfu_types_mesh.h"
#include "dfu_mesh.h"
#include "nrf51.h"
#include "bootloader_app_bridge.h"
#include "sha256.h"
#include "nrf_mbr.h"
#include "rtt_log.h"
#include "dfu_util.h"

#ifndef MISSING_ENTRY_BACKLOG_COUNT
#define MISSING_ENTRY_BACKLOG_COUNT (8)
#endif

#define WRITE_BUFFER_SIZE   (64) /* must be a multiple of 16 */
#define WRITE_BUFFER_ALIGN(p_addr) (((uint32_t) p_addr) & ~(WRITE_BUFFER_SIZE - 1))
#define WRITE_BUFFER_OFFSET(p_addr) (((uint32_t) p_addr) & (WRITE_BUFFER_SIZE - 1))

/*****************************************************************************
* Local typedefs
*****************************************************************************/
typedef struct
{
    uint32_t addr;
    uint16_t length;
} dfu_entry_t;

typedef struct
{
    uint32_t*       p_start_addr;
    uint32_t*       p_bank_addr;
    uint16_t        segment_count;
    bool            final_transfer;
    uint32_t        size;
    uint32_t*       p_write_pointer;
    uint32_t        first_invalid_byte_in_write_buffer;
    dfu_entry_t     p_missing_entry_backlog[MISSING_ENTRY_BACKLOG_COUNT];
    uint32_t        missing_entry_count;
} dfu_transfer_t;

/*****************************************************************************
* Static globals
*****************************************************************************/
static dfu_transfer_t   m_current_transfer;
static uint8_t          mp_write_buffer[WRITE_BUFFER_SIZE];
static uint32_t         m_write_buffer_busy;

/*****************************************************************************
* Static Functions
*****************************************************************************/
static dfu_entry_t* entry_in_missing_backlog(uint32_t p_start, uint16_t length);
static void entry_mark_as_missing(uint32_t p_start, uint16_t length);

static void entry_mark_as_not_missing(uint32_t p_start, uint16_t length)
{
    uint32_t found_entries = 0;
    for (uint32_t i = 0; i < MISSING_ENTRY_BACKLOG_COUNT; ++i)
    {
        dfu_entry_t* p_entry = &m_current_transfer.p_missing_entry_backlog[i];

        if (p_entry->addr != 0)
        {
            found_entries++;
            /* existing entry starts at same point as new */
            if (p_entry->addr == p_start)
            {
                if (length >= p_entry->length)
                {
                    p_entry->addr = 0;
                    p_entry->length = 0;
                    m_current_transfer.missing_entry_count--;
                }
                else
                {
                    p_entry->length -= length;
                    p_entry->addr = p_start + length;
                }
                return;
            }
            else if (p_entry->addr + p_entry->length == p_start + length)
            {
                /* found at end of a larger entry */
                p_entry->length = p_start - p_entry->addr;
                return;
            }
            else if (p_start > p_entry->addr &&
                     p_start < p_entry->addr + p_entry->length)
            {
                /* found in the middle of entry */
                uint32_t old_end = p_entry->addr + p_entry->length;
                p_entry->length = p_start - p_entry->addr;
                entry_mark_as_missing(p_start + length,
                        old_end - (p_start + length));
                return;
            }
        }
    }
}

static void entry_mark_as_missing(uint32_t p_start, uint16_t length)
{
    /* first, check for entries which can be merged with this */
    dfu_entry_t* p_existing_entry = entry_in_missing_backlog(p_start - 1, length + 1 + 1);
    if (p_existing_entry)
    {
        if (p_existing_entry->addr <= p_start)
        {
            if (p_existing_entry->length < p_start + length - p_existing_entry->addr)
            {
                p_existing_entry->length = p_start + length - p_existing_entry->addr;
            }
        }
        else
        {
            p_existing_entry->length += p_existing_entry->addr - p_start;
            p_existing_entry->addr = p_start;
        }
        return;
    }

    /* create new entry */
    for (uint32_t i = 0; i < MISSING_ENTRY_BACKLOG_COUNT; ++i)
    {
        if (m_current_transfer.p_missing_entry_backlog[i].addr == 0)
        {
            m_current_transfer.p_missing_entry_backlog[i].addr = p_start;
            m_current_transfer.p_missing_entry_backlog[i].length = length;
            m_current_transfer.missing_entry_count++;
            return;
        }
    }
    /* backlog is full, abort. */
    send_end_evt(DFU_END_ERROR_PACKET_LOSS);
}

static dfu_entry_t* entry_in_missing_backlog(uint32_t p_start, uint16_t length)
{
    for (uint32_t i = 0; i < MISSING_ENTRY_BACKLOG_COUNT; ++i)
    {
        dfu_entry_t* p_entry = &m_current_transfer.p_missing_entry_backlog[i];

        if (p_entry->addr != 0)
        {
            /* overlap: */
            if (
                 (
                  p_entry->addr >= p_start &&
                  p_entry->addr < p_start + length
                 )
                 ||
                 (
                  p_start >= p_entry->addr &&
                  p_start < p_entry->addr + p_entry->length
                 )
               )
            {
                return p_entry;
            }
        }
    }
    return NULL;
}

static void flash_write_buffer(void)
{
    m_write_buffer_busy = 1;

    flash_write((uint32_t*) ((uint32_t) m_current_transfer.p_bank_addr
                               - (uint32_t) m_current_transfer.p_start_addr
                               + (uint32_t) m_current_transfer.p_write_pointer),
                    mp_write_buffer, WRITE_BUFFER_SIZE);

    m_current_transfer.first_invalid_byte_in_write_buffer = 0;
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/
void dfu_transfer_init(void)
{
    m_write_buffer_busy = 0;
    memset(&m_current_transfer, 0, sizeof(m_current_transfer));
    memset(mp_write_buffer, 0xFF, WRITE_BUFFER_SIZE);
}

uint32_t dfu_transfer_start(uint32_t* p_start_addr,
        uint32_t* p_bank_addr,
        uint32_t size,
        uint32_t section_size,
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
        m_current_transfer.p_bank_addr = p_start_addr;
    }
    else
    {
        m_current_transfer.p_bank_addr = p_bank_addr;
    }

    /* erase all affected pages. */
    flash_erase((uint32_t*) PAGE_ALIGN(m_current_transfer.p_bank_addr),
                            PAGE_ALIGN(size + PAGE_SIZE - 1));

    m_current_transfer.p_start_addr = p_start_addr;
    m_current_transfer.segment_count = segment_count;
    m_current_transfer.final_transfer = final_transfer;
    m_current_transfer.p_write_pointer = m_current_transfer.p_start_addr;
    m_current_transfer.first_invalid_byte_in_write_buffer = 0;
    m_current_transfer.size = size;
    return NRF_SUCCESS;
}

uint32_t dfu_transfer_data(uint32_t p_addr, uint8_t* p_data, uint16_t length)
{
    if (m_write_buffer_busy)
    {
        __LOG(RTT_CTRL_TEXT_RED "TRANSFER WRITE BUFFER BUSY!\n");
        return NRF_ERROR_BUSY;
    }
    if (!IS_WORD_ALIGNED(p_addr))
    {
        /* unaligned */
        return NRF_ERROR_INVALID_ADDR;
    }
    if (!IS_WORD_ALIGNED(length))
    {
        return NRF_ERROR_INVALID_LENGTH;
    }

    bool buffer_incoming_entry;
    if (WRITE_BUFFER_ALIGN(p_addr) == (uint32_t) m_current_transfer.p_write_pointer)
    {
        /* still in current write buffer */
        buffer_incoming_entry = true;
        if (m_current_transfer.first_invalid_byte_in_write_buffer < WRITE_BUFFER_OFFSET(p_addr))
        {
            /* skipped an entry */
            entry_mark_as_missing(
                (uint32_t) (m_current_transfer.p_write_pointer + m_current_transfer.first_invalid_byte_in_write_buffer),
                WRITE_BUFFER_OFFSET(p_addr) - m_current_transfer.first_invalid_byte_in_write_buffer);
        }
        else if (m_current_transfer.first_invalid_byte_in_write_buffer > WRITE_BUFFER_OFFSET(p_addr))
        {
            dfu_entry_t* p_backlog_entry = entry_in_missing_backlog(p_addr, length);
            if (p_backlog_entry == NULL)
            {
                /* already have this entry buffered */
                return NRF_ERROR_INVALID_STATE;
            }
        }
    }
    else if (WRITE_BUFFER_ALIGN(p_addr) > (uint32_t) m_current_transfer.p_write_pointer)
    {
        /* This only happens if we miss the last entry on the previous buffer, mark that entry as
           invalid. */
        entry_mark_as_missing(
                PAGE_ALIGN(m_current_transfer.p_write_pointer) + m_current_transfer.first_invalid_byte_in_write_buffer,
                PAGE_SIZE - m_current_transfer.first_invalid_byte_in_write_buffer);

        /* iterating the write buffer */
        flash_write_buffer();
        buffer_incoming_entry = true;
        m_current_transfer.p_write_pointer = (uint32_t*)((uint32_t) m_current_transfer.p_write_pointer + WRITE_BUFFER_SIZE);
    }
    else /* entry belongs in a write buffer we've already flashed. */
    {
        dfu_entry_t* p_backlog_entry = entry_in_missing_backlog(p_addr, length);
        if (*((uint32_t*) p_addr) != 0xFFFFFFFF && p_backlog_entry != NULL)
        {
            /* Recovering an old entry, flash it individually. */
            uint32_t error_code = flash_write((uint32_t*) p_addr, p_data, length);
            if (error_code != NRF_SUCCESS)
            {
                return NRF_ERROR_BUSY;
            }
            buffer_incoming_entry = false;
        }
        else
        {
            /* already have this entry buffered */
            return NRF_ERROR_INVALID_STATE;
        }
    }

    entry_mark_as_not_missing(p_addr, length);

    if (buffer_incoming_entry)
    {
        memcpy(&mp_write_buffer[WRITE_BUFFER_OFFSET(p_addr)], p_data, length);

        if (m_current_transfer.first_invalid_byte_in_write_buffer < WRITE_BUFFER_OFFSET(p_addr) + length)
        {
            m_current_transfer.first_invalid_byte_in_write_buffer = WRITE_BUFFER_OFFSET(p_addr) + length;
        }

        if (m_current_transfer.first_invalid_byte_in_write_buffer == WRITE_BUFFER_SIZE)
        {
            /* last entry in page */
            flash_write_buffer();
            m_current_transfer.p_write_pointer = (uint32_t*)((uint32_t) m_current_transfer.p_write_pointer + WRITE_BUFFER_SIZE);
        }
    }
    return NRF_SUCCESS;
}

bool dfu_transfer_has_entry(uint32_t* p_addr, uint8_t* p_out_buffer, uint16_t len)
{
    uint32_t* p_storage_addr;
    if (WRITE_BUFFER_ALIGN(p_addr) > (uint32_t) m_current_transfer.p_write_pointer)
    {
        return false;
    }
    else if (WRITE_BUFFER_ALIGN(p_addr) == (uint32_t) m_current_transfer.p_write_pointer)
    {
        if (WRITE_BUFFER_OFFSET(p_addr) + len >= m_current_transfer.first_invalid_byte_in_write_buffer)
        {
            return false;
        }

        p_storage_addr = (uint32_t*) ((uint32_t) mp_write_buffer + (uint32_t) WRITE_BUFFER_OFFSET(p_addr));
    }
    else /* already flashed */
    {
        p_storage_addr = (uint32_t*) ((uint32_t) p_addr
                - (uint32_t) m_current_transfer.p_start_addr
                + (uint32_t) m_current_transfer.p_bank_addr);
    }

    if (entry_in_missing_backlog((uint32_t) p_addr, len))
    {
        return false;
    }

    if (p_out_buffer && len)
    {
        memcpy(p_out_buffer, p_storage_addr, len);
    }
    return true;
}

bool dfu_transfer_get_oldest_missing_entry(uint32_t* p_start_addr, uint32_t** pp_entry, uint32_t* p_len)
{
    *pp_entry = NULL;
    *p_len = 0;
    for (uint32_t i = 0; i < MISSING_ENTRY_BACKLOG_COUNT; ++i)
    {
        if (m_current_transfer.p_missing_entry_backlog[i].addr != 0 &&
            m_current_transfer.p_missing_entry_backlog[i].addr >= (uint32_t) p_start_addr &&
            (
             (m_current_transfer.p_missing_entry_backlog[i].addr < (uint32_t) *pp_entry)
             ||
             (*pp_entry == NULL)
            )
           )
        {
            *pp_entry = (uint32_t*) m_current_transfer.p_missing_entry_backlog[i].addr;
            *p_len = m_current_transfer.p_missing_entry_backlog[i].length;
        }
    }
    return (*pp_entry != NULL);
}

void dfu_transfer_sha256(sha256_context_t* p_hash_context)
{
    sha256_update(p_hash_context,
                  (uint8_t*) m_current_transfer.p_bank_addr,
                  m_current_transfer.size);
}

void dfu_transfer_end(void)
{
    if (m_current_transfer.first_invalid_byte_in_write_buffer != 0)
    {
        flash_write_buffer();
    }
}

void dfu_transfer_flash_write_complete(uint8_t* p_write_src)
{
    if (p_write_src == mp_write_buffer)
    {
        m_write_buffer_busy = 0;
        memset(mp_write_buffer, 0xFF, WRITE_BUFFER_SIZE);
    }
    else if (m_write_buffer_busy)
    {
        __LOG(RTT_CTRL_TEXT_CYAN "Flash end not for us (got 0x%x, wants 0x%x)\n",
                p_write_src, mp_write_buffer);
    }
}
