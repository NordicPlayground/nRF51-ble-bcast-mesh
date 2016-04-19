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
#include "dfu_mesh.h"
#include "dfu_types_mesh.h"
#include "bootloader_mesh.h"
#include "nrf51.h"
#include "nrf_flash.h"
#include "sha256.h"
#include "app_error.h"

#ifndef MISSING_ENTRY_BACKLOG_COUNT
#define MISSING_ENTRY_BACKLOG_COUNT (64)
#endif

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
    uint32_t*       p_current_page;
    uint32_t        first_invalid_byte_on_page;
    dfu_entry_t     p_missing_entry_backlog[MISSING_ENTRY_BACKLOG_COUNT];
    uint32_t        missing_entry_count;
} dfu_transfer_t;
/*****************************************************************************
* Static globals
*****************************************************************************/
static dfu_transfer_t   m_current_transfer;
static uint8_t          mp_page_buffer[PAGE_SIZE];

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
    bootloader_abort(BL_END_ERROR_PACKET_LOSS);
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

static void flash_page(void)
{
    nrf_flash_store((uint32_t*) ((uint32_t) PAGE_ALIGN(m_current_transfer.p_bank_addr)
            - (uint32_t) PAGE_ALIGN(m_current_transfer.p_start_addr)
            + (uint32_t) m_current_transfer.p_current_page),
            mp_page_buffer, PAGE_SIZE, 0);

    memset(mp_page_buffer, 0xFF, PAGE_SIZE);

    m_current_transfer.first_invalid_byte_on_page = 0;
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/
void dfu_init(void)
{
    memset(&m_current_transfer, 0, sizeof(m_current_transfer));
    memset(mp_page_buffer, 0xFF, PAGE_SIZE);
}

uint32_t dfu_start(uint32_t* p_start_addr, uint32_t* p_bank_addr, uint32_t size, uint32_t section_size, bool final_transfer)
{
    dfu_init();
    uint16_t segment_count = (((size + (uint32_t) p_start_addr) & 0xFFFFFFF0) - ((uint32_t) p_start_addr & 0xFFFFFFF0)) / 16;
    uint32_t* p_end_addr = p_start_addr + size / 4;
    if (p_bank_addr == p_start_addr || p_bank_addr == NULL)
    {
        bool reflash_front = false;
        bool single_page = false;

        /* buffer front */
        if (PAGE_OFFSET(p_start_addr) != 0)
        {
            reflash_front = true;
            memcpy(mp_page_buffer,
                   (void*) PAGE_ALIGN(p_start_addr),
                   (uint32_t) PAGE_OFFSET(p_start_addr));
        }

        /* single page transfer */
        if (PAGE_ALIGN(p_start_addr) == PAGE_ALIGN(p_end_addr))
        {
            single_page = true;
            memcpy(&mp_page_buffer[PAGE_OFFSET(p_end_addr)],
                    (uint32_t*) p_end_addr, PAGE_SIZE - PAGE_OFFSET(p_end_addr));
        }
        
        /* Wipe all but the last page. This operation is super slow */
        if (single_page)
        {
            nrf_flash_erase((uint32_t*) PAGE_ALIGN(p_start_addr),
                            PAGE_SIZE);
        }
        else
        {
            uint32_t count = (PAGE_ALIGN(p_end_addr) - PAGE_ALIGN(p_start_addr)) / PAGE_SIZE;
            nrf_flash_erase((uint32_t*) PAGE_ALIGN(p_start_addr),
                            count * PAGE_SIZE);
        }

        /* Restore the beginning of the first page */
        if (reflash_front)
        {
            nrf_flash_store((uint32_t*) PAGE_ALIGN(p_start_addr),
                            mp_page_buffer,
                            PAGE_OFFSET(p_start_addr), 0);
        }

        /* Restore the end of a single-page transfer */
        if (single_page)
        {
            nrf_flash_store(p_end_addr,
                            &mp_page_buffer[PAGE_OFFSET(p_end_addr)],
                            PAGE_SIZE - PAGE_OFFSET(p_end_addr), 0);
        }
        else if (PAGE_OFFSET(p_end_addr) != 0)
        {
            /* Erase last page, but retain unaffected data. */
            memcpy(mp_page_buffer, p_end_addr, PAGE_SIZE - (uint32_t) PAGE_OFFSET(p_end_addr));
            nrf_flash_erase((uint32_t*) PAGE_ALIGN(p_end_addr), PAGE_SIZE);

            nrf_flash_store(p_end_addr,
                    mp_page_buffer,
                    PAGE_SIZE - (uint32_t) PAGE_OFFSET(p_end_addr), 0);
        }

        m_current_transfer.p_bank_addr = p_start_addr;
    }
    else /* dual bank */
    {
        /* dual banked transfers must be page-aligned to fit MBR */
        if ((uint32_t) p_start_addr & (PAGE_SIZE - 1))
        {
            return NRF_ERROR_INVALID_ADDR;
        }
        uint32_t page_count = 1 + (segment_count * 16) / PAGE_SIZE;
        nrf_flash_erase((uint32_t*) PAGE_ALIGN(p_bank_addr), page_count * PAGE_SIZE); /* This breaks the app */
        m_current_transfer.p_bank_addr = p_bank_addr;
    }
    m_current_transfer.p_start_addr = p_start_addr;
    m_current_transfer.segment_count = segment_count;
    m_current_transfer.final_transfer = final_transfer;
    m_current_transfer.p_current_page = (uint32_t*) (PAGE_ALIGN(m_current_transfer.p_start_addr));
    m_current_transfer.first_invalid_byte_on_page = PAGE_OFFSET(p_start_addr);
    m_current_transfer.size = size;

    memset(mp_page_buffer, 0xFF, PAGE_SIZE);
    return NRF_SUCCESS;
}

uint32_t dfu_data(uint32_t p_addr, uint8_t* p_data, uint16_t length)
{
    if (((uint32_t) p_addr) & 0x03)
    {
        /* unaligned */
        return NRF_ERROR_INVALID_ADDR;
    }
    if (length & 0x03)
    {
        return NRF_ERROR_INVALID_LENGTH;
    }
    
    bool buffer_incoming_entry;
    if (PAGE_ALIGN(p_addr) == (uint32_t) m_current_transfer.p_current_page)
    {
        buffer_incoming_entry = true;
        if (m_current_transfer.first_invalid_byte_on_page < PAGE_OFFSET(p_addr))
        {
            entry_mark_as_missing(
                PAGE_ALIGN(m_current_transfer.p_current_page) + m_current_transfer.first_invalid_byte_on_page,
                PAGE_OFFSET(p_addr) - m_current_transfer.first_invalid_byte_on_page);
        }
        else if (m_current_transfer.first_invalid_byte_on_page > PAGE_OFFSET(p_addr))
        {
            dfu_entry_t* p_backlog_entry = entry_in_missing_backlog(p_addr, length);
            if (p_backlog_entry == NULL)
            {
                /* already have this entry buffered */
                return NRF_ERROR_INVALID_STATE;
            }
        }
    }
    else if (PAGE_ALIGN(p_addr) > (uint32_t) m_current_transfer.p_current_page)
    {
        /* This only happens if we miss the last entry on the previous page, mark that entry as
           invalid. */
        entry_mark_as_missing(
                PAGE_ALIGN(m_current_transfer.p_current_page) + m_current_transfer.first_invalid_byte_on_page,
                PAGE_SIZE - m_current_transfer.first_invalid_byte_on_page);

        /* moving to next page */
        flash_page();
        buffer_incoming_entry = true;
        m_current_transfer.p_current_page = (uint32_t*)((uint32_t) m_current_transfer.p_current_page + PAGE_SIZE);
    }
    else
    {
        dfu_entry_t* p_backlog_entry = entry_in_missing_backlog(p_addr, length);
        if (p_backlog_entry != NULL)
        {
            /* Recovering an old entry, flash it individually. */
            nrf_flash_store((uint32_t*) p_addr, p_data, length, 0);
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
        memcpy(&mp_page_buffer[PAGE_OFFSET(p_addr)], p_data, length);
        if (m_current_transfer.first_invalid_byte_on_page < PAGE_OFFSET(p_addr) + length)
        {
            m_current_transfer.first_invalid_byte_on_page = PAGE_OFFSET(p_addr) + length;
        }

        if (m_current_transfer.first_invalid_byte_on_page == PAGE_SIZE)
        {
            /* last entry in page */
            flash_page();
            m_current_transfer.p_current_page = (uint32_t*)((uint32_t) m_current_transfer.p_current_page + PAGE_SIZE);
        }
    }
    return NRF_SUCCESS;
}

bool dfu_has_entry(uint32_t* p_addr, uint8_t* p_out_buffer, uint16_t len)
{
    uint32_t* p_storage_addr;
    if (PAGE_ALIGN(p_addr) > (uint32_t) m_current_transfer.p_current_page)
    {
        return false;
    }
    else if (PAGE_ALIGN(p_addr) == (uint32_t) m_current_transfer.p_current_page)
    {
        if (PAGE_OFFSET(p_addr) + len >= m_current_transfer.first_invalid_byte_on_page)
        {
            return false;
        }

        p_storage_addr = (uint32_t*) ((uint32_t) mp_page_buffer + (uint32_t) PAGE_OFFSET(p_addr));
    }
    else /* old page */
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

bool dfu_get_oldest_missing_entry(uint32_t* p_start_addr, uint32_t** pp_entry, uint32_t* p_len)
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

void dfu_sha256(sha256_context_t* p_hash_context)
{
    sha256_update(p_hash_context,
                  (uint8_t*) m_current_transfer.p_bank_addr,
                  m_current_transfer.size);
}

void dfu_end(void)
{
    if (m_current_transfer.first_invalid_byte_on_page != 0)
    {
        flash_page();
    }
    if (m_current_transfer.p_bank_addr != m_current_transfer.p_start_addr &&
        m_current_transfer.p_bank_addr != NULL)
    {
#if 0
        /* move the bank with MBR. NRF_UICR->BOOTLOADERADDR must have been set. */
        sd_mbr_command_t sd_mbr_cmd;

        sd_mbr_cmd.command               = SD_MBR_COMMAND_COPY_BL;
        sd_mbr_cmd.params.copy_bl.bl_src = (uint32_t *)(bl_image_start);
        sd_mbr_cmd.params.copy_bl.bl_len = bootloader_settings.bl_image_size / sizeof(uint32_t);

        if (sd_mbr_command(&sd_mbr_cmd) != NRF_SUCCESS)
        {
            bootloader_abort(BL_END_ERROR_MBR_FAIL);
        }
#endif
    }
}

