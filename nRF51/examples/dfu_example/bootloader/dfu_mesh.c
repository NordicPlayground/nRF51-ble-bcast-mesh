#include <string.h>
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

#define PAGE_ALIGN(p_pointer)       (((uint32_t) p_pointer) & (~((uint32_t) (PAGE_SIZE - 1))))
#define PAGE_OFFSET(p_pointer)      (((uint32_t) p_pointer) & (PAGE_SIZE - 1))

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
    uint32_t*       p_current_page;
    uint32_t        first_invalid_byte_on_page;
    dfu_entry_t     p_missing_entry_backlog[MISSING_ENTRY_BACKLOG_COUNT];
    uint32_t        missing_entry_count;
} dfu_transfer_t;
/*****************************************************************************
* Static globals
*****************************************************************************/
static dfu_transfer_t   m_current_transfer;
static uint32_t         mp_page_buffer[PAGE_SIZE / 4];

/*****************************************************************************
* Static Functions
*****************************************************************************/
static dfu_entry_t* entry_in_missing_backlog(uint32_t p_start, uint16_t length);

static void entry_mark_as_not_missing(uint32_t p_start, uint16_t length)
{
    uint32_t found_entries = 0;
    for (uint32_t i = 0; i < MISSING_ENTRY_BACKLOG_COUNT && found_entries < m_current_transfer.missing_entry_count; ++i)
    {
        dfu_entry_t* p_entry = &m_current_transfer.p_missing_entry_backlog[i];

        if (p_entry->addr != 0)
        {
            found_entries++;
            /* perfect match */
            if (p_entry->addr == p_start && p_entry->length == length)
            {
                p_entry->addr = 0;
                p_entry->length = 0;
                m_current_transfer.missing_entry_count--;
                return;
            }

            /* existing entry starts inside new */
            if (p_entry->addr > p_start &&
                p_entry->addr < p_start + length)
            {
                if (p_entry->addr + p_entry->length > p_start + length)
                {
                    p_entry->length = p_entry->addr + p_entry->length - p_start;
                }
                else
                {
                    p_entry->length = length;
                }
                p_entry->addr = p_start;
                return;
            }

            /* new entry starts inside original */
            if (p_entry->addr + p_entry->length > p_start &&
                p_entry->addr + p_entry->length < p_start + length)
            {
                if (p_entry->length < p_start + length - p_entry->addr)
                {
                    p_entry->length = p_start + length - p_entry->addr;
                }
                return;
            }
        }
    }
}

static void entry_mark_as_missing(uint32_t p_start, uint16_t length)
{
    /* first, check for entries which can be merged with this */
    dfu_entry_t* p_existing_entry = entry_in_missing_backlog(p_start - 1, length + 2);
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
    uint32_t entries_encountered = 0;
    for (uint32_t i = 0; i < MISSING_ENTRY_BACKLOG_COUNT && entries_encountered < m_current_transfer.missing_entry_count; ++i)
    {
        dfu_entry_t* p_entry = &m_current_transfer.p_missing_entry_backlog[i];

        if (p_entry->addr != 0)
        {
            entries_encountered++;
            /* overlap: */
            if (
                 (
                  p_entry->addr >= p_start &&
                  p_entry->addr <= p_start + length
                 )
                 ||
                 (
                  p_entry->addr + p_entry->length > p_start &&
                  p_entry->addr + p_entry->length < p_start + length
                 )
               )
            {
                return p_entry;
            }
        }
    }
    return NULL;
}

static bool flash_page(void)
{
    nrf_flash_store(m_current_transfer.p_current_page, (uint8_t*) mp_page_buffer, PAGE_SIZE, 0);

    for (uint32_t i = 0; i < PAGE_SIZE / 4; ++i)
    {
        mp_page_buffer[i] = 0xFFFFFFFF;
    }

    m_current_transfer.p_current_page = (uint32_t*)((uint32_t) m_current_transfer.p_current_page + PAGE_SIZE);
    m_current_transfer.first_invalid_byte_on_page = 0;
    
    return true;
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/
void dfu_init(void)
{
    m_current_transfer.first_invalid_byte_on_page = 0;
    m_current_transfer.p_current_page = NULL;
    memset(m_current_transfer.p_missing_entry_backlog, 0, sizeof(m_current_transfer.p_missing_entry_backlog));
    m_current_transfer.missing_entry_count = 0;
}

void dfu_start(uint32_t* p_start_addr, uint32_t* p_bank_addr, uint16_t segment_count, bool final_transfer)
{
    dfu_init();
    uint32_t* p_end_addr = (uint32_t*) SEGMENT_ADDR(segment_count, (uint32_t) p_start_addr);
    if (p_bank_addr == p_start_addr || p_bank_addr == NULL)
    {
        if (PAGE_OFFSET(p_start_addr) != 0)
        {
            memcpy(mp_page_buffer, (void*) PAGE_ALIGN(p_start_addr), (uint32_t) PAGE_OFFSET(p_start_addr));
        }

        /* Wipe all but the last page. This operation is super slow */
        nrf_flash_erase((uint32_t*) PAGE_ALIGN(p_start_addr), PAGE_ALIGN(p_end_addr) - PAGE_ALIGN(p_start_addr));

        /* Restore the beginning of the first page */
        if (PAGE_OFFSET(p_start_addr) != 0)
        {
            nrf_flash_store((uint32_t*) PAGE_ALIGN(p_start_addr), (uint8_t*) mp_page_buffer, (uint32_t) PAGE_OFFSET(p_start_addr), 0);
        }

        /* Erase last page, but retain unaffected data. */
        if (PAGE_OFFSET(p_end_addr) != 0)
        {
            memcpy(mp_page_buffer, p_end_addr, PAGE_SIZE - (uint32_t) PAGE_OFFSET(p_end_addr));
        }
        nrf_flash_erase((uint32_t*) PAGE_ALIGN(p_end_addr), PAGE_SIZE);
        if (PAGE_OFFSET(p_end_addr) != 0)
        {
            nrf_flash_store(p_end_addr, (uint8_t*) mp_page_buffer, PAGE_SIZE - (uint32_t) PAGE_OFFSET(p_end_addr), 0);
        }

        m_current_transfer.p_bank_addr = p_start_addr;
    }
    else /* double bank */
    {

        uint32_t page_count = 1 + (segment_count * 16) / 1024;
        nrf_flash_erase((uint32_t*) PAGE_ALIGN(p_bank_addr), page_count * PAGE_SIZE); /* This breaks the app */
        m_current_transfer.p_bank_addr = p_bank_addr;
        NRF_UICR->BOOTLOADERADDR = (uint32_t) p_start_addr;
    }
    m_current_transfer.p_start_addr = p_start_addr;
    m_current_transfer.segment_count = segment_count;
    m_current_transfer.final_transfer = final_transfer;
    m_current_transfer.p_current_page = (uint32_t*) (PAGE_ALIGN(m_current_transfer.p_start_addr));
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

    /* if entry stretches over several pages, take them one at a time, recursively */
    if (PAGE_OFFSET(p_addr) + length >= PAGE_SIZE)
    {
        uint16_t first_length = PAGE_SIZE - PAGE_OFFSET(p_addr) + length;
        uint32_t error_code = dfu_data(p_addr, p_data, first_length);
        if (error_code != NRF_SUCCESS)
        {
            return error_code;
        }

        p_addr = (uint32_t) (((uint8_t*) p_addr) + first_length);
        p_data += first_length;
        length -= first_length;
    }


    bool buffer_incoming_entry;
    if (PAGE_ALIGN(p_addr) == (uint32_t) m_current_transfer.p_current_page)
    {
        buffer_incoming_entry = true;
        if (m_current_transfer.first_invalid_byte_on_page < PAGE_ALIGN(p_addr))
        {
            entry_mark_as_missing(m_current_transfer.first_invalid_byte_on_page,
                    PAGE_ALIGN(p_addr) - m_current_transfer.first_invalid_byte_on_page);
        }
    }
    else if (PAGE_ALIGN(p_addr) > (uint32_t) m_current_transfer.p_current_page)
    {
        /* This only happens if we miss the last entry on the previous page, mark that entry as
           invalid. */
        entry_mark_as_missing(m_current_transfer.first_invalid_byte_on_page,
                PAGE_SIZE - m_current_transfer.first_invalid_byte_on_page);

        /* moving to next page */
        flash_page();
        buffer_incoming_entry = true;
    }
    else
    {
        /* Recovering an old entry, flash it individually. */
        nrf_flash_store((uint32_t*) p_addr, p_data, length, 0);
        buffer_incoming_entry = false;
    }
    
    entry_mark_as_not_missing(p_addr, length);
    
    if (buffer_incoming_entry)
    {
        memcpy(&mp_page_buffer[PAGE_OFFSET(p_addr)], p_data, length);

        m_current_transfer.first_invalid_byte_on_page = PAGE_OFFSET(p_addr) + length;

        if (m_current_transfer.first_invalid_byte_on_page == PAGE_SIZE)
        {
            /* last entry in page */
            flash_page();
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

        p_storage_addr = m_current_transfer.p_current_page + PAGE_OFFSET(p_addr);
    }
    else /* old page */
    {
        p_storage_addr = p_addr;
    }

    if (entry_in_missing_backlog((uint32_t) p_addr, len))
    {
        return false;
    }

    memcpy(p_out_buffer, p_storage_addr, len);
    return true;
}

void dfu_sha256(uint8_t* p_hash)
{
    sha256_context_t sha256_context;
    sha256_init(&sha256_context);
    sha256_update(&sha256_context, (uint8_t*) m_current_transfer.p_start_addr,
            SEGMENT_ADDR(m_current_transfer.segment_count, (uint32_t) m_current_transfer.p_start_addr) - (uint32_t) m_current_transfer.p_start_addr);
    sha256_final(&sha256_context, p_hash);
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
        /* move the bank with MBR */
        //TODO
    }
}

