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

  4. This software must only be used in a processor manufactured by Nordic
  Semiconductor ASA, or in a processor manufactured by a third party that
  is used in combination with a processor manufactured by Nordic Semiconductor.


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
#include "records.h"
#include "journal.h"
#include "nrf_flash.h"
#include "dfu_types_mesh.h"
#include "nrf.h"
#include "nrf_error.h"
#include <string.h>

static dfu_record_t* mp_records;
static uint32_t m_records_size;
static uint32_t m_index;
static uint16_t* mp_missing;
static uint32_t m_missing_count_max;
static uint32_t m_missing_count; /* current count of missing records (disregarding "holes") */
static uint32_t m_missing_recovered_count; /* number of "holes" in the missing-array */
static uint8_t m_assembly_page[PAGE_SIZE];

#define FULL_ADDRESS(short_addr)    (((uint32_t) (short_addr)) << 8)
#define PAGE_LOCAL_ADDR(full_addr)  ((full_addr) & 0x03FF)

#define INVALID_PAGE_INDEX          (0xFFFF)
#define INVALID_SHORT_ADDR          (0xFFFF)
#define INVALID_SEQ_NUM             (0xFFFF)
#define RECORDS_SIZE_MARGIN           (m_records_size / 10)

static void missing_records_invalidate(void)
{
    for (uint32_t i = 0; i < m_missing_count_max; ++i)
    {
        mp_missing[i] = INVALID_SEQ_NUM;
    }
}

static void records_invalidate(void)
{
    for (uint32_t i = 0; i < m_records_size; ++i)
    {
        mp_records[i].short_addr = INVALID_SHORT_ADDR;
    }
}

static bool record_mark_not_missing(uint16_t seq_num)
{
    for (uint32_t i = 0; i < m_missing_count; ++i)
    {
        if (mp_missing[i] == seq_num)
        {
            mp_missing[i] = INVALID_SEQ_NUM;
            if (i == m_missing_count - 1)
            {
                m_missing_count--;
            }
            else
            {
                m_missing_recovered_count++;
            }
            return true;
        }
    }
    return false;
}

static uint16_t get_first_page_index(bool reset_search_index)
{
    static uint32_t index = 0;
    if (reset_search_index)
    {
        index = 0;
    }
    
    for (; index < m_index; ++index)
    {
        if (mp_records[index].short_addr != INVALID_SHORT_ADDR)
        {
            return PAGE_INDEX(FULL_ADDRESS(mp_records[index].short_addr));
        }
    }
    return INVALID_PAGE_INDEX;
}

static dfu_record_t* get_next_record(uint16_t page_index, bool reset_search_index)
{
    static uint32_t index = 0;
    if (reset_search_index)
    {
        index = 0;
    }
    for (; index < m_index; ++index)
    {
        if (PAGE_INDEX(FULL_ADDRESS(mp_records[index].short_addr)) == page_index)
        {
            return &mp_records[index];
        }
    }

    return NULL;
}

uint32_t records_init(uint32_t missing_record_count)
{
    m_missing_count_max = missing_record_count;
    
    /* shameless dirty hack, seizing all unallocated memory */
#pragma diag_suppress 170
    extern uint32_t __initial_sp;
    mp_missing = (uint16_t*) (&__initial_sp + 4);
#pragma diag_warning 170
    
    mp_records = (dfu_record_t*) (mp_missing + 2 * m_missing_count_max);
    volatile uint32_t last_addr = 0x20000000 + NRF_FICR->SIZERAMBLOCKS * NRF_FICR->NUMRAMBLOCK;
    if (last_addr < (uint32_t) mp_records)
    {
        return NRF_ERROR_NO_MEM;
    }
    m_records_size = (last_addr - ((uint32_t) mp_records)) / sizeof(dfu_record_t);

    missing_records_invalidate();
    records_invalidate();
    
    return NRF_SUCCESS;
}

void records_record_add(dfu_record_t* p_record, bool was_missing)
{
    memcpy(&mp_records[m_index++], p_record, sizeof(dfu_record_t));
    if (was_missing)
    {
        record_mark_not_missing(p_record->seq_num);
    }

    if (m_index == m_records_size - RECORDS_SIZE_MARGIN)
    {
        records_flash();
    }
}

void records_flash_page(uint32_t page_index)
{
    uint32_t* page_addr = (uint32_t*) (PAGE_SIZE * page_index);
    /* load old page from flash */
    memcpy(m_assembly_page, page_addr, PAGE_SIZE);

    /* apply records to page */
    dfu_record_t* p_rec = get_next_record(page_index, true);
    uint32_t record_count = 0;
    uint16_t lowest_seq_num = UINT16_MAX;
    uint16_t highest_seq_num = 0;
    while (p_rec != NULL)
    {
        record_count++;

        memcpy(m_assembly_page + PAGE_LOCAL_ADDR(FULL_ADDRESS(p_rec->short_addr)), p_rec->data, DFU_RECORD_SIZE);
        p_rec->short_addr = INVALID_SHORT_ADDR; /* invalidate */
        
        if (p_rec->seq_num > highest_seq_num)
        {
            highest_seq_num = p_rec->seq_num;
        }
        if (p_rec->seq_num < lowest_seq_num)
        {
            lowest_seq_num = p_rec->seq_num;
        }

        p_rec = get_next_record(page_index, false);
    }
    
    /* erase and flash */
    journal_invalidate(page_index);
    nrf_flash_erase(page_addr, PAGE_SIZE);
    nrf_flash_store(page_addr, m_assembly_page, PAGE_SIZE, 0);
    
    if (!records_missing_in_range(lowest_seq_num, highest_seq_num))
    {
        journal_complete(page_index);
    }
}

void records_flash(void)
{
    uint32_t page_index = get_first_page_index(true);
    
    while (page_index != INVALID_PAGE_INDEX)
    {
        records_flash_page(page_index);
        
        /* fetch next page */
        page_index = get_first_page_index(false);
    }
    m_index = 0;
}

bool records_record_get(uint16_t seq_num, dfu_record_t* p_record)
{
    for (uint32_t i = 0; i < m_index; ++i)
    {
        if (mp_records[i].seq_num == seq_num)
        {
            memcpy(p_record, &mp_records[i], sizeof(dfu_record_t));
            return true;
        }
    }
    return false;
}

void records_missing_report(uint16_t seq_num)
{
    if (m_missing_recovered_count == 0)
    {
        mp_missing[m_missing_count++] = seq_num;
    }
    else /* we have holes in the array, have to iterate through */
    {
        bool placed = false;
        for (uint32_t i = 0; i < m_missing_count; ++i)
        {
            if (mp_missing[i] == INVALID_SEQ_NUM)
            {
                mp_missing[i] = seq_num;
                m_missing_recovered_count--;
                placed = true;
                break;
            }
        }
        if (!placed)
        {
            mp_missing[m_missing_count++] = seq_num; /* shouldn't be necessary */
        }
    }
}

bool records_missing_in_range(uint16_t lowest_seq_num, uint16_t highest_seq_num)
{
    for (uint32_t i = 0; i < m_missing_count; ++i)
    {
        if (mp_missing[i] >= lowest_seq_num && mp_missing[i] <= highest_seq_num)
        {
            return true;
        }
    }
    return false;
}
bool records_is_missing(uint16_t seq_num)
{
    for (uint32_t i = 0; i < m_missing_count; ++i)
    {
        if (mp_missing[i] == seq_num)
        {
            return true;
        }
    }
    return false;
}

uint16_t records_missing_get(void)
{
    for (uint32_t i = 0; i < m_missing_count; ++i)
    {
        if (mp_missing[i] != INVALID_SEQ_NUM)
        {
            return mp_missing[i];
        }
    }
    return INVALID_SEQ_NUM;
}

void records_clear(void)
{
    missing_records_invalidate();
    records_invalidate();
}
