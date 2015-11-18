#include <string.h>
#include "dfu.h"
#include "bootloader.h"
#include "nrf51.h"
//#include "nrf_flash.h"
#include "sha256.h"

#ifndef PAGE_SIZE
#define PAGE_SIZE               (0x400)
#endif

#define PAGE_ALIGN(p_pointer)   (void*)(((uint32_t) p_pointer) & (~((uint32_t) (PAGE_SIZE - 1))))
#define PAGE_OFFSET(p_pointer)  (void*)(((uint32_t) p_pointer) & (PAGE_SIZE - 1))

typedef struct
{
    uint32_t* p_start_addr;
    uint32_t* p_end_addr;
    uint32_t* p_bank_addr;
    dfu_type_t type;
    uint16_t segment_count;
    bool final_transfer;
} dfu_transfer_t;

static dfu_transfer_t m_current_transfer;

static uint32_t mp_page_buffer[PAGE_SIZE / 4];

void dfu_init(void)
{

}

void dfu_start(uint32_t* p_start_addr, uint32_t* p_end_addr, dfu_type_t type, uint16_t segment_count, bool final_transfer)
{
    if (type != DFU_TYPE_BOOTLOADER)
    {
        if (PAGE_OFFSET(p_start_addr) != 0)
        {
            memcpy(mp_page_buffer, PAGE_ALIGN(p_start_addr), (uint32_t) PAGE_OFFSET(p_start_addr));
        }

        /* Wipe all but the last page. This operation is super slow */
        nrf_flash_erase(PAGE_ALIGN(p_start_addr), PAGE_ALIGN(p_end_addr) - PAGE_ALIGN(p_start_addr));

        /* Recover the beginning of the first page */
        if (PAGE_OFFSET(p_start_addr) != 0)
        {
            nrf_flash_store(PAGE_ALIGN(p_start_addr), mp_page_buffer, (uint32_t) PAGE_OFFSET(p_start_addr), 0);
        }

        /* Erase last page, but retain unaffected data. */
        if (PAGE_OFFSET(p_end_addr) != 0)
        {
            memcpy(mp_page_buffer, p_end_addr, PAGE_SIZE - (uint32_t) PAGE_OFFSET(p_end_addr));
        }
        nrf_flash_erase(PAGE_ALIGN(p_end_addr), PAGE_SIZE);
        if (PAGE_OFFSET(p_end_addr) != 0)
        {
            nrf_flash_store(p_end_addr, mp_page_buffer, PAGE_SIZE - (uint32_t) PAGE_OFFSET(p_end_addr), 0);
        }
    }
    else
    {
        /* Use app space */
        uint32_t page_count = PAGE_ALIGN(p_end_addr) - PAGE_ALIGN(p_start_addr);
        uint32_t* p_bank_start = (uint32_t*) ((uint32_t) PAGE_ALIGN(p_start_addr) - (page_count * PAGE_SIZE));
        nrf_flash_erase(p_bank_start, page_count * PAGE_SIZE); /* This breaks the app */
        m_current_transfer.p_bank_addr = p_bank_start;
        NRF_UICR->BOOTLOADERADDR = (uint32_t) p_start_addr;
    }
    m_current_transfer.p_start_addr = p_start_addr;
    m_current_transfer.p_end_addr = p_end_addr;
    m_current_transfer.type = type;
    m_current_transfer.segment_count = segment_count;
    m_current_transfer.final_transfer = final_transfer;
}

bool dfu_data(uint32_t* p_addr, uint8_t* p_data, uint16_t length)
{
    if (m_current_transfer.type == DFU_TYPE_BOOTLOADER)
    {
        p_addr = (p_addr - m_current_transfer.p_start_addr + m_current_transfer.p_bank_addr);
    }

    if (p_addr >= m_current_transfer.p_start_addr &&
        p_addr + length <= m_current_transfer.p_end_addr)
    {
        nrf_flash_store(p_addr, p_data, length, 0);
        return true;
    }
    else
    {
        return false;
    }
}

void dfu_sha256(uint8_t* p_hash)
{
    sha256_context_t sha256_context;
    sha256_init(&sha256_context);
    sha256_update(&sha256_context, m_current_transfer.p_start_addr,
            m_current_transfer.p_end_addr - m_current_transfer.p_start_addr);
    sha256_final(&sha256_context, p_hash);
}

void dfu_end(void)
{
    /* init MBR? */
}

