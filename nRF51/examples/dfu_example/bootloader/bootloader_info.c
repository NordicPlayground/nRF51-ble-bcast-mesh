#include <string.h>
#include "bootloader_info.h"
#include "bootloader_mesh.h"
#include "dfu_types_mesh.h"
#include "nrf_flash.h"
#include "app_error.h"
#include "nrf51.h"

typedef union
{
    struct
    {
        uint8_t len;
        uint8_t type;
        uint8_t data[];
    } meta_8_8;
    struct
    {
        uint8_t len;
        uint16_t type;
        uint8_t data[];
    } meta_8_16;
    struct
    {
        uint16_t len;
        uint8_t type;
        uint8_t data[];
    } meta_16_8;
    struct
    {
        uint16_t len;
        uint16_t type;
        uint8_t data[];
    } meta_16_16;
} bootloader_info_header_t;

/******************************************************************************
* Static functions
******************************************************************************/
static uint32_t info_header_get_len(bootloader_info_header_t* p_info_header)
{
    switch (bootloader_info_get()->metadata.len_length)
    {
        case 16:
            return (uint32_t) p_info_header->meta_16_16.len * 4;
        default: 
            return (uint32_t) p_info_header->meta_8_8.len * 4;
    }
}

static uint32_t info_header_get_type(bootloader_info_header_t* p_info_header)
{
    switch (bootloader_info_get()->metadata.len_length)
    {
        case 8:
            switch (bootloader_info_get()->metadata.type_length)
            {
                case 8:
                    return p_info_header->meta_8_8.type;
                case 16:
                    return p_info_header->meta_8_16.type;
            }
        case 16:
            switch (bootloader_info_get()->metadata.type_length)
            {
                case 8:
                    return p_info_header->meta_16_8.type;
                case 16:
                    return p_info_header->meta_16_16.type;
            }
    }
    return 0;
}

static void info_header_set_type(bootloader_info_header_t* p_info_header, bl_info_type_t type)
{
    switch (bootloader_info_get()->metadata.len_length)
    {
        case 8:
            switch (bootloader_info_get()->metadata.type_length)
            {
                case 8:
                    p_info_header->meta_8_8.type = type;
                case 16:
                    p_info_header->meta_8_16.type = type;
            }
        case 16:
            switch (bootloader_info_get()->metadata.type_length)
            {
                case 8:
                    p_info_header->meta_16_8.type = type;
                case 16:
                    p_info_header->meta_16_16.type = type;
            }
    }
}

static uint8_t* info_header_get_data(bootloader_info_header_t* p_info_header)
{
    switch (bootloader_info_get()->metadata.len_length)
    {
        case 8:
            switch (bootloader_info_get()->metadata.type_length)
            {
                case 8:
                    return p_info_header->meta_8_8.data;
                case 16:
                    return p_info_header->meta_8_16.data;
            }
        case 16:
            switch (bootloader_info_get()->metadata.type_length)
            {
                case 8:
                    return p_info_header->meta_16_8.data;
                case 16:
                    return p_info_header->meta_16_16.data;
            }
    }
    return NULL;
}

static void info_header_set_len(bootloader_info_header_t* p_info_header, uint32_t length)
{
    /* correct poor alignment */
    if (length & 0x03)
    {
        length = length & 0xFFFFFFFC + 4;
    }
    switch (bootloader_info_get()->metadata.len_length)
    {
        case 8:
            switch (bootloader_info_get()->metadata.type_length)
            {
                case 8:
                    p_info_header->meta_8_8.len = length / 4;
                case 16:
                    p_info_header->meta_8_16.len = length / 4;
            }
        case 16:
            switch (bootloader_info_get()->metadata.type_length)
            {
                case 8:
                    p_info_header->meta_16_8.len = length / 4;
                case 16:
                    p_info_header->meta_16_16.len = length / 4;
            }
    }
}


static inline bootloader_info_header_t* bootloader_info_iterate(bootloader_info_header_t* p_info_header)
{
    return (bootloader_info_header_t*) (((uint8_t*) p_info_header) + info_header_get_len(p_info_header));
}

static bootloader_info_header_t* bootloader_info_header_get(bl_info_entry_t* p_entry)
{
    if (p_entry == NULL || (uint32_t) p_entry == FLASH_SIZE) 
    {
        return NULL;
    }
    uint8_t offset = bootloader_info_get()->metadata.len_length / 8 + bootloader_info_get()->metadata.type_length / 8;
    return (bootloader_info_header_t*) ((uint8_t*) p_entry - offset);
}

static inline bootloader_info_header_t* bootloader_info_first_unused_get(uint32_t page_address)
{
    const uint32_t unused_entry_type = (((bootloader_info_t*) page_address)->metadata.type_length == 16 ?
            BL_INFO_TYPE_LAST16 : 
            BL_INFO_TYPE_LAST8);
    return bootloader_info_header_get(bootloader_info_entry_get(page_address, (bl_info_type_t) unused_entry_type));
}

/******************************************************************************
* Interface functions
******************************************************************************/
void bootloader_info_init(void)
{
    /* make sure we have an end-of-entries entry */
    const uint32_t end_of_entries_type = (bootloader_info_get()->metadata.type_length == 16 ?
            BL_INFO_TYPE_LAST16 : 
            BL_INFO_TYPE_LAST8);
    if (bootloader_info_first_unused_get(BOOTLOADER_INFO_ADDRESS) == NULL)
    {
        /* need to restore the bank */
        if (bootloader_info_first_unused_get(BOOTLOADER_INFO_BANK_ADDRESS) == NULL)
        {
            /* bank is invalid too, no way to recover */
            bootloader_abort(BL_END_ERROR_INVALID_PERSISTANT_STORAGE);
        }
        
        nrf_flash_erase((uint32_t*) BOOTLOADER_INFO_ADDRESS, PAGE_SIZE);
        nrf_flash_store((uint32_t*) BOOTLOADER_INFO_ADDRESS, (uint8_t*) BOOTLOADER_INFO_BANK_ADDRESS, PAGE_SIZE, 0);
    }
}

bl_info_entry_t* bootloader_info_entry_get(uint32_t page_address, bl_info_type_t type)
{
    bootloader_info_header_t* p_header = (bootloader_info_header_t*) ((bootloader_info_t*) page_address)->data;
    uint32_t iterations = 0;
    while ((bl_info_type_t) info_header_get_type(p_header) != type)
    {
        p_header = bootloader_info_iterate(p_header);
        if ((uint32_t) p_header > page_address + PAGE_SIZE || 
            ++iterations > PAGE_SIZE / 2)
        {
            return (bl_info_entry_t*) (FLASH_SIZE); /* out of bounds */
        }
    }
    return (bl_info_entry_t*) info_header_get_data(p_header);
}

bl_info_entry_t* bootloader_info_entry_put(bl_info_type_t type, bl_info_entry_t* p_entry, uint32_t length)
{
    if ((uint32_t) p_entry < 0x20000000) /* must come from RAM */
    {
        return NULL;
    }
    const uint32_t header_len = 
        bootloader_info_get()->metadata.len_length / 8 + 
        bootloader_info_get()->metadata.type_length / 8;
    
    /* previous entry, to be invalidated after we've stored the current entry. */
    bootloader_info_header_t* p_old_header = bootloader_info_header_get(bootloader_info_entry_get(BOOTLOADER_INFO_ADDRESS, type));
    
    /* find first unused space */
    bootloader_info_header_t* p_new_header = bootloader_info_first_unused_get(BOOTLOADER_INFO_ADDRESS);
    if (p_new_header == NULL || 
        p_new_header >= (bootloader_info_header_t*) (BOOTLOADER_INFO_ADDRESS + PAGE_SIZE))
    {
        /* entry page has overflowed. A reset is needed. */
        bootloader_info_reset();
        p_new_header = bootloader_info_first_unused_get(BOOTLOADER_INFO_ADDRESS);
        if (p_new_header == NULL || 
            p_new_header >= (bootloader_info_header_t*) (BOOTLOADER_INFO_ADDRESS + PAGE_SIZE))
        {
            bootloader_abort(BL_END_ERROR_NO_MEM);
        }
    }
    
    if ((uint32_t) p_new_header & 0x03)
    {
        APP_ERROR_CHECK(NRF_ERROR_INVALID_ADDR);
    }
    
    /* store new entry in the available space */
    uint8_t buffer[PAGE_SIZE];
    info_header_set_type((bootloader_info_header_t*) buffer, type);
    info_header_set_len((bootloader_info_header_t*) buffer, length + header_len);
    memcpy(&buffer[header_len], p_entry, length);
    
    /* add end-of-entries-entry */
    info_header_set_type((bootloader_info_header_t*) &buffer[header_len + length], 
        bootloader_info_get()->metadata.type_length == 16 ?
            BL_INFO_TYPE_LAST16 : 
            BL_INFO_TYPE_LAST8);
    
    nrf_flash_store((uint32_t*) p_new_header, buffer, header_len + length, 0);
    
    /* invalidate old entry of this type */
    if (p_old_header != NULL)
    {
        /* TODO: optimization: check if the write only adds 0-bits to the current value, 
                 in which case we can just overwrite the current. */
        
        if ((uint32_t) p_old_header & 0x03)
        {
            APP_ERROR_CHECK(NRF_ERROR_INVALID_ADDR);
        }
        
        bootloader_info_header_t old_header;
        memcpy(&old_header, p_old_header, header_len);
        info_header_set_type(&old_header, BL_INFO_TYPE_INVALID);
        
        uint32_t len = header_len;
        if (header_len & 0x03)
        {
            len = (header_len & 0xFFFFFFFC) + 4;
        }
        nrf_flash_store((uint32_t*) p_old_header, (uint8_t*) &old_header, len, 0);
    }    
    return (bl_info_entry_t*) info_header_get_data(p_new_header);
}

void bootloader_info_reset(void)
{
    /* create new page from valid entries */
    uint8_t new_page[PAGE_SIZE];
    memset(new_page, 0xFF, PAGE_SIZE);
    uint8_t* p_new_page_next_space = &new_page[0];
    
    bootloader_info_header_t* p_info = ((bootloader_info_header_t*) bootloader_info_get()->data);
    const uint32_t unused_entry_type = (1 << bootloader_info_get()->metadata.type_length) - 1;
    
    while ((uint32_t) p_info < (uint32_t) (bootloader_info_get() + PAGE_SIZE))
    {
        bl_info_type_t type = (bl_info_type_t) info_header_get_type(p_info); 
        
        if (type == unused_entry_type)
        {
            break;
        }
        
        if (type != BL_INFO_TYPE_INVALID)
        {
            uint32_t len = info_header_get_len(p_info);
            memcpy(p_new_page_next_space, p_info, len);
            p_new_page_next_space += len;
        }
        p_info = bootloader_info_iterate(p_info);
    }
    
    if (p_new_page_next_space < &new_page[PAGE_SIZE])
    {
        info_header_set_type((bootloader_info_header_t*) p_new_page_next_space, 
            bootloader_info_get()->metadata.type_length == 16 ?
            BL_INFO_TYPE_LAST16 : 
            BL_INFO_TYPE_LAST8);
    }
    /* bank page */
    nrf_flash_erase((uint32_t*) BOOTLOADER_INFO_BANK_ADDRESS, PAGE_SIZE);
    nrf_flash_store((uint32_t*) BOOTLOADER_INFO_BANK_ADDRESS, new_page, PAGE_SIZE, 0);
    
    /* reflash original */
    nrf_flash_erase((uint32_t*) BOOTLOADER_INFO_ADDRESS, PAGE_SIZE);
    nrf_flash_store((uint32_t*) BOOTLOADER_INFO_ADDRESS, (uint8_t*) BOOTLOADER_INFO_BANK_ADDRESS, PAGE_SIZE, 0);
}
