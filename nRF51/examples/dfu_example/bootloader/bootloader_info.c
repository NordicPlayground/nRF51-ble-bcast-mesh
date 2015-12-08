#include <stdio.h>
#include <string.h>
#include "bootloader_info.h"
#include "bootloader_mesh.h"
#include "dfu_types_mesh.h"
#include "nrf_flash.h"
#include "app_error.h"
#include "nrf51.h"

#define WORD_ALIGN(data) data = (((uint32_t) data + 4) & 0xFFFFFFFC)

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

static bootloader_info_t* mp_bl_info_page;
static bootloader_info_t* mp_bl_info_bank_page;
/******************************************************************************
* Static functions
******************************************************************************/
static uint32_t info_header_get_len(bootloader_info_header_t* p_info_header)
{
    switch (mp_bl_info_page->metadata.len_length)
    {
        case 16:
            return ((uint32_t) p_info_header->meta_16_16.len) * 4;
        default:
            return ((uint32_t) p_info_header->meta_8_8.len) * 4;
    }
}
static uint32_t info_header_get_type(bootloader_info_header_t* p_info_header)
{
    switch (mp_bl_info_page->metadata.len_length)
    {
        case 8:
            switch (mp_bl_info_page->metadata.type_length)
            {
                case 8:
                    return p_info_header->meta_8_8.type;
                case 16:
                    return p_info_header->meta_8_16.type;
            }
        case 16:
            switch (mp_bl_info_page->metadata.type_length)
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
    switch (mp_bl_info_page->metadata.len_length)
    {
        case 8:
            switch (mp_bl_info_page->metadata.type_length)
            {
                case 8:
                    p_info_header->meta_8_8.type = type;
                    break;
                case 16:
                    p_info_header->meta_8_16.type = type;
                    break;
            }
            break;
        case 16:
            switch (mp_bl_info_page->metadata.type_length)
            {
                case 8:
                    p_info_header->meta_16_8.type = type;
                    break;
                case 16:
                    p_info_header->meta_16_16.type = type;
                    break;
            }
            break;
    }
}

static uint8_t* info_header_get_data(bootloader_info_header_t* p_info_header)
{
    switch (mp_bl_info_page->metadata.len_length)
    {
        case 8:
            switch (mp_bl_info_page->metadata.type_length)
            {
                case 8:
                    return p_info_header->meta_8_8.data;
                case 16:
                    return p_info_header->meta_8_16.data;
            }
        case 16:
            switch (mp_bl_info_page->metadata.type_length)
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
        length += 4;
    }
    length /= 4;
    switch (mp_bl_info_page->metadata.len_length)
    {
        case 8:
            p_info_header->meta_8_8.len = (uint8_t) length;
            break;
        case 16:
            p_info_header->meta_16_8.len = (uint16_t) length;
            break;
    }
}


static void invalidate_entry(bootloader_info_header_t* p_header)
{
    /* TODO: optimization: check if the write only adds 0-bits to the current value,
       in which case we can just overwrite the current. */
    const uint32_t header_len =
        mp_bl_info_page->metadata.len_length / 8 +
        mp_bl_info_page->metadata.type_length / 8;

    if ((uint32_t) p_header & 0x03)
    {
        APP_ERROR_CHECK(NRF_ERROR_INVALID_ADDR);
    }

    bootloader_info_header_t old_header_ram;
    memcpy(&old_header_ram, p_header, header_len);
    info_header_set_type(&old_header_ram, BL_INFO_TYPE_INVALID);

    uint32_t len = header_len;
    if (len & 0x03)
    {
        WORD_ALIGN(len);
    }
    nrf_flash_store((uint32_t*) p_header, (uint8_t*) &old_header_ram, len, 0);
}

static inline bootloader_info_header_t* bootloader_info_iterate(bootloader_info_header_t* p_info_header)
{
    return (bootloader_info_header_t*) (((uint32_t) p_info_header) + info_header_get_len(p_info_header));
}

static bootloader_info_header_t* bootloader_info_header_get(bl_info_entry_t* p_entry)
{
    if (p_entry == NULL || (uint32_t) p_entry == FLASH_SIZE)
    {
        return NULL;
    }
    uint8_t offset = mp_bl_info_page->metadata.len_length / 8
        + mp_bl_info_page->metadata.type_length / 8;
    return (bootloader_info_header_t*) ((uint8_t*) p_entry - offset);
}

static inline bootloader_info_header_t* bootloader_info_first_unused_get(bootloader_info_t* p_bl_info_page)
{
    const uint32_t unused_entry_type =
            (((bootloader_info_t*) p_bl_info_page)->metadata.type_length == 16 ?
            BL_INFO_TYPE_LAST16 :
            BL_INFO_TYPE_LAST8);

    bl_info_entry_t* p_entry = bootloader_info_entry_get(
            (uint32_t*) p_bl_info_page,
            (bl_info_type_t) unused_entry_type);

    return (p_entry ? bootloader_info_header_get(p_entry) : NULL);
}

static bootloader_info_header_t* reset_with_replace(
        bl_info_type_t replace_type,
        bl_info_entry_t* p_entry,
        uint32_t entry_length)
{
    /* create new page from valid entries, assumes intact info page */
    uint8_t new_page[PAGE_SIZE];
    bootloader_info_header_t* p_replace_position = NULL;
    memset(new_page, 0xFF, PAGE_SIZE);
    memcpy(new_page, mp_bl_info_page, mp_bl_info_page->metadata.metadata_len);
    uint8_t* p_new_page_next_space = &new_page[mp_bl_info_page->metadata.metadata_len];

    bootloader_info_header_t* p_info =
        ((bootloader_info_header_t*)
         ((uint8_t*) mp_bl_info_page + mp_bl_info_page->metadata.metadata_len));

    const uint32_t unused_entry_type =
        mp_bl_info_page->metadata.type_length == 16 ?
        BL_INFO_TYPE_LAST16 :
        BL_INFO_TYPE_LAST8;
    const uint32_t header_len =
        mp_bl_info_page->metadata.len_length / 8 +
        mp_bl_info_page->metadata.type_length / 8;

    while ((uint32_t) p_info < (uint32_t) ((uint32_t) mp_bl_info_page + PAGE_SIZE))
    {
        bl_info_type_t type = (bl_info_type_t) info_header_get_type(p_info);

        if (type == unused_entry_type)
        {
            break;
        }

        if (type != BL_INFO_TYPE_INVALID)
        {
            if (type == replace_type)
            {
                /* replace */
                p_replace_position = (bootloader_info_header_t*) p_new_page_next_space;
                uint32_t len = header_len + entry_length;
                if (len & 0x03)
                {
                    WORD_ALIGN(len);
                }

                if (p_new_page_next_space + len > new_page + PAGE_SIZE)
                {
                    APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
                }
                info_header_set_len(p_replace_position, len);
                info_header_set_type(p_replace_position, type);
                memset(p_new_page_next_space + len - 3, 0xFF, 3); /* sanitize padding */
                memcpy(p_new_page_next_space + header_len, p_entry, entry_length);
                p_new_page_next_space += len;
            }
            else
            {
                /* restore */
                uint32_t len = info_header_get_len(p_info);
                if (p_new_page_next_space + len > new_page + PAGE_SIZE)
                {
                    APP_ERROR_CHECK(NRF_ERROR_INTERNAL);
                }
                memcpy(p_new_page_next_space, p_info, len);
                p_new_page_next_space += len;
            }
        }
        p_info = bootloader_info_iterate(p_info);
    }

    if (p_new_page_next_space < &new_page[PAGE_SIZE])
    {
        info_header_set_type((bootloader_info_header_t*) p_new_page_next_space,
            mp_bl_info_page->metadata.type_length == 16 ?
            BL_INFO_TYPE_LAST16 :
            BL_INFO_TYPE_LAST8);
    }

    /* bank page */
    nrf_flash_erase((uint32_t*) mp_bl_info_bank_page, PAGE_SIZE);
    nrf_flash_store((uint32_t*) mp_bl_info_bank_page, new_page, PAGE_SIZE, 0);

    /* reflash original */
    nrf_flash_erase((uint32_t*) mp_bl_info_page, PAGE_SIZE);
    nrf_flash_store((uint32_t*) mp_bl_info_page, new_page, PAGE_SIZE, 0);

    return p_replace_position;
}
/******************************************************************************
* Interface functions
******************************************************************************/
uint32_t bootloader_info_init(uint32_t* p_bl_info_page, uint32_t* p_bl_info_bank_page)
{
    if ((uint32_t) p_bl_info_page & (PAGE_SIZE - 1) ||
        (uint32_t) p_bl_info_bank_page & (PAGE_SIZE - 1))
    {
        return NRF_ERROR_INVALID_ADDR; /* must be page aligned */
    }

    mp_bl_info_page      = (bootloader_info_t*) p_bl_info_page;
    mp_bl_info_bank_page = (bootloader_info_t*) p_bl_info_bank_page;

    /* make sure we have an end-of-entries entry */
    if (bootloader_info_first_unused_get(mp_bl_info_page) == NULL)
    {
        /* need to restore the bank */
        if (bootloader_info_first_unused_get(mp_bl_info_bank_page) == NULL)
        {
            /* bank is invalid too, no way to recover */
            bootloader_abort(BL_END_ERROR_INVALID_PERSISTANT_STORAGE);
        }

        nrf_flash_erase((uint32_t*) mp_bl_info_page, PAGE_SIZE);
        nrf_flash_store((uint32_t*) mp_bl_info_page, (uint8_t*) mp_bl_info_bank_page, PAGE_SIZE, 0);
    }

    return NRF_SUCCESS;
}

bootloader_info_t* bootloader_info_get(void)
{
    return (bootloader_info_t*) (mp_bl_info_page);
}

bl_info_entry_t* bootloader_info_entry_get(uint32_t* p_bl_info_page, bl_info_type_t type)
{
    bootloader_info_header_t* p_header =
        (bootloader_info_header_t*)
        ((uint32_t) p_bl_info_page + ((bootloader_info_t*) p_bl_info_page)->metadata.metadata_len);
    uint32_t iterations = 0;
    bl_info_type_t iter_type = info_header_get_type(p_header);
    while (iter_type != type)
    {
        p_header = bootloader_info_iterate(p_header);
        if ((uint32_t) p_header > ((uint32_t) p_bl_info_page) + PAGE_SIZE ||
            (
             (iter_type == BL_INFO_TYPE_LAST8 || iter_type == BL_INFO_TYPE_LAST16) &&
             (iter_type != type)
            ) ||
            ++iterations > PAGE_SIZE / 2)
        {
            return NULL; /* out of bounds */
        }
        iter_type = info_header_get_type(p_header);
    }
    return (bl_info_entry_t*) info_header_get_data(p_header);
}

bl_info_entry_t* bootloader_info_entry_put(bl_info_type_t type,
                                           bl_info_entry_t* p_entry,
                                           uint32_t length)
{
    if (PAGE_ALIGN((uint32_t) p_entry) == (uint32_t) mp_bl_info_page) /* Can't come from our info page */
    {
        return NULL;
    }
    const uint32_t header_len =
        mp_bl_info_page->metadata.len_length / 8 +
        mp_bl_info_page->metadata.type_length / 8;

    /* previous entry, to be invalidated after we've stored the current entry. */
    bootloader_info_header_t* p_old_header =
        bootloader_info_header_get(bootloader_info_entry_get((uint32_t*) mp_bl_info_page, type));

    /* find first unused space */
    bootloader_info_header_t* p_new_header =
        bootloader_info_first_unused_get(mp_bl_info_page);

    /* we have to be able to fit the end-of-entries entry at the last word, therefore check entry
       length + 4 */
    if (p_new_header == NULL ||
        ((uint32_t) p_new_header + header_len + length + 4) >= (uint32_t) ((uint32_t) mp_bl_info_page + PAGE_SIZE))
    {
        /* entry page has overflowed. A reset is needed. */
        p_new_header = reset_with_replace(type, p_entry, length);

        p_old_header = NULL;
    }
    else
    {
        if ((uint32_t) p_new_header & 0x03)
        {
            APP_ERROR_CHECK(NRF_ERROR_INVALID_ADDR);
        }

        /* store new entry in the available space */
        uint8_t buffer[PAGE_SIZE];
        uint32_t entry_length = header_len + length;
        if (entry_length & 0x03)
        {
            WORD_ALIGN(entry_length);
        }

        memset(&buffer[entry_length - 4], 0xFF, 8); /* pad the end-of-entries entry */
        info_header_set_type((bootloader_info_header_t*) &buffer[0], type);
        info_header_set_len((bootloader_info_header_t*) &buffer[0], length + header_len);
        memcpy(&buffer[header_len], p_entry, length);

        /* add end-of-entries-entry */
        info_header_set_type((bootloader_info_header_t*) &buffer[entry_length],
            (mp_bl_info_page->metadata.type_length == 16 ?
                BL_INFO_TYPE_LAST16 :
                BL_INFO_TYPE_LAST8));

        nrf_flash_store((uint32_t*) p_new_header, buffer, entry_length + 4, 0);

        /* invalidate old entry of this type */
        if (p_old_header != NULL)
        {
            invalidate_entry(p_old_header);
        }
    }
    return (bl_info_entry_t*) info_header_get_data(p_new_header);
}

void bootloader_info_reset(void)
{
    reset_with_replace(BL_INFO_TYPE_INVALID, NULL, 0);
}

