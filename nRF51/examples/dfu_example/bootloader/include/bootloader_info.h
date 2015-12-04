#ifndef BOOTLOADER_INFO_H__
#define BOOTLOADER_INFO_H__
#include <stdint.h>
#include <stdbool.h>
#include "dfu_types_mesh.h"

#define DFU_PUBLIC_KEY_LEN      (512)

#define BL_INFO_LEN_PUBLIC_KEY  (DFU_PUBLIC_KEY_LEN)
#define BL_INFO_LEN_SEGMENT     (9)
#define BL_INFO_LEN_FWID        (14)
#define BL_INFO_LEN_JOURNAL     ((2 * PAGE_SIZE) / 8)
#define BL_INFO_LEN_FLAGS       (1)

typedef enum
{
    BL_INFO_TYPE_INVALID            = 0x00,
    BL_INFO_TYPE_ECDSA_PUBLIC_KEY   = 0x01,
    BL_INFO_TYPE_VERSION            = 0x02,
    BL_INFO_TYPE_JOURNAL            = 0x03,
    BL_INFO_TYPE_FLAGS              = 0x04,

    BL_INFO_TYPE_SEGMENT_SD         = 0x10,
    BL_INFO_TYPE_SEGMENT_BL         = 0x11,
    BL_INFO_TYPE_SEGMENT_APP        = 0x12,

    BL_INFO_TYPE_LAST8              = 0x7F,
    BL_INFO_TYPE_LAST16             = 0x7FFF,
} bl_info_type_t;

typedef struct
{
    uint32_t start;
    uint32_t length;
} bl_info_segment_t;

typedef fwid_t bl_info_version_t;

typedef struct
{
    uint32_t sd_intact          :  1;
    uint32_t bl_intact          :  1;
    uint32_t app_intact         :  1;
    uint32_t page_is_invalid    :  1;
} bl_info_flags_t;

typedef union
{
    uint8_t             public_key[BL_INFO_LEN_PUBLIC_KEY];
    bl_info_segment_t   segment;
    bl_info_version_t   version;
    uint8_t             journal[BL_INFO_LEN_JOURNAL];
    bl_info_flags_t     flags;
} bl_info_entry_t;

typedef struct
{
    struct
    {
        uint8_t metadata_len; /* in bytes */
        uint16_t len_length;  /* in bits */
        uint16_t type_length; /* in bits */
    } metadata;
    uint8_t data[];
} bootloader_info_t;



inline bootloader_info_t* bootloader_info_get(void)
{
    return (bootloader_info_t*) (BOOTLOADER_INFO_ADDRESS + DFU_PUBLIC_KEY_LEN);
}

void bootloader_info_init(void);
bl_info_entry_t* bootloader_info_entry_get(uint32_t page_address, bl_info_type_t type);
bl_info_entry_t* bootloader_info_entry_put(bl_info_type_t type, bl_info_entry_t* p_entry, uint32_t length); /* p_entry must point to RAM */
void bootloader_info_reset(void);


#endif /* BOOTLOADER_INFO_H__ */
