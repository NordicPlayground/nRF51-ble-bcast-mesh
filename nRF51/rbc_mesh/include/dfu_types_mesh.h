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
#ifndef _DFU_TYPES_H__
#define _DFU_TYPES_H__

#include <stdint.h>
#include "rbc_mesh.h"

#ifndef PAGE_SIZE
#define PAGE_SIZE                   (0x400)
#endif
#ifndef PAGE_COUNT
#define PAGE_COUNT                   (256)
#endif

#define FLASH_SIZE                  (PAGE_SIZE * PAGE_COUNT)

#define SEGMENT_ADDR(segment_id, start_addr) ((segment_id == 1)? \
                                                (uint32_t) start_addr : \
                                                (((uint32_t) start_addr) & 0xFFFFFFF0) + ((segment_id) - 1) * 16)

#define ADDR_SEGMENT(addr, start_addr) (uint32_t)(((((uint32_t) addr) - ((uint32_t) start_addr & 0xFFFFFFFF0)) >> 4) + 1)

#define PAGE_ALIGN(p_pointer)       (((uint32_t) p_pointer) & (~((uint32_t) (PAGE_SIZE - 1))))
#define PAGE_OFFSET(p_pointer)      (((uint32_t) p_pointer) & (PAGE_SIZE - 1))

#define SOFTDEVICE_INFORMATION_BASE     0x0003000                                                       /**< Location in the SoftDevice image which holds the SoftDevice informations. */
#define SOFTDEVICE_INFORMATION          ((SOFTDEVICE_INFORMATION_Type *) SOFTDEVICE_INFORMATION_BASE)   /**< Make SoftDevice information accessible through the structure. */
#define NRF_UICR_BOOT_START_ADDRESS     (NRF_UICR_BASE + 0x14)                                          /**< Register where the bootloader start address is stored in the UICR register. */

#define APP_START_ADDRESS           SOFTDEVICE_INFORMATION->softdevice_size

#define BOOTLOADER_START_ADDRESS    (FLASH_SIZE - 16 * PAGE_SIZE)
#define BOOTLOADER_INFO_ADDRESS     (FLASH_SIZE - 1 * PAGE_SIZE)
#define BOOTLOADER_INFO_BANK_ADDRESS (FLASH_SIZE - 2 * PAGE_SIZE)
#define BOOTLOADER_MAX_SIZE         (BOOTLOADER_INFO_ADDRESS - BOOTLOADER_START_ADDRESS)

#define DFU_SD_BANK_ADDRESS         (BOOTLOADER_START_ADDRESS / 2)
#define DFU_SD_MAX_SIZE             (BOOTLOADER_START_ADDRESS - DFU_SD_BANK_ADDRESS)

#define DFU_APP_BANK_ADDRESS        ((BOOTLOADER_START_ADDRESS - APP_START_ADDRESS) / 2 + APP_START_ADDRESS)
#define DFU_APP_MAX_SIZE            (BOOTLOADER_START_ADDRESS - DFU_APP_BANK_ADDRESS)

#define DFU_BL_BANK_ADDRESS         (DFU_APP_BANK_ADDRESS)
#define DFU_BL_MAX_SIZE             (BOOTLOADER_MAX_SIZE)

#define SEGMENT_LENGTH              (16)

#define DATA_SEQ_END                (0xFFFF)

#define DFU_AUTHORITY_MAX           (0x07)

#define DFU_FWID_LEN_APP            (10)
#define DFU_FWID_LEN_BL             (2)
#define DFU_FWID_LEN_SD             (2)

#define DFU_PACKET_LEN_FWID         (2 + 2 + 2 + 4 + 2 + 4)
#define DFU_PACKET_LEN_STATE_SD     (2 + 1 + 1 + 4 + DFU_FWID_LEN_SD)
#define DFU_PACKET_LEN_STATE_BL     (2 + 1 + 1 + 4 + DFU_FWID_LEN_BL)
#define DFU_PACKET_LEN_STATE_APP    (2 + 1 + 1 + 4 + DFU_FWID_LEN_APP)
#define DFU_PACKET_LEN_START        (2 + 2 + 4 + 4 + 2 + 2 + 1)
#define DFU_PACKET_LEN_DATA         (2 + 2 + 4 + SEGMENT_LENGTH)
#define DFU_PACKET_LEN_DATA_REQ     (2 + 2 + 4)
#define DFU_PACKET_LEN_DATA_RSP     (2 + 2 + 4 + SEGMENT_LENGTH)

#define SERIAL_PACKET_OVERHEAD      (1)

typedef uint16_t segment_t;
typedef uint16_t seq_t;

typedef enum
{
    DFU_PACKET_TYPE_DATA_RSP    = 0xFFFA,
    DFU_PACKET_TYPE_DATA_REQ    = 0xFFFB,
    DFU_PACKET_TYPE_DATA        = 0xFFFC,
    DFU_PACKET_TYPE_STATE       = 0xFFFD,
    DFU_PACKET_TYPE_FWID        = 0xFFFE,
} dfu_packet_type_t;

typedef enum
{
    DFU_TYPE_NONE       = 0,
    DFU_TYPE_SD         = (1 << 0),
    DFU_TYPE_BOOTLOADER = (1 << 1),
    DFU_TYPE_APP        = (1 << 2),
    DFU_TYPE_BL_INFO    = (1 << 3),
} dfu_type_t;

typedef struct __attribute((packed))
{
    uint32_t company_id;
    uint16_t app_id;
    uint32_t app_version;
} app_id_t;

typedef struct __attribute((packed))
{
    uint8_t id;
    uint8_t ver;
} bl_id_t;

typedef struct __attribute((packed))
{
    uint16_t sd;
    bl_id_t bootloader;
    app_id_t app;
} fwid_t;

typedef union __attribute((packed))
{
    app_id_t app;
    bl_id_t bootloader;
    uint16_t sd;
} fwid_union_t;

typedef struct __attribute((packed))
{
    uint16_t packet_type;
    union __attribute((packed))
    {
        struct __attribute((packed))
        {
            uint16_t version;
            uint8_t data[RBC_MESH_VALUE_MAX_LEN];
        } raw;
        fwid_t fwid;
        struct __attribute((packed))
        {
            uint8_t dfu_type    : 4;
            uint8_t _rfu1       : 4;
            uint8_t authority   : 3;
            uint8_t flood       : 1;
            uint8_t relay_node  : 1;
            uint8_t _rfu2       : 3;
            uint32_t transaction_id;
            fwid_union_t fwid;
        } state;
        struct __attribute((packed))
        {
            uint16_t segment;
            uint32_t transaction_id;
            uint32_t start_address;
            uint32_t length; /* in words */
            uint16_t signature_length;
            uint8_t diff        : 1;
            uint8_t single_bank : 1;
            uint8_t first       : 1;
            uint8_t last        : 1;
            uint8_t _rfu        : 4;
        } start;
        struct __attribute((packed))
        {
            uint16_t segment;
            uint32_t transaction_id;
            uint8_t data[SEGMENT_LENGTH];
        } data;
        struct __attribute((packed))
        {
            uint16_t segment;
            uint32_t transaction_id;
        } req_data;
        struct __attribute((packed))
        {
            uint16_t segment;
            uint32_t transaction_id;
            uint8_t data[SEGMENT_LENGTH];
        } rsp_data;
    } payload;
} dfu_packet_t;

#endif /* _DFU_TYPES_H__ */
