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
#ifndef _DFU_TYPES_H__
#define _DFU_TYPES_H__

#include <stdint.h>

#ifndef PAGE_SIZE
#define PAGE_SIZE                   (0x400)
#endif

#define PAGE_COUNT                  (NRF_FICR->CODESIZE)
#define FLASH_SIZE                  (PAGE_SIZE * PAGE_COUNT)

#define SEGMENT_ADDR(segment_id, start_addr) (((start_addr) & 0xFFFFFFF0) + ((segment_id) - 1) * 16)

#define SOFTDEVICE_INFORMATION_BASE     0x0003000                                                       /**< Location in the SoftDevice image which holds the SoftDevice informations. */
#define SOFTDEVICE_INFORMATION          ((SOFTDEVICE_INFORMATION_Type *) SOFTDEVICE_INFORMATION_BASE)   /**< Make SoftDevice information accessible through the structure. */

#define APP_START_ADDRESS           SOFTDEVICE_INFORMATION->softdevice_size

#define BOOTLOADER_START_ADDRESS    (0x0003C000)
#define BOOTLOADER_INFO_ADDRESS     (0x0003FC00)
#define BOOTLOADER_MAX_SIZE         (BOOTLOADER_INFO_ADDRESS - BOOTLOADER_START_ADDRESS)

#define DFU_SD_BANK_ADDRESS         (BOOTLOADER_START_ADDRESS / 2)
#define DFU_SD_MAX_SIZE             (BOOTLOADER_START_ADDRESS - DFU_SD_BANK_ADDRESS)

#define DFU_APP_BANK_ADDRESS        ((BOOTLOADER_START_ADDRESS - APP_START_ADDRESS) / 2 + APP_START_ADDRESS)
#define DFU_APP_MAX_SIZE            (BOOTLOADER_START_ADDRESS - DFU_APP_BANK_ADDRESS)

#define DFU_BL_BANK_ADDRESS         (DFU_APP_BANK_ADDRESS)
#define DFU_BL_MAX_SIZE             (BOOTLOADER_MAX_SIZE)

typedef enum
{
    DFU_TYPE_APP,
    DFU_TYPE_SOFTDEVICE,
    DFU_TYPE_BOOTLOADER
} dfu_type_t;

typedef struct
{
    uint32_t size;
    dfu_type_t dfu_type;
    uint32_t start_addr;
    uint32_t app_id;
    uint32_t bank_addr; /* ignored by fw */
    uint16_t image_crc;
    uint8_t using_crc; /* flag for whether or not the image_crc is actually used */
} dfu_bootloader_info_t;

#endif /* _DFU_TYPES_H__ */
