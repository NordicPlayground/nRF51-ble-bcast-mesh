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
#ifndef DFU_TYPES_H__
#define DFU_TYPES_H__

#include <stdint.h>
#include <stdbool.h>

#ifdef NRF51
#define PAGE_SIZE          (0x400)

#ifndef FLASH_SIZE
#define FLASH_SIZE         (PAGE_SIZE * 256)
#endif

#define END_OF_RAM         (0x20000000 + NRF_FICR->SIZERAMBLOCKS * NRF_FICR->NUMRAMBLOCK)

#define BOOTLOADERADDR()   (NRF_UICR->BOOTLOADERADDR)
#endif

#ifdef NRF52
#define PAGE_SIZE          (0x1000)

#ifndef FLASH_SIZE
#define FLASH_SIZE         (1024 * 512)
#endif

#define END_OF_RAM         (0x20000000 + NRF_FICR->INFO.RAM * 1024)

#define BOOTLOADERADDR()   (NRF_UICR->NRFFW[0])
#endif


#define WORD_SIZE          (4)


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

#define BOOTLOADER_INFO_ADDRESS     (FLASH_SIZE - 1 * PAGE_SIZE)
#define BOOTLOADER_INFO_BANK_ADDRESS (FLASH_SIZE - 2 * PAGE_SIZE)

#define SEGMENT_LENGTH              (16)

#define DFU_AUTHORITY_MAX           (0x07)

#define DFU_FWID_LEN_APP            (10)
#define DFU_FWID_LEN_BL             (2)
#define DFU_FWID_LEN_SD             (2)

#define DFU_PACKET_LEN_FWID         (2 + 2 + 1 + 1 + 4 + 2 + 4)
#define DFU_PACKET_LEN_STATE_SD     (2 + 1 + 1 + 4 + DFU_FWID_LEN_SD)
#define DFU_PACKET_LEN_STATE_BL     (2 + 1 + 1 + 4 + DFU_FWID_LEN_BL)
#define DFU_PACKET_LEN_STATE_APP    (2 + 1 + 1 + 4 + DFU_FWID_LEN_APP)
#define DFU_PACKET_LEN_START        (2 + 2 + 4 + 4 + 4 + 2 + 1)
#define DFU_PACKET_LEN_DATA         (2 + 2 + 4 + SEGMENT_LENGTH)
#define DFU_PACKET_LEN_DATA_REQ     (2 + 2 + 4)
#define DFU_PACKET_LEN_DATA_RSP     (2 + 2 + 4 + SEGMENT_LENGTH)

#define DFU_PACKET_ADV_OVERHEAD     (1 /* adv_type */ + 2 /* UUID */) /* overhead inside adv data */
#define DFU_PACKET_OVERHEAD         (MESH_PACKET_BLE_OVERHEAD + 1 + DFU_PACKET_ADV_OVERHEAD) /* dfu packet total overhead */
#define SERIAL_PACKET_OVERHEAD      (1)

#define DFU_PUBLIC_KEY_LEN          (64)
#define DFU_SIGNATURE_LEN           (64)

#define BL_INFO_LEN_PUBLIC_KEY      (DFU_PUBLIC_KEY_LEN)
#define BL_INFO_LEN_SEGMENT         (sizeof(bl_info_segment_t))
#define BL_INFO_LEN_FWID            (sizeof(fwid_t))
#define BL_INFO_LEN_FLAGS           (sizeof(bl_info_flags_t))
#define BL_INFO_LEN_SIGNATURE       (DFU_SIGNATURE_LEN)
#define BL_INFO_LEN_BANK_SIGNED     (sizeof(bl_info_bank_t))
#define BL_INFO_LEN_BANK            (BL_INFO_LEN_BANK_SIGNED - DFU_SIGNATURE_LEN)

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

typedef enum
{
    BL_INFO_TYPE_INVALID            = 0x00,
    BL_INFO_TYPE_ECDSA_PUBLIC_KEY   = 0x01,
    BL_INFO_TYPE_VERSION            = 0x02,
    BL_INFO_TYPE_FLAGS              = 0x04,

    BL_INFO_TYPE_SEGMENT_SD         = 0x10,
    BL_INFO_TYPE_SEGMENT_BL         = 0x11,
    BL_INFO_TYPE_SEGMENT_APP        = 0x12,

    BL_INFO_TYPE_SIGNATURE_SD       = 0x1A,
    BL_INFO_TYPE_SIGNATURE_BL       = 0x1B,
    BL_INFO_TYPE_SIGNATURE_APP      = 0x1C,
    BL_INFO_TYPE_SIGNATURE_BL_INFO  = 0x1D,

    BL_INFO_TYPE_BANK_BASE          = 0x20, /**< Only for adding offset to get the correct entry. */
    BL_INFO_TYPE_BANK_SD            = 0x21,
    BL_INFO_TYPE_BANK_BL            = 0x22,
    BL_INFO_TYPE_BANK_APP           = 0x24,
    BL_INFO_TYPE_BANK_BL_INFO       = 0x28,

    BL_INFO_TYPE_TEST               = 0x100,

    BL_INFO_TYPE_LAST               = 0x7FFF,
    BL_INFO_TYPE_UNUSED             = 0xFFFF,
} bl_info_type_t;

typedef struct
{
    uint32_t start;
    uint32_t length;
} bl_info_segment_t;

typedef enum
{
    DFU_END_SUCCESS,                             /**< The transfer ended successfully. */
    DFU_END_FWID_VALID,                          /**< The FWID was valid, and the bootloader stopped operation. */
    DFU_END_APP_ABORT,                           /**< The application requested to abort the transfer. */
    DFU_END_ERROR_PACKET_LOSS,
    DFU_END_ERROR_UNAUTHORIZED,
    DFU_END_ERROR_NO_START,
    DFU_END_ERROR_TIMEOUT,
    DFU_END_ERROR_NO_MEM,
    DFU_END_ERROR_INVALID_PERSISTENT_STORAGE,
    DFU_END_ERROR_SEGMENT_VIOLATION,
    DFU_END_ERROR_MBR_CALL_FAILED,
    DFU_END_ERROR_INVALID_TRANSFER,
    DFU_END_ERROR_BANK_IN_BOOTLOADER_AREA,
    DFU_END_ERROR_BANK_AND_DESTINATION_OVERLAP    /**< When copying the finished bank to its intended destination, it will have to overwrite itself. */
} dfu_end_t;

typedef enum
{
    DFU_STATE_INITIALIZED,      /**< The DFU module has been initialized, but not started. */
    DFU_STATE_FIND_FWID,        /**< There's no DFU operation in progress. */
    DFU_STATE_DFU_REQ,          /**< Beaconing requests for transfers. */
    DFU_STATE_READY,            /**< Ready to receive a transfer. */
    DFU_STATE_TARGET,           /**< Receiving a transfer. */
    DFU_STATE_VALIDATE,         /**< Validating and finishing up a transfer. */
    DFU_STATE_STABILIZE,        /**< Waiting for metadata about validated transfer to be written. */
    DFU_STATE_RELAY_CANDIDATE,  /**< Beaconing intent to relay a transfer. */
    DFU_STATE_RELAY             /**< Passively relaying a transfer. */
} dfu_state_t;

/** The various roles a device can play in a dfu transfer. */
typedef enum
{
    DFU_ROLE_NONE,      /**< No role. */
    DFU_ROLE_TARGET,    /**< The target role. A receiver of a transfer. */
    DFU_ROLE_RELAY,     /**< The relay role. A passive forwarding role. */
    DFU_ROLE_SOURCE     /**< The source role. The originator of a transfer. */
} dfu_role_t;

/** Current state of a transfer. */
typedef struct
{
    dfu_role_t      role;           /**< This device's intended role in the transfer. */
    dfu_type_t      type;           /**< The DFU type of the transfer. */
    fwid_union_t    fwid;           /**< The FWID of the new data in the transfer. */
    dfu_state_t     state;          /**< The current global state of the transfer. */
    uint8_t         data_progress;  /**< The progress of the transfer in percent (0-100). */
} dfu_transfer_state_t;

/** DFU Bank info structure. */
typedef struct
{
    dfu_type_t dfu_type;    /**< DFU type of the bank. */
    fwid_union_t fwid;      /**< Firmware ID of the bank. */
    bool is_signed;         /**< Flag indicating whether the bank is signed with an encryption key. */
    uint32_t* p_start_addr; /**< Start address of the bank. */
    uint32_t length;        /**< Length of the firmware in the bank. */
} dfu_bank_info_t;

typedef fwid_t bl_info_version_t;

typedef struct
{
    uint32_t sd_intact          :  1;
    uint32_t bl_intact          :  1;
    uint32_t app_intact         :  1;
    uint32_t page_is_invalid    :  1;
} bl_info_flags_t;

/**
 * State of info bank. Written to allow state machine progression being stored
 * in flash without needing erase.
 */
typedef enum
{
    BL_INFO_BANK_STATE_IDLE         = 0xFF, /**< The bank has not been touched since it got transferred. */
    BL_INFO_BANK_STATE_FLASH_FW     = 0xFE, /**< In the process of flashing the bank. */
    BL_INFO_BANK_STATE_FLASH_META   = 0xFC, /**< In the process of flashing metadata (signature and firmware) */
    BL_INFO_BANK_STATE_FLASHED      = 0xF8, /**< The bank has been flashed, and can be invalidated. */
} bl_info_bank_state_t;

typedef struct
{
    uint32_t*               p_bank_addr;
    uint32_t                length;
    fwid_union_t            fwid;
    bool                    has_signature;
    bl_info_bank_state_t    state;
    uint8_t                 signature[BL_INFO_LEN_SIGNATURE];
} bl_info_bank_t;

typedef union
{
    uint8_t             public_key[BL_INFO_LEN_PUBLIC_KEY];
    uint8_t             signature[BL_INFO_LEN_SIGNATURE];
    bl_info_segment_t   segment;
    bl_info_version_t   version;
    bl_info_flags_t     flags;
    bl_info_bank_t      bank;
} bl_info_entry_t;

#endif /* DFU_TYPES_H__ */
