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

#ifndef BL_IF_H__
#define BL_IF_H__

#include <stdint.h>
#include "dfu_types_mesh.h"

#define BL_IF_VERSION   (1)

#define TX_REPEATS_INF  (0xFF)

typedef enum
{
    /* general opcodes */
    BL_CMD_TYPE_INIT = 0x00,
    BL_CMD_TYPE_ENABLE,
    BL_CMD_TYPE_RX,
    BL_CMD_TYPE_TIMEOUT,
    BL_CMD_TYPE_ECHO,

    /* DFU */
    BL_CMD_TYPE_DFU_START_TARGET = 0x20,
    BL_CMD_TYPE_DFU_START_RELAY,
    BL_CMD_TYPE_DFU_START_SOURCE,
    BL_CMD_TYPE_DFU_ABORT,
    BL_CMD_TYPE_DFU_BANK_FLASH,                 /**< Tell the bootloader to flash a bank. App will restart afterwards. */
    BL_CMD_TYPE_DFU_BANK_INFO_GET,              /**< Get info on the bank of the given type. */

    /* info opcodes */
    BL_CMD_TYPE_INFO_GET = 0x30,
    BL_CMD_TYPE_INFO_PUT,
    BL_CMD_TYPE_INFO_ERASE,

    /* uECC opcodes */
    BL_CMD_TYPE_UECC_SHARED_SECRET = 0x40,
    BL_CMD_TYPE_UECC_MAKE_KEY,
    BL_CMD_TYPE_UECC_VALID_PUBLIC_KEY,
    BL_CMD_TYPE_UECC_COMPUTE_PUBLIC_KEY,
    BL_CMD_TYPE_UECC_SIGN,
    BL_CMD_TYPE_UECC_VERIFY,

    /* Flash status */
    BL_CMD_TYPE_FLASH_WRITE_COMPLETE = 0x60,
    BL_CMD_TYPE_FLASH_ERASE_COMPLETE,
    BL_CMD_TYPE_FLASH_ALL_COMPLETE,

} bl_cmd_type_t;

typedef enum
{
    /* Generic */
    BL_EVT_TYPE_ECHO = 0x00,                /**< Response to an echo command. */
    BL_EVT_TYPE_ERROR,                      /**< Request to reset a previously requested timer. */
    BL_EVT_TYPE_BANK_AVAILABLE,             /**< A DFU has been received and banked, and is available for flashing. */

    /* DFU */
    BL_EVT_TYPE_DFU_ABORT = 0x20,           /**< DFU aborted, and went back to idle. */
    BL_EVT_TYPE_DFU_NEW_FW,                 /**< A neighbor device has a newer version of our firmware. */
    BL_EVT_TYPE_DFU_REQ,                    /**< Request to participate in a transfer. */
    BL_EVT_TYPE_DFU_START,                  /**< The transfer started. */
    BL_EVT_TYPE_DFU_DATA_SEGMENT_RX,        /**< The DFU module got a new packet in the ongoing DFU transfer. */
    BL_EVT_TYPE_DFU_END,                    /**< The transfer ended. */

    /* Flash */
    BL_EVT_TYPE_FLASH_ERASE = 0x50,         /**< Request to erase a flash section. */
    BL_EVT_TYPE_FLASH_WRITE,                /**< Request to write to a flash section. */

    /* TX */
    BL_EVT_TYPE_TX_RADIO = 0x60,            /**< Request to transmit a packet over the radio. */
    BL_EVT_TYPE_TX_SERIAL,                  /**< Request to transmit a packet over the serial connection. */
    BL_EVT_TYPE_TX_ABORT,                   /**< Request to stop transmitting a given packet. */

    /* Timer */
    BL_EVT_TYPE_TIMER_SET = 0x70,           /**< Request to start a timer. */
    BL_EVT_TYPE_TIMER_ABORT,                /**< Request to abort a running timer. */
} bl_evt_type_t;

typedef enum
{
    BL_RADIO_INTERVAL_TYPE_EXPONENTIAL,
    BL_RADIO_INTERVAL_TYPE_REGULAR,
    BL_RADIO_INTERVAL_TYPE_REGULAR_SLOW,
} bl_radio_interval_type_t;

typedef enum
{
    FLASH_OP_TYPE_NONE  = 0x00,
    FLASH_OP_TYPE_WRITE = 0x01,
    FLASH_OP_TYPE_ERASE = 0x02,
    FLASH_OP_TYPE_ALL   = 0x03
} flash_op_type_t;

typedef union
{
    struct
    {
        uint32_t    start_addr;
        uint32_t    length;
    } erase;
    struct
    {
        uint32_t    start_addr;
        uint8_t*    p_data;
        uint32_t    length;
    } write;
} flash_op_t;

/** uECC */
typedef enum
{
    UECC_CURVE_SECP160R1,
    UECC_CURVE_SECP192R1,
    UECC_CURVE_SECP224R1,
    UECC_CURVE_SECP256R1,
    UECC_CURVE_SECP256K1,
} uECC_curve_t;

typedef struct bl_evt bl_evt_t;
typedef struct bl_cmd bl_cmd_t;

typedef uint32_t (*bl_if_cb_evt_t)(bl_evt_t* p_bl_evt);
typedef uint32_t (*bl_if_cmd_handler_t)(bl_cmd_t* p_bl_cmd);

struct bl_cmd
{
    bl_cmd_type_t type;
    union
    {
        struct
        {
            uint8_t bl_if_version;
            bl_if_cb_evt_t event_callback;
            uint8_t timer_count;
            uint8_t tx_slots;
            bool in_app;
        } init;
        struct
        {
            dfu_packet_t* p_dfu_packet;
            uint32_t length;
        } rx;
        struct
        {
            uint8_t timer_index;
        } timeout;
        struct
        {
            char str[16];
        } echo;

        union
        {
            union
            {
                struct
                {
                    dfu_type_t      type;
                    fwid_union_t    fwid;
                    uint32_t*       p_bank_start;
                } target;
                struct
                {
                    dfu_type_t      type;
                    fwid_union_t    fwid;
                    uint32_t        transaction_id; /**< Set to 0 if unknown */
                } relay;
            } start;
            struct
            {
                dfu_type_t          bank_dfu_type; /**< There's only ever one bank of each DFU type. Specify which bank should be flashed. */
            } bank_flash;
            struct
            {
                dfu_type_t          bank_dfu_type; /**< There's only ever one bank of each DFU type. Specify which bank to get. */
                dfu_bank_info_t*    p_bank_info;
            } bank_info_get;
        } dfu;
        union
        {
            struct
            {
                bl_info_type_t      type;
                bl_info_entry_t*    p_entry;
            } get;
            struct
            {
                bl_info_type_t      type;
                bl_info_entry_t*    p_entry;
                uint32_t            length;
            } put;
            struct
            {
                bl_info_type_t type;
            } erase;
        } info;
        union
        {
            struct
            {
                const uint8_t*  p_public_key;
                const uint8_t*  p_private_key;
                uint8_t*        p_secret;
                uECC_curve_t    curve;
            } shared_secret;
            struct
            {
                uint8_t*        p_public_key;
                uint8_t*        p_private_key;
                uECC_curve_t    curve;
            } make_key;
            struct
            {
                const uint8_t*  p_public_key;
                uECC_curve_t    curve;
            } valid_public_key;
            struct
            {
                const uint8_t*  p_private_key;
                uint8_t*        p_public_key;
                uECC_curve_t    curve;
            } compute_public_key;
            struct
            {
                const uint8_t*  p_private_key;
                const uint8_t*  p_hash;
                uint32_t        hash_size;
                uint8_t*        p_signature;
                uECC_curve_t    curve;
            } sign;
            struct
            {
                const uint8_t*  p_public_key;
                const uint8_t*  p_hash;
                uint32_t        hash_size;
                uint8_t*        p_signature;
                uECC_curve_t    curve;
            } verify;
        } uecc;
        union
        {
            struct
            {
                void* p_data;
            } write;
            struct
            {
                void* p_dest;
            } erase;
        } flash;
    } params;
};

struct bl_evt
{
    bl_evt_type_t type;
    union
    {
        struct
        {
            char str[16];
        } echo;
        struct
        {
            uint32_t error_code;
            const char* p_file;
            uint32_t line;
        } error;
        union
        {
            struct
            {
                dfu_role_t      role;
                dfu_state_t     state;
                dfu_type_t      dfu_type;
                fwid_union_t    fwid;
                uint8_t         authority;
                uint32_t        transaction_id;
            } req;
            struct
            {
                dfu_state_t     state;
                dfu_type_t      fw_type;
                fwid_union_t    fwid;
            } new_fw;
            struct
            {
                dfu_role_t      role;
                dfu_type_t      dfu_type;
                fwid_union_t    fwid;
            } start;
            struct
            {
                uint16_t        received_segment;
                uint16_t        total_segments;
            } data_segment;
            struct
            {
                dfu_role_t      role;
                dfu_type_t      dfu_type;
                fwid_union_t    fwid;
            } end;
            struct
            {
                dfu_end_t       reason;
            } abort;
        } dfu;
        struct
        {
            dfu_type_t      bank_dfu_type;
            bool            is_signed;
            fwid_union_t    bank_fwid;
            fwid_union_t    current_fwid;
            uint32_t*       p_bank_addr;
            uint32_t        bank_length;
        } bank_available;

        flash_op_t flash;

        union
        {
            struct
            {
                dfu_packet_t*               p_dfu_packet;
                uint32_t                    length;
                bl_radio_interval_type_t    interval_type;
                uint8_t                     tx_count;
                uint8_t                     tx_slot;
            } radio;
            struct
            {
                dfu_packet_t*   p_dfu_packet;
                uint32_t        length;
            } serial;
            struct
            {
                uint8_t         tx_slot;
            } abort;
        } tx;
        union
        {
            struct
            {
                uint32_t index;
                uint32_t delay_us;
            } set;
            struct
            {
                uint32_t index;
            } abort;
        } timer;
    } params;
};


#endif /* BL_IF_H__ */

