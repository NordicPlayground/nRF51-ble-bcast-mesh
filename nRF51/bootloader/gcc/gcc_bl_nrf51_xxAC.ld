/* Linker script for Nordic Semiconductor nRF5 devices
 *
 * Version: Sourcery G++ 4.5-1
 * Support: https://support.codesourcery.com/GNUToolchain/
 *
 * Copyright (c) 2007, 2008, 2009, 2010 CodeSourcery, Inc.
 *
 * The authors hereby grant permission to use, copy, modify, distribute,
 * and license this software and its documentation for any purpose, provided
 * that existing copyright notices are retained in all copies and that this
 * notice is included verbatim in any distributions.  No written agreement,
 * license, or royalty fee is required for any of the authorized uses.
 * Modifications to this software may be copyrighted by their authors
 * and need not follow the licensing terms described here, provided that
 * the new terms are clearly indicated on the first page of each file where
 * they apply.
 */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)
MEMORY
{
  FLASH (rx) : ORIGIN = 0x0003A000, LENGTH = 22K
  RAM (rwx) :  ORIGIN = 0x20000004, LENGTH = 0x7cfc
  CORE_BL_RAM (rwx) :  ORIGIN = 0x20007D00, LENGTH = 0x2fc
  UICR_BOOTADDR (rx) : ORIGIN = 0x10001014, LENGTH = 0x04
}

SECTIONS
{
    .core_data :
    {
        . = ALIGN(4);
        _build/sha256.o(.data*)
        _build/bootloader_app_bridge.o(.data*)
        _build/bootloader_info.o(.data*)
        _build/uECC.o(.data*)
        _build/dfu_mesh.o(.data*)
        _build/dfu_transfer_mesh.o(.data*)
        _build/dfu_bank.o(.data*)
        _build/dfu_util.o(.data*)
        . = ALIGN(4);
    } > CORE_BL_RAM

    .core_bss :
    {
        . = ALIGN(4);
        _build/sha256.o(.bss*)
        _build/bootloader_app_bridge.o(.bss*)
        _build/bootloader_info.o(.bss*)
        _build/uECC.o(.bss*)
        _build/dfu_mesh.o(.bss*)
        _build/dfu_transfer_mesh.o(.bss*)
        _build/dfu_bank.o(.bss*)
        _build/dfu_util.o(.bss*)
        . = ALIGN(4);
    } > CORE_BL_RAM
}

INCLUDE "gcc_nrf51_common.ld"
