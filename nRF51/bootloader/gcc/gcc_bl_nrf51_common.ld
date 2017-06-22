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

/* This file only defines the contents of the sections, which are common for all chip versions. */
SECTIONS
{
    .core_data :
    {
        __core_ram_start__ = .;
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
        __core_ram_end__ = .;
    } > CORE_BL_RAM
    .core_bss :
    {
        __core_ram_start__ = .;
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
        __core_ram_end__ = .;
    } > CORE_BL_RAM

    /* Ensures the Bootloader start address in flash is written to UICR when flashing the image. */
    .uicrBootStartAddress :
    {
      KEEP(*(.uicrBootStartAddress))
    } > UICR_BOOTADDR
}

