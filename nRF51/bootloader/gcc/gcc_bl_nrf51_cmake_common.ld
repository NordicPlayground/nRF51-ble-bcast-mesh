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
        *sha256.obj (.data*)
        *bootloader_app_bridge.obj (.data*)
        *bootloader_info.obj (.data*)
        *uECC.obj (.data*)
        *dfu_mesh.obj (.data*)
        *dfu_transfer_mesh.obj (.data*)
        *dfu_bank.obj (.data*)
        *dfu_util.obj (.data*)
        . = ALIGN(4);
        __core_ram_end__ = .;
    } > CORE_BL_RAM
    .core_bss :
    {
        __core_ram_start__ = .;
        . = ALIGN(4);
        *sha256.obj (.bss*)
        *bootloader_app_bridge.obj (.bss*)
        *bootloader_info.obj (.bss*)
        *uECC.obj (.bss*)
        *dfu_mesh.obj (.bss*)
        *dfu_transfer_mesh.obj (.bss*)
        *dfu_bank.obj (.bss*)
        *dfu_util.obj (.bss*)
        . = ALIGN(4);
        __core_ram_end__ = .;
    } > CORE_BL_RAM

    /* Ensures the Bootloader start address in flash is written to UICR when flashing the image. */
    .uicrBootStartAddress :
    {
      KEEP(*(.uicrBootStartAddress))
    } > UICR_BOOTADDR
}

