/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)
MEMORY
{
  FLASH (rx) : ORIGIN = 0x00037C00, LENGTH = 28K
  RAM (rwx) :  ORIGIN = 0x20002000, LENGTH = 24K
  UICR_BOOTADDR (rx) : ORIGIN = 0x10001014, LENGTH = 0x04
}

SECTIONS
{
  /* Ensures the Bootloader start address in flash is written to UICR when flashing the image. */
  .uicrBootStartAddress :
  {
    KEEP(*(.uicrBootStartAddress))
  } > UICR_BOOTADDR
}

INCLUDE "gcc_nrf51_common.ld"
