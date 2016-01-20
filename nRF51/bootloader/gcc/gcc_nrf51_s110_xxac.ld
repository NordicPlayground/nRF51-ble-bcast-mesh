/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00039C00, LENGTH = 17K
  RAM (rwx) :  ORIGIN = 0x20002000, LENGTH = 24K
  UICR_BOOTADDR (rx) : ORIGIN = 0x10001014, LENGTH = 0x04
}

INCLUDE "gcc_nrf51_common.ld"
