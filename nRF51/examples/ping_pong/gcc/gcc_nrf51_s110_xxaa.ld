/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00018000, LENGTH = 160K 
  RAM (rwx) :  ORIGIN = 0x20004000, LENGTH = 16K
}

INCLUDE "gcc_nrf51_common.ld"
