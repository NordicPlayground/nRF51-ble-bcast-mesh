/* Linker script to configure memory regions. */

SEARCH_DIR(.)
GROUP(-lgcc -lc -lnosys)

MEMORY
{
  FLASH (rx) : ORIGIN = 0x00018000, LENGTH = 160K 
  RAM (rwx) :  ORIGIN = 0x20006000, LENGTH = 8K
}

INCLUDE "gcc_nrf51_common.ld"
