#ifndef BOOTLOADER_H__
#define BOOTLOADER_H__

#include <stdint.h>
#include <stdbool.h>
#include "dfu_types_mesh.h"

#define SD_VERSION_INVALID                  (0x0000)
#define APP_VERSION_INVALID                 (0x00000000)

typedef enum
{
    BL_STATE_FIND_FWID,
    BL_STATE_DFU_REQ,
    BL_STATE_DFU_READY,
    BL_STATE_DFU_TARGET,
    BL_STATE_VALIDATE,
} bl_state_t;

typedef enum
{
    BL_END_SUCCESS,
    BL_END_FWID_VALID,
    BL_END_ERROR_PACKET_LOSS,
    BL_END_ERROR_UNAUTHORIZED,
    BL_END_ERROR_NO_START,
    BL_END_ERROR_TIMEOUT,
    BL_END_ERROR_NO_MEM,
    BL_END_ERROR_INVALID_PERSISTANT_STORAGE,
    BL_END_ERROR_SEGMENT_VIOLATION,
} bl_end_t;

void bootloader_init(void);

void bootloader_rx(dfu_packet_t* p_packet, uint16_t length);
void bootloader_abort(bl_end_t end_reason);
void bootloader_rtc_irq_handler(void);

#endif /* BOOTLOADER_H__ */
