#ifndef BOOTLOADER_H__
#define BOOTLOADER_H__

#include <stdint.h>
#include <stdbool.h>
#include "dfu.h"

#define BOOTLOADER_DATA_POS_APP_ID          (0x3F000) /* TODO: change */
#define BOOTLOADER_DATA_POS_SD_ID           (0x300C)
#define BOOTLOADER_DATA_POS_BOOTLOADER_ID   (0x3FA00) /* TODO: change */

#define BOOTLOADER_DATA_POS_PUBLIC_KEY      (0x3FB00) /* TODO: change */

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
    BL_END_ERROR_PACKET_LOSS,
    BL_END_ERROR_UNAUTHORIZED,
    BL_END_ERROR_NO_START,
    BL_END_ERROR_TIMEOUT,
} bl_end_t;

void bootloader_init(void);

void bootloader_rx(dfu_packet_t* p_packet);
void bootloader_abort(bl_end_t end_reason);

#endif /* BOOTLOADER_H__ */
