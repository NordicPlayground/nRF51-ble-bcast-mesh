#ifndef _MESH_ACI_H__
#define _MESH_ACI_H__

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
  ACI_STATUS_SUCCESS                                        = 0x00,
  ACI_STATUS_ERROR_UNKNOWN                                  = 0x80,
  ACI_STATUS_ERROR_INTERNAL                                 = 0x81,
  ACI_STATUS_ERROR_CMD_UNKNOWN                              = 0x82,
  ACI_STATUS_ERROR_DEVICE_STATE_INVALID                     = 0x83,
  ACI_STATUS_ERROR_INVALID_LENGTH                           = 0x84,
  ACI_STATUS_ERROR_INVALID_PARAMETER                        = 0x85,
  ACI_STATUS_ERROR_BUSY                                     = 0x86,
  ACI_STATUS_ERROR_INVALID_DATA                             = 0x87,
  ACI_STATUS_RESERVED_START                                 = 0xF0,
  ACI_STATUS_RESERVED_END                                   = 0xFF

} __packed aci_status_code_t;

void mesh_aci_init(void);

void mesh_aci_loop(void);

#endif /* _MESH_ACI_H__ */
