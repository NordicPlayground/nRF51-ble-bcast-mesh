/***********************************************************************************
Copyright (c) Nordic Semiconductor ASA
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

  1. Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

  2. Redistributions in binary form must reproduce the above copyright notice, this
  list of conditions and the following disclaimer in the documentation and/or
  other materials provided with the distribution.

  3. Neither the name of Nordic Semiconductor ASA nor the names of other
  contributors to this software may be used to endorse or promote products
  derived from this software without specific prior written permission.

  4. This software must only be used in a processor manufactured by Nordic
  Semiconductor ASA, or in a processor manufactured by a third party that
  is used in combination with a processor manufactured by Nordic Semiconductor.


THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
************************************************************************************/
#ifndef _MESH_ACI_H__
#define _MESH_ACI_H__

#include "rbc_mesh.h"

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

/** @brief Initialize serial handler */
void mesh_aci_init(void);

/** @brief asynchronous command handler */
void mesh_aci_command_check(void);

/** @brief rbc_mesh event handler */
void mesh_aci_rbc_event_handler(rbc_mesh_event_t* evt);

#endif /* _MESH_ACI_H__ */
