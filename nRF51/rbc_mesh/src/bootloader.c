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

#include "dfu_types.h"
#include "nrf_flash.h"
#include "timeslot_handler.h"
#include "version_handler.h"
#include "rbc_mesh.h"
#include "event_handler.h"
#include "transport_control.h"

#include "nrf_sdm.h"
#include "nrf51.h"
#include "bootloader_util.h"
#include "boards.h"
#include <string.h>
#include <stdbool.h>

#define FULL_ADDRESS(short_addr) (((uint32_t) (short_addr)) << 8);

#define MAX_NUMBER_INTERRUPTS   (32)
#define HANDLE_COUNT            (20)
/*****************************************************************************
* Static globals
*****************************************************************************/
struct 
{
    uint16_t length;
    uint8_t data[28];
} g_srv_data[HANDLE_COUNT];
/******************************************************************************
* Static Functions
*****************************************************************************/


/*****************************************************************************
* Dummy Functions
*****************************************************************************/
void SVC_Handler(void)
{
    __BKPT(0);
}


uint32_t mesh_srv_char_val_set(uint8_t handle, uint8_t* data, uint16_t length)
{
    if (handle > HANDLE_COUNT || handle == 0)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    g_srv_data[handle].length = length;
    memcpy(g_srv_data[handle].data, data, length);
    return NRF_SUCCESS;
}

uint32_t mesh_srv_char_val_get(uint8_t handle, uint8_t* data, uint16_t* length)
{
    if (handle > HANDLE_COUNT || handle == 0)
    {
        return NRF_ERROR_INVALID_ADDR;
    }
    *length = g_srv_data[handle].length;
    memcpy(data, g_srv_data[handle].data, g_srv_data[handle].length);
    return NRF_SUCCESS;
}

/*****************************************************************************
* Interface Functions
*****************************************************************************/

