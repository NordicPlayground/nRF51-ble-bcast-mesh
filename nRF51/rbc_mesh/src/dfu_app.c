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

#include "dfu_app.h"
#include "nrf_error.h"

/*****************************************************************************
* Local defines
*****************************************************************************/

/*****************************************************************************
* Local typedefs
*****************************************************************************/

/*****************************************************************************
* Static globals
*****************************************************************************/

/*****************************************************************************
* Static functions
*****************************************************************************/

/*****************************************************************************
* Interface functions
*****************************************************************************/
uint32_t dfu_init(void)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

uint32_t dfu_jump_to_bootloader(void)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

uint32_t dfu_request(dfu_type_t type, fwid_union_t* p_fwid, uint32_t* p_bank_addr)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

uint32_t dfu_abort(void)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

uint32_t dfu_bank_info_get(dfu_type_t type, dfu_bank_info_t* p_bank_info)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

uint32_t dfu_bank_flash(dfu_type_t bank_type)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

uint32_t dfu_state_get(dfu_state_t* p_dfu_state)
{
    return NRF_ERROR_NOT_SUPPORTED;
}


uint32_t dfu_cmd_send(bl_cmd_t* p_cmd)
{
    return NRF_ERROR_NOT_SUPPORTED;
}

uint32_t dfu_evt_handler(bl_evt_t* p_evt)
{
    return NRF_ERROR_NOT_SUPPORTED;
}


