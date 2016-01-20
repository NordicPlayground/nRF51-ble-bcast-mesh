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
#ifndef BOOTLOADER_APP_H__
#define BOOTLOADER_APP_H__
#include <stdint.h>
#include <stdbool.h>
#include "dfu_types_mesh.h"

/** 
* Callback type for notifying the application of incoming dfu-transfers, and
* get authorization to start the bootloader. Return true to accept the dfu 
* transfer, false to reject it and resume normal operation. The device will 
* be reset immediately, and may not be available for regular operation for 
* several minutes.
*/
typedef bool (*bootloader_authorize_cb_t)(dfu_type_t type, fwid_union_t* p_fwid);

/** 
* Manually trigger the bootloader. The device will be reset immediately, and 
* may not be available for regular operation for several minutes. Will trigger
* the authorize callback, which must return true for the bootloader to start.
* If successful and authorized, this function call will not return.
*
* @param[in] type Type of transfer expected, or DFU_TYPE_NONE.
* @param[in] p_fwid FWID union to identify the incoming transfer.
* 
* @return NRF_ERROR_BUSY The application rejected the bootloader start request.
* @return NRF_ERROR_FORBIDDEN The NRF_UICR->BOOTLOADERADDR persistent register 
*   has not been set, and the bootloader could not start.
*/
uint32_t bootloader_start(dfu_type_t type, fwid_union_t* p_fwid);

/**
* Set the authorize callback function pointer. The function will be called for
* each call to bootloader_start(), and must return true to authorize the request for 
* starting the bootloader.
* 
* @param[in] authorize_callback Function pointer to the authorization callback 
*   function.
*/
void bootloader_authorize_callback_set(bootloader_authorize_cb_t authorize_callback);

#endif /* BOOTLOADER_APP_H__ */
