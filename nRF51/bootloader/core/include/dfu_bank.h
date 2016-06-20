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

#ifndef DFU_BANK_H__
#define DFU_BANK_H__

#include "dfu_types_mesh.h"
#include "bl_if.h"

/**
 * Look for available banks, finalize any ongoing flash operations.
 *
 * @param[out] p_bank_flash_started Set by the function. Indicates whether a
 * bank flash was started as a result of the call.
 */
void dfu_bank_scan(bool* p_bank_flash_started);

/**
 * Flash the bank of the given type.
 * If called from the application, this will trigger a restart of the chip. Any
 * data stored in non-volatile memory will be erased.
 *
 * @param[in] dfu_type The type of bank to flash.
 *
 * @return NRF_SUCCESS The given bank was successfully flashed.
 * @return NRF_ERROR_INVALID_PARAM The given DFU type is not available.
 * @return NRF_ERROR_INVALID_STATE A bank flash is already underway.
 * @return NRF_ERROR_NOT_FOUND There are no banks of the given type available.
 */
uint32_t dfu_bank_flash(dfu_type_t dfu_type);

/** Check whether a bank of the given type exists. */
bool dfu_bank_is_available(dfu_type_t dfu_type);

void dfu_bank_on_flash_idle(void);

bool dfu_bank_transfer_in_progress(void);

#endif /* DFU_BANK_H__ */

