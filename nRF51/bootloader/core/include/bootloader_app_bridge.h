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

#ifndef BOOTLOADER_APP_BRIDGE_H__
#define BOOTLOADER_APP_BRIDGE_H__

#include <stdint.h>
#include "bl_if.h"

#ifndef APP_ERROR_CHECK
#ifdef DEBUG

#define APP_ERROR_CHECK_BOOL(expr) do {\
    if (!(expr))\
    {\
        bootloader_error_post(0, __FILE__, __LINE__);\
    }\
} while(0)
#define APP_ERROR_CHECK(error) do {\
    if (error != NRF_SUCCESS)\
    {\
        bootloader_error_post(error, __FILE__, __LINE__);\
    }\
} while(0)
#else
#define APP_ERROR_CHECK_BOOL(expr) do {\
    if (!(expr))\
    {\
        bootloader_error_post(0, NULL, 0);\
    }\
} while(0)
#define APP_ERROR_CHECK(error) do {\
    if (error != NRF_SUCCESS)\
    {\
        bootloader_error_post(error, NULL, 0);\
    }\
} while(0)
#endif
#endif

uint32_t bootloader_app_bridge_init(void);
uint32_t bootloader_evt_send(bl_evt_t* p_evt);
uint32_t bl_cmd_handler(bl_cmd_t* p_bl_cmd);
uint32_t flash_write(void* p_dest, void* p_data, uint32_t length);
uint32_t flash_erase(void* p_dest, uint32_t length);
uint32_t bootloader_error_post(uint32_t error, const char* file, uint32_t line);

//uint32_t timer_set(uint32_t delay_us);
//uint32_t timer_abort(void);

uint32_t tx_abort(uint8_t slot);
void send_end_evt(dfu_end_t end_reason);
bool bootloader_is_in_application(void);

#endif /* BOOTLOADER_APP_BRIDGE_H__ */

