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

#include <stdint.h>
#include "nrf_flash.h"
#ifdef NRF51
#include "nrf.h"
#include "nrf51_bitfields.h"
#else
#include "nrf.h"
#endif

/** @brief Function for erasing a page in flash.
 *
 * @param page_address Address of the first word in the page to be erased.
 */
void nrf_flash_erase(uint32_t * page_address, uint32_t size)
{
    uint8_t num_pages = (size + NRF_FLASH_PAGE_SIZE - 1) / NRF_FLASH_PAGE_SIZE;

    for(uint8_t  i = 0; i < num_pages ; i++)
    {
        // Turn on flash erase enable and wait until the NVMC is ready:
        NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Een << NVMC_CONFIG_WEN_Pos);

        while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
        {
            // Do nothing.
        }

        // Erase page:
        NRF_NVMC->ERASEPAGE = (uint32_t)page_address;

        while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
        {
        // Do nothing.
        }

        // Turn off flash erase enable and wait until the NVMC is ready:
        NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

        while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
        {
            // Do nothing.
        }
        page_address += NRF_FLASH_PAGE_SIZE / sizeof(uint32_t *);
    }
}


/** @brief Function for filling a page in flash with a value.
 *
 * @param[in] address Address of the first word in the page to be filled.
 * @param[in] value Value to be written to flash.
 */
void nrf_flash_store(uint32_t * p_dest, uint8_t * p_src, uint32_t size, uint32_t offset)
{
    static uint8_t buffer[4]  __attribute__((aligned (4)));
    uint8_t cnt = 0;
    uint8_t has_content = 0;

    p_dest += offset / 4;

    for(uint32_t i=0; i < size ; i++)
    {
        has_content |= ~(*p_src);
        buffer[cnt++]  = *p_src;

        p_src++;

        if(cnt == 4)
        {
            cnt = 0;
            if (has_content)
            {
                // Turn on flash write enable and wait until the NVMC is ready:
                NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Wen << NVMC_CONFIG_WEN_Pos);

                while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
                {
                // Do nothing.
                }

                *p_dest = *(uint32_t *)buffer;

                while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
                {
                    // Do nothing.
                }

                // Turn off flash write enable and wait until the NVMC is ready:
                NRF_NVMC->CONFIG = (NVMC_CONFIG_WEN_Ren << NVMC_CONFIG_WEN_Pos);

                while (NRF_NVMC->READY == NVMC_READY_READY_Busy)
                {
                    // Do nothing.
                }
            }
            has_content = 0;
            p_dest++;
        }
    }
}
