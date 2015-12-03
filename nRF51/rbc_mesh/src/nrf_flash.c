/* Copyright (c) 2014 Nordic Semiconductor. All Rights Reserved.
 *
 * The information contained herein is property of Nordic Semiconductor ASA.
 * Terms and conditions of usage are described in detail in NORDIC
 * SEMICONDUCTOR STANDARD SOFTWARE LICENSE AGREEMENT.
 *
 * Licensees are granted free, non-transferable use of the information. NO
 * WARRANTY of ANY KIND is provided. This heading must NOT be removed from
 * the file.
 *
 */

#include <stdint.h>
#include "nrf_flash.h"
#include "nrf.h"

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
