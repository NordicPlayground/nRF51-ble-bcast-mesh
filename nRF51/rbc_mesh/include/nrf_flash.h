#ifndef NRF_FLASH_H__
#define NRF_FLASH_H__

#include <stdint.h>

#define NRF_FLASH_PAGE_SIZE    ((uint16_t)NRF_FICR->CODEPAGESIZE)                          /**< Size of one flash page. */

/** @brief Function for erasing a page in flash.
 *
 * @param page_address Address of the first word in the page to be erased.
 */
void nrf_flash_erase(uint32_t * page_address, uint32_t size);

/** @brief Function for filling a page in flash with a value.
 *
 * @param[in] address Address of the first word in the page to be filled.
 * @param[in] value Value to be written to flash.
 */
void nrf_flash_store(uint32_t * p_dest, uint8_t * p_src, uint32_t size, uint32_t offset);

#endif //NRF_FLASH_H__

/** @} */
