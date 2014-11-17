#include "led_config.h"
#include "nrf_soc.h"
/**
* @brief configure LEDs for easily visible status check
*/
void led_config(uint8_t led, uint8_t conf)
{
#ifdef BOARD_PCA10000    
  if (!conf)
  {
    NRF_GPIO->OUTSET = (1 << (led - 1 + LED_0));
  }
  else
  {
    NRF_GPIO->OUTCLR = (1 << (led - 1 + LED_0));
  }
#endif
#ifdef BOARD_PCA10001
  if (conf)
  {
    NRF_GPIO->OUTSET = (1 << (led - 1 + LED_0));
  }
  else
  {
    NRF_GPIO->OUTCLR = (1 << (led - 1 + LED_0));
  }
#endif
} 

