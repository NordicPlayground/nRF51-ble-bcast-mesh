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

#include "rbc_mesh.h"
#include "cmd_if.h"
#include "timeslot_handler.h"

#include "softdevice_handler.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "SEGGER_RTT.h"
#include "boards.h"
#include "utils.h"
#include "nrf_delay.h"


#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#define MESH_ACCESS_ADDR        (RBC_MESH_ACCESS_ADDRESS_BLE_ADV)
#define MESH_INTERVAL_MIN_MS    (400)
#define MESH_CHANNEL            (38)
#define MESH_CLOCK_SRC          (NRF_CLOCK_LFCLKSRC_XTAL_75_PPM)

#define INVALID_HANDLE          (RBC_MESH_INVALID_HANDLE)

#define max_links 100

volatile uint16_t gs_handle   __attribute__((at(0x0001FC04))) __attribute__((used)) ;
volatile uint16_t gs_state  __attribute__((at(0x0001FC00))) __attribute__((used)) ;

static rbc_mesh_value_handle_t m_handle = INVALID_HANDLE;
static uint8_t m_data[RBC_MESH_VALUE_MAX_LEN];
static uint16_t throughput[max_links];
extern void UART0_IRQHandler(void);


const uint8_t leds_list[LEDS_NUMBER] = LEDS_LIST;


void blink(uint8_t LED,uint8_t number)
{
  for(int i=0; i < number; i++)
  {
      nrf_gpio_pin_set(21);
			nrf_gpio_pin_set(22);
			nrf_gpio_pin_set(23);
			nrf_gpio_pin_clear(LED);
      //nrf_delay_ms(100);
      //nrf_gpio_pin_set(LED);
			//nrf_delay_ms(100);
  }
}

 
/**
* @brief Handle an incoming command, and act accordingly.
*/
static bool cmd_rx(char* cmd, uint32_t len)
{

	int j;

	//___LOG(0,"cmd_rx:%s",&cmd);

	if (*cmd == 0x53) //"S" for Start
	{
		if (m_handle == 0)
		{
			for (int i=0;i<max_links;i++)
			{
				throughput[i] = 0;
			}
			__LOG(0,"throughput clr \r\n");
		}
		__LOG(0,"%d is STARTING\r\n", (int) m_handle);
		return true;

	}
	else if (*cmd == 0x54) //"T" for sTop
	{
		__LOG(0,"%d is STOPPING\r\n", (int) m_handle);
		if (m_handle == 0)
		{
			
			//__LOG(0,"d%2xp%4x\r\n", 0, (int)throughput[0]);
			for (j=max_links-1;j>0;j--)
			{
					if (throughput[j] != 0)
					{
							break;
					}
			}

			__LOG(0,"data_start,");
			for (int i=0;i<j;i++)
			{
				__LOG(0,"%5x,",(int)throughput[i]); //
			}
			__LOG(0,"data_end\r\n");
			return true;
		}
	}

	else if (*cmd == 0x50) //"P" for Ping
	{
		__LOG(0,"%d is PINGING\r\n", (int) m_handle);
		return true;
	}
	
	else if (*cmd == 0x48) //"H" for How Many
	{
		__LOG(0,"%d RECEIVED %d \r\n", (int) m_handle, (int) throughput[m_handle]);
		return true;
	}

	return false;
}
/**
* @brief General error handler.
*/
static void error_loop(void)
{
    nrf_gpio_pin_clear(LED_START + 1);
    nrf_gpio_pin_set(LED_START + 2);
    while (1)
    {
        UART0_IRQHandler();
    }
}

/**
* @brief Softdevice crash handler, never returns
* 
* @param[in] pc Program counter at which the assert failed
* @param[in] line_num Line where the error check failed 
* @param[in] p_file_name File where the error check failed
*/
void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{
    __LOG(0,"SD ERROR: %s:L%d\r\n", (const char*) p_file_name, (int) line_num);
    error_loop();
}

/**
* @brief App error handle callback. Called whenever an APP_ERROR_CHECK() fails.
*   Never returns.
* 
* @param[in] error_code The error code sent to APP_ERROR_CHECK()
* @param[in] line_num Line where the error check failed 
* @param[in] p_file_name File where the error check failed
*/
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    __LOG(0,"APP ERROR: %s:L%d - E:%X\r\n", p_file_name, (int) line_num, (int) error_code);
    error_loop();
}

void HardFault_Handler(void)
{
    __LOG(0,"HARDFAULT\r\n");
    error_loop();
}

int rtt_send(char const *p_buf, unsigned buf_size)
{
  return SEGGER_RTT_Write(0, p_buf, buf_size);
}

int rtt_read(char *p_buf, unsigned buf_size)
{
  return SEGGER_RTT_Read(0, p_buf, buf_size);
}

/**
* @brief RBC_MESH framework event handler. Handles events coming from the mesh. 
*
* @param[in] evt RBC event propagated from framework
*/
static void rbc_mesh_event_handler(rbc_mesh_event_t* evt)
{ 
    //static const char cmd[] = {'U', 'C', 'N', 'I', 'T'};
	  
		switch (evt->event_type)
		{
				case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
				case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
				case RBC_MESH_EVENT_TYPE_NEW_VAL:  
						if (evt->value_handle == m_handle || m_handle == 0)
						{
								nrf_gpio_pin_toggle(LED_RGB_RED);
								if (m_handle == 0)
								{
										if ( ((uint8_t) evt->value_handle) < max_links)
										{
											rbc_mesh_value_set(evt->value_handle, m_data, 1); /* short ack */
											throughput[(uint8_t)evt->value_handle]++;
											throughput[0]++;
										}
										
								}
								else
								{
										m_data[6]++;
										nrf_gpio_pin_set(LED_RGB_GREEN);	
										rbc_mesh_value_set(evt->value_handle, m_data, RBC_MESH_VALUE_MAX_LEN);
								}
						}
						else
						{
								//rbc_mesh_value_disable(evt->value_handle);
						}
						break;
				case RBC_MESH_EVENT_TYPE_INITIALIZED: break;
				case RBC_MESH_EVENT_TYPE_TX: break;
		}
 
}


/**
* @brief RTT_RESPOND    read rtt incoming data and respond with RTT
*
*/
int RTT_respond(void)
{
	char send[128];
  char read[128];
  uint16_t read_len = 0;
	while (true)
	{
		read_len = rtt_read(read,128);
		nrf_gpio_pin_toggle(23);
		if (read_len!=0)
		{
			if (cmd_rx(&read[0], read_len))
				 continue;
			else
				 rtt_send(send, 18);
		}
		else
		{
			return true;
		}
	}
}



/** @brief main function */
int main(void)
{   
		bool start=true;
		nrf_gpio_range_cfg_output(0, 32);
    for (uint32_t i = LED_START; i <= LED_STOP; ++i)
    {
        nrf_gpio_pin_set(i);
    }
				
    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigDownBuffer(0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
    SEGGER_RTT_ConfigUpBuffer  (0, NULL, NULL, 0, SEGGER_RTT_MODE_BLOCK_IF_FIFO_FULL);
		 /*	
		uint32_t time_ms = 500; //Time(in miliseconds) between consecutive compare events.
    uint32_t time_ticks;
    uint32_t err_code = NRF_SUCCESS;
		
		err_code = nrf_drv_timer_init(&TIMER_LED, NULL, timer_led_event_handler);
    APP_ERROR_CHECK(err_code);
    
    time_ticks = nrf_drv_timer_ms_to_ticks(&TIMER_LED, time_ms);
    
    nrf_drv_timer_extended_compare(
         &TIMER_LED, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
    
    nrf_drv_timer_enable(&TIMER_LED);

    while(1)
    {
        __WFI();
    }
		
 	
		while (true)
		{
			for (int i=0;i<max_links;i++)
			{
				__LOG(0,"%5x,Z",i); //(int)throughput[i]
			}
		}
	
  
	
    __LOG(0,(char *) &start_msg[0]);
    __LOG(0,"Program init\r\n", __FUNCTION__);
    */
    
   
    /*
		uint32_t num=0;
    while (1)
    {
      num++;
      sprintf(&send[0], "W%7d\r\n", (int)num);
      rtt_send(send,10);
    
			read_len = rtt_read(read,128);
      if (read_len!=0)
      {
        rtt_send(read, read_len);
      }
		
      nrf_delay_ms(10);
    }
      */
		
		
		/* Enable Softdevice (including sd_ble before framework) */
    SOFTDEVICE_HANDLER_INIT(MESH_CLOCK_SRC, NULL);
    softdevice_ble_evt_handler_set(rbc_mesh_ble_evt_handler);
    softdevice_sys_evt_handler_set(rbc_mesh_sd_evt_handler);
  	
  	/* Init the rbc_mesh */
    rbc_mesh_init_params_t init_params;

    init_params.access_addr     = MESH_ACCESS_ADDR;
    init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
    init_params.channel         = MESH_CHANNEL;
    init_params.lfclksrc        = MESH_CLOCK_SRC;
    
    uint32_t error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);
    
    ble_gap_addr_t addr;
    sd_ble_gap_address_get(&addr);
    memcpy(m_data, addr.addr, 6);

    
		

    
	  //cmd_init(cmd_rx);
    
		/*		
		while(gs_release == 0xFFFF)
		{
			nrf_delay_ms(1000);
			blink(LED_RGB_RED,1);
			sprintf(&send[0], "RESET\r\n");
			rtt_send(send,7);
		}
		
		while(true)
		{
			nrf_delay_ms(1000);
			if (gs_handle == 0x0000)
			{
				blink(LED_RGB_GREEN,1);
				sprintf(&send[0], "CENTRAL\r\n");
				rtt_send(send,9);
			}
			else 
			{
				blink(LED_RGB_BLUE,1);
				sprintf(&send[0], "NOT CENTRAL\r\n");
				rtt_send(send,13);
			}
		}
	*/
	
    rbc_mesh_event_t evt;
    while (true)
    {
			if (gs_state == 0xFFFF)
			{
				nrf_gpio_pin_clear(LED_RGB_BLUE);
				RTT_respond();
				m_handle = (rbc_mesh_value_handle_t) gs_handle;
			}	
			if (gs_state == 0x0FFF)
			{
				if  (start == true)
				{
					if (m_handle == 0)
					{
						for (int i=0;i<max_links;i++)
						{
							throughput[i] = 0;
						}
					}
					else
					{
						m_data[6]++;
						rbc_mesh_value_set(m_handle, m_data, RBC_MESH_VALUE_MAX_LEN);
						rbc_mesh_persistence_set(m_handle, true);
						nrf_gpio_pin_set(LED_RGB_BLUE);
						nrf_gpio_pin_clear(LED_RGB_GREEN);
					}
					start = false;
					nrf_gpio_pin_set(LED_RGB_BLUE);
					nrf_gpio_pin_clear(LED_RGB_GREEN);	
				}
				
				sd_app_evt_wait();
			
				if (rbc_mesh_event_get(&evt) == NRF_SUCCESS)
				{
						rbc_mesh_event_handler(&evt);
						rbc_mesh_packet_release(evt.data);
				}
			}
			if (gs_state == 0x00FF)
			{
				RTT_respond();
			}
			if (gs_state == 0x000F)
			{
			}	
			if (gs_state == 0x0000)
			{
			}	
		}
}

