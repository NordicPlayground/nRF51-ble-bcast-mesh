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

#include "rbc_mesh.h"
#include "nrf_adv_conn.h"
#include "led_config.h"
#include "mesh_aci.h"

#include "softdevice_handler.h"
#include "app_error.h"
#include "nrf_gpio.h"
#include "boards.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>

#include "SEGGER_RTT.h"

#if defined(WITH_ACK_SLAVE)||defined(WITHOUT_ACK_SLAVE)
#include "handle.h"
#endif

/* Debug macros for debugging with logic analyzer */
#define SET_PIN(x) NRF_GPIO->OUTSET = (1 << (x))
#define CLEAR_PIN(x) NRF_GPIO->OUTCLR = (1 << (x))
#define TICK_PIN(x) do { SET_PIN((x)); CLEAR_PIN((x)); }while(0)

#define MESH_ACCESS_ADDR        (RBC_MESH_ACCESS_ADDRESS_BLE_ADV)
#define MESH_INTERVAL_MIN_MS    (100)
#define MESH_CHANNEL            (38)
#define MESH_CLOCK_SOURCE       (NRF_CLOCK_LFCLKSRC_XTAL_75_PPM)


#if defined(WITH_ACK_MASTER) || defined (WITHOUT_ACK_MASTER)

#define NODE_NUMBER  105
#define SLAVE_NUMBER (NODE_NUMBER-1)

static char UpBuffer0[BUFFER_SIZE_UP];
static uint8_t packet_array [300] ;
static uint16_t packet_index = 0 ;
static uint32_t packet_count[NODE_NUMBER] __attribute__((at(0x20004688)));
static uint8_t handle_array [NODE_NUMBER] ;
static uint8_t flag [NODE_NUMBER];
static uint8_t last_value[NODE_NUMBER];
static uint8_t present_value [NODE_NUMBER];
static uint32_t global_packet_count = 0;

#endif

#if defined(WITH_ACK_SLAVE)

static char UpBuffer0[BUFFER_SIZE_UP];
uint32_t node_handle;
static uint32_t total_tx_number __attribute__((at(0x20004688)));
static uint32_t packet_rcv __attribute__((at(0x2000468C))) ;
static uint8_t node_data [RBC_MESH_VALUE_MAX_LEN ] ;
static uint32_t tx_time=0;
static uint8_t control=0;
#endif

#if defined(WITHOUT_ACK_SLAVE)

static char UpBuffer0[BUFFER_SIZE_UP];
uint32_t node_handle ;
static uint8_t node_data [RBC_MESH_VALUE_MAX_LEN ] ;
static uint32_t packet_count __attribute__((at(0x20004688)));
static uint32_t packet_rcv __attribute__((at(0x2000468C))) ;

#endif



/**
* @brief General error handler.
*/
static void error_loop(void)
{
    led_config(2, 1);
    led_config(3, 0);
    while (true)
    {
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
    error_loop();
}

void HardFault_Handler(void)
{
    error_loop();
}

/**
* @brief Softdevice event handler
*/
void sd_ble_evt_handler(ble_evt_t* p_ble_evt)
{
    rbc_mesh_ble_evt_handler(p_ble_evt);
    nrf_adv_conn_evt_handler(p_ble_evt);
}
/**
* @brief RBC_MESH framework event handler. Defined in rbc_mesh.h. Handles
*   events coming from the mesh. Sets LEDs according to data
*
* @param[in] evt RBC event propagated from framework
*/
static void rbc_mesh_event_handler(rbc_mesh_event_t* p_evt)
{
    TICK_PIN(28);
    
    #if defined(WITH_ACK_MASTER) || defined (WITHOUT_ACK_MASTER)|| defined (WITH_ACK_SLAVE)||defined(WITHOUT_ACK_SLAVE)
    
	uint32_t error_code;
	char time_buffer[100];
	int index_count = 0;
    
    #endif
	
	
    switch (p_evt->type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
					
                  #if defined(WITH_ACK_MASTER)
				  error_code = rbc_mesh_value_set(p_evt->params.rx.value_handle,&p_evt->params.rx.p_data[0],1);
                  APP_ERROR_CHECK(error_code);
                  break;
                  #endif
                  
                  #if defined(WITHOUT_ACK_MASTER)
                  error_code =rbc_mesh_value_disable(p_evt->params.rx.value_handle);
                  APP_ERROR_CHECK(error_code);
                  break;
                  #endif
          
					
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
				  
                  #if defined(WITH_ACK_MASTER)	   				      
				  error_code = rbc_mesh_value_set(p_evt->params.rx.value_handle,&p_evt->params.rx.p_data[0],1);
                  APP_ERROR_CHECK(error_code);
                  break;
                  #endif
        
                  #if defined(WITHOUT_ACK_MASTER)
                  error_code =rbc_mesh_value_disable(p_evt->params.rx.value_handle); 
                  APP_ERROR_CHECK(error_code);        
                  #endif
        
                  #if defined (WITH_ACK_SLAVE)
        
                  packet_rcv= packet_rcv+1;
				  if(p_evt->params.rx.value_handle!= node_handle)
							   {
								    rbc_mesh_value_disable(p_evt->params.rx.value_handle);
							   }
				  else if (p_evt->params.rx.value_handle == node_handle)
							   {
									if (p_evt->params.rx.p_data[0] >= node_data[0] )
										 {										 
											 node_data[0] = p_evt->params.rx.p_data[0] + 1 ;	
											 error_code = rbc_mesh_value_set(node_handle,&node_data[0],RBC_MESH_VALUE_MAX_LEN);
											 APP_ERROR_CHECK(error_code);
											 error_code = rbc_mesh_tx_event_set(node_handle, true);
                                             APP_ERROR_CHECK(error_code);				 
											 index_count= snprintf (time_buffer,100, " hdl %d <%d> Rx <%d> rabout<%dus> \n",p_evt->params.rx.value_handle,p_evt->params.rx.timestamp_us,p_evt->params.rx.p_data[0],p_evt->params.rx.timestamp_us-tx_time );
											 control=0;
											 SEGGER_RTT_Write(0, time_buffer,index_count );
											 SEGGER_RTT_printf(0,"\n");
											 nrf_gpio_pin_toggle(LED_2);
											 
									     }
							   }
							
				  break;

                  #endif
        
                  #if defined (WITHOUT_ACK_SLAVE)
                     packet_rcv= packet_rcv+1;
				     if(p_evt->params.rx.value_handle!= node_handle)
					     {
						    rbc_mesh_value_disable(p_evt->params.rx.value_handle);
					     }
							
					break;         
                  #endif
        				   
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
					
                  #if defined(WITH_ACK_MASTER) || defined (WITHOUT_ACK_MASTER)	
        
				    if (p_evt->params.rx.value_handle == handle_array[p_evt->params.rx.value_handle])
						{
								if (flag [p_evt->params.rx.value_handle] == 0)
								 {
									 last_value  [p_evt->params.rx.value_handle] = p_evt->params.rx.p_data[0] ;
									 flag [p_evt->params.rx.value_handle] = 1;
								 }
					
								else if (flag [p_evt->params.rx.value_handle] == 1)
								 {
									 present_value [p_evt->params.rx.value_handle] = p_evt->params.rx.p_data[0] ;

								 }
								
								if ((present_value[p_evt->params.rx.value_handle] > last_value[p_evt->params.rx.value_handle])|| (present_value[p_evt->params.rx.value_handle] == 0))
									
								 { 
									 if (packet_index == 300 ) 
										    packet_index = 0 ;
									 
									 packet_array [packet_index] = last_value [p_evt->params.rx.value_handle] ;
									 packet_count[p_evt->params.rx.value_handle] ++ ;
									 global_packet_count ++;
									 packet_index ++ ;
									 
                                     #if defined(WITH_ACK_MASTER)
									 last_value[p_evt->params.rx.value_handle] = present_value[p_evt->params.rx.value_handle] ;
                                     #endif
                                     
                                     #if defined(WITHOUT_ACK_MASTER)
                                     last_value[p_evt->params.rx.value_handle] = p_evt->params.rx.p_data[0] ;
									 #endif
								 }
							}
						
					  #if defined(WITH_ACK_MASTER)	
				      error_code = rbc_mesh_value_set(p_evt->params.rx.value_handle,&p_evt->params.rx.p_data[0],1);
                      APP_ERROR_CHECK(error_code);
                      index_count= snprintf (time_buffer,150, " M: hdl %d <%d> Rx <%d> tx ver <%d> vdlta <%d>\n",p_evt->params.rx.value_handle,p_evt->params.rx.timestamp_us,p_evt->params.rx.p_data[0],p_evt->params.rx.p_data[0],p_evt->params.rx.version_delta); 
                      #endif
                            
                      #if defined(WITHOUT_ACK_MASTER)	
                      error_code =rbc_mesh_value_disable(p_evt->params.rx.value_handle); 
                      APP_ERROR_CHECK(error_code);
                      index_count= snprintf (time_buffer,50, " M:hdl %d <%d> Rx %d %d vdlta <%d> \n",p_evt->params.rx.value_handle,p_evt->params.rx.timestamp_us,p_evt->params.rx.p_data[0],packet_count[p_evt->params.rx.value_handle],p_evt->params.rx.version_delta);     
                      #endif
                            
                      SEGGER_RTT_Write(0, time_buffer,index_count );
					  nrf_gpio_pin_toggle(LED_1);
							
                      break;
                   #endif
                            
                   #if defined (WITH_ACK_SLAVE) 
                            
                      packet_rcv= packet_rcv+1;
				      if(p_evt->params.rx.value_handle!= node_handle)
							   {
								    rbc_mesh_value_disable(p_evt->params.rx.value_handle);
							   }
					  else if (p_evt->params.rx.value_handle == node_handle)
							   {
									if (p_evt->params.rx.p_data[0] >= node_data[0] )
										 {
											 node_data[0] = p_evt->params.rx.p_data[0] + 1 ;	
											 uint32_t error_code = rbc_mesh_value_set(node_handle,&node_data[0],RBC_MESH_VALUE_MAX_LEN);
											 APP_ERROR_CHECK(error_code);
											 error_code = rbc_mesh_tx_event_set(node_handle, true);
                                             APP_ERROR_CHECK(error_code);
                                             index_count= snprintf (time_buffer,100, " hdl %d <%d> Rx <%d> vdlta <%d> rabout<%dus> \n",p_evt->params.rx.value_handle,p_evt->params.rx.timestamp_us,p_evt->params.rx.p_data[0],p_evt->params.rx.version_delta,p_evt->params.rx.timestamp_us-tx_time );
											 control=0;
											 SEGGER_RTT_Write(0, time_buffer,index_count );
											 SEGGER_RTT_printf(0,"\n");
											 nrf_gpio_pin_toggle(LED_3);
											 
									    }
							   }
            
                     break;                            
                   #endif   
  
                   #if defined (WITHOUT_ACK_SLAVE)
                      packet_rcv= packet_rcv+1;
				      if(p_evt->params.rx.value_handle!= node_handle)
					     {
						    rbc_mesh_value_disable(p_evt->params.rx.value_handle);
					     }
            
                     break;
                   #endif                               
                            
                            
        case RBC_MESH_EVENT_TYPE_TX:	
            
             #if defined (WITH_ACK_SLAVE)
              if (control==0)
					 {
						tx_time = p_evt->params.tx.timestamp_us ;
						control=1;
					 } 

              index_count= snprintf (time_buffer,100, " hdl %d <%d> Tx <%d> \n",p_evt->params.rx.value_handle,p_evt->params.tx.timestamp_us,node_data[0]);					
     		  SEGGER_RTT_Write(0, time_buffer,index_count );
              total_tx_number ++;
	          nrf_gpio_pin_toggle(LED_1);
              break;
             #endif
            #if defined (WITHOUT_ACK_SLAVE)
              if(p_evt->params.rx.value_handle!= node_handle)
				{
					rbc_mesh_value_disable(p_evt->params.rx.value_handle);
				}
			  packet_count=packet_count +1 ;	 
			  node_data[0] = node_data[0] + 1 ;	 
			  error_code = rbc_mesh_value_set(node_handle,&node_data[0],RBC_MESH_VALUE_MAX_LEN);
			  APP_ERROR_CHECK(error_code); 		  
              index_count= snprintf (time_buffer,50, " hdl %d <%d> Tx %d  %d\n",p_evt->params.tx.value_handle,p_evt->params.tx.timestamp_us,p_evt->params.tx.p_data[0],packet_count);
			  SEGGER_RTT_Write(0, time_buffer,index_count );			
              nrf_gpio_pin_toggle(LED_1);	 
	  		  break;       
            #endif
        case RBC_MESH_EVENT_TYPE_INITIALIZED:
            /* init BLE gateway softdevice application: */
            nrf_adv_conn_init();
            break;
    }
}


/**
* @brief Initialize GPIO pins, for LEDs and debugging
*/
void gpio_init(void)
{
    nrf_gpio_range_cfg_output(LED_START, LED_STOP);

    for (uint32_t i = 0; i < LEDS_NUMBER; ++i)
    {
        nrf_gpio_pin_set(LED_START + i);
    }

#if defined(BOARD_PCA10001) || defined(BOARD_PCA10028)
    nrf_gpio_range_cfg_output(0, 32);
#endif



}


void LFCK_clock_initialization()
{
    /* Start 32KHz crystal oscillator */
    NRF_CLOCK->EVENTS_LFCLKSTARTED = 0;
    NRF_CLOCK->TASKS_LFCLKSTART    = 1;

    /* Wait for the external oscillator to start up */
    while (NRF_CLOCK->EVENTS_LFCLKSTARTED == 0)
    {
     
    }		
}


/** @brief main function */
int main(void)
{
    /* init leds and pins */
    gpio_init();
    NRF_GPIO->OUTSET = (1 << 4);
    /* Enable Softdevice (including sd_ble before framework */
    SOFTDEVICE_HANDLER_INIT(MESH_CLOCK_SOURCE, NULL);
    softdevice_ble_evt_handler_set(sd_ble_evt_handler); /* app-defined event handler, as we need to send it to the nrf_adv_conn module and the rbc_mesh */
    softdevice_sys_evt_handler_set(rbc_mesh_sd_evt_handler);
    
    
    
    
    #if defined(WITH_ACK_MASTER) || defined (WITHOUT_ACK_MASTER)	
    
    for (uint8_t i=0;i<NODE_NUMBER;i++)
		{
		    handle_array[i]= i;
			flag[i]=0;
			last_value[i]=0;
			present_value[i]=0;
			packet_count[i] = 0;
		}
    #endif
        
    rbc_mesh_init_params_t init_params;

    init_params.access_addr = MESH_ACCESS_ADDR;
    init_params.interval_min_ms = MESH_INTERVAL_MIN_MS;
    init_params.channel = MESH_CHANNEL;
    init_params.lfclksrc = MESH_CLOCK_SOURCE;
    init_params.tx_power = RBC_MESH_TXPOWER_0dBm;
		
    uint32_t error_code = rbc_mesh_init(init_params);
    APP_ERROR_CHECK(error_code);

	
	ble_gap_addr_t my_addr_new;
    error_code = sd_ble_gap_address_get(&my_addr_new);
    APP_ERROR_CHECK(error_code);
	
    #if defined (WITH_ACK_SLAVE)
        
    total_tx_number=0;
    for (uint8_t i=0; i < RBC_MESH_VALUE_MAX_LEN ; i++)
		{
			node_data[i] = 1 ;  
		}
		
    error_code = rbc_mesh_value_set(node_handle,&node_data[0],(RBC_MESH_VALUE_MAX_LEN-1));
    APP_ERROR_CHECK(error_code);
    error_code = rbc_mesh_tx_event_set(node_handle, true);
    APP_ERROR_CHECK(error_code);	
    error_code =rbc_mesh_persistence_set(node_handle, true);
	APP_ERROR_CHECK(error_code);    
      
    #endif   

    #if defined (WITHOUT_ACK_SLAVE)  
    packet_count=0;
    for (uint8_t i=0; i < RBC_MESH_VALUE_MAX_LEN ; i++)
	  {
		node_data[i] = 1 ;  
	  }
		
    error_code = rbc_mesh_value_set(node_handle,&node_data[0],(RBC_MESH_VALUE_MAX_LEN-1));
	APP_ERROR_CHECK(error_code);	
    error_code = rbc_mesh_tx_event_set(node_handle,true);
    APP_ERROR_CHECK(error_code);	
	error_code = rbc_mesh_persistence_set(node_handle, true);
	APP_ERROR_CHECK(error_code);
    #endif
      
    #if defined(WITH_ACK_MASTER) || defined (WITHOUT_ACK_MASTER)|| defined (WITH_ACK_SLAVE)||defined(WITHOUT_ACK_SLAVE)    
    SEGGER_RTT_Init();
    SEGGER_RTT_ConfigUpBuffer(0, "UpBuffer0", UpBuffer0, BUFFER_SIZE_UP, SEGGER_RTT_MODE_NO_BLOCK_SKIP);
    #else
    /* init BLE gateway softdevice application: */
    nrf_adv_conn_init();
    #endif 

    /* fetch events */
    rbc_mesh_event_t evt;
    while (true)
    {
        if (rbc_mesh_event_get(&evt) == NRF_SUCCESS)
        {
            rbc_mesh_event_handler(&evt);
            rbc_mesh_event_release(&evt);
        }

        sd_app_evt_wait();
    }

}







