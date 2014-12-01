#include "rbc_mesh.h"

#include "mesh_aci.h"

#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <stdio.h>
  

/**
* @brief Softdevice crash handler, resets chip
* 
* @param[in] pc Program counter at which the assert failed
* @param[in] line_num Line where the error check failed 
* @param[in] p_file_name File where the error check failed
*/
void sd_assert_handler(uint32_t pc, uint16_t line_num, const uint8_t* p_file_name)
{
    NVIC_SystemReset();
}

/**
* @brief App error handle callback. Called whenever an APP_ERROR_CHECK() fails.
* 
* @param[in] error_code The error code sent to APP_ERROR_CHECK()
* @param[in] line_num Line where the error check failed 
* @param[in] p_file_name File where the error check failed
*/
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name)
{
    NVIC_SystemReset();
}

/**
* @brief Hardfault handler, resets chip
*/
void HardFault_Handler(void)
{
    NVIC_SystemReset();
}

/**
* @brief Softdevice event handler, forwards SD events to rbc_mesh framework 
*/
void SD_EVT_IRQHandler(void)
{
    rbc_mesh_sd_irq_handler();
}

/**
* @brief RBC_MESH framework event handler. Defined in rbc_mesh.h. Handles
*   events coming from the mesh. 
*
* @param[in] evt RBC event propagated from framework
*/
void rbc_mesh_event_handler(rbc_mesh_event_t* evt)
{
    switch (evt->event_type)
    {
        case RBC_MESH_EVENT_TYPE_CONFLICTING_VAL:
        case RBC_MESH_EVENT_TYPE_NEW_VAL:
        case RBC_MESH_EVENT_TYPE_UPDATE_VAL:
            break;
    }
}


int main(void)
{
    uint32_t error_code = 
        sd_softdevice_enable(NRF_CLOCK_LFCLKSRC_XTAL_75_PPM, sd_assert_handler);
    
    ble_enable_params_t ble_enable_params;
    ble_enable_params.gatts_enable_params.service_changed = 0;
    
    error_code = sd_ble_enable(&ble_enable_params);
    APP_ERROR_CHECK(error_code);
    
    error_code = rbc_mesh_init(0xA541A68F, 38, 1, 100);
    APP_ERROR_CHECK(error_code);
    
    mesh_aci_init();
    
    while (true)
    {
        mesh_aci_loop();
    }
}

