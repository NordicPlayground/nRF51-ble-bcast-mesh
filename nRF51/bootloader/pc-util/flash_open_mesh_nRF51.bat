SETLOCAL
REM @echo on



SET seggerid=%~1
 
nrfjprog -s %seggerid% --eraseall --family NRF51

nrfjprog -s %seggerid% --program ..\softdevices_nRF51_nRF52\s110_nrf51_8.0.0\s110_nrf51_8.0.0_softdevice.hex --family NRF51 

nrfjprog -s %seggerid% --program .\bin\bootloader_serial_xxAC.hex --family NRF51 

del .\pc-util\example.hex
.\pc-util\device_page .\pc-util\example --nrf51
nrfjprog -s %seggerid% --program .\pc-util\example.hex --family NRF51

nrfjprog -s %seggerid% --program ..\examples\BLE_Gateway\bin\rbc_gateway_example_serial_nRF51422_xxAC.hex --family NRF51


.\pc-util\nrfutil.exe dfu genpkg --application ..\examples\BLE_Gateway\bin\rbc_gateway_example_serial_blinky_PCA10031_nRF51422_xxAC.hex --company-id 0x00000059 --application-id 1 --application-version 2 --sd-req 0x0064 --mesh dfu_test.zip



REM Reset the nRF5x to get it ready for DFU
nrfjprog -s %seggerid% --reset --family NRF51

