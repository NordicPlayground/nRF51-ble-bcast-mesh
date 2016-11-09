SETLOCAL
REM @echo on


REM assign the segger id 
SET seggerid=%~1

REM Erase the nRF51  
nrfjprog -s %seggerid% --eraseall --family NRF51

REM Program nRF51 with softdevice hex file
nrfjprog -s %seggerid% --program ..\softdevices\s110_nrf51_8.0.0\s110_nrf51_8.0.0_softdevice.hex --family NRF51 

REM Program nRF51 with bootloader hex file
nrfjprog -s %seggerid% --program .\bin\bootloader_serial_xxAC.hex --family NRF51 

REM Generate a fresh device page for nRF51
del .\pc-util\example.hex
.\pc-util\device_page .\pc-util\example --nrf51
s
REM Program nRF51 with device page
nrfjprog -s %seggerid% --program .\pc-util\example.hex --family NRF51

REM Program nRF51 with application hex
nrfjprog -s %seggerid% --program ..\examples\BLE_Gateway\bin\rbc_gateway_example_serial_nRF51422_xxAC.hex --family NRF51

REM Generate the DFU package
SET new_application=..\examples\BLE_Gateway\bin\rbc_gateway_example_serial_blinky_PCA10031_nRF51422_xxAC.hex

.\pc-util\nrfutil.exe dfu genpkg --application %new_application% --company-id 0x00000059 --application-id 1 --application-version 2 --sd-req 0x0064 --mesh dfu_test.zip

REM Reset the nRF5x to get it ready for DFU
nrfjprog -s %seggerid% --reset --family NRF51

