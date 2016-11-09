SETLOCAL
REM @echo on

REM assign the segger id 
SET seggerid=%~1
 
REM Erase the nRF52  
nrfjprog -s %seggerid% --eraseall --family NRF52

REM Program nRF52 with softdevice hex file
nrfjprog -s %seggerid% --program ..\softdevices\s132_nrf52_3.0.0\s132_nrf52_3.0.0_softdevice.hex --family NRF52 

REM Program nRF52 with bootloader hex file
nrfjprog -s %seggerid% --program .\bin\bootloader_serial_nrf52_xxAA.hex --family NRF52 

REM Generate a fresh device page for nRF52
del .\pc-util\example52.hex
.\pc-util\device_page .\pc-util\example52 --nrf52

REM Program nRF52 with device page
nrfjprog -s %seggerid% --program .\pc-util\example52.hex --family NRF52

REM Program nRF52 with application hex
nrfjprog -s %seggerid% --program ..\examples\BLE_Gateway\bin\rbc_gateway_example_serial_nRF52832_xxAA.hex --family NRF52

REM Generate the DFU package
SET new_application=..\examples\BLE_Gateway\bin\rbc_gateway_example_serial_blinky_PCA10040_nRF52832_xxAA.hex
.\pc-util\nrfutil.exe dfu genpkg --application %new_application% --company-id 0x00000059 --application-id 1 --application-version 2 --sd-req 0x0084 --mesh dfu_test.zip

REM Reset the nRF52 to get it ready for DFU
nrfjprog -s %seggerid% --reset --family NRF52

