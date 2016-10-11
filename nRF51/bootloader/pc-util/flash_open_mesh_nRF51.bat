SETLOCAL
REM @echo on

REM parameter 1 = nRF51/bootloader folder
SET bootloaderfolder=%~f1

REM parameter 2 = folder with the S110 v8.0.0 softdevice
SET softdevicefolder=%~f2

REM Parameter 3 = Segger ID of the gateway nRF5x board
SET seggerid=%3

nrfjprog -s %seggerid% --eraseall --family NRF51

cd %softdevicefolder%

nrfjprog -s %seggerid% --program s110_nrf51_8.0.0_softdevice.hex --family NRF51 

cd %bootloaderfolder%

nrfjprog -s %seggerid% --program .\bin\bootloader_serial_xxAC.hex --family NRF51 


del .\pc-util\example.hex
.\pc-util\device_page .\pc-util\example --nrf51
nrfjprog -s %seggerid% --program .\pc-util\example.hex --family NRF51


nrfjprog -s %seggerid% --program ..\examples\BLE_Gateway\bin\rbc_gateway_example_serial.hex --family NRF51


..\..\..\pc-nrfutil\nrfutil.exe dfu genpkg --application test\nrf51422_xxac_PCA10028_S110_Blinky_large.hex --company-id 0x00000059 --application-id 1 --application-version 2 --sd-req 0x0064 --mesh dfu_test.zip



REM Reset the nRF5x to get it ready for DFU
nrfjprog -s %seggerid% --reset --family NRF51

