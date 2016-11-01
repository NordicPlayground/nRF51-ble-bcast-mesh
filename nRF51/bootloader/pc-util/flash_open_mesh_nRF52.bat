SETLOCAL
REM @echo on


SET seggerid=%~1

nrfjprog -s %seggerid% --eraseall --family NRF52

nrfjprog -s %seggerid% --program ..\softdevices_nRF51_nRF52\s132_nrf52_3.0.0\s132_nrf52_3.0.0_softdevice.hex --family NRF52 

nrfjprog -s %seggerid% --program .\bin\bootloader_serial_nrf52_xxAA.hex --family NRF52 

del .\pc-util\example52.hex
.\pc-util\device_page .\pc-util\example52 --nrf52
nrfjprog -s %seggerid% --program .\pc-util\example52.hex --family NRF52

nrfjprog -s %seggerid% --program ..\examples\BLE_Gateway\bin\rbc_gateway_example_serial_nRF52832_xxAA.hex --family NRF52



.\pc-util\nrfutil.exe dfu genpkg --application ..\examples\BLE_Gateway\bin\rbc_gateway_example_serial_blinky_PCA10040_nRF52832_xxAA.hex --company-id 0x00000059 --application-id 1 --application-version 2 --sd-req 0x0084 --mesh dfu_test.zip



REM Reset the nRF5x to get it ready for DFU
nrfjprog -s %seggerid% --reset --family NRF52

