SETLOCAL
REM @echo off
REM parameter 1 = nRF51/bootloader folder
SET bootloaderfolder=%~f1
REM parameter 2 = folder with the S110 v8.0.0 softdevice
SET softdevicefolder=%~f2
nrfjprog --eraseall --family NRF52
cd %softdevicefolder%
REM nrfjprog --program s111_nrf51_8.0.0_softdevice.hex
nrfjprog --program s132_nrf52_3.0.0_softdevice.hex --family NRF52 
cd %bootloaderfolder%
REM  nrfjprog --program .\bin\bootloader_serial_xxAC_new_3.hex
nrfjprog --program "C:\Users\anba\nordic sdk\nRF5_SDK_12.0.0_12f24da\examples\nRF51-ble-broadcast-mesh-private\nRF51\bootloader\arm\_build_serial\bootloader_serial_xxAA.hex" --family NRF52 
del .\pc-util\example52.hex
.\pc-util\device_page .\pc-util\example52
nrfjprog --program .\pc-util\example52.hex --family NRF52  
REM nrfjprog --program .\test\rbc_gateway_example_serial_trsn_21_sep_16_2.hex
nrfjprog --program "C:\Users\anba\nordic sdk\nRF5_SDK_12.0.0_12f24da\examples\nRF51-ble-broadcast-mesh-private\nRF51\examples\BLE_Gateway\arm\_build\rbc_gateway_example_52.hex"
nrfjprog --reset --family NRF52  
C:\Users\anba\Desktop\pc-nrfutil\nrfutil.exe dfu genpkg --application test/nrf52832_xxaa.hex --company-id 0x00000059 --application-id 1 --application-version 2 --sd-req 0x0084 --mesh dfu_test.zip