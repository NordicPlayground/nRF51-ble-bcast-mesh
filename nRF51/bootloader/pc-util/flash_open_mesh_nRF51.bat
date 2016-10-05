SETLOCAL
REM @echo off
REM parameter 1 = nRF51/bootloader folder
SET bootloaderfolder=%~f1
REM parameter 2 = folder with the S110 v8.0.0 softdevice
SET softdevicefolder=%~f2
nrfjprog --eraseall --family NRF51
cd %softdevicefolder%
REM nrfjprog --program s111_nrf51_8.0.0_softdevice.hex
nrfjprog --program s110_nrf51_8.0.0_softdevice.hex --family NRF51 
cd %bootloaderfolder%
REM  nrfjprog --program .\bin\bootloader_serial_xxAC_new_3.hex
nrfjprog --program "C:\Users\anba\nordic sdk\nRF51_SDK_8.1.0_b6ed55f\examples\nRF51-ble-broadcast-mesh-private\nRF51\bootloader\arm\_build_serial\bootloader_serial_xxAC.hex" --family NRF51 
del .\pc-util\example.hex
.\pc-util\device_page .\pc-util\example
nrfjprog --program .\pc-util\example.hex --family NRF51  
REM nrfjprog --program .\test\rbc_gateway_example_serial_trsn_21_sep_16_2.hex
nrfjprog --program "C:\Users\anba\nordic sdk\nRF51_SDK_8.1.0_b6ed55f\examples\nRF51-ble-broadcast-mesh-private\nRF51\examples\BLE_Gateway\arm\_build\rbc_gateway_example_serial.hex"
nrfjprog --reset --family NRF51  
C:\Users\anba\Desktop\pc-nrfutil\nrfutil.exe dfu genpkg --application test/nrf51422_xxac_PCA10031_S110_Blinky.hex --company-id 0x00000059 --application-id 1 --application-version 2 --sd-req 0x0064 --mesh dfu_test.zip