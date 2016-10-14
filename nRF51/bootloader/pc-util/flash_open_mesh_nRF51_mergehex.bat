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
SET softdevice=%softdevicefolder%\s110_nrf51_8.0.0_softdevice.hex


cd %bootloaderfolder%
SET bootloader=.\bin\bootloader_serial_xxAC.hex 


del .\pc-util\example.hex
.\pc-util\device_page .\pc-util\example --nrf51
SET device_page=.\pc-util\example.hex



SET App=.\test\nrf51422_xxac_PCA10031_S110_Blinky.hex


mergehex -m  %softdevice% %bootloader% %device_page% -o .\test\softdevice_bootloader_serial_device_page.hex
mergehex -m  .\test\softdevice_bootloader_serial_device_page.hex %App% -o .\test\softdevice_bootloader_serial_device_page_app.hex
nrfjprog -s %seggerid% --program .\test\softdevice_bootloader_serial_device_page_app.hex --family NRF51


REM ..\..\..\pc-nrfutil\nrfutil.exe dfu genpkg --application test\nrf51422_xxac_PCA10031_S110_Blinky.hex --company-id 0x00000059 --application-id 1 --application-version 2 --sd-req 0x0064 --mesh dfu_test.zip

REM C:\python27\Scripts\nrfutil dfu genpkg --application test\nrf51422_xxac_PCA10031_S110_Blinky.hex --company-id 0x00000059 --application-id 1 --application-version 2 --sd-req 0x0064 --mesh dfu_test.zip


REM Reset the nRF5x to get it ready for DFU
nrfjprog -s %seggerid% --reset --family NRF51

