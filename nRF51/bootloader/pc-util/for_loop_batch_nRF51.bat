SETLOCAL
ECHO ON

REM parameter 1 = nRF51/bootloader folder
REM parameter 2 = folder with the S110 v8.0.0 softdevice


nrfjprog -i > .\tmp_remove_me.txt
FOR /f %%i IN (.\tmp_remove_me.txt) DO "%~dp0/flash_open_mesh_nRF51.bat" "%~f1" "%~f2" %%i

del .\tmp_remove_me.txt