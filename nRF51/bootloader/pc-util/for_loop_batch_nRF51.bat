SETLOCAL
ECHO ON

REM Store the segger ids of all connected nRF51 devices 
nrfjprog -i > .\tmp_remove_me.txt

REM Program each nRF51 device with new hex files
FOR /f %%i IN (.\tmp_remove_me.txt) DO .\pc-util\flash_open_mesh_nRF51.bat %%i

REM Delete the temporary file
del .\tmp_remove_me.txt