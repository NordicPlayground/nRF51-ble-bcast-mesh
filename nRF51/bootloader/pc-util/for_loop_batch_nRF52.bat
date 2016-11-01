SETLOCAL
ECHO ON


nrfjprog -i > .\tmp_remove_me.txt

FOR /f %%i IN (.\tmp_remove_me.txt) DO .\pc-util\flash_open_mesh_nRF52.bat %%i

del .\tmp_remove_me.txt