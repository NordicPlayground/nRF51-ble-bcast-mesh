@echo off
set currdir=%cd%
for /D %%D in ("*") DO (
    echo Entering %%D
    cd %%D\arm
    for %%P in ("*.uvpro*") DO (
        echo Building %%P
        UV4 -c %%P -j0
        UV4 -b %%P -z -j0 -o log.txt
        if ERRORLEVEL 2 (
            echo ERROR building %%P:
            type log.txt
            goto :cleanup
        )
        echo %%P built successfully.
    )
    cd %currdir%
)
echo ----------------------
echo All builds successful!

:cleanup
cd %currdir%
