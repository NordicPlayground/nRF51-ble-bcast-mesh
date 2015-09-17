@echo off
for /D %%D in ("*") DO (
    echo Entering %%D
    cd %%D\arm
    for %%P in ("*.uvpro*") DO (
        echo Building %%P
        UV4 -b %%P -z -j0 -o log.txt
        if ERRORLEVEL 2 (
            echo ERROR building %%P:
            type log.txt
            cd ..\..
            goto :eof
        )
        echo %%P built successfully.
    )
    cd ..\..
)
echo ----------------------
echo All builds successful!
