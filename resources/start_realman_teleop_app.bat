@echo off
REM =============================================
REM  Auto Restart Program Launcher for Windows
REM  Supports: Conda environment + auto restart
REM =============================================

REM ---- USER CONFIG ----
REM set "APP_DIR=%UserProfile%\realman_teleop_app"
set "APP_DIR=D:\wk\Codehub\0\phimate\dist\realman_teleop_app"
set "LAUNCH_SCRIPT=realman_teleop"
REM set "CONDA_PATH=%UserProfile%\miniconda3"
set "CONDA_PATH=%APP_DIR%\realman_teleop_env"
set "ENV_NAME=realman_teleop_env"

REM ----------------------

echo [%date% %time%] Starting program... 

REM ---- change to app dir ----
cd /d "%APP_DIR%"
if errorlevel 1 (
    echo [%date% %time%] ERROR: cannot change directory %APP_DIR% 
    exit /b 1
)


:run_main
echo [%date% %time%] Launching %LAUNCH_SCRIPT% ... 

REM ---- activate conda ----
REM call "%CONDA_PATH%\Scripts\activate.bat"
REM call conda activate "%ENV_NAME%"

%CONDA_PATH%\python.exe -m "%LAUNCH_SCRIPT%"
echo [%date% %time%] Program exited. 
::exit /b 0
pause
