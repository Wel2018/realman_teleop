@echo off
REM =========================================
REM Realman Teleop App
REM wel2018
REM 2026-01-09
REM =========================================

REM 1. 3d鼠标运行时：将 hidapi.dll 拷贝到 C:\Windows\System32 下
REM 2. 将 start_realman_teleop_app.bat 启动脚本复制到 C:\ProgramData\Microsoft\Windows\Start Menu\Programs\StartUp 下

REM 获取当前脚本所在目录
set SCRIPT_DIR=%~dp0

REM hidapi.dll 所在路径（假设和 bat 在同一目录）
set HID_DLL=%SCRIPT_DIR%\resources\hidapi.dll
set LAUNCH_BAT=%SCRIPT_DIR%start_realman_teleop_app.bat

REM System32 目标路径
set SYSTEM32_DIR=C:\Windows\System32
set AUTOSTART_DIR=C:\ProgramData\Microsoft\Windows\Start Menu\Programs\StartUp

echo ""
echo SCRIPT_DIR=%SCRIPT_DIR%
echo HID_DLL=%HID_DLL%
echo SYSTEM32_DIR=%SYSTEM32_DIR%
echo AUTOSTART_DIR=%AUTOSTART_DIR%
echo ""

REM 检查管理员权限
net session >nul 2>&1
if %errorLevel% neq 0 (
    echo [ERROR] use root!
    pause
    exit /b 1
)

REM 拷贝 hidapi.dll（存在则覆盖）
if exist "%HID_DLL%" (
    echo [INFO] copy hidapi.dll to System32...
    copy /Y "%HID_DLL%" "%SYSTEM32_DIR%" >nul
) else (
    echo [ERROR] not found! hidapi.dll：%HID_DLL%
    pause
    exit /b 1
)

REM 拷贝 LAUNCH_BAT（存在则覆盖）
if exist "%LAUNCH_BAT%" (
    echo [INFO] copy %LAUNCH_BAT% to %AUTOSTART_DIR%...
    copy /Y "%LAUNCH_BAT%" "%AUTOSTART_DIR%" >nul
) else (
    echo [ERROR] not found：%LAUNCH_BAT%
    pause
    exit /b 1
)

echo [INFO] Everything is ok!
pause
REM exit /b 0
