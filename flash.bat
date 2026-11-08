@echo off
setlocal

REM ============================================================
REM mcuFlash.bat - ESP8266 firmware flashing wrapper
REM Usage: mcuFlash -COM 3 -f firmware.bin
REM ============================================================

set "COM_PORT="
set "FIRMWARE="
set "BAUD=115200"

:parse_args
if "%~1"=="" goto args_done

if /I "%~1"=="-COM" (
    if "%~2"=="" goto usage
    set "COM_PORT=%~2"
    shift
    shift
    goto parse_args
)

if /I "%~1"=="-f" (
    if "%~2"=="" goto usage
    set "FIRMWARE=%~2"
    shift
    shift
    goto parse_args
)

if /I "%~1"=="-baud" (
    if "%~2"=="" goto usage
    set "BAUD=%~2"
    shift
    shift
    goto parse_args
)

echo ERROR: Unknown argument: %~1
goto usage

:args_done
if not defined COM_PORT (
    echo ERROR: COM port is required.
    goto usage
)

if not defined FIRMWARE (
    echo ERROR: Firmware file is required.
    goto usage
)

where python >nul 2>nul
if errorlevel 1 (
    echo ERROR: Python was not found in PATH.
    exit /b 2
)

python -m esptool version >nul 2>nul
if errorlevel 1 (
    echo ERROR: esptool is not installed for this Python.
    echo Install it with: python -m pip install esptool
    exit /b 3
)

if not exist "%FIRMWARE%" (
    echo ERROR: Firmware file not found: %FIRMWARE%
    exit /b 4
)

echo.
echo ESP8266 Firmware Flash
echo   COM port : COM%COM_PORT%
echo   Firmware : %FIRMWARE%
echo   Baud rate: %BAUD%
echo.

python -m esptool --chip esp8266 --port COM%COM_PORT% --baud %BAUD% write-flash 0x0 "%FIRMWARE%"
set "FLASH_RESULT=%ERRORLEVEL%"

if not "%FLASH_RESULT%"=="0" (
    echo.
    echo ERROR: Flashing failed with exit code %FLASH_RESULT%.
    exit /b %FLASH_RESULT%
)

echo.
echo SUCCESS: Firmware flashed successfully.
exit /b 0

:usage
echo.
echo Usage:
echo   mcuFlash -COM 3 -f firmware.bin
echo.
echo Optional baud rate:
echo   mcuFlash -COM 3 -f firmware.bin -baud 460800
echo.
echo Requirements:
echo   Python in PATH
echo   esptool installed: python -m pip install esptool
exit /b 1
