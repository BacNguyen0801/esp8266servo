@echo off
setlocal
cd /d "%~dp0"

set "APP_NAME=eMotor"
set "EXE_PATH=dist\%APP_NAME%.exe"

if exist "dist\%APP_NAME%.exe" rmdir /s /q "dist\%APP_NAME%.exe"

echo Building %APP_NAME%...
set "PYTHON_CMD="
where py.exe >nul 2>nul
if not errorlevel 1 (
    py -3 -c "import sys" >nul 2>nul
    if not errorlevel 1 set "PYTHON_CMD=py -3"
)
if not defined PYTHON_CMD (
    where python.exe >nul 2>nul
    if not errorlevel 1 set "PYTHON_CMD=python"
)
if not defined PYTHON_CMD (
    echo ERROR: Python not found. Install Python and enable "Add Python to PATH".
    exit /b 1
)

%PYTHON_CMD% -m PyInstaller ^
    --onefile ^
    --noupx ^
    --windowed ^
    --clean ^
    --exclude-module tkinter ^
    --exclude-module numpy ^
    --name %APP_NAME% ^
    tools\pyUI.py
if not exist "%EXE_PATH%" (
    echo.
    echo Build failed. Make sure PyInstaller is installed:
    echo %PYTHON_CMD% -m pip install pyinstaller
    exit /b 1
)
