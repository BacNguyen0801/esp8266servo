@echo off
setlocal
cd /d "%~dp0"

set "APP_NAME=eMotor"
set "EXE_PATH=dist\%APP_NAME%.exe"

if exist "dist\%APP_NAME%.exe" rmdir /s /q "dist\%APP_NAME%.exe"

echo Building %APP_NAME%...
python -m PyInstaller ^
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
    echo python -m pip install pyinstaller
    exit /b 1
)
