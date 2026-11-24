
@echo off
setlocal EnableExtensions

REM ============================================================
REM ESP Firmware Downloader and Flasher
REM ============================================================

set "REPO=BacNguyen0801/esp8266servo"
set "BRANCH=main"
set "ARTIFACT=esp8266-nodemcu-firmware"
set "FIRMWARE=firmware.bin"
set "BAUD=115200"
set "COM_PORT="
set "LOCAL_FIRMWARE="
set "CHIP=esp8266"

set "DOWNLOAD_DIR=%~dp0downloads"
set "ZIP_FILE=%DOWNLOAD_DIR%\firmware.zip"
set "EXTRACT_DIR=%DOWNLOAD_DIR%\extracted"
set "FIRMWARE_PATH=%EXTRACT_DIR%\firmware.bin"

REM ============================================================
REM Parse arguments
REM ============================================================

:parse_args

if "%~1"=="" goto args_done

if /I "%~1"=="-COM" (
    if "%~2"=="" goto usage
    set "COM_PORT=%~2"
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

if /I "%~1"=="-chip" (
    if "%~2"=="" goto usage
    set "CHIP=%~2"
    shift
    shift
    goto parse_args
)

if /I "%~1"=="-f" (
    if "%~2"=="" goto usage
    set "LOCAL_FIRMWARE=%~2"
    shift
    shift
    goto parse_args
)

if /I "%~1"=="--firmware" (
    if "%~2"=="" goto usage
    set "LOCAL_FIRMWARE=%~2"
    shift
    shift
    goto parse_args
)

echo ERROR: Unknown argument %~1
goto usage

:args_done

if not defined COM_PORT goto usage

if /I "%CHIP%"=="esp8266" (
    set "CHIP=esp8266"
    set "ARTIFACT=esp8266-nodemcu-firmware"
) else if /I "%CHIP%"=="esp32" (
    set "CHIP=esp32"
    set "ARTIFACT=esp32dev-firmware"
) else (
    echo ERROR: Unsupported chip %CHIP%. Use esp8266 or esp32.
    exit /b 1
)

REM Normalize COM port
if /I "%COM_PORT:~0,3%"=="COM" (
    set "COM_PORT=%COM_PORT:~3%"
)
set "COM_PORT=COM%COM_PORT%"

echo.
echo ============================================================
echo       AUTOMATIC ESP FIRMWARE FLASHER
echo ============================================================
echo Repository : %REPO%
echo Branch     : %BRANCH%
echo Chip       : %CHIP%
echo COM Port   : %COM_PORT%
echo Baud Rate  : %BAUD%
if defined LOCAL_FIRMWARE (
    echo Firmware   : %LOCAL_FIRMWARE%
)

echo ============================================================

REM ============================================================
REM [1/4] Check prerequisites
REM ============================================================

echo.
echo [1/4] Checking Python, esptool and curl...

where python >nul 2>nul
if errorlevel 1 (
    echo ERROR: Python not found.
    exit /b 1
)

where curl.exe >nul 2>nul
if errorlevel 1 (
    echo ERROR: curl.exe not found.
    exit /b 1
)

python -m esptool version
if errorlevel 1 (
    echo Installing esptool...
    python -m pip install --upgrade esptool
    if errorlevel 1 exit /b 1
)

if defined LOCAL_FIRMWARE (
    if not exist "%LOCAL_FIRMWARE%" (
        echo ERROR: Local firmware file not found: %LOCAL_FIRMWARE%
        exit /b 1
    )
    set "FIRMWARE_PATH=%LOCAL_FIRMWARE%"
    goto flash_local
)

if not exist "%DOWNLOAD_DIR%" mkdir "%DOWNLOAD_DIR%"

if exist "%EXTRACT_DIR%" rmdir /s /q "%EXTRACT_DIR%"
mkdir "%EXTRACT_DIR%"

REM ============================================================
REM [2/4] Find and download latest successful GitHub artifact
REM ============================================================

echo.
echo [2/4] Searching latest successful GitHub build...

powershell.exe -NoProfile -ExecutionPolicy Bypass -Command "$ErrorActionPreference='Stop'; try { $repo=$env:REPO; $branch=$env:BRANCH; $artifactName=$env:ARTIFACT; $zip=$env:ZIP_FILE; $extract=$env:EXTRACT_DIR; $fwName=$env:FIRMWARE; $fwPath=$env:FIRMWARE_PATH; $headers=@{'User-Agent'='ESP8266-Flasher';'Accept'='application/vnd.github+json';'X-GitHub-Api-Version'='2022-11-28'}; if($env:GITHUB_TOKEN) { $headers.Authorization='Bearer '+$env:GITHUB_TOKEN; Write-Host 'Using GITHUB_TOKEN for GitHub API authentication' }; $url='https://api.github.com/repos/'+$repo+'/actions/runs?branch='+$branch+'&status=success&per_page=100'; $runs=(Invoke-RestMethod -Uri $url -Headers $headers).workflow_runs; $selected=$null; $artifact=$null; foreach($run in $runs) { if($run.head_branch -ne $branch -or $run.conclusion -ne 'success') { continue }; Write-Host ('Checking build '+$run.id+'...'); $au='https://api.github.com/repos/'+$repo+'/actions/runs/'+$run.id+'/artifacts?per_page=100'; $arts=(Invoke-RestMethod -Uri $au -Headers $headers).artifacts; $found=$arts | Where-Object { $_.name -eq $artifactName -and -not $_.expired } | Select-Object -First 1; if($found) { $selected=$run; $artifact=$found; break } }; if(-not $artifact) { throw 'No successful firmware artifact found on the configured branch' }; Write-Host ('Build ID: '+$selected.id); Write-Host ('Commit: '+$selected.head_sha); Write-Host ('Artifact: '+$artifact.name); Write-Host ('Artifact ID: '+$artifact.id); $download='https://api.github.com/repos/'+$repo+'/actions/artifacts/'+$artifact.id+'/zip'; $curlArgs=@('--location','--fail','--silent','--show-error','--output',$zip); if($env:GITHUB_TOKEN) { $curlArgs+=@('--header',('Authorization: Bearer '+$env:GITHUB_TOKEN)) }; $curlArgs+=$download; Write-Host 'Downloading artifact ZIP...'; & curl.exe @curlArgs; if($LASTEXITCODE -ne 0) { throw ('curl download failed with exit code '+$LASTEXITCODE+'. Check GitHub access or GITHUB_TOKEN.') }; if(-not (Test-Path $zip) -or (Get-Item $zip).Length -eq 0) { throw 'Downloaded ZIP is missing or empty' }; Write-Host 'Extracting firmware...'; Expand-Archive -Path $zip -DestinationPath $extract -Force; $fw=Get-ChildItem -Path $extract -Filter $fwName -File -Recurse | Select-Object -First 1; if(-not $fw -or $fw.Length -eq 0) { throw ('Firmware file '+$fwName+' missing or empty in artifact') }; if($fw.FullName -ne $fwPath) { Copy-Item -Path $fw.FullName -Destination $fwPath -Force }; Write-Host ('Firmware downloaded: '+$fwPath) -ForegroundColor Green; exit 0 } catch { Write-Host ('ERROR: '+$_.Exception.Message) -ForegroundColor Red; exit 1 }"

if errorlevel 1 (
    echo.
    echo ERROR: GitHub firmware download failed.
    echo.
    echo If GitHub returns 401, configure GITHUB_TOKEN.
    exit /b 1
)

REM ============================================================
REM [3/4] Flash firmware (esptool detects the chip)
REM ============================================================

:flash_local

echo.
echo [3/4] Flashing %CHIP% firmware...
echo Firmware: %FIRMWARE_PATH%
for %%I in ("%FIRMWARE_PATH%") do set "ESP32_DIR=%%~dpI"

if /I "%CHIP%"=="esp8266" (
    python -m esptool ^
        --chip esp8266 ^
        --port %COM_PORT% ^
        --baud %BAUD% ^
        write-flash ^
        0x0 ^
        "%FIRMWARE_PATH%"
) else (
    if not exist "%ESP32_DIR%\bootloader.bin" (
        echo ERROR: ESP32 bootloader.bin not found beside firmware: %ESP32_DIR%
        exit /b 1
    )
    if not exist "%ESP32_DIR%\partitions.bin" (
        echo ERROR: ESP32 partitions.bin not found beside firmware: %ESP32_DIR%
        exit /b 1
    )

    python -m esptool ^
        --chip esp32 ^
        --port %COM_PORT% ^
        --baud %BAUD% ^
        write-flash ^
        --flash-mode dio ^
        --flash-freq 40m ^
        --flash-size detect ^
        0x1000 "%ESP32_DIR%\bootloader.bin" ^
        0x8000 "%ESP32_DIR%\partitions.bin" ^
        0x10000 "%FIRMWARE_PATH%"
)

if errorlevel 1 (
    echo.
    echo ERROR: Firmware flashing failed.
    exit /b 1
)

REM ============================================================
REM [4/4] Complete
REM ============================================================

echo.
echo ============================================================
echo       FIRMWARE UPDATE SUCCESSFUL
echo ============================================================
echo Repository : %REPO%
echo Branch     : %BRANCH%
echo Chip       : %CHIP%
echo Firmware   : %FIRMWARE%
echo COM Port   : %COM_PORT%
echo ============================================================

exit /b 0

:usage
echo.
echo Usage:
echo   flash.bat -COM 3
echo   flash.bat -COM COM3
echo   flash.bat -COM 3 -baud 460800
echo   flash.bat -COM 4 -chip esp8266
echo   flash.bat -COM 4 -chip esp32
echo   flash.bat -COM 4 -chip esp8266 -f .pio\build\nodemcuv2\firmware.bin
echo   flash.bat -COM 4 -chip esp32 -f .pio\build\esp32dev\firmware.bin
echo.
exit /b 1
