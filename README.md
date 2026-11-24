# ESP8266 Servo

## Build firmware

PlatformIO builds firmware for both supported boards by default:

```powershell
pio run
```

The output binaries are separate:

- ESP8266 NodeMCU: `.pio/build/nodemcuv2/firmware.bin`
- ESP32 Dev Module: `.pio/build/esp32dev/firmware.bin`

To build only one target, use `pio run -e nodemcuv2` or
`pio run -e esp32dev`.

To download and flash a firmware automatically:

```bat
flash.bat -COM 4 -chip esp8266
flash.bat -COM 4 -chip esp32
```

For a locally built ESP32 image, place `bootloader.bin` and `partitions.bin`
beside `firmware.bin` in `.pio/build/esp32dev`, then use:

```bat
flash.bat -COM 4 -chip esp32 -f .pio\build\esp32dev\firmware.bin
```

## Wi-Fi connection

The firmware's `WifiConnection` module stores credentials in LittleFS and
automatically attempts to connect to the saved network during startup. The
module exposes SSID/password getters and setters, plus connect, disconnect,
reconnect, connection-status, and credential-change methods. Call `begin()`
before using it. To seed a new device, copy `src/wifi_defaults.h.example` to
`src/wifi_defaults.h` and set the defaults there. That local file is ignored by
Git; on first startup the defaults are saved to LittleFS.

## Build and sign the Windows EXE

The repo includes a signed-build flow for the packaged Windows app.

### Build only

```powershell
build_eMotor.bat
```

This builds the PyInstaller EXE and then attempts to sign it if a certificate is configured.

### Sign the built EXE manually

```powershell
set SIGN_CERT_PATH=C:\certs\mycert.pfx
set SIGN_CERT_PASSWORD=your-password
call sign_eMotor.bat dist\eMotor.exe
```

Or by certificate subject:

```powershell
set SIGN_CERT_SUBJECT="My Company Code Signing"
call sign_eMotor.bat dist\eMotor.exe
```

### Notes

- If `signtool.exe` is not installed, the build still completes but signing is skipped.
- If no certificate is configured, the batch file prints the required setup steps.
- On a managed Windows machine, an enterprise application-control policy may still block an unsigned or unapproved binary even after a successful build.
