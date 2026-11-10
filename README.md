# ESP8266 Servo

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
