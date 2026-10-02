@echo off
REM Double-click this to watch the live IMU angles.
REM
REM ASCII only on purpose: a .bat with Chinese in it gets mangled by the
REM console code page, and the whole point of this file is that it works when
REM double-clicked without anyone having to think about encoding.
REM
REM It tries the official py launcher first and falls back to python, because
REM on this machine plain "python" in some shells resolves to the MSYS2 build
REM under C:\msys64, which is not the one that has the packages installed.

cd /d "%~dp0"

echo.
echo   Live roll / pitch / yaw from the running board, over SWD.
echo   Lay the module flat and still: roll and pitch should both read near 0.
echo   Press Ctrl-C to stop.
echo.

where py >nul 2>nul
if %errorlevel%==0 (
    py tools\watch_imu.py %*
) else (
    python tools\watch_imu.py %*
)

echo.
pause
