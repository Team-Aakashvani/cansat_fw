@echo off
title Aakashvani Ground Station
cd /d "%~dp0"

where python >nul 2>nul
if errorlevel 1 (
    echo Python is not installed. Install Python 3.11 or newer from https://www.python.org/downloads/
    echo and tick "Add python.exe to PATH" during installation. Then run this file again.
    pause
    exit /b 1
)

python -c "import customtkinter, serial, bleak, websockets, matplotlib, PIL, esptool" >nul 2>nul
if errorlevel 1 (
    echo First start: installing the ground station packages. This needs internet and takes a minute...
    python -m pip install --disable-pip-version-check -r tools\requirements.txt
    if errorlevel 1 (
        echo.
        echo Package installation failed. Check the internet connection and try again.
        pause
        exit /b 1
    )
)

python tools\aakashvani_dock.py
if errorlevel 1 (
    echo.
    echo The ground station exited with an error.
    pause
)
