@echo off
title "Aakashvani Mission Control & Flasher Dock"
cd /d "%~dp0"
python tools\aakashvani_dock.py
if errorlevel 1 (
    echo.
    echo Application exited with an error.
    pause
)
