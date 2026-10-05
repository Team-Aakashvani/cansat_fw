@echo off
title Aakashvani Ground Station
cd /d "%~dp0"
python tools\aakashvani_dock.py
if errorlevel 1 (
    echo.
    echo The ground station exited with an error.
    pause
)
