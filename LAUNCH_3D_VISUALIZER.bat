@echo off
title Aakashvani 3D attitude
cd /d "%~dp0"
python tools\launch_3d_visualizer.py
if errorlevel 1 (
    echo.
    echo The 3D viewer exited with an error.
    pause
)
