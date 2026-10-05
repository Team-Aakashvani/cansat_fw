@echo off
title "CAN-7USAT 3D Assembled Body & IMU Attitude Visualizer"
cd /d "%~dp0"
python tools\launch_3d_visualizer.py
if errorlevel 1 (
    echo.
    echo 3D Visualizer exited with an error.
    pause
)
