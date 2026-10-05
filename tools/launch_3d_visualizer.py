"""
AAKASHVANI — CanSat 3D Attitude Visualizer Desktop Launcher
===========================================================
Launches the real-time 3D CanSat body attitude viewer in a dedicated
desktop window (PyQt6 WebEngine) or browser with live telemetry stream.
"""

import os
import sys
import time
import webbrowser
import threading

# Add tools directory to path
TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
if TOOLS_DIR not in sys.path:
    sys.path.insert(0, TOOLS_DIR)

import telemetry_3d_server

def main():
    print("=========================================================")
    print(" CAN-7USAT — 3D Attitude & Body Visualizer Launcher")
    print("=========================================================")

    # 1. Attach to existing server or start standalone server
    server_started_by_launcher = False
    if not telemetry_3d_server.is_port_in_use(8055):
        print("[Launcher] Starting standalone telemetry & WebSocket server on ports 8055/8765...")
        telemetry_3d_server.start_server(http_port=8055, ws_port=8765)
        server_started_by_launcher = True
    else:
        print("[Launcher] Connected to active Aakashvani Dock telemetry server.")
    viewer_url = "http://127.0.0.1:8055/"
    print(f"[Launcher] 3D Viewer endpoint: {viewer_url}")

    # 2. Try launching with PyQt6 QWebEngine for a native desktop application feel
    use_pyqt = True
    try:
        from PyQt6.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QWidget
        from PyQt6.QtWebEngineWidgets import QWebEngineView
        from PyQt6.QtCore import QUrl
        from PyQt6.QtGui import QIcon
    except ImportError:
        use_pyqt = False

    if use_pyqt:
        print("[Launcher] Launching dedicated PyQt6 Desktop Window...")
        app = QApplication(sys.argv)
        app.setApplicationName("CAN-7USAT 3D Attitude Visualizer")

        window = QMainWindow()
        window.setWindowTitle("AAKASHVANI — CAN-7USAT 3D Assembled Body & IMU Attitude Visualizer")
        window.resize(1280, 800)
        window.setMinimumSize(960, 640)

        # Central WebEngine Widget
        web_view = QWebEngineView()
        web_view.load(QUrl(viewer_url))

        central_widget = QWidget()
        layout = QVBoxLayout(central_widget)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(web_view)

        window.setCentralWidget(central_widget)
        window.show()

        try:
            sys.exit(app.exec())
        except SystemExit:
            if server_started_by_launcher:
                telemetry_3d_server.stop_server()
    else:
        print("[Launcher] PyQt6 not available; opening in default web browser...")
        webbrowser.open(viewer_url)
        print("[Launcher] Press Ctrl+C in terminal to stop server.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            if server_started_by_launcher:
                telemetry_3d_server.stop_server()
            print("\nExited.")

if __name__ == "__main__":
    main()
