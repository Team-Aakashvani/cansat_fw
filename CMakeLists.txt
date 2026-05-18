# ===========================================================================
# cansat_fw — Top-level CMakeLists.txt
# CAN-7USAT 2026 | ESP32-S3 WROOM-1 | ESP-IDF v5.x
# ===========================================================================
cmake_minimum_required(VERSION 3.20)

# Include ESP-IDF build system
include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Project name and language standards
project(cansat_fw
    VERSION 1.0.0
    LANGUAGES C CXX
)

# C++17 required throughout for structured bindings, if-constexpr, std::array, etc.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# C11 for C components (FreeRTOS wrappers, drivers)
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
