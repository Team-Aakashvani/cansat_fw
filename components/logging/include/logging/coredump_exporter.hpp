/**
 * @file coredump_exporter.hpp
 * @brief Utility to export crash coredumps from flash to SD card.
 */
#pragma once

#include "esp_err.h"

namespace logging {

class CoredumpExporter {
public:
    /**
     * @brief Check for core dump in flash and export it to SD card if present.
     * Should be called after SD card is mounted.
     * 
     * Output file: "/sdcard/CRASH_<boot_count>.bin"
     * 
     * @return ESP_OK if processed (or none found), error otherwise.
     */
    static esp_err_t check_and_export() noexcept;
};

} // namespace logging
