/**
 * @file system_init.hpp
 * @brief Shared core initialization for all CAN-7USAT hardware roles.
 */
#pragma once

#include "esp_err.h"

namespace system_init {

/**
 * @brief Initialize core hardware and system services shared by all roles.
 * Includes NVS, basic logging, and boot count tracking.
 * @return ESP_OK on success.
 */
esp_err_t core_init() noexcept;

/**
 * @brief Initialize SD card logging if available on the hardware.
 * @return ESP_OK on success.
 */
esp_err_t logging_init() noexcept;

} // namespace system_init
