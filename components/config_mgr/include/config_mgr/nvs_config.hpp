/**
 * @file nvs_config.hpp
 * @brief NVS-backed persistent configuration store.
 *
 * Stores and retrieves calibration and mission parameters that must survive
 * power cycles:
 *   - team_id         (uint16, default from compile-time config)
 *   - ground_alt_m    (float, set by CAL command or BIT)
 *   - mag_cal[3]      (float[3], optional hard-iron offset)
 *   - baro_offset_pa  (float, factory trim)
 *   - boot_count      (uint32, incremented each boot)
 *
 * All reads return default values if the key is not found (first boot).
 * All writes are committed immediately (small data, write-once-per-flight).
 *
 * Thread safety: Each method opens/closes NVS atomically. Safe from any task.
 */
#pragma once

#include "nav/config.hpp"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_err.h"
#include <cstdint>

namespace config_mgr {

class NVSConfig {
public:
    static constexpr const char* NAMESPACE = "cansat_cfg";

    NVSConfig() noexcept = default;

    /**
     * @brief Initialise NVS flash and the cansat_cfg namespace.
     * Erases and re-initialises NVS if the partition is corrupted.
     * @return ESP_OK on success.
     */
    esp_err_t init() noexcept;

    // ----- Getters -----------------------------------------------------------

    uint16_t get_team_id()        const noexcept;
    float    get_ground_alt_m()   const noexcept;
    float    get_baro_offset_pa() const noexcept;
    uint32_t get_boot_count()     const noexcept;
    bool     get_bit_override()   const noexcept;
    void     get_mag_cal(float out[3]) const noexcept;

    // ----- Setters -----------------------------------------------------------

    esp_err_t set_team_id       (uint16_t id)         noexcept;
    esp_err_t set_ground_alt_m  (float alt_m)         noexcept;
    esp_err_t set_baro_offset_pa(float offset_pa)     noexcept;
    esp_err_t set_bit_override  (bool enable)         noexcept;
    esp_err_t set_mag_cal       (const float cal[3])  noexcept;

    /// Increment and persist boot counter. Call once at startup after init().
    uint32_t  increment_boot_count() noexcept;

private:
    bool ready_ = false;

    esp_err_t open_rw(nvs_handle_t& out) const noexcept;
    esp_err_t open_ro(nvs_handle_t& out) const noexcept;
};

} // namespace config_mgr
