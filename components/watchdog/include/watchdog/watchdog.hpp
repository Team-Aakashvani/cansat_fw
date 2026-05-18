/**
 * @file watchdog.hpp
 * @brief Task-level watchdog monitor using ESP-IDF TWDT.
 *
 * Each critical task registers itself and must call ping() within its timeout.
 * If any registered task misses its deadline, the TWDT triggers a system reset.
 *
 * Critical tasks and their timeouts:
 *   - nav_task        : 200ms  (100Hz IMU loop + EKF)
 *   - control_task    : 200ms  (100Hz attitude control)
 *   - telemetry_task  : 5s     (1Hz beacon, allows 3 retries)
 *   - logging_task    : 10s    (SD flush)
 *   - lora_task       : 5s     (1Hz TX window)
 *
 * Wraps ESP-IDF esp_task_wdt_* APIs. Must be called after
 * esp_task_wdt_init() in main.
 *
 * Thread safety: register() must be called from the owning task.
 *                ping() must be called from the owning task.
 */
#pragma once

#include "esp_task_wdt.h"
#include "esp_err.h"
#include <cstdint>
#include <cstddef>

namespace watchdog {

/// Unique task names for watchdog registration.
static constexpr const char* WDT_NAV      = "wdt_nav";
static constexpr const char* WDT_CTRL     = "wdt_ctrl";
static constexpr const char* WDT_TELEM    = "wdt_telem";
static constexpr const char* WDT_LOGGING  = "wdt_log";
static constexpr const char* WDT_LORA     = "wdt_lora";

class Watchdog {
public:
    /// TWDT timeout in seconds (hardware reset if any subscriber misses).
    static constexpr uint32_t TWDT_TIMEOUT_S = 15;

    Watchdog() noexcept = default;

    /**
     * @brief Initialise TWDT.
     * @param panic_on_timeout  If true, panic + coredump on WDT expiry.
     *                          If false, reset silently.
     * @return ESP_OK on success.
     */
    esp_err_t init(bool panic_on_timeout = true) noexcept;

    /**
     * @brief Register the calling task with the TWDT.
     * Call once from each task during its startup phase.
     * @return ESP_OK on success.
     */
    static esp_err_t register_task() noexcept;

    /**
     * @brief Deregister the calling task from TWDT.
     * Call before a task exits.
     */
    static void deregister_task() noexcept;

    /**
     * @brief Reset the watchdog for the calling task.
     * Must be called within TWDT_TIMEOUT_S seconds.
     */
    static void ping() noexcept;

    /// Return true if TWDT is armed.
    bool is_armed() const noexcept { return armed_; }

private:
    bool armed_ = false;
};

} // namespace watchdog
