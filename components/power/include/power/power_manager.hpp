/**
 * @file power_manager.hpp
 * @brief Battery monitoring, brownout protection, and power-state reporting.
 *
 * Aggregates INA260 (bus voltage/current) + MAX17048 (fuel gauge) data and
 * provides a unified PowerState to the system. Triggers warnings and safe
 * shutdown when voltage drops below thresholds defined in PowerConfig.
 *
 * Brownout callback: registered via ESP-IDF brownout detector. On brownout,
 * the system is placed into a low-power safe state (motors disarmed, SD
 * flushed) before reset.
 *
 * Thread safety: get_state() returns a snapshot; update() should be called
 * from a single task.
 */
#pragma once

#include "drivers/ina260.hpp"
#include "drivers/max17048.hpp"
#include "nav/config.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <cstdint>
#include <functional>

namespace power {

enum class PowerStatus : uint8_t {
    NORMAL       = 0,
    LOW_VOLTAGE  = 1,   ///< Below POWER_CFG.voltage_warn_v
    CRITICAL     = 2,   ///< Below POWER_CFG.voltage_crit_v
    BROWNOUT     = 3,   ///< Hardware brownout imminent
};

struct PowerState {
    float       voltage_v;
    float       current_a;
    float       power_w;
    float       soc_pct;         ///< State-of-charge 0..100%
    float       crate_pct_hr;    ///< Charge rate (%/hr, negative = discharging)
    PowerStatus status;
    bool        valid;
};

using LowPowerCallback = std::function<void(PowerStatus)>;

class PowerManager {
public:
    PowerManager() noexcept = default;

    /**
     * @brief Initialise INA260 and MAX17048.
     * @param i2c  Shared I2C bus.
     * @return ESP_OK on success.
     */
    esp_err_t init(hal::I2CBus& i2c) noexcept;

    /**
     * @brief Read latest sensor data and update internal PowerState.
     * Call from a periodic task (e.g. 1Hz).
     */
    void update() noexcept;

    /**
     * @brief Register callback invoked when power status transitions.
     * Called from the task that calls update(). Must be lightweight.
     */
    void set_low_power_callback(LowPowerCallback cb) noexcept { cb_ = cb; }

    /// Thread-safe snapshot of current power state.
    PowerState get_state() const noexcept;

    bool is_critical() const noexcept {
        return get_state().status >= PowerStatus::CRITICAL;
    }

private:
    drivers::INA260    ina260_{};
    drivers::MAX17048  max17048_{};

    mutable SemaphoreHandle_t mutex_ = nullptr;
    PowerState        state_{};
    PowerStatus       prev_status_ = PowerStatus::NORMAL;
    LowPowerCallback  cb_{};
    bool              ready_ = false;
};

} // namespace power
