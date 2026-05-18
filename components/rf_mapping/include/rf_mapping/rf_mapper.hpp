/**
 * @file rf_mapper.hpp
 * @brief Directional RF mapping mission subsystem.
 *
 * Sweeps a directional antenna via servo (0-180 deg) and samples RSSI
 * from the CC1101 scanner, synchronized with GNSS position and mission time.
 */
#pragma once

#include "nav/flight_computer.hpp"
#include "drivers/cc1101.hpp"
#include "control/motor_mixer.hpp"
#include "logging/sd_logger.hpp"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <atomic>

namespace rf_mapping {

class RFMapper {
public:
    RFMapper() noexcept = default;

    /**
     * @brief Initialize the mapper.
     * @param fc   Flight computer (for position/time snapshots)
     * @param scan CC1101 scanner driver
     * @param motors Motor mixer (for servo control)
     * @param sd   SD logger (for data export)
     * @param fc_mutex Mutex protecting the flight computer state
     */
    void init(nav::FlightComputer& fc, 
              drivers::CC1101& scan,
              control::MotorMixer& motors,
              logging::SDLogger& sd,
              SemaphoreHandle_t fc_mutex) noexcept;

    /// Start the mapping mission task.
    void start() noexcept;

    /// Stop the mapping mission.
    void stop() noexcept;

    /// Is the mapping mission currently active?
    bool is_running() const noexcept { return active_.load(); }

private:
    static void task_entry(void* arg) noexcept;
    void run() noexcept;

    nav::FlightComputer* fc_     = nullptr;
    drivers::CC1101*     scan_   = nullptr;
    control::MotorMixer* motors_ = nullptr;
    logging::SDLogger*   sd_     = nullptr;
    SemaphoreHandle_t    fc_mutex_ = nullptr;

    std::atomic<bool>    active_{false};
    TaskHandle_t         task_   = nullptr;

    // Sweep parameters
    static constexpr float SCAN_FREQ_HZ = 433e6; // Example scan freq
    static constexpr float STEP_DEG      = 5.0f;
    static constexpr uint32_t SETTLE_MS  = 100;
};

} // namespace rf_mapping
