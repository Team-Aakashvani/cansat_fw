/**
 * @file power_manager.cpp
 * @brief Power manager implementation.
 */
#include "power/power_manager.hpp"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "PowerMgr";

namespace power {

esp_err_t PowerManager::init(hal::I2CBus& i2c) noexcept {
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) return ESP_ERR_NO_MEM;

    esp_err_t ret = ina260_.init(i2c);   // uses default 0x40
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "INA260 init failed: %d", ret);
        return ret;
    }
    ret = max17048_.init(i2c);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "MAX17048 init failed: %d — SOC unavailable", ret);
        // Non-fatal: INA260 is sufficient for basic voltage monitoring
    }
    ready_ = true;
    ESP_LOGI(TAG, "PowerManager ready");
    return ESP_OK;
}

void PowerManager::update() noexcept {
    if (!ready_) return;

    drivers::PowerData  pwr  = ina260_.read();
    drivers::FuelData   fuel = max17048_.read();

    PowerStatus new_status = PowerStatus::NORMAL;
    if (pwr.valid) {
        if (pwr.voltage_v < (float)nav::POWER_CFG.bat_critical_v) {
            new_status = PowerStatus::CRITICAL;
        } else if (pwr.voltage_v < (float)nav::POWER_CFG.bat_low_v) {
            new_status = PowerStatus::LOW_VOLTAGE;
        }
    }

    {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        state_.voltage_v   = pwr.valid  ? pwr.voltage_v   : 0.0f;
        state_.current_a   = pwr.valid  ? pwr.current_a   : 0.0f;
        state_.power_w     = pwr.valid  ? pwr.power_w     : 0.0f;
        state_.soc_pct     = fuel.valid ? fuel.soc_pct    : -1.0f;
        state_.crate_pct_hr= fuel.valid ? fuel.crate_pct_hr: 0.0f;
        state_.status      = new_status;
        state_.valid       = pwr.valid;
        xSemaphoreGive(mutex_);
    }

    if (new_status != prev_status_) {
        ESP_LOGW(TAG, "Power status: %d → %d (V=%.2fV SoC=%.1f%%)",
                 (int)prev_status_, (int)new_status,
                 (double)state_.voltage_v, (double)state_.soc_pct);
        prev_status_ = new_status;
        if (cb_) cb_(new_status);
    }
}

PowerState PowerManager::get_state() const noexcept {
    PowerState snap{};
    if (mutex_) {
        xSemaphoreTake(mutex_, portMAX_DELAY);
        snap = state_;
        xSemaphoreGive(mutex_);
    }
    return snap;
}

} // namespace power
