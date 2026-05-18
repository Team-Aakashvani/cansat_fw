/**
 * @file watchdog.cpp
 * @brief Task-level watchdog implementation.
 */
#include "watchdog/watchdog.hpp"
#include "esp_log.h"

static const char* TAG = "WDT";

namespace watchdog {

esp_err_t Watchdog::init(bool panic_on_timeout) noexcept {
    esp_task_wdt_config_t cfg{};
    cfg.timeout_ms     = TWDT_TIMEOUT_S * 1000u;
    cfg.idle_core_mask = 0;          // Don't watch idle tasks
    cfg.trigger_panic  = panic_on_timeout;

    esp_err_t ret = esp_task_wdt_reconfigure(&cfg);
    if (ret == ESP_ERR_INVALID_STATE) {
        // TWDT not yet initialised — init it
        ret = esp_task_wdt_init(&cfg);
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "TWDT init failed: %d", ret);
        return ret;
    }
    armed_ = true;
    ESP_LOGI(TAG, "TWDT armed (%lus timeout, panic=%d)",
             (unsigned long)TWDT_TIMEOUT_S, (int)panic_on_timeout);
    return ESP_OK;
}

esp_err_t Watchdog::register_task() noexcept {
    esp_err_t ret = esp_task_wdt_add(nullptr);  // nullptr = current task
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "WDT register_task failed: %d", ret);
    }
    return ret;
}

void Watchdog::deregister_task() noexcept {
    esp_task_wdt_delete(nullptr);
}

void Watchdog::ping() noexcept {
    esp_task_wdt_reset();
}

} // namespace watchdog
