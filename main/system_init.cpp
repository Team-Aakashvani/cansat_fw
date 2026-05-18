#include "system_init.hpp"
#include "config_mgr/nvs_config.hpp"
#include "esp_log.h"
#include "esp_system.h"

static const char* TAG = "SysInit";
static config_mgr::NVSConfig nvs_cfg;

namespace system_init {

esp_err_t core_init() noexcept {
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "   AAKASHVANI — CAN-7USAT india 2026            ");
    ESP_LOGI(TAG, "   (C) 2026 SVNIT. All Rights Reserved.         ");
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "ESP-IDF %s | CPU @ %dMHz",
             esp_get_idf_version(),
             CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);

    // 1. NVS + Config
    esp_err_t ret = nvs_cfg.init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "NVS init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    uint32_t boot_cnt = nvs_cfg.increment_boot_count();
    ESP_LOGI(TAG, "Boot #%lu", (unsigned long)boot_cnt);

    return ESP_OK;
}

} // namespace system_init
