/**
 * @file nvs_config.cpp
 * @brief NVS-backed persistent configuration implementation.
 */
#include "config_mgr/nvs_config.hpp"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "NVSConfig";

namespace config_mgr {

esp_err_t NVSConfig::init() noexcept {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW(TAG, "NVS corrupt — erasing");
        nvs_flash_erase();
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "nvs_flash_init: %d", ret);
        return ret;
    }
    ready_ = true;
    ESP_LOGI(TAG, "NVS ready (team_id=%u ground_alt=%.1fm boot=%lu)",
             (unsigned)get_team_id(),
             (double)get_ground_alt_m(),
             (unsigned long)get_boot_count());
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// Getters
// ---------------------------------------------------------------------------
uint16_t NVSConfig::get_team_id() const noexcept {
    nvs_handle_t nvs;
    uint16_t val = (uint16_t)nav::TELEM_CFG.team_id;  // compile-time default
    if (open_ro(nvs) == ESP_OK) {
        nvs_get_u16(nvs, "team_id", &val);
        nvs_close(nvs);
    }
    return val;
}

float NVSConfig::get_ground_alt_m() const noexcept {
    nvs_handle_t nvs;
    float val = 0.0f;
    if (open_ro(nvs) == ESP_OK) {
        uint32_t raw = 0;
        if (nvs_get_u32(nvs, "gnd_alt", &raw) == ESP_OK) {
            memcpy(&val, &raw, sizeof(float));
        }
        nvs_close(nvs);
    }
    return val;
}

float NVSConfig::get_baro_offset_pa() const noexcept {
    nvs_handle_t nvs;
    float val = 0.0f;
    if (open_ro(nvs) == ESP_OK) {
        uint32_t raw = 0;
        if (nvs_get_u32(nvs, "baro_off", &raw) == ESP_OK) {
            memcpy(&val, &raw, sizeof(float));
        }
        nvs_close(nvs);
    }
    return val;
}

uint32_t NVSConfig::get_boot_count() const noexcept {
    nvs_handle_t nvs;
    uint32_t val = 0;
    if (open_ro(nvs) == ESP_OK) {
        nvs_get_u32(nvs, "boot_cnt", &val);
        nvs_close(nvs);
    }
    return val;
}

void NVSConfig::get_mag_cal(float out[3]) const noexcept {
    out[0] = out[1] = out[2] = 0.0f;
    nvs_handle_t nvs;
    if (open_ro(nvs) == ESP_OK) {
        size_t sz = 3 * sizeof(float);
        nvs_get_blob(nvs, "mag_cal", out, &sz);
        nvs_close(nvs);
    }
}

// ---------------------------------------------------------------------------
// Setters
// ---------------------------------------------------------------------------
esp_err_t NVSConfig::set_team_id(uint16_t id) noexcept {
    nvs_handle_t nvs;
    esp_err_t ret = open_rw(nvs);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_u16(nvs, "team_id", id);
    if (ret == ESP_OK) ret = nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "team_id set to %u", (unsigned)id);
    return ret;
}

esp_err_t NVSConfig::set_ground_alt_m(float alt_m) noexcept {
    nvs_handle_t nvs;
    esp_err_t ret = open_rw(nvs);
    if (ret != ESP_OK) return ret;
    uint32_t raw;
    memcpy(&raw, &alt_m, sizeof(float));
    ret = nvs_set_u32(nvs, "gnd_alt", raw);
    if (ret == ESP_OK) ret = nvs_commit(nvs);
    nvs_close(nvs);
    ESP_LOGI(TAG, "ground_alt set to %.2f m", (double)alt_m);
    return ret;
}

esp_err_t NVSConfig::set_baro_offset_pa(float offset_pa) noexcept {
    nvs_handle_t nvs;
    esp_err_t ret = open_rw(nvs);
    if (ret != ESP_OK) return ret;
    uint32_t raw;
    memcpy(&raw, &offset_pa, sizeof(float));
    ret = nvs_set_u32(nvs, "baro_off", raw);
    if (ret == ESP_OK) ret = nvs_commit(nvs);
    nvs_close(nvs);
    return ret;
}

esp_err_t NVSConfig::set_mag_cal(const float cal[3]) noexcept {
    nvs_handle_t nvs;
    esp_err_t ret = open_rw(nvs);
    if (ret != ESP_OK) return ret;
    ret = nvs_set_blob(nvs, "mag_cal", cal, 3 * sizeof(float));
    if (ret == ESP_OK) ret = nvs_commit(nvs);
    nvs_close(nvs);
    return ret;
}

uint32_t NVSConfig::increment_boot_count() noexcept {
    nvs_handle_t nvs;
    uint32_t cnt = 0;
    if (open_rw(nvs) == ESP_OK) {
        nvs_get_u32(nvs, "boot_cnt", &cnt);
        ++cnt;
        nvs_set_u32(nvs, "boot_cnt", cnt);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    return cnt;
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------
esp_err_t NVSConfig::open_rw(nvs_handle_t& out) const noexcept {
    return nvs_open(NAMESPACE, NVS_READWRITE, &out);
}

esp_err_t NVSConfig::open_ro(nvs_handle_t& out) const noexcept {
    return nvs_open(NAMESPACE, NVS_READONLY, &out);
}

} // namespace config_mgr
