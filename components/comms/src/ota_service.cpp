/**
 * @file ota_service.cpp
 * @brief OTA service implementation.
 */
#include "comms/ota_service.hpp"
#include "esp_log.h"
#include "esp_ota_ops.h"

static const char* TAG = "OTAService";

namespace comms {

esp_err_t OTAService::start() noexcept {
    if (in_progress_) return ESP_ERR_INVALID_STATE;

    update_partition_ = esp_ota_get_next_update_partition(nullptr);
    if (!update_partition_) {
        ESP_LOGE(TAG, "No OTA partition found");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Starting OTA update to partition %s at 0x%08lx",
             update_partition_->label, (unsigned long)update_partition_->address);

    esp_err_t err = esp_ota_begin(update_partition_, OTA_SIZE_UNKNOWN, &update_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed: %d", err);
        return err;
    }

    in_progress_ = true;
    written_ = 0;
    return ESP_OK;
}

esp_err_t OTAService::write_chunk(const uint8_t* data, size_t len) noexcept {
    if (!in_progress_) return ESP_ERR_INVALID_STATE;

    esp_err_t err = esp_ota_write(update_handle_, data, len);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_write failed: %d", err);
        return err;
    }

    written_ += len;
    return ESP_OK;
}

esp_err_t OTAService::finish() noexcept {
    if (!in_progress_) return ESP_ERR_INVALID_STATE;

    esp_err_t err = esp_ota_end(update_handle_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_end failed: %d", err);
        return err;
    }

    err = esp_ota_set_boot_partition(update_partition_);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_set_boot_partition failed: %d", err);
        return err;
    }

    ESP_LOGI(TAG, "OTA update successful! Reboot required.");
    in_progress_ = false;
    return ESP_OK;
}

void OTAService::abort() noexcept {
    if (!in_progress_) return;
    esp_ota_abort(update_handle_);
    in_progress_ = false;
    written_ = 0;
    ESP_LOGW(TAG, "OTA update aborted");
}

} // namespace comms
