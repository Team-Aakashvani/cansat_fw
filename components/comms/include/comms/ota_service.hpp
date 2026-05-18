/**
 * @file ota_service.hpp
 * @brief ESP-IDF OTA firmware update wrapper.
 */
#pragma once

#include "esp_ota_ops.h"
#include "esp_err.h"
#include <cstdint>
#include <cstddef>

namespace comms {

class OTAService {
public:
    OTAService() noexcept = default;

    /**
     * @brief Start OTA update.
     * Finds the next OTA partition and begins the update process.
     */
    esp_err_t start() noexcept;

    /**
     * @brief Write a chunk of firmware data.
     */
    esp_err_t write_chunk(const uint8_t* data, size_t len) noexcept;

    /**
     * @brief Finish OTA update and set boot partition.
     */
    esp_err_t finish() noexcept;

    /**
     * @brief Abort OTA update.
     */
    void abort() noexcept;

    bool is_in_progress() const noexcept { return in_progress_; }
    size_t bytes_written() const noexcept { return written_; }

private:
    esp_ota_handle_t update_handle_ = 0;
    const esp_partition_t* update_partition_ = nullptr;
    bool in_progress_ = false;
    size_t written_ = 0;
};

} // namespace comms
