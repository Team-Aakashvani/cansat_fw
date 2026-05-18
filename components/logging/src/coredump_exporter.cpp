/**
 * @file coredump_exporter.cpp
 * @brief Coredump export implementation.
 */
#include "logging/coredump_exporter.hpp"
#include "esp_core_dump.h"
#include "esp_log.h"
#include "esp_partition.h"
#include "nvs_flash.h"
#include <cstdio>
#include <cstdlib>

static const char* TAG = "CoredumpExporter";

namespace logging {

esp_err_t CoredumpExporter::check_and_export() noexcept {
    size_t address = 0;
    size_t size = 0;

    // Check if coredump exists
    esp_err_t err = esp_core_dump_image_get(&address, &size);
    if (err == ESP_ERR_NOT_FOUND) {
        return ESP_OK; // Normal boot
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get core dump info: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGW(TAG, "!!! Crash detected in previous session !!!");
    ESP_LOGW(TAG, "Core dump size: %zu bytes. Exporting to SD card...", size);

    if (esp_core_dump_image_check() != ESP_OK) {
        ESP_LOGE(TAG, "Core dump integrity check failed (checksum mismatch)");
    }

    // Get boot count from NVS (already incremented by core_init)
    uint32_t boot_count = 0;
    nvs_handle_t nvs;
    if (nvs_open("cansat", NVS_READONLY, &nvs) == ESP_OK) {
        nvs_get_u32(nvs, "boot_count", &boot_count);
        nvs_close(nvs);
    }

    char fname[64];
    snprintf(fname, sizeof(fname), "/sdcard/CRASH_%04lu.bin", (unsigned long)boot_count);

    FILE* f = fopen(fname, "wb");
    if (!f) {
        ESP_LOGE(TAG, "Could not open %s for writing", fname);
        return ESP_FAIL;
    }

    const esp_partition_t* partition = esp_partition_find_first(
        ESP_PARTITION_TYPE_DATA, 
        ESP_PARTITION_SUBTYPE_DATA_COREDUMP, 
        NULL
    );

    if (!partition) {
        ESP_LOGE(TAG, "Coredump partition missing from table");
        fclose(f);
        return ESP_FAIL;
    }

    // Export in 4KB chunks
    uint8_t buffer[4096];
    size_t remaining = size;
    size_t offset = 0;
    
    while (remaining > 0) {
        size_t to_read = (remaining > sizeof(buffer)) ? sizeof(buffer) : remaining;
        err = esp_partition_read(partition, offset, buffer, to_read);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Flash read failed at %zu", offset);
            break;
        }
        
        if (fwrite(buffer, 1, to_read, f) != to_read) {
            ESP_LOGE(TAG, "SD write failed at %zu", offset);
            err = ESP_FAIL;
            break;
        }
        
        remaining -= to_read;
        offset += to_read;
    }

    fclose(f);

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "SUCCESS: Coredump saved to %s", fname);
        // Wipe it so we don't loop
        esp_core_dump_image_erase();
        ESP_LOGI(TAG, "Coredump partition cleared.");
    } else {
        ESP_LOGE(TAG, "Export FAILED.");
    }

    return err;
}

} // namespace logging
