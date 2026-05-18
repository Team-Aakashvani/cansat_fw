/**
 * @file sd_logger.cpp
 * @brief Buffered FAT SD-card logger implementation.
 */
#include "logging/sd_logger.hpp"
#include "esp_log.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "esp_timer.h"
#include <cstring>
#include <cstdio>

static const char* TAG     = "SDLogger";
static const char* MOUNT   = "/sdcard";

namespace logging {

// ---------------------------------------------------------------------------
// init
// ---------------------------------------------------------------------------
esp_err_t SDLogger::init(int clk_pin, int cmd_pin, int d0_pin) noexcept {
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;  // 40MHz

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.clk  = (gpio_num_t)clk_pin;
    slot.cmd  = (gpio_num_t)cmd_pin;
    slot.d0   = (gpio_num_t)d0_pin;
    slot.width = 1;                              // 1-bit SDIO (safe for all cards)
    slot.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP;

    esp_vfs_fat_sdmmc_mount_config_t mnt{};
    mnt.format_if_mount_failed = false;
    mnt.max_files              = 4;
    mnt.allocation_unit_size   = 16 * 1024;

    esp_err_t ret = esp_vfs_fat_sdmmc_mount(MOUNT, &host, &slot, &mnt, &card_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed (%d): %s", ret, esp_err_to_name(ret));
        return ret;
    }
    sdmmc_card_print_info(stdout, card_);
    mounted_ = true;

    // Read boot counter from NVS to create unique filename
    uint32_t boot_count = 0;
    nvs_handle_t nvs;
    if (nvs_open("cansat", NVS_READWRITE, &nvs) == ESP_OK) {
        nvs_get_u32(nvs, "boot_count", &boot_count);
        ++boot_count;
        nvs_set_u32(nvs, "boot_count", boot_count);
        nvs_commit(nvs);
        nvs_close(nvs);
    }

    char fname[64];
    snprintf(fname, sizeof(fname), "%s/CANSAT_%04lu.csv", MOUNT, (unsigned long)boot_count);
    fp_ = fopen(fname, "w");
    if (!fp_) {
        ESP_LOGE(TAG, "Failed to open log file: %s", fname);
        return ESP_FAIL;
    }
    write_header();
    ESP_LOGI(TAG, "Logging to %s", fname);
    return ESP_OK;
}

// ---------------------------------------------------------------------------
// write_line
// ---------------------------------------------------------------------------
bool SDLogger::write_line(const char* line) noexcept {
    if (!mounted_ || !fp_ || !line) return false;
    return ring_push(line);
}

// ---------------------------------------------------------------------------
// flush
// ---------------------------------------------------------------------------
void SDLogger::flush() noexcept {
    if (!mounted_ || !fp_) return;
    char tmp[LINE_BUF_LEN];
    while (ring_pop(tmp)) {
        if (fputs(tmp, fp_) == EOF) {
            ++flush_errors_;
            ESP_LOGW(TAG, "fputs failed");
        } else {
            ++lines_written_;
        }
    }
    if (fflush(fp_) != 0) {
        ++flush_errors_;
        ESP_LOGW(TAG, "fflush failed");
    }
}

// ---------------------------------------------------------------------------
// close
// ---------------------------------------------------------------------------
void SDLogger::close() noexcept {
    if (fp_) {
        flush();
        fclose(fp_);
        fp_ = nullptr;
    }
    if (mounted_) {
        esp_vfs_fat_sdcard_unmount(MOUNT, card_);
        mounted_ = false;
        ESP_LOGI(TAG, "SD card unmounted. %lu lines written.", (unsigned long)lines_written_);
    }
}

// ---------------------------------------------------------------------------
// Internal helpers
// ---------------------------------------------------------------------------
void SDLogger::write_header() noexcept {
    if (!fp_) return;
    // CAN-7USAT CSV column names
    fputs("TEAM_ID,MISSION_TIME,PACKET_COUNT,ALTITUDE,PRESSURE,TEMPERATURE,"
          "VOLTAGE,GNSS_TIME,LATITUDE,LONGITUDE,GNSS_ALT,SATS,"
          "TILT_X,TILT_Y,ROT_Z,SOFTWARE_STATE\n", fp_);
    fflush(fp_);
}

bool SDLogger::ring_push(const char* line) noexcept {
    if (ring_count_ >= RING_LINES) {
        ESP_LOGW(TAG, "Ring buffer full — dropping line");
        return false;
    }
    strncpy(ring_[ring_head_], line, LINE_BUF_LEN - 1);
    ring_[ring_head_][LINE_BUF_LEN - 1] = '\0';
    ring_head_ = (ring_head_ + 1) % RING_LINES;
    ++ring_count_;
    return true;
}

bool SDLogger::ring_pop(char* out) noexcept {
    if (ring_count_ == 0) return false;
    strncpy(out, ring_[ring_tail_], LINE_BUF_LEN - 1);
    out[LINE_BUF_LEN - 1] = '\0';
    ring_tail_ = (ring_tail_ + 1) % RING_LINES;
    --ring_count_;
    return true;
}

} // namespace logging
