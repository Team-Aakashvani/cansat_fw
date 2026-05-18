/**
 * @file sd_logger.hpp
 * @brief Buffered FAT SD-card telemetry logger.
 *
 * Opens a new CSV file on the SD card at boot, writes one telemetry line per
 * call to write_line(). Lines are buffered in RAM (configurable ring) and
 * flushed to disk either periodically or when the buffer is full.
 *
 * SD card is accessed via SDMMC peripheral (1-bit or 4-bit SDIO).
 * Filesystem: FAT32 on SDMMC.
 *
 * File naming: "/sdcard/CANSAT_<boot_count>.csv"
 * A header line matching the CAN-7USAT CSV column names is written first.
 *
 * Thread safety: write_line() is safe to call from any single task.
 * Do NOT call from multiple tasks simultaneously without external locking.
 */
#pragma once

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "nav/config.hpp"
#include <cstdio>
#include <cstdint>
#include <cstddef>

namespace logging {

class SDLogger {
public:
    static constexpr size_t LINE_BUF_LEN    = 256;
    static constexpr size_t RING_LINES      = 64;    ///< In-RAM ring buffer depth
    static constexpr uint32_t FLUSH_INTERVAL_MS = 5000;  ///< Auto-flush period

    SDLogger() noexcept = default;
    ~SDLogger() noexcept { close(); }

    /**
     * @brief Mount SD card and open log file.
     * @param clk_pin  SDMMC CLK GPIO.
     * @param cmd_pin  SDMMC CMD GPIO.
     * @param d0_pin   SDMMC DATA0 GPIO (D1–D3 optional for 4-bit).
     * @return ESP_OK on success; ESP_ERR_NOT_FOUND if no card detected.
     */
    esp_err_t init(int clk_pin, int cmd_pin, int d0_pin) noexcept;

    /**
     * @brief Write a CSV telemetry line to the SD card.
     * Line is added to the ring buffer. Flush happens automatically.
     * @param line  Null-terminated CSV string (including '\n').
     * @return true on success; false if card not mounted or buffer overflow.
     */
    bool write_line(const char* line) noexcept;

    /**
     * @brief Flush all buffered lines to disk.
     * Call from the logging task at the desired flush interval, or call
     * explicitly before power-off.
     */
    void flush() noexcept;

    /// Unmount the SD card safely.
    void close() noexcept;

    bool is_mounted() const noexcept { return mounted_; }
    uint32_t lines_written() const noexcept { return lines_written_; }
    uint32_t flush_errors()  const noexcept { return flush_errors_; }

private:
    FILE*    fp_           = nullptr;
    bool     mounted_      = false;
    uint32_t lines_written_= 0;
    uint32_t flush_errors_ = 0;

    // Simple ring buffer of fixed-length line slots
    char     ring_[RING_LINES][LINE_BUF_LEN];
    uint32_t ring_head_    = 0;   ///< Next write slot
    uint32_t ring_tail_    = 0;   ///< Next read/flush slot
    uint32_t ring_count_   = 0;   ///< Pending unflushed lines

    sdmmc_card_t* card_    = nullptr;

    void write_header() noexcept;
    bool ring_push(const char* line) noexcept;
    bool ring_pop(char* out) noexcept;
};

} // namespace logging
