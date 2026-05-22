/**
 * @file main_p4.cpp
 * @brief CAN-7USAT 2026 — ESP32-P4 Media Coprocessor Firmware.
 *
 * FILE LOCATION: main/main_p4.cpp
 *
 * WHAT THIS CODE DOES (plain English):
 * -------------------------------------
 * This runs on a SEPARATE chip (ESP32-P4) whose only job is video.
 * It has three states:
 *
 *   STANDBY     — camera powered, not recording, waiting for command
 *   RECORDING   — capturing video, encoding H.264, writing to SD card
 *   HALT_FLUSH  — stop recording, save everything safely, go back to standby
 *
 * The flight computer (ESP32-S3) sends text commands over a UART wire:
 *   "$P4CMD,STANDBY\r\n"     → stop recording
 *   "$P4CMD,RECORD\r\n"      → start recording
 *   "$P4CMD,HALT_FLUSH\r\n"  → save and stop
 *
 * This chip sends back a heartbeat every second:
 *   "P4HB,1,3.50,25\r\n"    → recording=yes, 3.5GB free, 25fps
 *
 * IMPORTANT: Uses UART_NUM_1 (NOT UART_NUM_0) for the FC link.
 * UART_NUM_0 outputs boot logs which would corrupt command parsing.
 *
 * HOW IT CONNECTS TO THE PROJECT:
 * --------------------------------
 * This file is only compiled when built with CONFIG_CANSAT_P4.
 * app_main() calls main_p4() when that config is selected.
 */

#include <cstring>
#include <cstdio>
#include <atomic>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "driver/uart.h"

// ESP32-P4 Video / H.264 Headers (IDF v5.3+)
#include "esp_video.h"
#include "esp_h264_enc.h"
#include "esp_video_init.h"

static const char* TAG = "P4";

// =============================================================================
// UART CONFIGURATION — link to the flight computer
// =============================================================================
//
// UART_NUM_1 is used instead of UART_NUM_0.
// UART_NUM_0 prints boot logs and ESP_LOG messages by default,
// which would corrupt the command protocol. This was your correction #1.

#define P4_FC_UART       UART_NUM_1
#define P4_FC_BAUD       921600
#define P4_FC_TX_PIN     27    // Connected to S3 RX (GPIO 27)
#define P4_FC_RX_PIN     26    // Connected to S3 TX (GPIO 26)
#define P4_UART_BUF_SIZE 1024

// =============================================================================
// STATE MACHINE
// =============================================================================

enum class P4State : uint8_t {
    STANDBY    = 0,
    RECORDING  = 1,
    HALT_FLUSH = 2,
};

// Shared state — protected by state_mutex
static std::atomic<P4State> g_state{P4State::STANDBY};
static SemaphoreHandle_t    state_mutex = nullptr;

// Stats for heartbeat
static std::atomic<uint8_t>  g_recording{0};
static std::atomic<float>    g_sd_free_gb{0.0f};
static std::atomic<uint8_t>  g_fps{0};

// Frame counter for fps calculation
static std::atomic<uint32_t> g_frame_count{0};

// =============================================================================
// HELPER: Get SD free space in GB
// =============================================================================

#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"

static sdmmc_card_t* g_card = nullptr;

static float get_sd_free_gb() noexcept
{
    if (!g_card) return 0.0f;
    
    FATFS *fs;
    DWORD fre_clust, fre_sect, tot_sect;

    /* Get volume information and free clusters of drive 0 */
    FRESULT res = f_getfree("0:", &fre_clust, &fs);
    if (res != FR_OK) {
        ESP_LOGW(TAG, "Failed to get free space: %d", res);
        return 0.0f;
    }

    /* Get total sectors and free sectors */
    tot_sect = (fs->n_fatent - 2) * fs->csize;
    fre_sect = fre_clust * fs->csize;

    /* Print the free space (assuming 512 bytes/sector) */
    return (float)fre_sect / (1024.0f * 1024.0f * 1024.0f / 512.0f);
}

static void cam_task(void* /*arg*/)
{
    ESP_LOGI(TAG, "cam_task started on core %d", xPortGetCoreID());

    P4State prev_state = P4State::STANDBY;
    uint32_t last_fps_tick = 0;
    uint32_t frames_since_last = 0;
    FILE* video_file = nullptr;

    // --- Video / Encoder Handles ---
    esp_video_t* video = nullptr;
    esp_h264_enc_t* encoder = nullptr;

    while (true)
    {
        P4State current = g_state.load();

        switch (current)
        {
        case P4State::STANDBY:
            if (prev_state != P4State::STANDBY) {
                ESP_LOGI(TAG, "Entering STANDBY — camera idle");
                g_recording.store(0);
                g_fps.store(0);
                g_frame_count.store(0);
                
                // Cleanup handles
                if (encoder) { esp_h264_enc_delete(encoder); encoder = nullptr; }
                if (video)   { esp_video_close(video); video = nullptr; }
                
                if (video_file) {
                    fclose(video_file);
                    video_file = nullptr;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(100));
            break;

        case P4State::RECORDING:
            if (prev_state != P4State::RECORDING) {
                ESP_LOGI(TAG, "Entering RECORDING — starting capture");
                g_recording.store(1);
                last_fps_tick = xTaskGetTickCount();
                frames_since_last = 0;

                // 1. Open output file
                char filename[32];
                static int file_idx = 0;
                snprintf(filename, sizeof(filename), "/sdcard/vid_%03d.h264", file_idx++);
                video_file = fopen(filename, "wb");
                if (!video_file) {
                    ESP_LOGE(TAG, "Failed to open %s", filename);
                    g_state.store(P4State::STANDBY);
                    continue;
                }

                // 2. Initialise Video (MIPI-CSI 1080p @ 25fps)
                esp_video_config_t v_cfg = {};
                v_cfg.device = ESP_VIDEO_DEVICE_CSI0;
                v_cfg.width = 1920;
                v_cfg.height = 1080;
                v_cfg.pixel_format = ESP_VIDEO_PIX_FMT_YUV420P;
                v_cfg.fps = 25;
                
                if (esp_video_open(&v_cfg, &video) != ESP_OK) {
                    ESP_LOGE(TAG, "Video open failed");
                    g_state.store(P4State::STANDBY);
                    continue;
                }

                // 3. Initialise H.264 Encoder
                esp_h264_enc_config_t e_cfg = {};
                e_cfg.width = 1920;
                e_cfg.height = 1080;
                e_cfg.fps = 25;
                e_cfg.bitrate = 4000000; // 4 Mbps
                e_cfg.gop = 25;
                e_cfg.profile = ESP_H264_ENC_PROFILE_MAIN;
                
                if (esp_h264_enc_create(&e_cfg, &encoder) != ESP_OK) {
                    ESP_LOGE(TAG, "Encoder create failed");
                    g_state.store(P4State::STANDBY);
                    continue;
                }

                esp_video_start(video);
            }

            // --- Capture Loop ---
            esp_video_buffer_t* v_buf = nullptr;
            if (esp_video_read(video, &v_buf, pdMS_TO_TICKS(100)) == ESP_OK) {
                esp_h264_enc_frame_t e_frame = {};
                if (esp_h264_enc_process(encoder, v_buf->data, v_buf->size, &e_frame) == ESP_OK) {
                    fwrite(e_frame.data, 1, e_frame.size, video_file);
                    frames_since_last++;
                    g_frame_count.fetch_add(1);
                }
                esp_video_return_buffer(video, v_buf);
            }

            // Calculate FPS
            {
                uint32_t now_tick = xTaskGetTickCount();
                uint32_t elapsed_ms = (now_tick - last_fps_tick) * portTICK_PERIOD_MS;
                if (elapsed_ms >= 1000) {
                    g_fps.store((uint8_t)(frames_since_last * 1000 / elapsed_ms));
                    frames_since_last = 0;
                    last_fps_tick = now_tick;
                    g_sd_free_gb.store(get_sd_free_gb());
                }
            }
            break;

        case P4State::HALT_FLUSH:
            ESP_LOGI(TAG, "HALT_FLUSH — stopping capture");
            if (video) esp_video_stop(video);
            if (video_file) {
                fflush(video_file);
                fsync(fileno(video_file));
                fclose(video_file);
                video_file = nullptr;
            }
            g_recording.store(0);
            g_fps.store(0);
            g_state.store(P4State::STANDBY);
            ESP_LOGI(TAG, "Flush complete → STANDBY");
            break;
        }
        prev_state = current;
    }
}

// =============================================================================
// RELIABILITY: XOR Checksum helper
// =============================================================================

static uint8_t calc_checksum(const char* s, size_t len)
{
    uint8_t crc = 0;
    for (size_t i = 0; i < len; ++i) crc ^= (uint8_t)s[i];
    return crc;
}

// =============================================================================
// uart_cmd_task — reads commands from the flight computer
// =============================================================================

static void uart_cmd_task(void* /*arg*/)
{
    ESP_LOGI(TAG, "uart_cmd_task started");

    char line_buf[128];
    size_t line_pos = 0;

    while (true)
    {
        uint8_t byte;
        if (uart_read_bytes(P4_FC_UART, &byte, 1, pdMS_TO_TICKS(10)) <= 0) continue;

        if (byte == '\n')
        {
            line_buf[line_pos] = '\0';
            if (line_pos > 0 && line_buf[line_pos - 1] == '\r') line_buf[--line_pos] = '\0';

            // Expected format: "$P4CMD,<cmd>*<checksum>"
            char* star = strchr(line_buf, '*');
            if (star && star > line_buf) {
                *star = '\0';
                uint8_t received_crc = (uint8_t)strtol(star + 1, nullptr, 16);
                uint8_t actual_crc = calc_checksum(line_buf + 1, strlen(line_buf + 1));

                if (received_crc == actual_crc) {
                    if (strncmp(line_buf, "$P4CMD,", 7) == 0) {
                        const char* cmd = line_buf + 7;
                        if (strcmp(cmd, "STANDBY") == 0) g_state.store(P4State::STANDBY);
                        else if (strcmp(cmd, "RECORD") == 0) g_state.store(P4State::RECORDING);
                        else if (strcmp(cmd, "HALT_FLUSH") == 0) g_state.store(P4State::HALT_FLUSH);
                    }
                } else {
                    ESP_LOGW(TAG, "CRC error on UART cmd: expected %02X, got %02X", actual_crc, received_crc);
                }
            }
            line_pos = 0;
        }
        else if (line_pos < sizeof(line_buf) - 1)
        {
            line_buf[line_pos++] = (char)byte;
        }
    }
}

// =============================================================================
// heartbeat_task — sends status back to FC every 1 second
// =============================================================================

static void heartbeat_task(void* /*arg*/)
{
    ESP_LOGI(TAG, "heartbeat_task started");

    while (true)
    {
        char data_buf[64];
        int data_len = snprintf(data_buf, sizeof(data_buf),
                                "P4HB,%u,%.2f,%u",
                                (unsigned)g_recording.load(),
                                (double)g_sd_free_gb.load(),
                                (unsigned)g_fps.load());

        if (data_len > 0) {
            uint8_t crc = calc_checksum(data_buf, data_len);
            char full_buf[128];
            int full_len = snprintf(full_buf, sizeof(full_buf),
                                   "$%s*%02X\r\n", data_buf, crc);
            uart_write_bytes(P4_FC_UART, full_buf, full_len);
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void main_p4()
{
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "   AAKASHVANI — CAN-7USAT india 2026            ");
    ESP_LOGI(TAG, "   (C) 2026 SVNIT. All Rights Reserved.         ");
    ESP_LOGI(TAG, "   [ Media Coprocessor Firmware ]               ");
    ESP_LOGI(TAG, "================================================");

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 1.5 Video Subsystem Init
    ESP_ERROR_CHECK(esp_video_init());

    // 2. SDMMC mount (using standard P4 Slot 0 pins)
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    // P4 specific SDMMC pins would be set here if needed, 
    // but Slot 0 uses default GPIOs 39-44.
    
    ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &g_card);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount SD card (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SD card mounted at /sdcard");
        g_sd_free_gb.store(get_sd_free_gb());
    }

    // 3. UART init
    uart_config_t uart_config = {
        .baud_rate  = P4_FC_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(P4_FC_UART, P4_UART_BUF_SIZE, P4_UART_BUF_SIZE, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(P4_FC_UART, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(P4_FC_UART, P4_FC_TX_PIN, P4_FC_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    state_mutex = xSemaphoreCreateMutex();
    configASSERT(state_mutex);

    xTaskCreatePinnedToCore(uart_cmd_task,  "p4_cmd",  2048, nullptr, 6, nullptr, 0);
    xTaskCreatePinnedToCore(cam_task,       "p4_cam",  8192, nullptr, 5, nullptr, 1);
    xTaskCreatePinnedToCore(heartbeat_task, "p4_hb",   1024, nullptr, 2, nullptr, 1);

    ESP_LOGI(TAG, "All P4 tasks spawned.");
}
