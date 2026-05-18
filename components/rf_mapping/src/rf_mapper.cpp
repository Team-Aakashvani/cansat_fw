#include "rf_mapping/rf_mapper.hpp"
#include "esp_log.h"
#include <cstdio>
#include <cstring>
#include <algorithm>

static const char* TAG = "RFMapper";

namespace rf_mapping {

void RFMapper::init(nav::FlightComputer& fc, 
                  drivers::CC1101& scan,
                  control::MotorMixer& motors,
                  logging::SDLogger& sd,
                  SemaphoreHandle_t fc_mutex) noexcept {
    fc_ = &fc;
    scan_ = &scan;
    motors_ = &motors;
    sd_ = &sd;
    fc_mutex_ = fc_mutex;
}

void RFMapper::start() noexcept {
    if (active_.load()) return;
    active_.store(true);
    BaseType_t ret = xTaskCreatePinnedToCore(task_entry, "rf_map", 4096, this, 3, &task_, 1);
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to spawn RF mapping task");
        active_.store(false);
        return;
    }
    ESP_LOGI(TAG, "RF Mapping mission started");
}

void RFMapper::stop() noexcept {
    active_.store(false);
    // Task will exit on next loop iteration
}

void RFMapper::task_entry(void* arg) noexcept {
    static_cast<RFMapper*>(arg)->run();
    vTaskDelete(nullptr);
}

void RFMapper::run() noexcept {
    float angle = 0.0f;
    bool direction_forward = true;

    // Ensure scanner is in RX mode
    scan_->set_frequency((uint32_t)SCAN_FREQ_HZ);

    while (active_.load()) {
        // 1. Move servo
        motors_->set_servo_angle((double)angle);
        vTaskDelay(pdMS_TO_TICKS(SETTLE_MS));

        // 2. Sample RSSI
        // set_frequency handles SPI bus acquisition
        scan_->set_frequency((uint32_t)SCAN_FREQ_HZ); 
        vTaskDelay(pdMS_TO_TICKS(10)); // Settle PLL
        int8_t rssi = scan_->read_rssi_dbm();

        // 3. Snapshot FC state
        nav::FlightComputerOutput fc_out{};
        bool valid = false;
        if (fc_mutex_) {
            xSemaphoreTake(fc_mutex_, portMAX_DELAY);
            fc_out = fc_->last_output;
            valid = fc_->output_valid;
            xSemaphoreGive(fc_mutex_);
        }

        // 4. Log data synchronized
        if (valid) {
            char log_buf[128];
            // Format: RFMAP,Time,PosX,PosY,PosZ,Angle,RSSI
            snprintf(log_buf, sizeof(log_buf), "RFMAP,%.3f,%.2f,%.2f,%.2f,%.1f,%d\n",
                     fc_out.t_s,
                     fc_out.imm.nav.p(0), // East (m)
                     fc_out.imm.nav.p(1), // North (m)
                     fc_out.imm.nav.p(2), // Up (m)
                     (double)angle,
                     (int)rssi);
            sd_->write_line(log_buf);
            ESP_LOGD(TAG, "Sample: A=%.1f RSSI=%d P=[%.1f, %.1f]", 
                     (double)angle, (int)rssi, fc_out.imm.nav.p(0), fc_out.imm.nav.p(1));
        }

        // 5. Update angle for sweep
        if (direction_forward) {
            angle += STEP_DEG;
            if (angle >= 180.0f) {
                angle = 180.0f;
                direction_forward = false;
            }
        } else {
            angle -= STEP_DEG;
            if (angle <= 0.0f) {
                angle = 0.0f;
                direction_forward = true;
            }
        }
    }
    
    // Return servo to neutral on exit
    motors_->set_servo_angle(90.0);
    ESP_LOGI(TAG, "RF Mapping mission task stopped");
}

} // namespace rf_mapping
