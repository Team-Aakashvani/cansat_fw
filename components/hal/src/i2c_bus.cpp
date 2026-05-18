/**
 * @file i2c_bus.cpp
 * @brief I2C bus implementation.
 */
#include "hal/i2c_bus.hpp"
#include "esp_log.h"
#include <cstring>

static const char* TAG = "I2CBus";

namespace hal {

I2CBus::~I2CBus() noexcept {
    if (bus_) {
        i2c_master_bus_rm_device(nullptr);
        i2c_del_master_bus(bus_);
        bus_ = nullptr;
    }
    if (mutex_) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

esp_err_t I2CBus::init(i2c_port_t port, int sda_pin, int scl_pin,
                        uint32_t speed_hz) noexcept {
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Mutex create failed");
        return ESP_ERR_NO_MEM;
    }

    i2c_master_bus_config_t cfg{};
    cfg.i2c_port      = port;
    cfg.sda_io_num    = (gpio_num_t)sda_pin;
    cfg.scl_io_num    = (gpio_num_t)scl_pin;
    cfg.clk_source    = I2C_CLK_SRC_DEFAULT;
    cfg.glitch_ignore_cnt = 7;
    cfg.flags.enable_internal_pullup = true;
    cfg.intr_priority = 0;

    esp_err_t ret = i2c_new_master_bus(&cfg, &bus_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "I2C port %d initialised (SDA=%d SCL=%d @%luHz)",
             (int)port, sda_pin, scl_pin, (unsigned long)speed_hz);
    return ESP_OK;
}

esp_err_t I2CBus::write_reg(uint8_t addr, uint8_t reg,
                             const uint8_t* data, size_t len) noexcept {
    if (!bus_ || !mutex_) return ESP_ERR_INVALID_STATE;

    // Build payload: [reg, data...]
    uint8_t buf[len + 1];
    buf[0] = reg;
    memcpy(buf + 1, data, len);

    esp_err_t ret = ESP_FAIL;
    for (int attempt = 0; attempt < I2C_RETRIES; ++attempt) {
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE)
            return ESP_ERR_TIMEOUT;

        i2c_master_dev_handle_t dev = nullptr;
        i2c_device_config_t dev_cfg{};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address  = addr;
        dev_cfg.scl_speed_hz    = 400000;

        ret = i2c_master_bus_add_device(bus_, &dev_cfg, &dev);
        if (ret == ESP_OK) {
            ret = i2c_master_transmit(dev, buf, len + 1,
                                      pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            i2c_master_bus_rm_device(dev);
        }
        xSemaphoreGive(mutex_);
        if (ret == ESP_OK) break;
        vTaskDelay(1);
    }
    return ret;
}

esp_err_t I2CBus::read_reg(uint8_t addr, uint8_t reg,
                            uint8_t* buf, size_t len) noexcept {
    if (!bus_ || !mutex_) return ESP_ERR_INVALID_STATE;

    esp_err_t ret = ESP_FAIL;
    for (int attempt = 0; attempt < I2C_RETRIES; ++attempt) {
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE)
            return ESP_ERR_TIMEOUT;

        i2c_master_dev_handle_t dev = nullptr;
        i2c_device_config_t dev_cfg{};
        dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
        dev_cfg.device_address  = addr;
        dev_cfg.scl_speed_hz    = 400000;

        ret = i2c_master_bus_add_device(bus_, &dev_cfg, &dev);
        if (ret == ESP_OK) {
            ret = i2c_master_transmit_receive(dev, &reg, 1, buf, len,
                                              pdMS_TO_TICKS(I2C_TIMEOUT_MS));
            i2c_master_bus_rm_device(dev);
        }
        xSemaphoreGive(mutex_);
        if (ret == ESP_OK) break;
        vTaskDelay(1);
    }
    return ret;
}

bool I2CBus::probe(uint8_t addr) noexcept {
    if (!bus_) return false;
    return (i2c_master_probe(bus_, addr,
                             pdMS_TO_TICKS(I2C_TIMEOUT_MS)) == ESP_OK);
}

} // namespace hal
