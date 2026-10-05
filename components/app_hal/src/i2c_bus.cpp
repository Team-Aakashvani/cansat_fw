/**
 * @file i2c_bus.cpp
 * @brief I2C bus implementation.
 */
#include "app_hal/i2c_bus.hpp"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include <cstring>

static const char* TAG = "I2CBus";

namespace hal {

I2CBus::~I2CBus() noexcept {
    if (bus_) {
        for (int i = 0; i < 128; ++i) {
            if (dev_cache_[i]) {
                i2c_master_bus_rm_device(dev_cache_[i]);
                dev_cache_[i] = nullptr;
            }
        }
        i2c_del_master_bus(bus_);
        bus_ = nullptr;
    }
    if (mutex_) {
        vSemaphoreDelete(mutex_);
        mutex_ = nullptr;
    }
}

i2c_master_dev_handle_t I2CBus::get_device(uint8_t addr) noexcept {
    if (addr >= 128) return nullptr;
    if (dev_cache_[addr]) return dev_cache_[addr];

    i2c_device_config_t dev_cfg{};
    dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    dev_cfg.device_address  = addr;
    dev_cfg.scl_speed_hz    = (speed_hz_ > 0) ? speed_hz_ : 100000;

    esp_err_t ret = i2c_master_bus_add_device(bus_, &dev_cfg, &dev_cache_[addr]);
    if (ret != ESP_OK) {
        dev_cache_[addr] = nullptr;
        return nullptr;
    }
    return dev_cache_[addr];
}

esp_err_t I2CBus::init(i2c_port_t port, int sda_pin, int scl_pin,
                        uint32_t speed_hz) noexcept {
    speed_hz_ = speed_hz;
    mutex_ = xSemaphoreCreateMutex();
    if (!mutex_) {
        ESP_LOGE(TAG, "Mutex create failed");
        return ESP_ERR_NO_MEM;
    }

    for (int i = 0; i < 128; ++i) dev_cache_[i] = nullptr;

    // 1. Diagnose and unstick I2C lines if held low by hung slave
    gpio_reset_pin((gpio_num_t)sda_pin);
    gpio_reset_pin((gpio_num_t)scl_pin);
    gpio_set_direction((gpio_num_t)sda_pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_set_direction((gpio_num_t)scl_pin, GPIO_MODE_INPUT_OUTPUT_OD);
    gpio_pullup_en((gpio_num_t)sda_pin);
    gpio_pullup_en((gpio_num_t)scl_pin);
    gpio_set_level((gpio_num_t)sda_pin, 1);
    gpio_set_level((gpio_num_t)scl_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    int sda_raw = gpio_get_level((gpio_num_t)sda_pin);
    int scl_raw = gpio_get_level((gpio_num_t)scl_pin);
    ESP_LOGI(TAG, "Pre-init I2C pin state: SDA (GPIO%d) = %d, SCL (GPIO%d) = %d",
             sda_pin, sda_raw, scl_pin, scl_raw);

    if (sda_raw == 0 || scl_raw == 0) {
        ESP_LOGW(TAG, "I2C bus held LOW! Pulsing SCL 9 times to unwedge slaves...");
        for (int i = 0; i < 9; ++i) {
            gpio_set_level((gpio_num_t)scl_pin, 0);
            esp_rom_delay_us(10);
            gpio_set_level((gpio_num_t)scl_pin, 1);
            esp_rom_delay_us(10);
        }
        // Send STOP condition: SDA low -> SCL high -> SDA high
        gpio_set_level((gpio_num_t)sda_pin, 0);
        esp_rom_delay_us(10);
        gpio_set_level((gpio_num_t)scl_pin, 1);
        esp_rom_delay_us(10);
        gpio_set_level((gpio_num_t)sda_pin, 1);
        esp_rom_delay_us(10);

        sda_raw = gpio_get_level((gpio_num_t)sda_pin);
        scl_raw = gpio_get_level((gpio_num_t)scl_pin);
        ESP_LOGI(TAG, "Post-recovery I2C pin state: SDA=%d, SCL=%d", sda_raw, scl_raw);
        if (sda_raw == 0 || scl_raw == 0) {
            ESP_LOGE(TAG, "CRITICAL: Bus line still LOW! Check for ground short, inverted wiring, or missing pull-up on SDA/SCL.");
        }
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

esp_err_t I2CBus::write_raw(uint8_t addr, const uint8_t* data, size_t len) noexcept {
    if (!bus_ || !mutex_) return ESP_ERR_INVALID_STATE;

    esp_err_t ret = ESP_FAIL;
    for (int attempt = 0; attempt < I2C_RETRIES; ++attempt) {
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE)
            return ESP_ERR_TIMEOUT;

        i2c_master_dev_handle_t dev = get_device(addr);
        if (dev) {
            ret = i2c_master_transmit(dev, data, len, (int)I2C_TIMEOUT_MS);
        }
        xSemaphoreGive(mutex_);
        if (ret == ESP_OK) break;
        vTaskDelay(1);
    }
    return ret;
}

esp_err_t I2CBus::read_raw(uint8_t addr, uint8_t* buf, size_t len) noexcept {
    if (!bus_ || !mutex_) return ESP_ERR_INVALID_STATE;

    esp_err_t ret = ESP_FAIL;
    for (int attempt = 0; attempt < I2C_RETRIES; ++attempt) {
        if (xSemaphoreTake(mutex_, pdMS_TO_TICKS(I2C_TIMEOUT_MS)) != pdTRUE)
            return ESP_ERR_TIMEOUT;

        i2c_master_dev_handle_t dev = get_device(addr);
        if (dev) {
            ret = i2c_master_receive(dev, buf, len, (int)I2C_TIMEOUT_MS);
        }
        xSemaphoreGive(mutex_);
        if (ret == ESP_OK) break;
        vTaskDelay(1);
    }
    return ret;
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

        i2c_master_dev_handle_t dev = get_device(addr);
        if (dev) {
            ret = i2c_master_transmit(dev, buf, len + 1, (int)I2C_TIMEOUT_MS);
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

        i2c_master_dev_handle_t dev = get_device(addr);
        if (dev) {
            ret = i2c_master_transmit_receive(dev, &reg, 1, buf, len, (int)I2C_TIMEOUT_MS);
        }
        xSemaphoreGive(mutex_);
        if (ret == ESP_OK) break;
        vTaskDelay(1);
    }
    return ret;
}

bool I2CBus::probe(uint8_t addr) noexcept {
    if (!bus_) return false;
    return (i2c_master_probe(bus_, addr, 50) == ESP_OK);
}

} // namespace hal
