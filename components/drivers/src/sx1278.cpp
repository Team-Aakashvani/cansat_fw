#include "drivers/sx1278.hpp"
#include "nav/config.hpp"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <cstring>

static const char* TAG = "SX1278";
static drivers::SX1278* g_sx1278 = nullptr;

namespace drivers {

esp_err_t SX1278::init(hal::SPIBus& spi, int cs_pin, int rst_pin, int irq_pin) noexcept {
    spi_     = &spi;
    rst_pin_ = rst_pin;
    irq_pin_ = irq_pin;
    g_sx1278 = this;

    // Configure RST and IRQ GPIOs
    gpio_reset_pin((gpio_num_t)rst_pin);
    gpio_set_direction((gpio_num_t)rst_pin, GPIO_MODE_OUTPUT);
    gpio_reset_pin((gpio_num_t)irq_pin);
    gpio_set_direction((gpio_num_t)irq_pin, GPIO_MODE_INPUT);

    // Add SPI device
    if (spi_->add_device(cs_pin, 8000000, dev_) != ESP_OK) {
        ESP_LOGE(TAG, "SPI device add failed");
        return ESP_FAIL;
    }

    // Hardware reset
    gpio_set_level((gpio_num_t)rst_pin, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level((gpio_num_t)rst_pin, 1);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Verify chip ID (SX1278 = 0x12)
    uint8_t ver = read_reg(REG_VERSION);
    if (ver != 0x12) {
        ESP_LOGE(TAG, "SX1278 not found (ver=0x%02X)", ver);
        return ESP_ERR_NOT_FOUND;
    }

    // Enter sleep mode first (required to set LoRa mode)
    write_reg(REG_OP_MODE, MODE_LORA_SLEEP);
    vTaskDelay(pdMS_TO_TICKS(10));

    // Set frequency and modem config
    set_frequency(nav::TELEM_CFG.lora_frequency_hz);
    configure_modem();

    // FIFO base addresses
    write_reg(REG_FIFO_TX_BASE, 0x00);
    write_reg(REG_FIFO_RX_BASE, 0x00);

    // LNA: max gain, AGC on
    write_reg(REG_LNA, 0x23);

    // Go to standby
    write_reg(REG_OP_MODE, MODE_LORA_STDBY);
    vTaskDelay(pdMS_TO_TICKS(10));

    // DIO0 → TX Done / RX Done
    write_reg(REG_DIO_MAP1, 0x00);

    ready_ = true;
    ESP_LOGI(TAG, "SX1278 ready (ver=0x%02X, freq=%luHz)",
             ver, (unsigned long)nav::TELEM_CFG.lora_frequency_hz);
    return ESP_OK;
}

esp_err_t SX1278::transmit(const uint8_t* data, uint8_t len) noexcept {
    if (!ready_ || !spi_) return ESP_ERR_INVALID_STATE;

    spi_device_acquire_bus(dev_, portMAX_DELAY);

    // Standby
    write_reg(REG_OP_MODE, MODE_LORA_STDBY);

    // Set FIFO address and write data
    write_reg(REG_FIFO_ADDR_PTR, 0x00);
    spi_->write_reg(dev_, REG_FIFO, data, len);
    write_reg(REG_PAYLOAD_LEN, len);

    // Start TX
    write_reg(REG_OP_MODE, MODE_LORA_TX);

    spi_device_release_bus(dev_);

    // Wait for TX done (poll IRQ flags, timeout 3s)
    for (int i = 0; i < 3000; i += 10) {
        vTaskDelay(pdMS_TO_TICKS(10));
        uint8_t flags = read_reg(REG_IRQ_FLAGS);
        if (flags & 0x08) {  // TxDone bit
            write_reg(REG_IRQ_FLAGS, 0xFF);  // Clear all IRQs
            write_reg(REG_OP_MODE, MODE_LORA_STDBY);
            return ESP_OK;
        }
    }
    ESP_LOGW(TAG, "TX timeout");
    write_reg(REG_OP_MODE, MODE_LORA_STDBY);
    return ESP_ERR_TIMEOUT;
}

esp_err_t SX1278::transmit_str(const char* str) noexcept {
    if (!str) return ESP_ERR_INVALID_ARG;
    size_t n = strlen(str);
    if (n > 255) n = 255;
    return transmit((const uint8_t*)str, (uint8_t)n);
}

esp_err_t SX1278::start_rx(LoRaRxCallback cb) noexcept {
    if (!ready_) return ESP_ERR_INVALID_STATE;
    rx_cb_ = cb;
    write_reg(REG_IRQ_FLAGS_MASK, 0x3F);  // Enable RxDone IRQ only
    write_reg(REG_FIFO_ADDR_PTR, 0x00);
    write_reg(REG_OP_MODE, MODE_LORA_RXCONT);
    return ESP_OK;
}

void SX1278::stop_rx() noexcept {
    if (ready_) write_reg(REG_OP_MODE, MODE_LORA_STDBY);
}

esp_err_t SX1278::read_packet(LoRaRxPacket& pkt, uint32_t timeout_ms) noexcept {
    if (!ready_) return ESP_ERR_INVALID_STATE;

    // Polling loop for RxDone flag
    uint32_t elapsed = 0;
    uint8_t flags = 0;
    while (elapsed <= timeout_ms) {
        flags = read_reg(REG_IRQ_FLAGS);
        if (flags & 0x40) break; // RxDone
        vTaskDelay(pdMS_TO_TICKS(10));
        elapsed += 10;
        if (timeout_ms == 0) return ESP_ERR_TIMEOUT;
    }

    if (!(flags & 0x40)) return ESP_ERR_TIMEOUT;

    // Acquire bus to ensure atomic sequence of reading packet metadata and FIFO
    spi_device_acquire_bus(dev_, portMAX_DELAY);

    write_reg(REG_IRQ_FLAGS, 0xFF); // Clear all IRQs

    pkt.crc_ok  = !(flags & 0x20);  // PayloadCrcError bit
    pkt.len     = read_reg(REG_RX_NB_BYTES);
    last_rssi_  = (int8_t)(read_reg(REG_PKT_RSSI) - 164);
    pkt.rssi_dbm = last_rssi_;
    
    // SNR is 2's complement, signed byte, step 0.25dB
    int8_t raw_snr = (int8_t)read_reg(REG_PKT_SNR);
    pkt.snr_db = (float)raw_snr / 4.0f;

    uint8_t rx_addr = read_reg(REG_FIFO_RX_CURR);
    write_reg(REG_FIFO_ADDR_PTR, rx_addr);
    
    if (pkt.len > 0) {
        spi_->read_reg(dev_, REG_FIFO, pkt.data, pkt.len);
    }

    spi_device_release_bus(dev_);

    return pkt.crc_ok ? ESP_OK : ESP_ERR_INVALID_CRC;
}

void SX1278::write_reg(uint8_t reg, uint8_t val) noexcept {
    spi_->write_reg(dev_, reg, &val, 1);
}

uint8_t SX1278::read_reg(uint8_t reg) noexcept {
    uint8_t val = 0;
    spi_->read_reg(dev_, reg, &val, 1);
    return val;
}

void SX1278::set_frequency(uint32_t freq_hz) noexcept {
    // Fstep = 32MHz / 2^19 = 61.035 Hz
    uint64_t frf = ((uint64_t)freq_hz << 19) / 32000000UL;
    write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
    write_reg(REG_FRF_LSB, (uint8_t)(frf));
}

void SX1278::configure_modem() noexcept {
    const nav::TelemetryConfig& T = nav::TELEM_CFG;

    // PA config: PA_BOOST with max power
    write_reg(REG_PA_CONFIG, 0x8F | ((T.lora_tx_power_dbm - 2) & 0x0F));
    write_reg(REG_PA_DAC, 0x87);  // +20dBm mode if needed

    // Modem config 1: BW=7(125kHz), CR=5(4/5), explicit header
    write_reg(REG_MODEM_CONFIG1, (T.lora_bandwidth << 4) | (T.lora_cr << 1) | 0x00);

    // Modem config 2: SF10, CRC on
    write_reg(REG_MODEM_CONFIG2, (T.lora_sf << 4) | 0x04);

    // Modem config 3: LDRO auto, AGC auto
    write_reg(REG_MODEM_CONFIG3, 0x04);

    // Preamble length
    write_reg(REG_PREAMBLE_MSB, (T.lora_preamble_len >> 8) & 0xFF);
    write_reg(REG_PREAMBLE_LSB, T.lora_preamble_len & 0xFF);

    // Sync word (team-specific NETID)
    write_reg(REG_SYNC_WORD, T.lora_sync_word);
}

} // namespace drivers
