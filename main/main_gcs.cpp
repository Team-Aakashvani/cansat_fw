/**
 * CAN-7USAT 2026 — GCS Bridge Firmware
 * =====================================
 *
 * FILE:  main/main_gcs.cpp
 *
 * Bridging radio <-> USB serial for the Ground Station.
 */

#include <cstring>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"
#include "hal/spi_bus.hpp"
#include "comms/xbee_link.hpp"

// PIN DEFINITIONS — ESP32-S3 WROOM-1
#define GCS_XBEE_TX   34
#define GCS_XBEE_RX   32

#define GCS_BAUD_RATE 921600
#define GCS_UART      UART_NUM_0

static const char* TAG = "GCS";

#define PRI_GCS_TASK  5
#define STK_GCS       4096
#define UPLINK_BUF_SZ 128

static const char UPLINK_PREFIX[] = "$UPLINK,";

static void gcs_task(void* pvParameters)
{
    comms::XBeeLink* xbee = static_cast<comms::XBeeLink*>(pvParameters);
    uint8_t uplink_buf[UPLINK_BUF_SZ];
    size_t  uplink_pos = 0;

    ESP_LOGI(TAG, "GCS bridge task started");

    while (true)
    {
        // DOWNLINK: Radio -> USB
        if (xbee->spin())
        {
            const uint8_t* rx_data = xbee->last_rx_data();
            size_t         rx_len  = xbee->last_rx_len();
            if (rx_data && rx_len > 0)
            {
                uart_write_bytes(GCS_UART, rx_data, rx_len);
            }
        }

        // UPLINK: USB -> Radio
        uint8_t byte;
        while (uart_read_bytes(GCS_UART, &byte, 1, 0) > 0)
        {
            if (byte == '\n')
            {
                uplink_buf[uplink_pos] = '\0';
                if (uplink_pos > sizeof(UPLINK_PREFIX) - 1 &&
                    strncmp((const char*)uplink_buf, UPLINK_PREFIX, sizeof(UPLINK_PREFIX) - 1) == 0)
                {
                    const char* cmd_part = (const char*)uplink_buf + (sizeof(UPLINK_PREFIX) - 1);
                    xbee->enqueue_packet(cmd_part, strlen(cmd_part));
                    ESP_LOGI(TAG, "Uplink: %s", cmd_part);
                }
                uplink_pos = 0;
            }
            else if (byte != '\r')
            {
                if (uplink_pos < UPLINK_BUF_SZ - 1) uplink_buf[uplink_pos++] = byte;
                else uplink_pos = 0;
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
    }
}

void main_gcs()
{
    ESP_LOGI(TAG, "================================================");
    ESP_LOGI(TAG, "   AAKASHVANI — CAN-7USAT india 2026            ");
    ESP_LOGI(TAG, "   (C) 2026 SVNIT. All Rights Reserved.         ");
    ESP_LOGI(TAG, "   [ Ground Station Bridge ]                    ");
    ESP_LOGI(TAG, "================================================");

    uart_config_t uart_config = {
        .baud_rate  = GCS_BAUD_RATE,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_DISABLE,
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    ESP_ERROR_CHECK(uart_driver_install(GCS_UART, 2048, 2048, 0, nullptr, 0));
    ESP_ERROR_CHECK(uart_param_config(GCS_UART, &uart_config));

    static hal::UARTBus xbee_uart;
    ESP_ERROR_CHECK(xbee_uart.init(UART_NUM_2, GCS_XBEE_TX, GCS_XBEE_RX, nav::TELEM_CFG.xbee_baud));

    static comms::XBeeLink xbee;
    ESP_ERROR_CHECK(xbee.init(xbee_uart));
    xbee.set_promiscuous(true);

    xTaskCreatePinnedToCore(gcs_task, "gcs", STK_GCS, &xbee, PRI_GCS_TASK, nullptr, 1);
    ESP_LOGI(TAG, "Bridge Live.");
}
