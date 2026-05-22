/**
 * CAN-7USAT 2026 — GCS Bridge Firmware
 * =====================================
 *
 * FILE:  main/main_gcs_pin.cpp
 *
 * Migration to XBee UART.
 */

#include <cstring>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/uart.h"

#include "hal/uart_bus.hpp"
#include "comms/xbee_link.hpp"


// =============================================================================
// PIN DEFINITIONS — from the ESP32-S3 WROOM-1 board pinout
// =============================================================================

#define GCS_XBEE_TX   34
#define GCS_XBEE_RX   32

// USB serial speed — must match parser.py
#define GCS_BAUD_RATE 921600


// =============================================================================
// CONSTANTS
// =============================================================================

static const char* TAG = "GCS";

#define PRI_GCS_TASK  5
#define STK_GCS       4096
#define GCS_UART      UART_NUM_0
#define UPLINK_BUF_SZ 128

static const char UPLINK_PREFIX[] = "$UPLINK,";


// =============================================================================
// THE GCS TASK — runs forever, bridging radio ↔ USB
// =============================================================================

static void gcs_task(void* pvParameters)
{
    comms::XBeeLink* xbee = static_cast<comms::XBeeLink*>(pvParameters);

    uint8_t uplink_buf[UPLINK_BUF_SZ];
    size_t  uplink_pos = 0;

    ESP_LOGI(TAG, "GCS bridge task started — listening for packets...");

    while (true)
    {
        // DOWNLINK — radio packet arrived? → write to USB
        if (xbee->spin())
        {
            const uint8_t* rx_data = xbee->last_rx_data();
            size_t         rx_len  = xbee->last_rx_len();

            if (rx_data != nullptr && rx_len > 0)
            {
                uart_write_bytes(GCS_UART, rx_data, rx_len);
                ESP_LOGD(TAG, "Downlink: %u bytes sent to USB", (unsigned)rx_len);
            }
        }

        // UPLINK — laptop sent a command? → send over radio
        // Line-buffered: collect bytes until '\n', then parse
        {
            uint8_t byte;
            int bytes_read = uart_read_bytes(GCS_UART, &byte, 1, 0);

            while (bytes_read > 0)
            {
                if (byte == '\n')
                {
                    uplink_buf[uplink_pos] = '\0';

                    if (uplink_pos > sizeof(UPLINK_PREFIX) - 1 &&
                        strncmp((const char*)uplink_buf, UPLINK_PREFIX,
                                sizeof(UPLINK_PREFIX) - 1) == 0)
                    {
                        // Skip "$UPLINK," prefix (8 bytes)
                        const char* cmd_part = (const char*)uplink_buf + (sizeof(UPLINK_PREFIX) - 1);
                        xbee->enqueue_packet(cmd_part, strlen(cmd_part));
                        ESP_LOGI(TAG, "Uplink: %s", cmd_part);
                    }
                    else
                    {
                        ESP_LOGD(TAG, "Ignored non-uplink line: %s",
                                 (const char*)uplink_buf);
                    }

                    uplink_pos = 0;
                }
                else if (byte != '\r')
                {
                    if (uplink_pos < UPLINK_BUF_SZ - 1)
                    {
                        uplink_buf[uplink_pos++] = byte;
                    }
                    else
                    {
                        ESP_LOGW(TAG, "Uplink buffer overflow — discarding");
                        uplink_pos = 0;
                    }
                }

                bytes_read = uart_read_bytes(GCS_UART, &byte, 1, 0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// =============================================================================
// main_gcs() — boot sequence
// =============================================================================

void main_gcs()
{
    ESP_LOGI(TAG, "========================================");
    ESP_LOGI(TAG, " CAN-7USAT 2026 — GCS Bridge Starting  ");
    ESP_LOGI(TAG, "========================================");

    // STEP 0: Install the USB UART driver
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
    ESP_ERROR_CHECK(uart_set_pin(GCS_UART,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE,
        UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "USB UART initialised");

    // STEP 1: Initialise the XBee UART
    static hal::UARTBus xbee_uart;
    ESP_ERROR_CHECK(xbee_uart.init(UART_NUM_2, GCS_XBEE_TX, GCS_XBEE_RX, nav::TELEM_CFG.xbee_baud));

    ESP_LOGI(TAG, "XBee UART initialised");

    // STEP 2: Initialise the XBee link
    static comms::XBeeLink xbee;
    ESP_ERROR_CHECK(xbee.init(xbee_uart));
    xbee.set_promiscuous(true);

    ESP_LOGI(TAG, "XBee radio initialised (Promiscuous mode ON)");

    // STEP 3: Start the GCS bridge task on Core 1
    xTaskCreatePinnedToCore(
        gcs_task,
        "gcs",
        STK_GCS,
        &xbee,
        PRI_GCS_TASK,
        nullptr,
        1
    );

    ESP_LOGI(TAG, "GCS bridge task launched on Core 1");
    ESP_LOGI(TAG, "Bridge is live — XBee <-> USB serial");
}
