/**
 * @file wifi_ota_manager.h
 * @brief Manages ESP32 Wi-Fi Access Point, ArduinoOTA updates, and TCP MSP/Terminal telemetry server.
 * @author Wastl Kraus
 * @license MIT
 */

#ifndef WIFI_OTA_MANAGER_H
#define WIFI_OTA_MANAGER_H

#include <Arduino.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Wi-Fi Soft-AP ("Flight32_AP"), ArduinoOTA, and TCP Telemetry Server on Port 2323.
 */
void wifi_ota_init();
void wifi_ota_stop();
bool wifi_ota_is_initialized();

/**
 * @brief Service ArduinoOTA and handle incoming TCP telemetry connections/bytes.
 * Must be called regularly in loop() or a FreeRTOS task.
 */
void wifi_ota_handle();

/**
 * @brief Check if bytes are available to read from a connected wireless TCP client.
 * @return Number of available bytes.
 */
int wifi_ota_available();

/**
 * @brief Read a single byte from the connected wireless TCP client.
 * @return Byte value (0-255) or -1 if no byte is available.
 */
int wifi_ota_read();

/**
 * @brief Write a buffer of bytes to the connected wireless TCP client.
 * @param buf Pointer to data buffer.
 * @param len Length of data in bytes.
 * @return Number of bytes written.
 */
size_t wifi_ota_write(const uint8_t *buf, size_t len);

/**
 * @brief Check if an active TCP client is currently connected.
 * @return true if connected, false otherwise.
 */
bool wifi_ota_is_client_connected();

#ifdef __cplusplus
}
#endif

#endif // WIFI_OTA_MANAGER_H
