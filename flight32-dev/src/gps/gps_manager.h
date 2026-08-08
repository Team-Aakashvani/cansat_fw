/**
 * @file gps_manager.h
 * @brief GPS receiver management using TinyGPSPlus on UART1 (GPIO 13 RX / 14 TX).
 * @author Wastl Kraus
 * @license MIT
 */

#pragma once

#include <Arduino.h>
#include <TinyGPSPlus.h>

struct GpsData
{
    bool has_fix = false;
    uint8_t satellites = 0;
    double latitude = 0.0;
    double longitude = 0.0;
    float altitude_meters = 0.0f;
    float speed_mps = 0.0f;
    float course_deg = 0.0f; // Course Over Ground (True North degrees when moving)
    uint32_t last_update_ms = 0;
};

class GpsManager
{
public:
    GpsManager() = default;

    /**
     * @brief Initializes HardwareSerial (Serial1) on GPIO 13 (RX) and GPIO 14 (TX).
     * @param baud_rate GPS baud rate (default 9600 for standard Ublox NEO-M8N / NMEA GPS).
     */
    void begin(uint32_t baud_rate = 9600);

    /**
     * @brief Reads available bytes from Serial1 and feeds TinyGPSPlus parser.
     *        Should be called frequently in a task or loop.
     */
    void update();

    /**
     * @brief Gets current parsed GPS data.
     */
    GpsData getData() const;

    /**
     * @brief Computes kinematic True North Yaw alignment offset if moving > 1.5 m/s.
     * @param current_gyro_yaw_deg Current gyroscope Yaw angle in degrees.
     * @param aligned_yaw_deg Output aligned True North Yaw angle.
     * @return true if moving fast enough and valid alignment computed.
     */
    bool getKinematicYawAlignment(float current_gyro_yaw_deg, float &aligned_yaw_deg) const;

private:
    TinyGPSPlus _gps;
    GpsData _data;
};

// Global GPS manager instance declaration
extern GpsManager gpsManager;
