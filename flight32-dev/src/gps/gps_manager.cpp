/**
 * @file gps_manager.cpp
 * @brief Implements GPS receiver parsing and Kinematic Yaw Alignment on UART1.
 * @author Wastl Kraus
 * @license MIT
 */

#include "gps_manager.h"
#include <HardwareSerial.h>

constexpr uint8_t GPS_RX_PIN = 13;
constexpr uint8_t GPS_TX_PIN = 14;

GpsManager gpsManager;

void GpsManager::begin(uint32_t baud_rate)
{
    Serial1.begin(baud_rate, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
    Serial.printf("[GPS] UART1 initialized on RX=GPIO%u, TX=GPIO%u at %u bps.\n",
                  GPS_RX_PIN, GPS_TX_PIN, baud_rate);
}

void GpsManager::update()
{
    while (Serial1.available() > 0)
    {
        uint8_t c = Serial1.read();
        _gps.encode(c);
    }

    if (_gps.location.isUpdated() || _gps.satellites.isUpdated() || _gps.course.isUpdated())
    {
        _data.has_fix = _gps.location.isValid() && (_gps.location.age() < 3000);
        _data.satellites = _gps.satellites.isValid() ? _gps.satellites.value() : 0;
        _data.latitude = _gps.location.isValid() ? _gps.location.lat() : 0.0;
        _data.longitude = _gps.location.isValid() ? _gps.location.lng() : 0.0;
        _data.altitude_meters = _gps.altitude.isValid() ? (float)_gps.altitude.meters() : 0.0f;
        _data.speed_mps = _gps.speed.isValid() ? (float)_gps.speed.mps() : 0.0f;
        _data.course_deg = _gps.course.isValid() ? (float)_gps.course.deg() : 0.0f;
        _data.last_update_ms = millis();
    }
}

GpsData GpsManager::getData() const
{
    return _data;
}

bool GpsManager::getKinematicYawAlignment(float current_gyro_yaw_deg, float &aligned_yaw_deg) const
{
    // Check if we have a valid GPS fix and vehicle is moving faster than 1.5 m/s (~5.4 km/h)
    if (!_data.has_fix || _data.speed_mps < 1.5f || millis() - _data.last_update_ms > 2000)
    {
        return false;
    }

    // When moving forward, GPS Course Over Ground (COG) represents True North heading
    aligned_yaw_deg = _data.course_deg;
    return true;
}
