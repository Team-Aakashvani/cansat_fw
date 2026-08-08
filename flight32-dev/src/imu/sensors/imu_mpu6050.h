/**
 * @file imu_mpu6050.h
 * @brief Defines the wrapper for the MPU6050 sensor.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#pragma once

#include "../imu_sensor.h"
#include "../../config/imu_config.h"
#include <MPU9250_WE.h>
#include <Adafruit_BMP280.h>
#include "../../utils/MahonyAHRS.h"

class ImuMpu6050 : public ImuSensor
{
public:
    ImuMpu6050();
    ~ImuMpu6050();
    bool begin(bool useDMP = false, ImuGyroRangeIndex gyroRange = ImuGyroRangeIndex::GYRO_RANGE_250DPS, ImuAccelRangeIndex accelRange = ImuAccelRangeIndex::ACCEL_RANGE_2G, ImuLpfBandwidthIndex lpf = ImuLpfBandwidthIndex::LPF_256HZ_N_0MS) override;
    void calibrate() override;
    void read() override;

    ImuAxisData getGyroscopeOffset() override; // Removed const to match base class
    void setGyroscopeOffset(const ImuAxisData &offset) override;
    ImuAxisData getAccelerometerOffset() override; // Removed const to match base class
    void setAccelerometerOffset(const ImuAxisData &offset) override;
    ImuQuaternionData getQuaternion() const override;

    // Removed static LpfBandwidth getLpfBandwidthFromIndex(uint8_t index);
    uint16_t getI2CErrorCount() const override { return _i2c_error_count; }
    bool isSensorHealthy() const override { return _is_healthy; }
    bool isBaroHealthy() const override { return _baro_healthy; }
    bool isMagHealthy() const override { return _mag_healthy; }

private:
    MPU9250_WE _sensor; // Use MPU9250_WE for 9-DOF (MPU6500 + AK8963 Magnetometer)
    Adafruit_BMP280 _baro; // BMP280 Barometer sensor
    ImuQuaternionData _quaternion;
    static uint16_t _i2c_error_count;
    bool _is_healthy = false;
    bool _mag_healthy = false;
    bool _baro_healthy = false;

    MahonyAHRS _mahony_filter; // Mahony filter instance
};
