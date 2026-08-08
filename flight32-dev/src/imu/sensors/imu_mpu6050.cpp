/**
 * @file imu_mpu6050.cpp
 * @brief Implements the wrapper for the MPU6050 sensor.
 * @author Wastl Kraus - derdoktor667
 * @license MIT
 */

#include <Arduino.h>
#include "imu_mpu6050.h"
#include "../../config/imu_config.h"
#include "../../com_manager.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <Wire.h>

// Static members initialization
uint16_t ImuMpu6050::_i2c_error_count = 0;

ImuMpu6050::ImuMpu6050() : _sensor(&Wire, MPU6050_I2C_ADDRESS), _baro(&Wire), _mahony_filter(MAHONY_SAMPLE_RATE_HZ) // Initialize Mahony filter with sample rate
{
    _data_mutex = xSemaphoreCreateMutex();
    if (_data_mutex == nullptr)
    {
        com_send_log(ComMessageType::LOG_ERROR, "Failed to create IMU data mutex");
    }
}

ImuMpu6050::~ImuMpu6050()
{
    if (_data_mutex != nullptr)
    {
        vSemaphoreDelete(_data_mutex);
    }
}

bool ImuMpu6050::begin(bool useDMP_unused, ImuGyroRangeIndex gyroRange, ImuAccelRangeIndex accelRange, ImuLpfBandwidthIndex lpf)
{

    Wire.begin(MPU6050_I2C_SDA, MPU6050_I2C_SCL);

    // Initialize the MPU9250 / MPU6500 / MPU6050 (handle various clone WHO_AM_I codes)
    uint8_t who = _sensor.whoAmI();
    com_send_log(ComMessageType::LOG_INFO, "MPU WHO_AM_I register: 0x%02X", who);

    bool init_ok = _sensor.init();
    if (!init_ok)
    {
        init_ok = _sensor.MPU6500_WE::init();
    }
    if (!init_ok && (who == 0x68 || who == 0x70 || who == 0x71 || who == 0x73 || who == 0x75))
    {
        com_send_log(ComMessageType::LOG_INFO, "Valid MPU chip detected (WHO_AM_I=0x%02X), continuing initialization.", who);
        init_ok = true;
    }

    if (!init_ok)
    {
        com_send_log(ComMessageType::LOG_ERROR, "Failed to initialize MPU sensor (WHO_AM_I=0x%02X).", who);
        _i2c_error_count++;
        _is_healthy = false;

        return false;
    }

    // Configure Gyroscope and Accelerometer ranges
    _sensor.setGyrRange(static_cast<MPU9250_gyroRange>(gyroRange));

    _sensor.setAccRange(static_cast<MPU9250_accRange>(accelRange));

    _sensor.setGyrDLPF(static_cast<MPU9250_dlpf>(lpf));
    _sensor.setAccDLPF(static_cast<MPU9250_dlpf>(lpf));

    // Initialize AK8963 Magnetometer via auxiliary I2C bypass
    uint8_t whoMag = _sensor.whoAmIMag();
    com_send_log(ComMessageType::LOG_INFO, "AK8963 Magnetometer WHO_AM_I: 0x%02X (expected 0x48)", whoMag);
    _mag_healthy = _sensor.initMagnetometer();
    if (_mag_healthy)
    {
        _sensor.setMagOpMode(AK8963_CONT_MODE_100HZ);
        com_send_log(ComMessageType::LOG_INFO, "AK8963 Magnetometer initialized successfully.");
    }
    else
    {
        com_send_log(ComMessageType::LOG_WARN, "AK8963 Magnetometer not detected or init failed.");
    }

    // Initialize BMP280 Barometer (try 0x76 first, then 0x77)
    _baro_healthy = _baro.begin(0x76);
    if (!_baro_healthy)
    {
        _baro_healthy = _baro.begin(0x77);
    }
    if (_baro_healthy)
    {
        _baro.setSampling(Adafruit_BMP280::MODE_NORMAL,
                          Adafruit_BMP280::SAMPLING_X2,
                          Adafruit_BMP280::SAMPLING_X16,
                          Adafruit_BMP280::FILTER_X16,
                          Adafruit_BMP280::STANDBY_MS_1);
        com_send_log(ComMessageType::LOG_INFO, "BMP280 Barometer initialized successfully.");
    }
    else
    {
        com_send_log(ComMessageType::LOG_WARN, "BMP280 Barometer not detected at 0x76/0x77.");
    }

    _is_healthy = true;

    return true;
}

void ImuMpu6050::calibrate()
{
    if (!_is_healthy) return;
    com_send_log(ComMessageType::LOG_INFO, "Calibrating MPU6500_WE...");
    _sensor.autoOffsets(); // Assuming autoOffset() performs both gyro and accel calibration
    com_send_log(ComMessageType::LOG_INFO, "MPU6500_WE Calibration complete.");
}

void ImuMpu6050::read()
{
    if (!_is_healthy) return;
    // Reading raw sensor data.
    xyzFloat accelData = _sensor.getGValues();  // getGValues for accelerometer
    xyzFloat gyroData = _sensor.getGyrValues(); // getGyrValues for gyroscope
    xyzFloat magData = {0.0f, 0.0f, 0.0f};

    if (_mag_healthy)
    {
        magData = _sensor.getMagValues(); // getMagValues for magnetometer (uT)
        _mahony_filter.update(gyroData.x, gyroData.y, gyroData.z, accelData.x, accelData.y, accelData.z, magData.x, magData.y, magData.z);
    }
    else
    {
        _mahony_filter.updateIMU(gyroData.x, gyroData.y, gyroData.z, accelData.x, accelData.y, accelData.z);
    }

    if (xSemaphoreTake(_data_mutex, portMAX_DELAY) == pdTRUE)
    {
        _data.accelX = static_cast<int16_t>(accelData.x * ACCEL_G_TO_MG_FACTOR); // Convert G to mg
        _data.accelY = static_cast<int16_t>(accelData.y * ACCEL_G_TO_MG_FACTOR);
        _data.accelZ = static_cast<int16_t>(accelData.z * ACCEL_G_TO_MG_FACTOR);
        _data.gyroX = static_cast<int16_t>(gyroData.x * GYRO_DEGS_TO_MDEGS_FACTOR); // Convert deg/s to mdeg/s
        _data.gyroY = static_cast<int16_t>(gyroData.y * GYRO_DEGS_TO_MDEGS_FACTOR);
        _data.gyroZ = static_cast<int16_t>(gyroData.z * GYRO_DEGS_TO_MDEGS_FACTOR);
        _data.magX = magData.x;
        _data.magY = magData.y;
        _data.magZ = magData.z;

        if (_baro_healthy)
        {
            _data.temp = _baro.readTemperature();
            _data.pressure = _baro.readPressure() / 100.0f; // Pa to hPa
            _data.altitude = _baro.readAltitude(1013.25f);  // sea level hPa
        }
        else
        {
            _data.temp = _sensor.getTemperature();
            _data.pressure = 1013.25f;
            _data.altitude = 0.0f;
        }

        // Retrieve quaternions from Mahony filter
        _quaternion.w = _mahony_filter.q4;
        _quaternion.x = _mahony_filter.q5;
        _quaternion.y = _mahony_filter.q6;
        _quaternion.z = _mahony_filter.q7;

        xSemaphoreGive(_data_mutex);
    }
}

ImuAxisData ImuMpu6050::getGyroscopeOffset()
{
    xyzFloat gyrOffset = _sensor.getGyrOffsets(); // Assuming getGyrOffsets returns xyzFloat
    return {(float)gyrOffset.x,
            (float)gyrOffset.y,
            (float)gyrOffset.z};
}

void ImuMpu6050::setGyroscopeOffset(const ImuAxisData &offset)
{
    xyzFloat gyrOffset = {offset.x, offset.y, offset.z}; // Create xyzFloat from ImuAxisData
    _sensor.setGyrOffsets(gyrOffset);                    // Assuming setGyrOffsets takes xyzFloat
}

ImuAxisData ImuMpu6050::getAccelerometerOffset()
{
    xyzFloat accOffset = _sensor.getAccOffsets(); // Assuming getAccOffsets returns xyzFloat
    return {(float)accOffset.x,
            (float)accOffset.y,
            (float)accOffset.z};
}

void ImuMpu6050::setAccelerometerOffset(const ImuAxisData &offset)
{
    xyzFloat accOffset = {offset.x, offset.y, offset.z}; // Create xyzFloat from ImuAxisData
    _sensor.setAccOffsets(accOffset);                    // Assuming setAccOffsets takes xyzFloat
}

ImuQuaternionData ImuMpu6050::getQuaternion() const
{
    ImuQuaternionData temp_quat = {0, 0, 0, 0};
    if (xSemaphoreTake(_data_mutex, portMAX_DELAY) == pdTRUE)
    {
        temp_quat = _quaternion;
        xSemaphoreGive(_data_mutex);
    }
    return temp_quat;
}