#include "imu_handler.h"
#include <Wire.h>
#include <Arduino.h>

IMUHandler::IMUHandler() : data_ready_(false) {}

bool IMUHandler::begin()
{
    Wire.setPins(SDA_PIN, SCL_PIN); // Set custom SDA and SCL pins
    Wire.begin();                   // Initialize I2C with custom SDA and SCL pins
    Wire.setClock(400000);          // Set I2C frequency to 400kHz

    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A8G;           // 加速度計量程 ±8g
    setting.gyro_fs_sel = GYRO_FS_SEL::G1000DPS;        // 陀螺儀量程 ±1000°/s
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS; // 磁力計解析度 16位
    setting.gyro_fchoice = 0x03;                        // 啟用陀螺儀濾波
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;   // 陀螺儀低通濾波器 41Hz
    setting.accel_fchoice = 0x01;                       // 啟用加速度計濾波
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ; // 加速度計低通濾波器 45Hz

    if (!mpu9250_sensor_lib_.setup(0x68, setting))
    {
        Serial.println("MPU connection failed. Please check wiring");
        return false;
    }
    Serial.println("MPU connection successful");

    applyCalibration(); // Apply calibration settings if needed


    return true;
}


void IMUHandler::applyCalibration()
{
    mpu9250_sensor_lib_.setGyroBias(-1.27, -0.38, -1.76);
    mpu9250_sensor_lib_.setAccBias(-4.69, 58.14, 83.74);
    mpu9250_sensor_lib_.setMagBias(04.54, 368.57, -327.77);
    mpu9250_sensor_lib_.setMagScale(0.66, 1.26, 1.43);
}  


bool IMUHandler::update()
{   
    data_ready_ = false; // Assume no new data initially for this cycle
    if (mpu9250_sensor_lib_.update())
    {
        current_mpu_data_.accX = mpu9250_sensor_lib_.getAccX();
        current_mpu_data_.accY = mpu9250_sensor_lib_.getAccY();
        current_mpu_data_.accZ = mpu9250_sensor_lib_.getAccZ();

        current_mpu_data_.gyroX = mpu9250_sensor_lib_.getGyroX();
        current_mpu_data_.gyroY = mpu9250_sensor_lib_.getGyroY();
        current_mpu_data_.gyroZ = mpu9250_sensor_lib_.getGyroZ();

        current_attitude_.roll = mpu9250_sensor_lib_.getRoll();
        current_attitude_.pitch = mpu9250_sensor_lib_.getPitch();
        current_attitude_.yaw = mpu9250_sensor_lib_.getYaw();

        data_ready_ = true;
        return true; 
    }

    return false;
}

DroneTypes:: Attitude IMUHandler::getAttitude() const
{
    return current_attitude_;
}

DroneTypes::RawMPUData IMUHandler::getRawMPUData() const
{
    return current_mpu_data_;
}

bool IMUHandler::isDataReady() const
{
    return data_ready_;
}