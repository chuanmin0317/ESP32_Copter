#include "imu_handler.h"
#include <Wire.h>
#include <Arduino.h>

const char *NVS_NAMESPACE = "imu-cal";
const char *KEY_CAL_DONE = "cal_done";
const char *KEY_ACC_BIAS_X = "abias_x";
const char *KEY_ACC_BIAS_Y = "abias_y";
const char *KEY_ACC_BIAS_Z = "abias_z";
const char *KEY_GYRO_BIAS_X = "gbias_x";
const char *KEY_GYRO_BIAS_Y = "gbias_y";
const char *KEY_GYRO_BIAS_Z = "gbias_z";
const char *KEY_MAG_BIAS_X = "mbias_x";
const char *KEY_MAG_BIAS_Y = "mbias_y";
const char *KEY_MAG_BIAS_Z = "mbias_z";
const char *KEY_MAG_SCALE_X = "mscale_x";
const char *KEY_MAG_SCALE_Y = "mscale_y";
const char *KEY_MAG_SCALE_Z = "mscale_z";

IMUHandler::IMUHandler() : data_ready_(false), calibration_loaded_(false) {}

bool IMUHandler::begin()
{
    Wire.begin(SDA_PIN, SCL_PIN); // Initialize I2C with custom SDA and SCL pins
    Wire.setClock(400000);        // Set I2C frequency to 400kHz
    Serial.println("I2C Initialized.");
    delay(100);

    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A8G;           // 加速度計量程 ±8g
    setting.gyro_fs_sel = GYRO_FS_SEL::G1000DPS;        // 陀螺儀量程 ±1000°/s
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS; // 磁力計解析度 16位
    setting.gyro_fchoice = 0x03;                        // 啟用陀螺儀濾波
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_41HZ;   // 陀螺儀低通濾波器 41Hz
    setting.accel_fchoice = 0x01;                       // 啟用加速度計濾波
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ; // 加速度計低通濾波器 45Hz

    if (!mpu9250_sensor_lib_.setup(MPU9250_ADDRESS, setting))
    {
        Serial.println("MPU connection failed. Please check wiring");
        return false;
    }
    Serial.println("MPU connection successful");

    applyCalibration(); // Apply calibration settings if needed

    mpu9250_sensor_lib_.selectFilter(QuatFilterSel::MADGWICK);
    mpu9250_sensor_lib_.ahrs(true);

    mpu9250_sensor_lib_.setMagneticDeclination(-3.8); // Set magnetic declination for your location

    return true;
}

bool IMUHandler::loadCalibrationFromNVS(float abias[3], float gbias[3], float mbias[3], float mscale[3])
{
    Serial.println("Attempting to load calibration data from NVS...");

    if (!preferences_.begin(NVS_NAMESPACE, false))
    {
        Serial.println("Failed to open NVS namespace (read-write). Cannot save calibration.");
        return false;
    }

    if (!preferences_.isKey(KEY_CAL_DONE) || !preferences_.getBool(KEY_CAL_DONE, false))
    {
        Serial.println(" Calibration data not found or incomplete in NVS.");
        preferences_.end();
        return false;
    }

    float abx = mpu9250_sensor_lib_.getAccBiasX();
    float aby = mpu9250_sensor_lib_.getAccBiasY();
    float abz = mpu9250_sensor_lib_.getAccBiasZ();
    float gbx = mpu9250_sensor_lib_.getGyroBiasX();
    float gby = mpu9250_sensor_lib_.getGyroBiasY();
    float gbz = mpu9250_sensor_lib_.getGyroBiasZ();
    float mbx = mpu9250_sensor_lib_.getMagBiasX();
    float mby = mpu9250_sensor_lib_.getMagBiasY();
    float mbz = mpu9250_sensor_lib_.getMagBiasZ();
    float msx = mpu9250_sensor_lib_.getMagScaleX();
    float msy = mpu9250_sensor_lib_.getMagScaleY();
    float msz = mpu9250_sensor_lib_.getMagScaleZ();

    preferences_.putFloat(KEY_ACC_BIAS_X, abx);
    preferences_.putFloat(KEY_ACC_BIAS_Y, aby);
    preferences_.putFloat(KEY_ACC_BIAS_Z, abz);
    preferences_.putFloat(KEY_GYRO_BIAS_X, gbx);
    preferences_.putFloat(KEY_GYRO_BIAS_Y, gby);
    preferences_.putFloat(KEY_GYRO_BIAS_Z, gbz);
    preferences_.putFloat(KEY_MAG_BIAS_X, mbx);
    preferences_.putFloat(KEY_MAG_BIAS_Y, mby);
    preferences_.putFloat(KEY_MAG_BIAS_Z, mbz);
    preferences_.putFloat(KEY_MAG_SCALE_X, msx);
    preferences_.putFloat(KEY_MAG_SCALE_Y, msy);
    preferences_.putFloat(KEY_MAG_SCALE_Z, msz);

    preferences_.putBool(KEY_CAL_DONE, true); // Mark calibration as done

    preferences_.end();
    Serial.println("Calibration data saved successfully.");
    calibration_loaded_ = true;
    return true;
}

void IMUHandler::applyCalibration()
{
    float accBias[3], gyroBias[3], magBias[3], magScale[3];
    if (loadCalibrationFromNVS(accBias, gyroBias, magBias, magScale))
    {
        mpu9250_sensor_lib_.setAccBias(accBias[0], accBias[1], accBias[2]);
        mpu9250_sensor_lib_.setGyroBias(gyroBias[0], gyroBias[1], gyroBias[2]);
        mpu9250_sensor_lib_.setMagBias(magBias[0], magBias[1], magBias[2]);
        mpu9250_sensor_lib_.setMagScale(magScale[0], magScale[1], magScale[2]);
    }
    else
    {
        Serial.println("WARNING: Calibration data not found in NVS. Applying default values.");
        Serial.println("         Run calibration routine to improve sensor accuracy!");
        setAccBias(0.0f, 0.0f, 0.0f);
        setGyroBias(0.0f, 0.0f, 0.0f);
        setMagBias(0.0f, 0.0f, 0.0f);
        setMagScale(1.0f, 1.0f, 1.0f);
        calibration_loaded_ = false;
    }
}

bool IMUHandler::runCalibrationRoutine()
{
    Serial.println("\n--- Starting Sensor Calibration ---");
    delay(3000);
    mpu9250_sensor_lib_.calibrateAccelGyro();

    delay(5000);
    Serial.println("Starting Mag calibration NOW!");
    mpu9250_sensor_lib_.calibrateMag();
    saveCalibrationToNVS();

    Serial.println("--- Sensor Calibration Finished ---");
    return true;
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

DroneTypes::Attitude IMUHandler::getAttitude() const
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

void IMUHandler::setGyroBias(float bx, float by, float bz)
{
    mpu9250_sensor_lib_.setGyroBias(bx, by, bz);
    Serial.printf("Set Gyro Bias: X:%.2f Y:%.2f Z:%.2f\n", bx, by, bz);
}
void IMUHandler::setAccBias(float bx, float by, float bz)
{
    mpu9250_sensor_lib_.setAccBias(bx, by, bz);
    Serial.printf("Set Accel Bias: X:%.2f Y:%.2f Z:%.2f\n", bx, by, bz);
}
void IMUHandler::setMagBias(float bx, float by, float bz)
{
    mpu9250_sensor_lib_.setMagBias(bx, by, bz);
    Serial.printf("Set Mag Bias: X:%.2f Y:%.2f Z:%.2f\n", bx, by, bz);
}
void IMUHandler::setMagScale(float sx, float sy, float sz)
{
    mpu9250_sensor_lib_.setMagScale(sx, sy, sz);
    Serial.printf("Set Mag Scale: X:%.2f Y:%.2f Z:%.2f\n", sx, sy, sz);
}
