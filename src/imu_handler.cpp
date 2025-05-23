#include "imu_handler.h"
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

IMUHandler::IMUHandler() : data_ready_(false) {}

bool IMUHandler::begin()
{

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

    if (!preferences_.begin(NVS_NAMESPACE, true))
    {
        Serial.println(" Failed to open NVS namespace (read-only). Assuming no calibration data.");
        return false;
    }

    if (!preferences_.isKey(KEY_CAL_DONE) || !preferences_.getBool(KEY_CAL_DONE, false))
    {
        Serial.println(" Calibration data not found or incomplete in NVS.");
        preferences_.end();
        return false;
    }

    abias[0] = preferences_.getFloat(KEY_ACC_BIAS_X, 0.0f);
    abias[1] = preferences_.getFloat(KEY_ACC_BIAS_Y, 0.0f);
    abias[2] = preferences_.getFloat(KEY_ACC_BIAS_Z, 0.0f);

    gbias[0] = preferences_.getFloat(KEY_GYRO_BIAS_X, 0.0f);
    gbias[1] = preferences_.getFloat(KEY_GYRO_BIAS_Y, 0.0f);
    gbias[2] = preferences_.getFloat(KEY_GYRO_BIAS_Z, 0.0f);

    mbias[0] = preferences_.getFloat(KEY_MAG_BIAS_X, 0.0f);
    mbias[1] = preferences_.getFloat(KEY_MAG_BIAS_Y, 0.0f);
    mbias[2] = preferences_.getFloat(KEY_MAG_BIAS_Z, 0.0f);

    mscale[0] = preferences_.getFloat(KEY_MAG_SCALE_X, 1.0f);
    mscale[1] = preferences_.getFloat(KEY_MAG_SCALE_Y, 1.0f);
    mscale[2] = preferences_.getFloat(KEY_MAG_SCALE_Z, 1.0f);

    preferences_.end();
    Serial.println("Calibration data successfully loaded from NVS.");
    calibration_loaded_ = true;
    return true;
}

bool IMUHandler::saveCalibrationToNVS()
{
    Serial.println("Attempting to load calibration data from NVS...");

    if (!preferences_.begin(NVS_NAMESPACE, false))
    {
        Serial.println("Failed to open NVS namespace (read-write). Cannot save calibration.");
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
    Serial.println("\n--- IMU Calibration Check ---");
    Serial.printf("Type 'yes' in Serial Monitor within %lu seconds to start FULL calibration now.\n", CALIBRATION_PROMPT_TIMEOUT_MS / 1000);
    Serial.println("Otherwise, existing data (if found) or default values will be used.");

    bool start_calibration = false;
    String input_string = "";
    unsigned long start_time = millis();

    while (Serial.available())
        Serial.read();

    while (millis() - start_time < CALIBRATION_PROMPT_TIMEOUT_MS)
    {
        if (Serial.available() > 0)
        {
            input_string = Serial.readStringUntil('\n');
            input_string.trim();
            if (input_string.equalsIgnoreCase("yes"))
            {
                start_calibration = true;
                break;
            }
            else
            {
                Serial.println("Input received, skipping calibration prompt.");
                break;
            }
        }
        delay(50);
    }

    if (start_calibration)
    {
        runCalibrationRoutine();
        Serial.println("Calibration applied after routine.");
        calibration_loaded_ = true;
    }
    else
    {
        if (loadCalibrationFromNVS(accBias, gyroBias, magBias, magScale))
        {
            Serial.println("Applying existing calibration data from NVS.");
            setAccBias(accBias[0], accBias[1], accBias[2]);
            setGyroBias(gyroBias[0], gyroBias[1], gyroBias[2]);
            setMagBias(magBias[0], magBias[1], magBias[2]);
            setMagScale(magScale[0], magScale[1], magScale[2]);
            calibration_loaded_ = true;
        }
        else
        {
            Serial.println("Timeout or no valid data in NVS. Applying default calibration values.");
            Serial.println("Run 'calibration' command later for better accuracy!");
            setAccBias(0.0f, 0.0f, 0.0f);
            setGyroBias(0.0f, 0.0f, 0.0f);
            setMagBias(0.0f, 0.0f, 0.0f);
            setMagScale(1.0f, 1.0f, 1.0f);
            calibration_loaded_ = false;
        }
    }
    Serial.println("--- End IMU Calibration Check ---");
}

bool IMUHandler::runCalibrationRoutine()
{
    Serial.println("\n--- Starting Sensor Calibration ---");
    Serial.println("Keep the drone LEVEL and STILL for Accel/Gyro calibration...");
    delay(3000);
    mpu9250_sensor_lib_.calibrateAccelGyro();
    Serial.println("Accel/Gyro calibration DONE.");
    Serial.printf(" Accel Bias: X:%.2f Y:%.2f Z:%.2f\n", mpu9250_sensor_lib_.getAccBiasX(), mpu9250_sensor_lib_.getAccBiasY(), mpu9250_sensor_lib_.getAccBiasZ());
    Serial.printf(" Gyro Bias:  X:%.2f Y:%.2f Z:%.2f\n", mpu9250_sensor_lib_.getGyroBiasX(), mpu9250_sensor_lib_.getGyroBiasY(), mpu9250_sensor_lib_.getGyroBiasZ());

    Serial.println("\nPrepare for Magnetometer calibration.");
    Serial.println("Slowly move the drone in a figure-8 pattern in the air,");
    Serial.println("covering all orientations (pitch, roll, yaw).");
    delay(5000);
    Serial.println("Starting Mag calibration NOW!");
    mpu9250_sensor_lib_.calibrateMag();
    Serial.println("Magnetometer calibration DONE.");
    Serial.printf(" Mag Bias:  X:%.2f Y:%.2f Z:%.2f\n", mpu9250_sensor_lib_.getMagBiasX(), mpu9250_sensor_lib_.getMagBiasY(), mpu9250_sensor_lib_.getMagBiasZ());
    Serial.printf(" Mag Scale: X:%.2f Y:%.2f Z:%.2f\n", mpu9250_sensor_lib_.getMagScaleX(), mpu9250_sensor_lib_.getMagScaleY(), mpu9250_sensor_lib_.getMagScaleZ());

    bool save_success = saveCalibrationToNVS();

    Serial.println("--- Sensor Calibration Finished ---");
    return save_success;
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
        // Serial.printf("Roll:%.2f Pitch:%.2f Yaw:%.2f\n",
        //               current_attitude_.roll, current_attitude_.pitch, current_attitude_.yaw);
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