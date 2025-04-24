#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <MPU9250.h>
#include <Preferences.h>
#include "common_types.h"
#include "setting.h"

class IMUHandler
{
public:
    IMUHandler(); // Constructor

    /**
     * @brief Initializes the MPU9250 sensor.
     * Call this once in setup(). Assumes the I2C bus is already initialized.
     * @return true if initialization is successful, false otherwise.
     */
    bool begin();

    /**
     * @brief Reads the latest data from the MPU9250 sensor if available.
     * Call this periodically in the main loop.
     * @return true if reading is successful, false otherwise.
     */
    bool update();

    /**
     * @brief Gets the most recently read attitude data raw data.
     * @return Attitude struct containing roll, pitch, yaw.
     */
    DroneTypes::Attitude getAttitude() const;

    /**
     * @brief Gets the most recently read raw MPU data.
     * @return RawMPUData struct containing accelerometer and gyroscope data.
     */
    DroneTypes::RawMPUData getRawMPUData() const;

    /**
     * @brief Checks if new data was ready after the last update() call.
     * @return true if new data was available, false otherwise.
     */
    bool isDataReady() const;

    /**
     * @brief Runs the mpu9250 calibration routines (Accel/Gyro and Mag).
     * @return true if calibration seemed successful.
     */
    bool runCalibrationRoutine();

    void setGyroBias(float bx, float by, float bz);
    void setAccBias(float bx, float by, float bz);
    void setMagBias(float bx, float by, float bz);
    void setMagScale(float sx, float sy, float sz);

private:
    MPU9250 mpu9250_sensor_lib_;
    Preferences preferences_;

    DroneTypes::Attitude current_attitude_;
    DroneTypes::RawMPUData current_mpu_data_;
    bool data_ready_;

    /**
     * @brief Applies sensor calibration values.
     * Called by begin().
     */
    void applyCalibration();

    /**
     * @brief Attempts to load calibration data from NVS.
     * @param abias Output array for accelerometer bias (size 3).
     * @param gbias Output array for gyroscope bias (size 3).
     * @param mbias Output array for magnetometer bias (size 3).
     * @param mscale Output array for magnetometer scale (size 3).
     * @return true if calibration data was found and loaded, false otherwise.
     */
    bool loadCalibrationFromNVS(float abias[3], float gbias[3], float mbias[3], float mscale[3]);

    /**
     * @brief Saves the current calibration data (read from sensor library) to NVS.
     * @return true if saving was successful, false otherwise.
     */
    bool saveCalibrationToNVS();

    bool calibration_loaded_;
};

#endif // IMU_HANDLER_H