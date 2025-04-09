#ifndef IMU_HANDLER_H
#define IMU_HANDLER_H

#include <MPU9250.h>
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
    DroneTypes::RawMPUData getAttitude() const;

    /**
     * @brief Gets the most recently read raw MPU data.
     * @return RawMPUData struct containing accelerometer and gyroscope data.
     */
    DroneTypes::Attitude getRawMPUData() const;

    /**
     * @brief Checks if new data was ready after the last update() call.
     * @return true if new data was available, false otherwise.
     */
    bool isDataReady() const;

private:
    MPU9250 mpu9250_sensor;

    DroneTypes::RawMPUData current_mpu_data_;
    DroneTypes::Attitude current_attitude_;
    bool data_ready_;

    void applyCalibration();
};

#endif // IMU_HANDLER_H