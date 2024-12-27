#ifndef SENSORS_H
#define SENSORS_H

#include <MPU9250.h>

class SensorManger
{
public:
    bool begin(int sda, int scl, uint8_t address);

    void loadCalibration();

    void update();

    float getRoll();
    float getPitch();
    float getYaw();

private:
    MPU9250 mpu9250;
    bool initialized = false;
};

#endif