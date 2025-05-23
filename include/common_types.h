// include/common_types.h
#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h>

namespace DroneTypes
{

    struct RawMPUData
    {
        float accX;
        float accY;
        float accZ;
        float gyroX;
        float gyroY;
        float gyroZ;
    };

    struct Attitude
    {
        float roll;
        float pitch;
        float yaw;
    };

    struct RemoteInputRaw
    {
        uint16_t roll_raw;
        uint16_t pitch_raw;
        uint16_t yaw_raw;
        uint16_t throttle_raw;
    };

    struct ControlSetpoint
    {
        float roll_angle_setpoint;
        float pitch_angle_setpoint;
        float yaw_rate_setpoint;
        uint16_t throttle_value;
    };

    struct MotorCommands
    {
        uint16_t motor1;
        uint16_t motor2;
        uint16_t motor3;
        uint16_t motor4;
    };

    struct SensorData
    {
        Attitude attitude;
        RawMPUData raw_mpu;
    };

    struct RemoteData
    {
        ControlSetpoint setpoint;
        bool is_connected;
    };
}

#endif // COMMON_TYPES_H