// include/common_types.h
#ifndef COMMON_TYPES_H
#define COMMON_TYPES_H

#include <stdint.h>

namespace DroneTypes
{

    struct RawMPUData
    {
        int16_t accX;
        int16_t accY;
        int16_t accZ;
        int16_t gyroX;
        int16_t gyroY;
        int16_t gyroZ;
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

}

#endif // COMMON_TYPES_H