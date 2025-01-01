#ifndef _IMU_H_
#define _IMU_H_
#include "MyCommon.h"

typedef struct{
    int16_t accX;
    int16_t accY;
    int16_t accZ;
    int16_t gyroX;
    int16_t gyroY;
    int16_t gyroZ;
}st_Mpu;

typedef struct{
    float roll;
    float pitch;
    float yaw;
}st_Angle;

extern void MpuInit(void);
extern void GetImuData(st_Mpu *mpu, st_Angle *angle);
#endif
