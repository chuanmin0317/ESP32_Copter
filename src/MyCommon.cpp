#ifndef _ALL_DEFINED_H_
#define _ALL_DEFINED_H_

#include "MyCommon.h"

uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;


st_Mpu Mpu9250;
st_Angle Angle;
st_Controller Controller;

PidObject pidRateRoll;
PidObject pidRatePitch;
PidObject pidRateYaw;

PidObject pidRoll;
PidObject pidPitch;
PidObject pidYaw;

uint32_t Timer;

void PID_ParamInit(void);

void ALL_Init()
{
    ControllerInit();
    PID_ParamInit();
    MotorInit();
    MpuInit();
}

void PID_ParamInit(void)
{
    pidRateRoll.kp = 0;
    pidRatePitch.kp = 0;
    pidRateYaw.kp = 0;

    pidRateRoll.ki = 0;
    pidRatePitch.ki = 0;
    pidRateYaw.ki = 0;

    pidRateRoll.kd = 0;
    pidRatePitch.kd = 0;
    pidRateYaw.kd = 0;

    pidRoll.kp = 0;
    pidPitch.kp = 0;
    pidYaw.kp = 0;
}

#endif