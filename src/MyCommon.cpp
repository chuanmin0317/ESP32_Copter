#ifndef _ALL_DEFINED_H_
#define _ALL_DEFINED_H_

#include "MyCommon.h"

uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;
XboxSeriesXControllerESP32_asukiaaa::Core xboxController;

MPU9250 mpu9250;

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

void ALL_Init(){
    PID_ParamInit();
    ControllerInit();
    MotorInit();
    MpuInit();
}

void PID_ParamInit(void){
    pidRateRoll.kp = pidRatePitch.kp = 0.6;
    pidRateRoll.ki = pidRatePitch.ki = 3.5;
    pidRateRoll.kd = pidRatePitch.kd = 0.03;

    pidRateYaw.kp = 2;
    pidRateYaw.ki = 12;
    pidRateYaw.kd = 0;

    pidRoll.kp = pidPitch.kp = 2;
    pidRoll.ki = pidPitch.ki = 0;
    pidRoll.kd = pidPitch.kd = 0;
}

#endif