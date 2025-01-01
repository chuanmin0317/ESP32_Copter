#ifndef _MY_COMMON_H_
#define _MY_COMMON_H_

#include <Arduino.h>
#include <Wire.h>
#include <MPU9250.h>
#include <XboxSeriesXControllerESP32_asukiaaa.hpp>
#include "pid.h"
#include "remote.h"
#include "control.h"
#include "imu.h"
#include "config.h"

extern XboxSeriesXControllerESP32_asukiaaa::Core xboxController;
extern uint16_t joystickMax = XboxControllerNotificationParser::maxJoy;

extern MPU9250 mpu9250;

extern st_Mpu Mpu9250;
extern st_Angle Angle;
extern st_Controller Controller;

extern PidObject pidRateRoll;
extern PidObject pidRatePitch;
extern PidObject pidRateYaw;

extern PidObject pidRoll;
extern PidObject pidPitch;
extern PidObject pidYaw;

extern uint32_t Timer;

extern void ALL_Init();
#endif