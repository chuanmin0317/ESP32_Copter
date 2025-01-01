#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "pid.h"

extern void PidControl(float ControlTime);
extern void MotorControl(void);
extern void MotorInit(void);
#endif