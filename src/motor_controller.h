#ifndef MOTOR_CONTROLLER_H
#define MOTOR_CONTROLLER_H

#include "config.h"
#include <vector>

class Motor
{
public:
    Motor(int motorPin, int channel) : motorPin(motorPin), channel(channel), pwm(0) {}

    void begin();

    void setPWM(float pwm);

private:
    int motorPin;
    int channel;
    float pwm;
};

class MotorManger
{
public:
    void initialMotors();

private:
    Motor motor1 = Motor(MOTOR1_PIN, 0);
    Motor motor2 = Motor(MOTOR2_PIN, 1);
    Motor motor3 = Motor(MOTOR3_PIN, 2);
    Motor motor4 = Motor(MOTOR4_PIN, 3);
};

#endif