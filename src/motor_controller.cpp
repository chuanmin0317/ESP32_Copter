#include "motor_controller.h"
#include <Arduino.h>

void Motor::begin()
{
    ledcSetup(channel, 5000, 10);

    ledcAttachPin(motorPin, channel);
}

void Motor::setPWM(float pwm)
{
    this->pwm = pwm;
}

void MotorManger::initialMotors()
{
    motor1.begin();
    motor2.begin();
    motor3.begin();
    motor4.begin();
}