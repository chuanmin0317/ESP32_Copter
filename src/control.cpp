#include "control.h"
#include "MyCommon.h"
#include "pid.h"

PidObject *(pPidObject[6]) = {&pidRateRoll, &pidRatePitch, &pidRateYaw, &pidRoll, &pidPitch, &pidYaw};

void PidControl(float dt)
{
    pidRateRoll.measure = Mpu9250.gyroX;
    pidRatePitch.measure = Mpu9250.gyroY;
    pidRateYaw.measure = Mpu9250.gyroZ;

    pidRoll.measure = Angle.roll;
    pidPitch.measure = Angle.pitch;
    pidYaw.measure = Angle.yaw;

    CascadePID(&pidRateRoll, &pidRoll, dt);
    CascadePID(&pidRatePitch, &pidPitch, dt);
    CascadePID(&pidRateYaw, &pidYaw, dt);
}

void MotorInit()
{
    ledcSetup(0, 250, 10);
    ledcAttachPin(MOTOR1_PIN, 0);

    ledcSetup(1, 250, 10);
    ledcAttachPin(MOTOR2_PIN, 1);

    ledcSetup(2, 250, 10);
    ledcAttachPin(MOTOR3_PIN, 2);

    ledcSetup(3, 250, 10);
    ledcAttachPin(MOTOR4_PIN, 3);
}

int16_t motor[4];
#define MOTOR1 motor[0]
#define MOTOR2 motor[1]
#define MOTOR3 motor[2]
#define MOTOR4 motor[3]

int16_t LIMIT(int16_t value, int16_t max, int16_t min)
{
    if (value > max)
        return max;
    if (value < min)
        return min;
    return value;
}

void MotorControl()
{
    int16_t temp;
    temp = Controller.throttle - 1000;

    if (Controller.throttle < 1030)
    {
        MOTOR1 = MOTOR2 = MOTOR3 = MOTOR4 = 0;
        pidReset(pPidObject, 6);
        return;
    }

    temp = LIMIT(temp, 900, 0);

    MOTOR1 = temp - pidRateRoll.output + pidRatePitch.output - pidRateYaw.output; // 左前
    MOTOR2 = temp + pidRateRoll.output + pidRatePitch.output + pidRateYaw.output; // 右前
    MOTOR3 = temp + pidRateRoll.output - pidRatePitch.output - pidRateYaw.output; // 右後
    MOTOR4 = temp - pidRateRoll.output - pidRatePitch.output + pidRateYaw.output; // 左後

    MOTOR1 = LIMIT(MOTOR1, 1000, 0);
    MOTOR2 = LIMIT(MOTOR2, 1000, 0);
    MOTOR3 = LIMIT(MOTOR3, 1000, 0);
    MOTOR4 = LIMIT(MOTOR4, 1000, 0);

    ledcWrite(0, MOTOR1);
    ledcWrite(1, MOTOR2);
    ledcWrite(2, MOTOR3);
    ledcWrite(3, MOTOR4);
}