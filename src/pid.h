#ifndef _PID_H
#define _PID_H

typedef struct
{
    float desired;
    float prevError;
    float integral;
    float kp = 0;
    float ki = 0;
    float kd = 0;
    const float IntegLimitHigh = 400;
    const float IntegLimitLow = -400;
    float measure;
    float output;
    const float OutLimitHigh = 400;
    const float OutLimitLow = -400;
} PidObject;

extern void pidReset(PidObject **pid, const int len);
extern void pidUpdate(PidObject *pid, const float dt);
extern void CascadePID(PidObject *pidRate, PidObject *pidAng, const float dt);
#endif