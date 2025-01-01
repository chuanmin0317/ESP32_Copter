#include "pid.h"

void pidReset(PidObject **pid, const int len)
{
   for (int i = 0; i < len; i++)
   {
      (*pid)[i].integral = 0;
      (*pid)[i].prevError = 0;
      (*pid)[i].output = 0;
   }
}

void pidUpdate(PidObject *pid, const float dt)
{
    float error = pid->desired - pid->measure;
    pid->integral += error * dt;
    if (pid->integral > pid->IntegLimitHigh)
        pid->integral = pid->IntegLimitHigh;
    else if (pid->integral < pid->IntegLimitLow)
        pid->integral = pid->IntegLimitLow;
    float derivative = (error - pid->prevError) / dt;
    pid->output = pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    if (pid->output > pid->OutLimitHigh)
        pid->output = pid->OutLimitHigh;
    else if (pid->output < pid->OutLimitLow)
        pid->output = pid->OutLimitLow;
    pid->prevError = error;
}

void CascadePID(PidObject *pidRate, PidObject *pidAng, const float dt)
{
    pidUpdate(pidAng, dt);
    pidRate->desired = pidAng->output;
    pidUpdate(pidRate, dt);
}