#include "kalman.h"
#include "MyCommon.h"

void kalman_1d(struct _1_ekf_filter *ekf, float Rate, float Angle)
{
    ekf->Predict = ekf->Predict + Rate * ekf->Q;
    ekf->Kg = ekf->Predict / (ekf->Predict + ekf->R);
    ekf->out = ekf->out + ekf->Kg * (Angle - ekf->out);
    ekf->Predict = (1 - ekf->Kg) * ekf->Predict;
}

void KalmanAngle()
{
    for (int i = 0; i < 3; i++)
    {
        static struct _1_ekf_filter AngleFilter[3] = {
            {0, 0.004, 0.1, 0, 0},
            {0, 0.004, 0.1, 0, 0},
            {0, 0.004, 0.1, 0, 0}};
        if (i == 0)
        {
            kalman_1d(&AngleFilter[i], Mpu9250.gyroX, Angle.roll);
            Angle.roll = AngleFilter[i].out;
        }
        else if (i == 1)
        {
            kalman_1d(&AngleFilter[i], Mpu9250.gyroY, Angle.pitch);
            Angle.pitch = AngleFilter[i].out;
        }
        else
        {
            kalman_1d(&AngleFilter[i], Mpu9250.gyroZ, Angle.yaw);
            Angle.yaw = AngleFilter[i].out;
        }
    }
}