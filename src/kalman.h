#ifndef _KALMAN_H
#define _KALMAN_H

struct  _1_ekf_filter
{
    float Predict;
    float Q;
    float R;
    float Kg;
    float out;
};

extern void KalmanAngle();
#endif