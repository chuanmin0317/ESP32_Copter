#ifndef _KALMAN_H
#define _KALMAN_H

struct  _1_ekf_filter
{
    float LastP;
    float Now_P;
    float Q;
    float R;
    float Kg;
    float out;
};

extern void KalmanAngle();
#endif