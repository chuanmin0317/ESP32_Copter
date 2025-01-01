#ifndef _KALMAN_H
#define _KALMAN_H

struct  _1_ekf_filter
{
    float LastP;
    float NowP;
    float Q;
    float R;
    float Kg;
    float out;
};

extern void kalman_1d(struct _1_ekf_filter *ekf, float Rate, float Angle);
#endif