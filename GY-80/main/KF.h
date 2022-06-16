// KasBot V1  -  Kalman filter module

#ifndef _KF_H_
#define _KF_H_


float kalmanCalculate(float newAngle, float newRate, int looptime, int i);
void kalmanSetAngle(float angle, int i);
#endif