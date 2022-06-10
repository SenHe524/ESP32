// KasBot V1  -  Kalman filter module

#ifndef _KF_H_
#define _KF_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>


float kalmanCalculate(float newAngle, float newRate, int looptime);

#endif