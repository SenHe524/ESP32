#ifndef _IMU_H_
#define _IMU_H_

void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *Q_angle);
void IMU_update(float gx, float gy, float gz, float ax, float ay, float az, float *Q_angle);

#endif