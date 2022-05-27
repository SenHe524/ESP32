// KasBot V1  -  Kalman filter module
#include <stdio.h>
#include <string.h>
#include "KF.h"

float Q_angle  =  0.01; //0.001    //0.005
float Q_gyro   =  0.0003;  //0.003  //0.0003
float R_angle  =  0.01;  //0.03     //0.008

float P_00 = 0, P_01 = 0, P_10 = 0, P_11 = 0;	
float  y, S;
float K_0, K_1;





// Q_angle = 0.001f;
float Q_bias = 0.003f;
float R_measure = 0.03f;

float angle = 0.0f; // Reset the angle
float bias = 0.0f; // Reset bias
float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};



// The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
float Kalman_getAngle(float newAngle, float newRate, int looptime)
{
    // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
    // Modified by Kristian Lauszus
    // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

    // Discrete Kalman filter time update equations - Time Update ("Predict")
    // Update xhat - Project the state ahead
    /* Step 1 */
    float dt = (float)looptime / 1000;
    angle += dt * (newRate - bias);

    // Update estimation error covariance - Project the error covariance ahead
    /* Step 2 */
    P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_angle);
    P[0][1] -= dt * P[1][1];
    P[1][0] -= dt * P[1][1];
    P[1][1] += Q_bias * dt;

    // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
    // Calculate Kalman gain - Compute the Kalman gain
    /* Step 4 */
    S = P[0][0] + R_measure; // Estimate error
    /* Step 5 */
    float K[2]; // Kalman gain - This is a 2x1 vector
    K[0] = P[0][0] / S;
    K[1] = P[1][0] / S;

    // Calculate angle and bias - Update estimate with measurement zk (newAngle)
    /* Step 3 */
    y = newAngle - angle; // Angle difference
    /* Step 6 */
    angle += K[0] * y;
    bias += K[1] * y;

    // Calculate estimation error covariance - Update the error covariance
    /* Step 7 */
    float P00_temp = P[0][0];
    float P01_temp = P[0][1];

    P[0][0] -= K[0] * P00_temp;
    P[0][1] -= K[0] * P01_temp;
    P[1][0] -= K[1] * P00_temp;
    P[1][1] -= K[1] * P01_temp;

    return angle;
}

float kalmanCalculate(float newAngle, float newRate,int looptime)
{
float dt = (float)looptime / 1000;
float x_angle = 0;                                    // XXXXXXX arevoir
x_angle += dt * (newRate - bias);
P_00 +=  - dt * (P_10 + P_01) + Q_angle * dt;
P_01 +=  - dt * P_11;
P_10 +=  - dt * P_11;
P_11 +=  + Q_gyro * dt;

y = newAngle - x_angle;
S = P_00 + R_angle;
K_0 = P_00 / S;
K_1 = P_10 / S;

x_angle +=  K_0 * y;
bias  +=  K_1 * y;
P_00 -= K_0 * P_00;
P_01 -= K_0 * P_01;
P_10 -= K_1 * P_00;
P_11 -= K_1 * P_01;

return x_angle;
}