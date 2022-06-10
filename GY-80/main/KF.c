// KasBot V1  -  Kalman filter module
#include <stdio.h>
#include <string.h>
#include "KF.h"

// float  Angle_dif, S;
// float Q_Angle = 0.001f; //0.01f    //0.005f
// float Q_bias = 0.0003f;  //0.003  //0.0003
// float R_measure = 0.03f;
// float angle = 0.0f; // Reset the angle
// float bias = 0.0f; // Reset bias
// float P[2][2] = {{0.0f, 0.0f}, {0.0f, 0.0f}};


// // The angle should be in degrees and the rate should be in degrees per second and the delta time in seconds
// float kalmanCalculate(float newAngle, float newRate, int looptime)
// {
//     // KasBot V2  -  Kalman filter module - http://www.x-firm.com/?page_id=145
//     // Modified by Kristian Lauszus
//     // See my blog post for more information: http://blog.tkjelectronics.dk/2012/09/a-practical-approach-to-kalman-filter-and-how-to-implement-it

//     // Discrete Kalman filter time update equations - Time Update ("Predict")
//     // Update xhat - Project the state ahead
//     /* Step 1 */
//     float dt = (float)looptime / 1000;
//     angle += dt * (newRate - bias);

//     // Update estimation error covariance - Project the error covariance ahead
//     /* Step 2 */
//     P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Q_Angle);
//     P[0][1] -= dt * P[1][1];
//     P[1][0] -= dt * P[1][1];
//     P[1][1] += Q_bias * dt;

//     // Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
//     // Calculate Kalman gain - Compute the Kalman gain
//     /* Step 4 */
//     S = P[0][0] + R_measure; // Estimate error
//     /* Step 5 */
//     float K[2]; // Kalman gain - This is a 2x1 vector
//     K[0] = P[0][0] / S;
//     K[1] = P[1][0] / S;

//     // Calculate angle and bias - Update estimate with measurement zk (newAngle)
//     /* Step 3 */
//     Angle_dif = newAngle - angle; // Angle difference
//     /* Step 6 */
//     angle += K[0] * Angle_dif;
//     bias += K[1] * Angle_dif;

//     // Calculate estimation error covariance - Update the error covariance
//     /* Step 7 */
//     float P00_temp = P[0][0];
//     float P01_temp = P[0][1];

//     P[0][0] -= K[0] * P00_temp;
//     P[0][1] -= K[0] * P01_temp;
//     P[1][0] -= K[1] * P00_temp;
//     P[1][1] -= K[1] * P01_temp;

//     return angle;
// }

float angle; 
float q_bias; 
float rate; 

static float P[2][2] = {{ 1, 0 }, { 0, 1 }}; 

static const float R_angle = 0.5 ; 
static const float Q_angle = 0.001; 
static const float Q_gyro  = 0.003; 

float stateUpdate(const float gyro_m, float dt)
{ 
    float q; 
    float Pdot[4]; 
    q = gyro_m - q_bias; 
    Pdot[0] = Q_angle - P[0][1] - P[1][0];    /* 0,0 */ 
    Pdot[1] = - P[1][1];             /* 0,1 */ 
    Pdot[2] = - P[1][1];                 /* 1,0 */ 
    Pdot[3] = Q_gyro;     /* 1,1 */ 

    rate = q; 


    angle += q * dt; 

    P[0][0] += Pdot[0] * dt; 
    P[0][1] += Pdot[1] * dt; 
    P[1][0] += Pdot[2] * dt; 
    P[1][1] += Pdot[3] * dt; 

    return angle; 
} 

float kalmanCalculate(float newAngle, float newRate, int looptime)
{ 

    float angle_m = newAngle; 
    float angle_err = angle_m - angle; 
    float dt = (float)looptime / 1000;
    angle = stateUpdate(newRate, dt);
    float h_0 = 1; 

    const float PHt_0 = h_0*P[0][0]; /* + h_1*P[0][1] = 0*/ 
    const float PHt_1 = h_0*P[1][0]; /* + h_1*P[1][1] = 0*/ 

    float E = R_angle +(h_0 * PHt_0); 

    float K_0 = PHt_0 / E; 
    float K_1 = PHt_1 / E; 

    float Y_0 = PHt_0;  /*h_0 * P[0][0]*/ 
    float Y_1 = h_0 * P[0][1]; 
    
    P[0][0] -= K_0 * Y_0; 
    P[0][1] -= K_0 * Y_1; 
    P[1][0] -= K_1 * Y_0; 
    P[1][1] -= K_1 * Y_1; 


    angle += K_0 * angle_err; 
    q_bias += K_1 * angle_err; 

    return angle; 
}