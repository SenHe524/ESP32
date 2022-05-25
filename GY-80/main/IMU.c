#include "IMU.h"
#include <math.h>
#include "stdio.h"

/*************************百度知道代码宏定义****************************************/
// #define Kp 1.0f // proportional gain governs rate of convergence to accelerometer/magnetometer
// #define Ki 0.01f // integral gain governs rate of convergence of gyroscope biases
float invSqrt(float x);
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
// static float exInt = 0, eyInt = 0, ezInt = 0;

//不知名代码——AHRSupdate 宏定义
// AHRS algorithm update
static float exInt = 0, eyInt = 0, ezInt = 0;
static float halfT = 0.1;
static float k10 = 0.0f, k11 = 0.0f, k12 = 0.0f, k13 = 0.0f;
static float k20 = 0.0f, k21 = 0.0f, k22 = 0.0f, k23 = 0.0f;
static float k30 = 0.0f, k31 = 0.0f, k32 = 0.0f, k33 = 0.0f;
static float k40 = 0.0f, k41 = 0.0f, k42 = 0.0f, k43 = 0.0f;
#define Kp 1.0f // proportional gain governs rate of convergence toaccelerometer/magnetometer
// Kp比例增益 决定了加速度计和磁力计的收敛速度
#define Ki 0.001f // integral gain governs rate of convergenceof gyroscope biases
// Ki积分增益 决定了陀螺仪偏差的收敛速度

/*******论文代码宏定义---filterUpdate***********/
// System constants
#define deltat 0.001f									 // sampling period in seconds (shown as 1 ms)
#define gyroMeasError 3.14159265358979 * (5.0f / 180.0f) // gyroscope measurement error in rad/s (shown as 5 deg/s)
#define gyroMeasDrift 3.14159265358979 * (0.2f / 180.0f) // gyroscope measurement error in rad/s/s (shown as 0.2f deg/s/s)
#define beta sqrt(3.0f / 4.0f) * gyroMeasError			 // compute beta
#define zeta sqrt(3.0f / 4.0f) * gyroMeasDrift			 // compute zeta
// Global system variables
float a_x, a_y, a_z;							  // accelerometer measurements
float w_x, w_y, w_z;							  // gyroscope measurements in rad/s
float m_x, m_y, m_z;							  // magnetometer measurements
float SEq_1 = 1, SEq_2 = 0, SEq_3 = 0, SEq_4 = 0; // estimated orientation quaternion elements with initial conditions
float b_x = 1, b_z = 0;							  // reference direction of flux in earth frame
float w_bx = 0, w_by = 0, w_bz = 0;				  // estimate gyroscope biases error

/********百度知道代码*****/
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	// float halfT = 0.005;
	float tempq0, tempq1, tempq2, tempq3;
	// 先把这些用得到的值算好
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;

	//把加计的三维向量转成单位向量。
	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	norm = invSqrt(mx*mx + my*my + mz*mz);
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;
	/*
	这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
	根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
	所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
	*/
	// compute reference direction of flux
	//hx, hy, hz 表示将飞行器参考系上的地磁矢量转换到地理坐标系（参考坐标系）后的矢量
	hx = 2*mx*(0.5f - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
	hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5f - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
	hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5f - q1q1 - q2q2);
	bx = sqrt((hx*hx) + (hy*hy));//实质为误差函数，bx越接近hx，则估计姿态与电子罗盘测得姿态越接近
	bz = hz;
	// estimated direction of gravity and flux (v and w)
	//vx, vy, vz为将 标准单位重力 转换到飞行器参考系后 各个坐标轴上的分量（重力加速度）
	vx = 2 * (q1q3 - q0q2);
	vy = 2 * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	// //wx, wy, wz为将 bx与bz又重新 转换到飞行器参考系后 各个坐标轴上的分量。（地磁场）
	wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
	wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
	wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay*vz - az*vy) + (my*wz - mz*wy);
	ey = (az*vx - ax*vz) + (mz*wx - mx*wz);
	ez = (ax*vy - ay*vx) + (mx*wy - my*wx);
	/*
	axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
	axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
	那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
	向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
	这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。
	*/
	if (ex != 0.0f && ey != 0.0f && ez != 0.0f)
	{
		exInt = exInt + ex * Ki * halfT;
		eyInt = eyInt + ey * Ki * halfT;
		ezInt = ezInt + ez * Ki * halfT;
		// 用叉积误差来做PI修正陀螺零偏
		gx = gx + Kp * ex + exInt;
		gy = gy + Kp * ey + eyInt;
		gz = gz + Kp * ez + ezInt;
	}
	// 四元数微分方程
	tempq0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	tempq1 = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	tempq2 = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	tempq3 = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
	// 四元数规范化
	norm = invSqrt(tempq0 * tempq0 + tempq1 * tempq1 + tempq2 * tempq2 + tempq3 * tempq3);
	q0 = tempq0 * norm;
	q1 = tempq1 * norm;
	q2 = tempq2 * norm;
	q3 = tempq3 * norm;
	printf("Pitch: %f\n", -asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.2957795f);
	printf("Roll: %f\n", atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.2957795f);
	printf("Yaw: %f\n",-atan2(2*(q1*q2 + q0*q3),q0*q0+q1*q1-q2*q2-q3*q3) * 57.2957795f);
}

//*********代码来源：桌面文件-TESTVer.2.13******************//
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz)
{
	float norm;				  //用于单位化
	float hx, hy, hz, bx, bz; //
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	// auxiliary variables to reduce number of repeated operations  辅助变量减少重复操作次数
	float q0q0 = q0 * q0;
	float q0q1 = q0 * q1;
	float q0q2 = q0 * q2;
	float q0q3 = q0 * q3;
	float q1q1 = q1 * q1;
	float q1q2 = q1 * q2;
	float q1q3 = q1 * q3;
	float q2q2 = q2 * q2;
	float q2q3 = q2 * q3;
	float q3q3 = q3 * q3;
	// normalise the measurements  对加速度计和磁力计数据进行规范化
	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;
	norm = invSqrt(mx * mx + my * my + mz * mz);
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;
	// estimated direction of gravity and magnetic field (v and w)  //估计重力和磁场的方向
	// vx,vy,vz是重力加速度在物体坐标系的表示
	vx = 2.0f * (q1q3 - q0q2);
	vy = 2.0f * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	// compute reference direction of magnetic field  计算磁场的参考方向
	// hx,hy,hz是mx,my,mz在参考坐标系的表示
	hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
	hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
	hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
	// bx,by,bz是地球磁场在参考坐标系的表示
	bx = sqrtf((hx * hx) + (hy * hy));
	bz = hz;

	// wx,wy,wz是地磁场在物体坐标系的表示
	wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
	wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
	wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);

	// error is sum ofcross product between reference direction of fields and directionmeasured by sensors
	// ex,ey,ez是加速度计与磁力计测量出的方向与实际重力加速度与地磁场方向的误差，误差用叉积来表示，且加速度计与磁力计的权重是一样的
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);

	exInt = exInt + ex * halfT;
	eyInt = eyInt + ey * halfT;
	ezInt = ezInt + ez * halfT;
	// adjusted gyroscope measurements
	// PI调节陀螺仪数据
	gx = gx + Kp * ex + Ki * exInt;
	gy = gy + Kp * ey + Ki * eyInt;
	gz = gz + Kp * ez + Ki * ezInt;

	// RUNGE_KUTTA 法解微分方程
	k10 = 0.5 * (-gx * q1 - gy * q2 - gz * q3);
	k11 = 0.5 * (gx * q0 + gz * q2 - gy * q3);
	k12 = 0.5 * (gy * q0 - gz * q1 + gx * q3);
	k13 = 0.5 * (gz * q0 + gy * q1 - gx * q2);

	k20 = 0.5 * (halfT * (q0 + halfT * k10) + (halfT - gx) * (q1 + halfT * k11) + (halfT - gy) * (q2 + halfT * k12) + (halfT - gz) * (q3 + halfT * k13));
	k21 = 0.5 * ((halfT + gx) * (q0 + halfT * k10) + halfT * (q1 + halfT * k11) + (halfT + gz) * (q2 + halfT * k12) + (halfT - gy) * (q3 + halfT * k13));
	k22 = 0.5 * ((halfT + gy) * (q0 + halfT * k10) + (halfT - gz) * (q1 + halfT * k11) + halfT * (q2 + halfT * k12) + (halfT + gx) * (q3 + halfT * k13));
	k23 = 0.5 * ((halfT + gz) * (q0 + halfT * k10) + (halfT + gy) * (q1 + halfT * k11) + (halfT - gx) * (q2 + halfT * k12) + halfT * (q3 + halfT * k13));

	k30 = 0.5 * (halfT * (q0 + halfT * k20) + (halfT - gx) * (q1 + halfT * k21) + (halfT - gy) * (q2 + halfT * k22) + (halfT - gz) * (q3 + halfT * k23));
	k31 = 0.5 * ((halfT + gx) * (q0 + halfT * k20) + halfT * (q1 + halfT * k21) + (halfT + gz) * (q2 + halfT * k22) + (halfT - gy) * (q3 + halfT * k23));
	k32 = 0.5 * ((halfT + gy) * (q0 + halfT * k20) + (halfT - gz) * (q1 + halfT * k21) + halfT * (q2 + halfT * k22) + (halfT + gx) * (q3 + halfT * k23));
	k33 = 0.5 * ((halfT + gz) * (q0 + halfT * k20) + (halfT + gy) * (q1 + halfT * k21) + (halfT - gx) * (q2 + halfT * k22) + halfT * (q3 + halfT * k23));

	k40 = 0.5 * (2 * halfT * (q0 + 2 * halfT * k30) + (2 * halfT - gx) * (q1 + 2 * halfT * k31) + (2 * halfT - gy) * (q2 + 2 * halfT * k32) + (2 * halfT - gz) * (q3 + 2 * halfT * k33));
	k41 = 0.5 * ((2 * halfT + gx) * (q0 + 2 * halfT * k30) + 2 * halfT * (q1 + 2 * halfT * k31) + (2 * halfT + gz) * (q2 + 2 * halfT * k32) + (2 * halfT - gy) * (q3 + 2 * halfT * k33));
	k42 = 0.5 * ((2 * halfT + gy) * (q0 + 2 * halfT * k30) + (2 * halfT - gz) * (q1 + 2 * halfT * k31) + 2 * halfT * (q2 + 2 * halfT * k32) + (2 * halfT + gx) * (q3 + 2 * halfT * k33));
	k43 = 0.5 * ((2 * halfT + gz) * (q0 + 2 * halfT * k30) + (2 * halfT + gy) * (q1 + 2 * halfT * k31) + (2 * halfT - gx) * (q2 + 2 * halfT * k32) + 2 * halfT * (q3 + 2 * halfT * k33));

	q0 = q0 + 2 * halfT / 6.0 * (k10 + 2 * k20 + 2 * k30 + k40);
	q1 = q1 + 2 * halfT / 6.0 * (k11 + 2 * k21 + 2 * k31 + k41);
	q2 = q2 + 2 * halfT / 6.0 * (k12 + 2 * k22 + 2 * k32 + k42);
	q3 = q3 + 2 * halfT / 6.0 * (k13 + 2 * k23 + 2 * k33 + k43);

	// normalise quaternion
	norm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	q0 = q0 * norm;
	q1 = q1 * norm;
	q2 = q2 * norm;
	q3 = q3 * norm;

	printf("Pitch: %f\n", -asin(-2 * q1 * q3 + 2 * q0 * q2) * 57.2957795f);
	printf("Roll: %f\n", atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 57.2957795f);
	printf("Yaw: %f\n", -atan2(2 * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.2957795f);
}

/***************论文代码*******************/
// Function to compute one filter iteration
void filterUpdate(float w_x, float w_y, float w_z, float a_x, float a_y, float a_z, float m_x, float m_y, float m_z)
{
	// local system variables
	float norm;																	// vector norm
	float SEqDot_omega_1, SEqDot_omega_2, SEqDot_omega_3, SEqDot_omega_4;		// quaternion rate from gyroscopes elements
	float f_1, f_2, f_3, f_4, f_5, f_6;											// objective function elements
	float J_11or24, J_12or23, J_13or22, J_14or21, J_32, J_33,					// objective function Jacobian elements
		J_41, J_42, J_43, J_44, J_51, J_52, J_53, J_54, J_61, J_62, J_63, J_64; //
	float SEqHatDot_1, SEqHatDot_2, SEqHatDot_3, SEqHatDot_4;					// estimated direction of the gyroscope error
	float w_err_x, w_err_y, w_err_z;											// estimated direction of the gyroscope error (angular)
	float h_x, h_y, h_z;														// computed flux in the earth frame
	// axulirary variables to avoid reapeated calcualtions
	float halfSEq_1 = 0.5f * SEq_1;
	float halfSEq_2 = 0.5f * SEq_2;
	float halfSEq_3 = 0.5f * SEq_3;
	float halfSEq_4 = 0.5f * SEq_4;
	float twoSEq_1 = 2.0f * SEq_1;
	float twoSEq_2 = 2.0f * SEq_2;
	float twoSEq_3 = 2.0f * SEq_3;
	float twoSEq_4 = 2.0f * SEq_4;
	float twob_x = 2.0f * b_x;
	float twob_z = 2.0f * b_z;
	float twob_xSEq_1 = 2.0f * b_x * SEq_1;
	float twob_xSEq_2 = 2.0f * b_x * SEq_2;
	float twob_xSEq_3 = 2.0f * b_x * SEq_3;
	float twob_xSEq_4 = 2.0f * b_x * SEq_4;
	float twob_zSEq_1 = 2.0f * b_z * SEq_1;
	float twob_zSEq_2 = 2.0f * b_z * SEq_2;
	float twob_zSEq_3 = 2.0f * b_z * SEq_3;
	float twob_zSEq_4 = 2.0f * b_z * SEq_4;
	float SEq_1SEq_2;
	float SEq_1SEq_3 = SEq_1 * SEq_3;
	float SEq_1SEq_4;
	float SEq_2SEq_3;
	float SEq_2SEq_4 = SEq_2 * SEq_4;
	float SEq_3SEq_4;
	float twom_x = 2.0f * m_x;
	float twom_y = 2.0f * m_y;
	float twom_z = 2.0f * m_z;
	// normalise the accelerometer measurement
	norm = sqrt(a_x * a_x + a_y * a_y + a_z * a_z);
	a_x /= norm;
	a_y /= norm;
	a_z /= norm;
	// normalise the magnetometer measurement
	norm = sqrt(m_x * m_x + m_y * m_y + m_z * m_z);
	m_x /= norm;
	m_y /= norm;
	m_z /= norm;
	// compute the objective function and Jacobian
	f_1 = twoSEq_2 * SEq_4 - twoSEq_1 * SEq_3 - a_x;
	f_2 = twoSEq_1 * SEq_2 + twoSEq_3 * SEq_4 - a_y;
	f_3 = 1.0f - twoSEq_2 * SEq_2 - twoSEq_3 * SEq_3 - a_z;
	f_4 = twob_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twob_z * (SEq_2SEq_4 - SEq_1SEq_3) - m_x;
	f_5 = twob_x * (SEq_2 * SEq_3 - SEq_1 * SEq_4) + twob_z * (SEq_1 * SEq_2 + SEq_3 * SEq_4) - m_y;
	f_6 = twob_x * (SEq_1SEq_3 + SEq_2SEq_4) + twob_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3) - m_z;
	J_11or24 = twoSEq_3; // J_11 negated in matrix multiplication
	J_12or23 = 2.0f * SEq_4;
	J_13or22 = twoSEq_1; // J_12 negated in matrix multiplication
	J_14or21 = twoSEq_2;
	J_32 = 2.0f * J_14or21; // negated in matrix multiplication
	J_33 = 2.0f * J_11or24; // negated in matrix multiplication
	J_41 = twob_zSEq_3;		// negated in matrix multiplication
	J_42 = twob_zSEq_4;
	J_43 = 2.0f * twob_xSEq_3 + twob_zSEq_1; // negated in matrix multiplication
	J_44 = 2.0f * twob_xSEq_4 - twob_zSEq_2; // negated in matrix multiplication
	J_51 = twob_xSEq_4 - twob_zSEq_2;		 // negated in matrix multiplication
	J_52 = twob_xSEq_3 + twob_zSEq_1;
	J_53 = twob_xSEq_2 + twob_zSEq_4;
	J_54 = twob_xSEq_1 - twob_zSEq_3; // negated in matrix multiplication
	J_61 = twob_xSEq_3;
	J_62 = twob_xSEq_4 - 2.0f * twob_zSEq_2;
	J_63 = twob_xSEq_1 - 2.0f * twob_zSEq_3;
	J_64 = twob_xSEq_2;
	// compute the gradient (matrix multiplication)
	SEqHatDot_1 = J_14or21 * f_2 - J_11or24 * f_1 - J_41 * f_4 - J_51 * f_5 + J_61 * f_6;
	SEqHatDot_2 = J_12or23 * f_1 + J_13or22 * f_2 - J_32 * f_3 + J_42 * f_4 + J_52 * f_5 + J_62 * f_6;
	SEqHatDot_3 = J_12or23 * f_2 - J_33 * f_3 - J_13or22 * f_1 - J_43 * f_4 + J_53 * f_5 + J_63 * f_6;
	SEqHatDot_4 = J_14or21 * f_1 + J_11or24 * f_2 - J_44 * f_4 - J_54 * f_5 + J_64 * f_6;
	// normalise the gradient to estimate direction of the gyroscope error
	norm = sqrt(SEqHatDot_1 * SEqHatDot_1 + SEqHatDot_2 * SEqHatDot_2 + SEqHatDot_3 * SEqHatDot_3 + SEqHatDot_4 * SEqHatDot_4);
	SEqHatDot_1 = SEqHatDot_1 / norm;
	SEqHatDot_2 = SEqHatDot_2 / norm;
	SEqHatDot_3 = SEqHatDot_3 / norm;
	SEqHatDot_4 = SEqHatDot_4 / norm;
	// compute angular estimated direction of the gyroscope error
	w_err_x = twoSEq_1 * SEqHatDot_2 - twoSEq_2 * SEqHatDot_1 - twoSEq_3 * SEqHatDot_4 + twoSEq_4 * SEqHatDot_3;
	w_err_y = twoSEq_1 * SEqHatDot_3 + twoSEq_2 * SEqHatDot_4 - twoSEq_3 * SEqHatDot_1 - twoSEq_4 * SEqHatDot_2;
	w_err_z = twoSEq_1 * SEqHatDot_4 - twoSEq_2 * SEqHatDot_3 + twoSEq_3 * SEqHatDot_2 - twoSEq_4 * SEqHatDot_1;
	// compute and remove the gyroscope baises
	w_bx += w_err_x * deltat * zeta;
	w_by += w_err_y * deltat * zeta;
	w_bz += w_err_z * deltat * zeta;
	w_x -= w_bx;
	w_y -= w_by;
	w_z -= w_bz;
	// compute the quaternion rate measured by gyroscopes
	SEqDot_omega_1 = -halfSEq_2 * w_x - halfSEq_3 * w_y - halfSEq_4 * w_z;
	SEqDot_omega_2 = halfSEq_1 * w_x + halfSEq_3 * w_z - halfSEq_4 * w_y;
	SEqDot_omega_3 = halfSEq_1 * w_y - halfSEq_2 * w_z + halfSEq_4 * w_x;
	SEqDot_omega_4 = halfSEq_1 * w_z + halfSEq_2 * w_y - halfSEq_3 * w_x;
	// compute then integrate the estimated quaternion rate
	SEq_1 += (SEqDot_omega_1 - (beta * SEqHatDot_1)) * deltat;
	SEq_2 += (SEqDot_omega_2 - (beta * SEqHatDot_2)) * deltat;
	SEq_3 += (SEqDot_omega_3 - (beta * SEqHatDot_3)) * deltat;
	SEq_4 += (SEqDot_omega_4 - (beta * SEqHatDot_4)) * deltat;
	// normalise quaternion
	norm = sqrt(SEq_1 * SEq_1 + SEq_2 * SEq_2 + SEq_3 * SEq_3 + SEq_4 * SEq_4);
	SEq_1 /= norm;
	SEq_2 /= norm;
	SEq_3 /= norm;
	SEq_4 /= norm;
	// compute flux in the earth frame
	SEq_1SEq_2 = SEq_1 * SEq_2; // recompute axulirary variables
	SEq_1SEq_3 = SEq_1 * SEq_3;
	SEq_1SEq_4 = SEq_1 * SEq_4;
	SEq_3SEq_4 = SEq_3 * SEq_4;
	SEq_2SEq_3 = SEq_2 * SEq_3;
	SEq_2SEq_4 = SEq_2 * SEq_4;
	h_x = twom_x * (0.5f - SEq_3 * SEq_3 - SEq_4 * SEq_4) + twom_y * (SEq_2SEq_3 - SEq_1SEq_4) + twom_z * (SEq_2SEq_4 + SEq_1SEq_3);
	h_y = twom_x * (SEq_2SEq_3 + SEq_1SEq_4) + twom_y * (0.5f - SEq_2 * SEq_2 - SEq_4 * SEq_4) + twom_z * (SEq_3SEq_4 - SEq_1SEq_2);
	h_z = twom_x * (SEq_2SEq_4 - SEq_1SEq_3) + twom_y * (SEq_3SEq_4 + SEq_1SEq_2) + twom_z * (0.5f - SEq_2 * SEq_2 - SEq_3 * SEq_3);
	// normalise the flux vector to have only components in the x and z
	b_x = sqrt((h_x * h_x) + (h_y * h_y));
	b_z = h_z;
	printf("%f, %f, %f, %f-------------------------------------------------\n", SEq_1, SEq_2, SEq_3, SEq_4);
	printf("Pitch: %f\n", -asin(-2 * SEq_2 * SEq_4 + 2 * SEq_1 * SEq_3) * 57.2957795f);
	printf("Roll: %f\n", atan2(2 * SEq_3 * SEq_4 + 2 * SEq_1 * SEq_2, -2 * SEq_2 * SEq_2 - 2 * SEq_3 * SEq_3 + 1) * 57.2957795f);
	printf("Yaw: %f\n", -atan2(2 * (SEq_2 * SEq_3 + SEq_1 * SEq_4), SEq_1 * SEq_1 + SEq_2 * SEq_2 - SEq_3 * SEq_3 - SEq_4 * SEq_4) * 57.2957795f);
}

float invSqrt(float x)
{
	float halfx = 0.5f * x;
	float y = x;
	long i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (1.5f - (halfx * y * y));
	return y;
}
