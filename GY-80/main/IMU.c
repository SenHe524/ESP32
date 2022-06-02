#include "IMU.h"
#include <math.h>
#include "stdio.h"
#include "stdlib.h"
#include <stdbool.h>
/*************************互补滤波代码宏定义****************************************/
// #define Kp 1.0f // proportional gain governs rate of convergence to accelerometer/magnetometer
// #define Ki 0.01f // integral gain governs rate of convergence of gyroscope biases
float invSqrt(float x);
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f; // quaternion of sensor frame relative to auxiliary frame
float q0_temp, q1_temp, q2_temp, q3_temp;
// static float exInt = 0, eyInt = 0, ezInt = 0;

//不知名代码——AHRSupdate 宏定义
// AHRS algorithm update
static float exInt = 0, eyInt = 0, ezInt = 0;
static float FRE = 66.0f;
static float halfT;
static float k10 = 0.0f, k11 = 0.0f, k12 = 0.0f, k13 = 0.0f;
static float k20 = 0.0f, k21 = 0.0f, k22 = 0.0f, k23 = 0.0f;
static float k30 = 0.0f, k31 = 0.0f, k32 = 0.0f, k33 = 0.0f;
static float k40 = 0.0f, k41 = 0.0f, k42 = 0.0f, k43 = 0.0f;
#define Kp 2.0f // proportional gain governs rate of convergence toaccelerometer/magnetometer
// Kp比例增益 决定了加速度计和磁力计的收敛速度
#define Ki 0.005f // integral gain governs rate of convergenceof gyroscope biases
// Ki积分增益 决定了陀螺仪偏差的收敛速度

/*******************************互补滤波*******************************/
void IMU_AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *Q_angle)
{
	float norm;
	float hx, hy, hz, bx, bz;
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	halfT = 0.5f *(1.0f / FRE);
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
	float r31;
	//把加计的三维向量转成单位向量。
	norm = invSqrt(ax * ax + ay * ay + az * az);
	ax = ax * norm;
	ay = ay * norm;
	az = az * norm;

	norm = invSqrt(mx * mx + my * my + mz * mz);
	mx = mx * norm;
	my = my * norm;
	mz = mz * norm;
	// compute reference direction of flux
	/*计算地理坐标系下的磁场矢量b-xyz（参考值）。因为地理地磁水平夹角，我们已知是0度（抛去磁偏角的因素，固定向北），
	我定义b-y指向正北，所以by=某值，bx=0但地理参考地磁矢量在垂直面上也有分量bz，地球上每个地方都是不一样的。
	我们无法得知，也就无法用来融合（有更适合做垂直方向修正融合的加速度计），所以直接从测量值hz上复制过来，bz=hz。
	磁场水平分量，参考值和测量值的大小应该是一致的(bx*bx) + (by*by)) = ((hx*hx) + (hy*hy))。
	因为bx=0，所以就简化成(by*by)  = ((hx*hx) + (hy*hy))。可算出by。下面可以修改b-y和b-x那个轴指向正北。
	*/
	hx = 2.0f * mx * (0.5f - q2q2 - q3q3) + 2.0f * my * (q1q2 - q0q3) + 2.0f * mz * (q1q3 + q0q2);
	hy = 2.0f * mx * (q1q2 + q0q3) + 2.0f * my * (0.5f - q1q1 - q3q3) + 2.0f * mz * (q2q3 - q0q1);
	hz = 2.0f * mx * (q1q3 - q0q2) + 2.0f * my * (q2q3 + q0q1) + 2.0f * mz * (0.5f - q1q1 - q2q2);
	bx = sqrt((hx * hx) + (hy * hy)); //实质为误差函数，bx越接近hx，则估计姿态与电子罗盘测得姿态越接近
	// by = sqrtf((hx*hx) + (hy*hy));
	bz = hz;

	/*
	这是把四元数换算成《方向余弦矩阵》中的第三列的三个元素。
	根据余弦矩阵和欧拉角的定义，地理坐标系的重力向量，转到机体坐标系，正好是这三个元素。
	所以这里的vx\y\z，其实就是当前的欧拉角（即四元数）的机体坐标参照系上，换算出来的重力单位向量。
	*/
	// estimated direction of gravity and flux (v and w)
	// vx, vy, vz为将 标准单位重力 转换到飞行器参考系后 各个坐标轴上的分量（重力加速度）
	vx = 2.0f * (q1q3 - q0q2);
	vy = 2.0f * (q0q1 + q2q3);
	vz = q0q0 - q1q1 - q2q2 + q3q3;
	// //wx, wy, wz为将 bx与bz又重新 转换到飞行器参考系后 各个坐标轴上的分量。（地磁场）
	wx = 2.0f * bx * (0.5f - q2q2 - q3q3) + 2.0f * bz * (q1q3 - q0q2);
	wy = 2.0f * bx * (q1q2 - q0q3) + 2.0f * bz * (q0q1 + q2q3);
	wz = 2.0f * bx * (q0q2 + q1q3) + 2.0f * bz * (0.5f - q1q1 - q2q2);
	// error is sum of cross product between reference direction of fields and direction measured by sensors
	ex = (ay * vz - az * vy) + (my * wz - mz * wy);
	ey = (az * vx - ax * vz) + (mz * wx - mx * wz);
	ez = (ax * vy - ay * vx) + (mx * wy - my * wx);
	/*
	axyz是机体坐标参照系上，加速度计测出来的重力向量，也就是实际测出来的重力向量。
	axyz是测量得到的重力向量，vxyz是陀螺积分后的姿态来推算出的重力向量，它们都是机体坐标参照系上的重力向量。
	那它们之间的误差向量，就是陀螺积分后的姿态和加计测出来的姿态之间的误差。
	向量间的误差，可以用向量叉积（也叫向量外积、叉乘）来表示，exyz就是两个重力向量的叉积。
	这个叉积向量仍旧是位于机体坐标系上的，而陀螺积分误差也是在机体坐标系，而且叉积的大小与陀螺积分误差成正比，正好拿来纠正陀螺。（你可以自己拿东西想象一下）由于陀螺是对机体直接积分，所以对陀螺的纠正量会直接体现在对机体坐标系的纠正。
	*/

	exInt = exInt + ex * Ki * halfT * 2;
	eyInt = eyInt + ey * Ki * halfT * 2;
	ezInt = ezInt + ez * Ki * halfT * 2;
	// adjusted gyroscope measurements
	// PI调节陀螺仪数据
	gx = gx + Kp * ex + exInt;
	gy = gy + Kp * ey + eyInt;
	gz = gz + Kp * ez + ezInt;

	// 四元数微分方程
	q0_temp = q0 + (-q1 * gx - q2 * gy - q3 * gz) * halfT;
	q1_temp = q1 + (q0 * gx + q2 * gz - q3 * gy) * halfT;
	q2_temp = q2 + (q0 * gy - q1 * gz + q3 * gx) * halfT;
	q3_temp = q3 + (q0 * gz + q1 * gy - q2 * gx) * halfT;
	// 四元数规范化
	norm = invSqrt(q0_temp * q0_temp + q1_temp * q1_temp + q2_temp * q2_temp + q3_temp * q3_temp);
	q0 = q0_temp * norm;
	q1 = q1_temp * norm;
	q2 = q2_temp * norm;
	q3 = q3_temp * norm;
	q0q0 = q0 * q0;
	q0q1 = q0 * q1;
	q0q2 = q0 * q2;
	q0q3 = q0 * q3;
	q1q1 = q1 * q1;
	q1q2 = q1 * q2;
	q1q3 = q1 * q3;
	q2q2 = q2 * q2;
	q2q3 = q2 * q3;
	q3q3 = q3 * q3;
	//使用Z-Y-X顺规
	r31 = 2.0f * (q1q3 - q0q2); 
	if(r31 < 1){
		if(20 > -1){
			// X - φ
			Q_angle[0] = atan2(2.0f * (q2q3 + q0q1), -2.0f * (q1q1 + q2q2) + 1.0f) * 57.2957795f;
			// Y - θ
			Q_angle[1] = asin(-2.0f * q1q3 + 2.0f * q0q2) * 57.2957795f; 
			// Z - ψ
			Q_angle[2] = atan2(2.0f * (q1q2 + q0q3), -2.0f * (q2q2 + q3q3) + 1.0f) * 57.2957795f;
		}else{
			// X - φ
			Q_angle[0] = 0;
			// Y - θ
			Q_angle[1] = 90.00; 
			// Z - ψ
			Q_angle[2] = -atan2(2.0f * (q0q1 - q2q3), -2.0f * (q1q3 + q3q3) + 1.0f) * 57.2957795f;
		}
	}else{
		// X - φ
			Q_angle[0] = 0;
			// Y - θ
			Q_angle[1] = -90.00; 
			// Z - ψ
			Q_angle[2] = -atan2(2.0f * (q0q1 - q2q3), -2.0f * (q1q3 + q3q3) + 1.0f) * 57.2957795f;
	}
}
// Q_angle[0] = -asin(-2.0f * q1 * q3 + 2.0f * q0 * q2) * 57.2957795f;
			// Q_angle[1] = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 57.2957795f;
			// Q_angle[2] = -atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.2957795f;
//*********代码来源：桌面文件-TESTVer.2.13******************//
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz, float *Q_angle)
{
	float norm;				  //用于单位化
	float hx, hy, hz, bx, bz; //
	float vx, vy, vz, wx, wy, wz;
	float ex, ey, ez;
	halfT = 0.5f *(1.0f / FRE);
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


	exInt = exInt + ex * Ki * halfT;
	eyInt = eyInt + ey * Ki * halfT;
	ezInt = ezInt + ez * Ki * halfT;
	// adjusted gyroscope measurements
	// PI调节陀螺仪数据
	gx = gx + Kp * ex + exInt;
	gy = gy + Kp * ey + eyInt;
	gz = gz + Kp * ez + ezInt;

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

	q0_temp = q0 + 2 * halfT / 6.0 * (k10 + 2 * k20 + 2 * k30 + k40);
	q1_temp = q1 + 2 * halfT / 6.0 * (k11 + 2 * k21 + 2 * k31 + k41);
	q2_temp = q2 + 2 * halfT / 6.0 * (k12 + 2 * k22 + 2 * k32 + k42);
	q3_temp = q3 + 2 * halfT / 6.0 * (k13 + 2 * k23 + 2 * k33 + k43);

	// normalise quaternion
	norm = invSqrt(q0_temp * q0_temp + q1_temp * q1_temp + q2_temp * q2_temp + q3_temp * q3_temp);
	q0 = q0_temp * norm;
	q1 = q1_temp * norm;
	q2 = q2_temp * norm;
	q3 = q3_temp * norm;
	Q_angle[0] = -asin(-2.0f * q1 * q3 + 2.0f * q0 * q2) * 57.2957795f;
	Q_angle[1] = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 57.2957795f;
	Q_angle[2] = -atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.2957795f;
	// Q_angle[0] = asin(-2.0f * q1 * q3 + 2.0f * q0 * q2) * 57.2957795f;
	// Q_angle[1] = atan2(2.0f * q2 * q3 + 2.0f * q0 * q1, -2.0f * q1 * q1 - 2.0f * q2 * q2 + 1.0f) * 57.2957795f;
	// Q_angle[2] = atan2(2.0f * (q1 * q2 + q0 * q3), q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3) * 57.2957795f;
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
