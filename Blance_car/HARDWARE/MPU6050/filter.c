#include "filter.h"

float dt=0.005;		  //每5ms进行一次滤波     

float Complementary_Filter_x(float angle_m, float gyro_m)
{
	static float angle;
	float K1 =0.05; 
	angle = K1 * angle_m+ (1-K1) * (angle + gyro_m * dt);
	return angle;
}


float Kalman_Filter_x(float newAngle,float newGyro)		
{
	static float angle;
	static float Q_angle=0.001;//陀螺仪噪声协方差
	static float Q_gyro=0.003;//陀螺仪飘移噪声协方差
	static float R_angle=0.5;//角度测量噪声协方差
	static float Q_bias;
	static float P[2][2]={{1,0},{0,1}};
	static float measure,angle_err;
	static float K_0, K_1;
	
	//由先验估计方程X(K|K-1)=AX(K-1|K-1)+BU(K)+W(K)预测当前角度值
	angle = angle - Q_bias*dt + newGyro*dt;
	
	//预测协方差矩阵
	//由先验估计有系统参数A= 1  -dt
	//						0   1
	P[0][0] = P[0][0] + (Q_angle - P[0][1] - P[1][0])*dt + P[1][1]*dt*dt;
	P[0][1] = P[0][1] - P[1][1]*dt;
	P[1][0] = P[1][0] - P[1][1]*dt;
	P[1][1] = P[1][1] + Q_gyro;
	
	//建立测量方程,测量噪声已经被包含其中
	measure = newAngle;
	
	//计算卡尔曼增益
	K_0 = (P[0][0])/(P[0][0] + R_angle);
	K_1 = (P[1][0])/(P[0][0] + R_angle);
	
	//计算最优估计值,后验估计
	angle_err = measure - angle;
	angle = angle + K_0*angle_err;
	Q_bias = Q_bias + K_1*angle_err;
	
	//后验估计误差协方差
	P[0][0] = P[0][0] - K_0*(P[0][0]);
	P[0][1] = P[0][1] - K_0*(P[0][1]);
    P[1][0] = P[1][0] - K_1*(P[0][0]);
    P[1][1] = P[1][1] - K_1*(P[0][1]);
	
	return angle;
}
	


