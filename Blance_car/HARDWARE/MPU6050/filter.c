#include "filter.h"

float dt=0.005;		  //ÿ5ms����һ���˲�     

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
	static float Q_angle=0.001;//����������Э����
	static float Q_gyro=0.003;//������Ʈ������Э����
	static float R_angle=0.5;//�ǶȲ�������Э����
	static float Q_bias;
	static float P[2][2]={{1,0},{0,1}};
	static float measure,angle_err;
	static float K_0, K_1;
	
	//��������Ʒ���X(K|K-1)=AX(K-1|K-1)+BU(K)+W(K)Ԥ�⵱ǰ�Ƕ�ֵ
	angle = angle - Q_bias*dt + newGyro*dt;
	
	//Ԥ��Э�������
	//�����������ϵͳ����A= 1  -dt
	//						0   1
	P[0][0] = P[0][0] + (Q_angle - P[0][1] - P[1][0])*dt + P[1][1]*dt*dt;
	P[0][1] = P[0][1] - P[1][1]*dt;
	P[1][0] = P[1][0] - P[1][1]*dt;
	P[1][1] = P[1][1] + Q_gyro;
	
	//������������,���������Ѿ�����������
	measure = newAngle;
	
	//���㿨��������
	K_0 = (P[0][0])/(P[0][0] + R_angle);
	K_1 = (P[1][0])/(P[0][0] + R_angle);
	
	//�������Ź���ֵ,�������
	angle_err = measure - angle;
	angle = angle + K_0*angle_err;
	Q_bias = Q_bias + K_1*angle_err;
	
	//����������Э����
	P[0][0] = P[0][0] - K_0*(P[0][0]);
	P[0][1] = P[0][1] - K_0*(P[0][1]);
    P[1][0] = P[1][0] - K_1*(P[0][0]);
    P[1][1] = P[1][1] - K_1*(P[0][1]);
	
	return angle;
}
	


