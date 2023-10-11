#ifndef __FILTER_H
#define __FILTER_H

float Kalman_Filter_x(float newAngle,float newGyro);
float Complementary_Filter_x(float angle_m, float gyro_m);

#endif
