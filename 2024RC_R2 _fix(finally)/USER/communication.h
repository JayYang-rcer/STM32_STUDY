#ifndef __COMMUNICATION_H
#define __COMMUNICATION_H

#define START 0x11

enum FLAG{
	CHECK_HEADER_FLAG, 
	RECEIVE_DATA_FLAG, 
	CHECK_VALUE_FLAG, 
	FINAL_RECEIVE_FLAG
};


unsigned char serial_get_crc8_value(unsigned char *tem_array, unsigned char len);
void USART_Send_String(unsigned char *p, short sendSize, USART_TypeDef *usart);
void Usart_Send_Data(short RPM_1, short RPM_2, short RPM3, short RPM4, float RealAngle1, float RealAngle12, float RealAngle13, float RealAngle14, char ctrlFlag);
int STM32_READ_FROM_ROS_FLOAT(float *target_chassis_x, float *target_chassis_y, float *target_chassis_w, unsigned char *data3);
void Usart_Send_Data_Test(float RealAngle_2, float RealAngle_3, float RealAngle_4, char ctrlFlag);


#endif
