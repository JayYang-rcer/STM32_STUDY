#include "rm_motor.h"


/**
 * @brief  电机数据读取
 * @param  can数据接收结构体
 * @param  can数据接收数组
 * @return none
*/
void get_motor_measure(CAN_RxHeaderTypeDef *msg, uint8_t Data[8])                                    
{                                                                                         
    switch(msg -> StdId)  // 检测标准ID
	{
		case CHASSIS_M3508_M1_ID:	//底盘3508电机1
		{ 
			MOTO_REAL_INFO[0].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
			MOTO_REAL_INFO[0].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
			MOTO_REAL_INFO[0].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
			break;
		}
		
		case CHASSIS_M3508_M2_ID:	//底盘3508电机2
		{ 
			MOTO_REAL_INFO[1].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
			MOTO_REAL_INFO[1].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
			MOTO_REAL_INFO[1].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
			break;
		}
		
		case CHASSIS_M3508_M3_ID: 	//底盘3508电机3
		{ 
			MOTO_REAL_INFO[2].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
			MOTO_REAL_INFO[2].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速 
			MOTO_REAL_INFO[2].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
			break;
		}
		
		case CHASSIS_M3508_M4_ID:	//底盘3508电机4
		{ 
			MOTO_REAL_INFO[3].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
			MOTO_REAL_INFO[3].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
			MOTO_REAL_INFO[3].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
			break;
		}

		case M3508_M5_ID:
		{ 
			if(MOTO_REAL_INFO[4].Motor_Type == M_6020)
			{
				MOTO_REAL_INFO[4].GM6020.rotor_angle =    ((Data[0] << 8) |Data[1]);
				MOTO_REAL_INFO[4].GM6020.rotor_speed =    ((Data[2] << 8) |Data[3]);
				MOTO_REAL_INFO[4].GM6020.torque_current = ((Data[4] << 8) |Data[5]);
				MOTO_REAL_INFO[4].GM6020.rotor_angle =    Data[6];
			}
			else
			{
				MOTO_REAL_INFO[4].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
				MOTO_REAL_INFO[4].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
				MOTO_REAL_INFO[4].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
			}
			break;
		}

		case M3508_M6_ID:
		{ 
			if(MOTO_REAL_INFO[5].Motor_Type == M_6020)
			{
				MOTO_REAL_INFO[5].GM6020.rotor_angle =    ((Data[0] << 8) |Data[1]);
				MOTO_REAL_INFO[5].GM6020.rotor_speed =    ((Data[2] << 8) |Data[3]);
				MOTO_REAL_INFO[5].GM6020.torque_current = ((Data[4] << 8) |Data[5]);
				MOTO_REAL_INFO[5].GM6020.rotor_angle =    Data[6];
			}
			else
			{
				MOTO_REAL_INFO[5].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
				MOTO_REAL_INFO[5].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
				MOTO_REAL_INFO[5].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
			}
			break;
		}

		case M3508_M7_ID:
		{ 
			if(MOTO_REAL_INFO[6].Motor_Type == M_6020)
			{
				MOTO_REAL_INFO[6].GM6020.rotor_angle =    ((Data[0] << 8) |Data[1]);
				MOTO_REAL_INFO[6].GM6020.rotor_speed =    ((Data[2] << 8) |Data[3]);
				MOTO_REAL_INFO[6].GM6020.torque_current = ((Data[4] << 8) |Data[5]);
				MOTO_REAL_INFO[6].GM6020.rotor_angle =    Data[6];
			}
			else
			{
				MOTO_REAL_INFO[6].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  // 转子机械角度
				MOTO_REAL_INFO[6].RPM     = (uint16_t)((Data[2] << 8) |  Data[3]);  // 实际转子转速
				MOTO_REAL_INFO[6].CURRENT = (uint16_t)((Data[4] << 8) |  Data[5]);  // 实际转矩电流
			}
			break;
		}

		case BEYOND_M2006_M1_ID://顶部的2006
		{
			if(MOTO_REAL_INFO[7].Motor_Type == M_6020)
			{
				MOTO_REAL_INFO[7].GM6020.rotor_angle =    ((Data[0] << 8) |Data[1]);
				MOTO_REAL_INFO[7].GM6020.rotor_speed =    ((Data[2] << 8) |Data[3]);
				MOTO_REAL_INFO[7].GM6020.torque_current = ((Data[4] << 8) |Data[5]);
				MOTO_REAL_INFO[7].GM6020.rotor_angle =    Data[6];
			}
			else
			{
				MOTO_REAL_INFO[7].ANGLE   = (uint16_t)((Data[0] << 8) |  Data[1]);  //转子机械角度
				MOTO_REAL_INFO[7].RPM	  = (uint16_t)((Data[2] << 8) |	 Data[3]);  //实际转子转速
				MOTO_REAL_INFO[7].CURRENT = (uint16_t)((Data[4] << 8) |	 Data[5]);  //实际转矩电流
			}
			break;
		}
		
		default: break;
	}            
}



/**
 * @brief 发送电机数据,使用can总线控制大疆电机。3508和2006的can id设置是一样的，而且一条can指令最多可以控制8个电机（标识符0x200: 201 202 203 204, 标识符0x1FF: 205 206 207 208）
 * @brief GM6020则是一条指令只能控制4个电机(标识符为0x1FF: 205 206 207 208， 标识符0x2FF：209 20A 20B)   
 * @brief 考虑到一条CAN能够控制的电机数量是有限的。所以该函数控制可以控制8个电机（前四个可以控制2006或者3508，后四个可以控制2006或3508或6020）
 * @param NULL
 * @return NULL
*/
void M3508_Send_Currents(void)
{
    CAN_TxHeaderTypeDef			TxHeader1;
	uint8_t TxData1[8] = {0};
	uint32_t Send_Mail_Box1;
	
	//配置控制端
	TxHeader1.IDE = CAN_ID_STD;
	TxHeader1.RTR = CAN_RTR_DATA;
	TxHeader1.DLC = 0x08;
	
	//配置仲裁段和数据段
	TxHeader1.StdId = CAN_CHASSIS_ALL_ID;//0x200
	
	TxData1[0] = (uint8_t)(MOTO_REAL_INFO[0].TARGET_CURRENT >> 8);	//0x201
	TxData1[1] = (uint8_t) MOTO_REAL_INFO[0].TARGET_CURRENT;

	TxData1[2] = (uint8_t)(MOTO_REAL_INFO[1].TARGET_CURRENT >> 8);	//0x202
	TxData1[3] = (uint8_t) MOTO_REAL_INFO[1].TARGET_CURRENT;

	TxData1[4] = (uint8_t)(MOTO_REAL_INFO[2].TARGET_CURRENT >> 8);	//0x203
	TxData1[5] = (uint8_t) MOTO_REAL_INFO[2].TARGET_CURRENT;

	TxData1[6] = (uint8_t)(MOTO_REAL_INFO[3].TARGET_CURRENT >> 8);	//0x204
	TxData1[7] = (uint8_t) MOTO_REAL_INFO[3].TARGET_CURRENT;

	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &Send_Mail_Box1)!=HAL_OK)
	{
		Error_Handler();
	}


	CAN_TxHeaderTypeDef TxHeader2;
	uint8_t TxData2[8] = {0};
	uint32_t Send_Mail_Box2;
	
	// 配置控制段
	TxHeader2.IDE = CAN_ID_STD;//报文的11位标准标识符CAN_ID_STD表示本报文是标准帧
	TxHeader2.RTR = CAN_RTR_DATA;//报文类型标志RTR位CAN_ID_STD表示本报文的数据帧
	TxHeader2.DLC = 0x08;//数据段长度
	TxHeader2.TransmitGlobalTime = DISABLE;
	
	// 配置仲裁段和数据段
	TxHeader2.StdId = 0x1FF;  // 用于ID为 5 6 7 8 的电机(不论是3508还是2006)
	if(MOTO_REAL_INFO[4].Motor_Type == M_6020)
	{
		TxData2[0] = (uint8_t)(MOTO_REAL_INFO[4].GM6020.set_voltage >> 8);
		TxData2[1] = (uint8_t) MOTO_REAL_INFO[4].GM6020.set_voltage;	      //ID 5 
	}
	else
	{
		TxData2[0] = (uint8_t)(MOTO_REAL_INFO[4].TARGET_CURRENT >> 8);
		TxData2[1] = (uint8_t) MOTO_REAL_INFO[4].TARGET_CURRENT;	       
	}

	if(MOTO_REAL_INFO[4].Motor_Type == M_6020)
	{
		TxData2[2] = (uint8_t)(MOTO_REAL_INFO[5].GM6020.set_voltage >> 8);
		TxData2[3] = (uint8_t) MOTO_REAL_INFO[5].GM6020.set_voltage;	      //ID 6
	}
	else
	{
		TxData2[2] = (uint8_t)(MOTO_REAL_INFO[5].TARGET_CURRENT >> 8);
		TxData2[3] = (uint8_t) MOTO_REAL_INFO[5].TARGET_CURRENT;	       
	}

	if(MOTO_REAL_INFO[4].Motor_Type == M_6020)
	{
		TxData2[4] = (uint8_t)(MOTO_REAL_INFO[6].GM6020.set_voltage >> 8);
		TxData2[5] = (uint8_t) MOTO_REAL_INFO[6].GM6020.set_voltage;	      //ID 7
	}
	else
	{
		TxData2[4] = (uint8_t)(MOTO_REAL_INFO[6].TARGET_CURRENT >> 8);
		TxData2[5] = (uint8_t) MOTO_REAL_INFO[6].TARGET_CURRENT;	      
	}

	if(MOTO_REAL_INFO[4].Motor_Type == M_6020)
	{
		TxData2[6] = (uint8_t)(MOTO_REAL_INFO[7].GM6020.set_voltage >> 8);
		TxData2[7] = (uint8_t) MOTO_REAL_INFO[7].GM6020.set_voltage;	      //ID 8
	}
	else
	{
		TxData2[6] = (uint8_t)(MOTO_REAL_INFO[7].TARGET_CURRENT >> 8);
		TxData2[7] = (uint8_t) MOTO_REAL_INFO[7].TARGET_CURRENT;	      
	}
	
	
	if(HAL_CAN_AddTxMessage(&hcan1, &TxHeader2, TxData2, &Send_Mail_Box2)!=HAL_OK)
	{
		Error_Handler();
	}

}


/**
 * @brief M3508角度积分
 * @param 电机结构体
 * @return NULL
*/
void RM_MOTOR_Angle_Integral(MOTOR_REAL_INFO* RM_MOTOR)
{
	static float Delta_Pos = 0;
	float Deceleration_P = 0;

	//记录第一次进入时的数据
	if(!RM_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG)
	{
		RM_MOTOR->LAST_ANGLE = RM_MOTOR->ANGLE;
		RM_MOTOR->FIRST_ANGLE_INTEGRAL_FLAG = 1;
		return;
	}

	switch (RM_MOTOR->Motor_Type)
	{
		case M_3508:
			Deceleration_P = 19.0f;
			break;

		case M_2006:
			Deceleration_P = 36.0f;
			break;
		
		default:
			break;
	}	
	
	//计算角度变化
	if (RM_MOTOR->RPM >= 0)
	{
		/* code */
		if(RM_MOTOR->ANGLE < RM_MOTOR->LAST_ANGLE)
		{
			if (ABS(8191 + RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE < 1250))
			{
				/* code */
				Delta_Pos = ((float)(8191 + RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE)/8191.0f) * 360.0f;
				Delta_Pos = Delta_Pos / Deceleration_P;	//减速比
			}
		}
		else
		{
			Delta_Pos = ((float)(RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			Delta_Pos = Delta_Pos / Deceleration_P;	//减速比
		}

		//平通滤波
		if(Delta_Pos > 0)
		{
			RM_MOTOR->REAL_ANGLE += Delta_Pos;  // 积分	
		}
	}
	else
	{
		if(RM_MOTOR->ANGLE > RM_MOTOR->LAST_ANGLE)
		{
			if(ABS(8191 - RM_MOTOR->ANGLE + RM_MOTOR->LAST_ANGLE) < 1250)  // 利用两次CAN接收时间电机最大转动角度进行滤波			
			{
				Delta_Pos = ((float)(8191 - RM_MOTOR->ANGLE + RM_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
				Delta_Pos = Delta_Pos / Deceleration_P;	//减速比
			}
		}	
		else
		{
			Delta_Pos = ((float)(RM_MOTOR->ANGLE - RM_MOTOR->LAST_ANGLE) / 8191.0f) * 360.0f;
			Delta_Pos = Delta_Pos / Deceleration_P;	//减速比
		}
		
		// 滤波
		if(Delta_Pos < 0) 
			RM_MOTOR->REAL_ANGLE += Delta_Pos;  // 积分
	}

	// 存储角度值 
	RM_MOTOR->LAST_ANGLE = RM_MOTOR->ANGLE;
}


