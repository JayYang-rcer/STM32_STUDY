/***********************************************
閿熸枻鎷峰徃閿熸枻鎷烽敓鏂ゆ嫹瓒ｉ敓鐙＄》鎷烽敓鏂ゆ嫹閿熸枻鎷疯帪閿熸枻鎷烽敓鏂ゆ嫹閿熺潾鐧告嫹鍙�
鍝侀敓鐙★綇鎷稺HEELTEC
閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷穡heeltec.net
閿熺殕鎲嬫嫹閿熸枻鎷烽敓鏁欙綇鎷穝hop114407458.taobao.com 
閿熸枻鎷烽敓鏂ゆ嫹閫�: https://minibalance.aliexpress.com/store/4455017
閿熻姤鏈敓鏂ゆ嫹5.7
閿熺潾闈╂嫹鏃堕敓鎴掞細2021-04-29

 
Brand: WHEELTEC
Website: wheeltec.net
Taobao shop: shop114407458.taobao.com 
Aliexpress: https://minibalance.aliexpress.com/store/4455017
Version:5.7
Update閿熸枻鎷�2021-04-29

All rights reserved
***********************************************/
#include "stm32f10x.h"
#include "sys.h"
#include "MPU6050.h"
#include "oled.h"
#include "led.h"

u8 Flag_Stop=1;                 			//閿熸枻鎷烽敓閰碉级鐧告嫹閿熻鐤氫紮鎷烽敓渚ヮ剨鎷风瀻涔囬敓锟�
int Encoder_Left,Encoder_Right;                     						//閿熸枻鎷烽敓鎻唻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓锟�
u8 Flag_Stop;                               					//鍋滄閿熸枻鎷峰織浣嶉敓鏂ゆ嫹 閿熸枻鎷风ず閿熸枻鎷峰織浣� 榛橀敓鏂ゆ嫹鍋滄 閿熸枻鎷风ず閿熸枻鎷�
float Angle_Balance,Gyro_Balance;           		//骞抽敓鏂ゆ嫹閿熸枻鎷烽敓锟� 骞抽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷� 杞敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷�

int main(void)
{ 
	uart_init(115200);
	MY_NVIC_PriorityGroupConfig(2);	//閿熸枻鎷烽敓鏂ゆ嫹閿熷彨鏂嚖鎷烽敓鏂ゆ嫹
	delay_init();	    	            //閿熸枻鎷锋椂閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷峰閿熸枻鎷�	
	JTAG_Set(JTAG_SWD_DISABLE);     //閿熸埅鎲嬫嫹JTAG閿熸帴鍖℃嫹
	JTAG_Set(SWD_ENABLE);           //閿熸枻鎷稴WD閿熸帴鍖℃嫹 閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熺祳WD閿熸帴鍙ｇ鎷烽敓鏂ゆ嫹
	KEY_Init();                     //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷峰閿熸枻鎷�
	TB6612_Init(7199,0);   //閿熸枻鎷峰閿熸枻鎷稰WM 10KHZ閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹閿燂拷 閿熸枻鎷烽敓鏂ゆ嫹閿熺粸纭锋嫹閿熸枻鎷烽敓鏂ゆ嫹閿熸帴鍖℃嫹 閿熸枻鎷蜂负MiniBalance_PWM_Init(9999,35) 200HZ
	Encoder_Init_TIM2();            //閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鎺ュ尅鎷�
	Encoder_Init_TIM4();            //閿熸枻鎷峰閿熸枻鎷烽敓鏂ゆ嫹閿熸枻鎷烽敓鏂ゆ嫹2
	IIC_Init();                     //IIC閿熸枻鎷峰閿熸枻鎷�
	OLED_Init();                    //OLED閿熸枻鎷峰閿熸枻鎷�	    
	MPU6050_initialize();           //MPU6050閿熸枻鎷峰閿熸枻鎷�	
	DMP_Init();                     //閿熸枻鎷峰閿熸枻鎷稤MP 
	LED_Init();
	MiniBalance_EXTI_Init();        //MPU6050 5ms閿熸枻鎷锋椂閿熷彨鏂鎷峰閿熸枻鎷�
	while(1)
	{
		oled_show();
	} 
}

