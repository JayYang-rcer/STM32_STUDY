#include "show.h"



void oled_show(void)
{
		OLED_Display_On();  //显示屏打开
		//=============第一行显示小车模式=======================//	
		 OLED_ShowString(0,0,"DMP");
     OLED_ShowString(60,0,"Normal");
		//=============第三行显示编码器1=======================//	
		                      OLED_ShowString(00,20,"EncoLEFT");
		if( Encoder_Left<0)		OLED_ShowString(80,20,"-"),
		                      OLED_ShowNumber(95,20,-Encoder_Left,3,12);
		else                 	OLED_ShowString(80,20,"+"),
		                      OLED_ShowNumber(95,20, Encoder_Left,3,12);
  	//=============第四行显示编码器2=======================//		
		                      OLED_ShowString(00,30,"EncoRIGHT");
		if(Encoder_Right<0)		OLED_ShowString(80,30,"-"),
		                      OLED_ShowNumber(95,30,-Encoder_Right,3,12);
		else               		OLED_ShowString(80,30,"+"),
		                      OLED_ShowNumber(95,30,Encoder_Right,3,12);	
		//=============第六行显示角度与开关=======================//
		                      OLED_ShowString(0,50,"Angle");
		if(Angle_Balance<0)		OLED_ShowString(48,50,"-");
		if(Angle_Balance>=0)	OLED_ShowString(48,50,"+");
													OLED_ShowNumber(53,50,myabs((int)Angle_Balance),3,12);
		if(Flag_Stop)		      OLED_ShowString(90,50,"OFF");
		if(!Flag_Stop)	      OLED_ShowString(90,50,"ON ");
		//=============刷新=======================//
		OLED_Refresh_Gram();	
}

