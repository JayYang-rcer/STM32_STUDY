#include "FSM.h"


/****************************************************************************************************************************
2023/10/11 Author：Yang JianYi
该文件用于编写stm32端的状态机，Robot_Control_Mode函数为切换ros和32手柄控制的开关，开关为航模遥控的SWA按键(上拨为32控制，下拨为ros控制)
*****************************************************************************************************************************/

extern unsigned char USART_Receiver1;
// extern uint8_t test;

/**
 * @brief 机器人控制模式函数，手动or自动
*/
void Robot_Control_Mode(void)
{
    static uint8_t flag = 1;
    static uint8_t ros_start_init = 1;
    
    if (SWA >= 900 && SWA <= 1100)  //待机状态
    {
        Robot_stop();   //暂停机器人的所有机构执行
        debug_safe_printf("robot stop\r\n");
    }
    else  
    { //启动状态
        debug_safe_printf("robot start\r\n");
        if(SWB<=1600 && SWB>=1400)                      //stm32控制机器人，能接受到ros数据，但不会把控制机构的接口交给ros
        {
            ros_start_init = 1;

            ROBOT_CHASSI.SPEED_LIMI.Vw_MAX = 3.0f;
            ROBOT_CHASSI.SPEED_LIMI.Vx_MAX = 3.0f;
            ROBOT_CHASSI.SPEED_LIMI.Vy_MAX = 3.0f;

            ROBOT_CHASSI.Chassis_Controller_Flag = 1;   //开启手柄的底盘控制
            debug_safe_printf("hand\r\n");
            if(flag == 1)
                HAL_UART_AbortReceive(&huart1);//huart:串口指针。中止串口中断方式或者DMA方式的接收数据
                // test = 0;
            flag = 0;

            if(SWD>=1900 && SWD<=2100)  
            {
                //开启低移速模式
                ROBOT_CHASSI.SPEED_LIMI.Vw_MAX = 1.0f;
                ROBOT_CHASSI.SPEED_LIMI.Vx_MAX = 0.5f;
                ROBOT_CHASSI.SPEED_LIMI.Vy_MAX = 0.5f;
                if(SWC>=950 && SWC<=1050)
                {
                    ball_filter(TAKE_BALL);     //取球
                    debug_safe_printf("TAKE_BALL\r\n");
                }
                else if(SWC>=1450 && SWC<=1550)
                {
                    // ball_filter(THROW_BALL);
                    ball_filter(CONTROLLER_OFF);
                    u8_speed_set(0,U8_PWM_PIN6);
                    u8_speed_set(0,U8_PWM_PIN8);
                    debug_safe_printf("THROW_BALL\r\n");
                }
                else
                {
                    ball_filter(SAVE_BALL);
                    debug_safe_printf("SAVE_BALL\r\n");
                }
            }
            else
            {
                //关闭机器人的低移速模式
                ROBOT_CHASSI.SPEED_LIMI.Vw_MAX = 4.0f;
                ROBOT_CHASSI.SPEED_LIMI.Vx_MAX = 4.0f;
                ROBOT_CHASSI.SPEED_LIMI.Vy_MAX = 3.0f;
				if(SWC>=1950 && SWC<=2025)
				{
					ball_filter(CONTROLLER_OFF);    //暂停上层机构的执行 
					u8_speed_set(0,U8_PWM_PIN6);
					u8_speed_set(0,U8_PWM_PIN8);
				}
				else if(SWC>=950 && SWC<= 1050)
				{
					ball_filter(CONTROLLER_OFF);    //暂停上层机构的执行 
					u8_speed_set(0,U8_PWM_PIN6);
					u8_speed_set(0,U8_PWM_PIN8); 
				}
				else
				{
					ball_filter(CONTROLLER_OFF);     //暂停上层机构的执行 
					u8_speed_set(0,U8_PWM_PIN6);
					u8_speed_set(0,U8_PWM_PIN8);
				}
            }
        }
        else if(SWB<=2100 && SWB>=1900)      //ros控制，控制的接口交给ros处理
        {
            if(ros_start_init == 1)
            {
                Robot_stop();   //暂停机器人的所有机构执行

                ros_start_init = 0;
            }
            
            if(flag == 0)
            {
                // HAL_UART_MspInit(&huart1);
                
                HAL_UART_Receive_IT(&huart1, &USART_Receiver1, 1); //开启ros接收串口中断
            }
            
            debug_safe_printf("atuo\r\n");
            flag = 1;
        }
        else
        {
            Robot_stop();   //暂停机器人的所有机构执行
        }
    }
}


/**
 * @brief 关闭手柄的底盘控制，锁死底盘电机
*/
void Robot_stop(void)
{
    ROBOT_CHASSI.Chassis_Controller_Flag = 0;   
    ball_filter(CONTROLLER_OFF);
    u8_speed_set(0,U8_PWM_PIN6);
    u8_speed_set(0,U8_PWM_PIN8);
}
