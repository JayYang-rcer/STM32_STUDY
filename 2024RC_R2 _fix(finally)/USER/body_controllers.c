#include "body_controllers.h"

static int8_t door_satte; //门状态，1为开启，0为关闭

/**
 * @brief 筛球机构控制器，该控制机构采用VESC驱动N5065电机，电调ID从上往下是101，102，103设置（电调ID可以在VESC TOOL中进行设置）
 * @param 球的预期状态
 * @return 机构的当前状态
*/
int16_t ball_filter(BALL_STATUE EXPECT_BALL_STA)
{
	static int stop_ball = 0;
    switch (EXPECT_BALL_STA)
    {
        case SAVE_BALL: //存球，放入桶中
        {
            if(door_satte == 1)
            baffle_control(CLOSE);
			stop_ball = 1;
            VESC_MOTO_INFO[0].MOTOR_MODE = VESC_SPEED;
            VESC_MOTO_INFO[0].TARGET_eRPM = 7500;

            VESC_MOTO_INFO[1].MOTOR_MODE = VESC_SPEED;
            VESC_MOTO_INFO[1].TARGET_eRPM = 7500;
            

            u8_speed_set(10,N5065_PWM_PIN9);
            u8_speed_set(0,U8_PWM_PIN6);
            u8_speed_set(0,U8_PWM_PIN8);
            // VESC_MOTO_INFO[2].MOTOR_MODE = VESC_SPEED;
            // VESC_MOTO_INFO[2].TARGET_eRPM = 10000;
            // VESC_Control(&VESC_MOTO_INFO[2]);
            break;
        }

        case TAKE_BALL: //拿球并停在车里
        {
            if(door_satte == 1)
                baffle_control(CLOSE);
            VESC_MOTO_INFO[0].MOTOR_MODE = VESC_SPEED;
            VESC_MOTO_INFO[0].TARGET_eRPM = 0;
            
            VESC_MOTO_INFO[1].MOTOR_MODE = VESC_SPEED;
            VESC_MOTO_INFO[1].TARGET_eRPM = 0;

            u8_speed_set(0,N5065_PWM_PIN9);
            u8_speed_set(10,U8_PWM_PIN6);
            u8_speed_set(10,U8_PWM_PIN8);
            // VESC_MOTO_INFO[2].MOTOR_MODE = VESC_SPEED;
            // VESC_MOTO_INFO[2].TARGET_eRPM = 1000;   //暂时给1000电转速
            // VESC_Control(&VESC_MOTO_INFO[2]);

            break;
        }

        case THROW_BALL: //筛掉不要的球
        {
            VESC_MOTO_INFO[0].MOTOR_MODE = VESC_SPEED;
            VESC_MOTO_INFO[0].TARGET_eRPM = 0;

            VESC_MOTO_INFO[1].MOTOR_MODE = VESC_SPEED;
            VESC_MOTO_INFO[1].TARGET_eRPM = 0;

            if(door_satte == 0)
                baffle_control(OPEN);

            u8_speed_set(5,N5065_PWM_PIN9);
            u8_speed_set(10,U8_PWM_PIN6);
            u8_speed_set(10,U8_PWM_PIN8);
            // VESC_MOTO_INFO[2].MOTOR_MODE = VESC_SPEED;
            // VESC_MOTO_INFO[2].TARGET_eRPM = 7500;   //简单测试得到的结果，后续还要配合U8进行调试
            // VESC_Control(&VESC_MOTO_INFO[2]);

            break;
        }
		
		case INVERTED:  //电机反转，防止卡球，由于有一个VESC坏了，使用好盈电调临时替代，但效果并不理想
		{
            static uint16_t cnt;
            if(door_satte == 1)
                baffle_control(CLOSE);
			if(stop_ball == 1)
			{
                cnt++;
				VESC_MOTO_INFO[0].MOTOR_MODE = VESC_SPEED;
				VESC_MOTO_INFO[0].TARGET_eRPM = 0;

				VESC_MOTO_INFO[1].MOTOR_MODE = VESC_SPEED;
				VESC_MOTO_INFO[1].TARGET_eRPM = 0;
				u8_speed_set(0,N5065_PWM_PIN9);
				
				if(cnt>=1000)    //由于好盈电调的闭环需要编码器，而且好盈电调无法进行代码控制反转，所以需要Delay
				stop_ball = 0;
			
				VESC_MOTO_INFO[0].MOTOR_MODE = VESC_SPEED;
				VESC_MOTO_INFO[0].TARGET_eRPM = -10000;

				VESC_MOTO_INFO[1].MOTOR_MODE = VESC_SPEED;
				VESC_MOTO_INFO[1].TARGET_eRPM = -10000;
				
			}
                if(cnt>=2500)
                {
                    VESC_MOTO_INFO[0].MOTOR_MODE = VESC_SPEED;
                    VESC_MOTO_INFO[0].TARGET_eRPM = 15000;

                    VESC_MOTO_INFO[1].MOTOR_MODE = VESC_SPEED;
                    VESC_MOTO_INFO[1].TARGET_eRPM = 15000;
                    
                    u8_speed_set(30,N5065_PWM_PIN9);
                }

                cnt = 0;
			break;
		}

        case CONTROLLER_OFF:    //关闭机构控制
        {   
            if(door_satte == 1)
                baffle_control(CLOSE);
            VESC_MOTO_INFO[0].MOTOR_MODE = VESC_SPEED;
            VESC_MOTO_INFO[0].TARGET_eRPM = 0;

            VESC_MOTO_INFO[1].MOTOR_MODE = VESC_SPEED;
            VESC_MOTO_INFO[1].TARGET_eRPM = 0;

            u8_speed_set(0,N5065_PWM_PIN9);
            // VESC_MOTO_INFO[2].MOTOR_MODE = VESC_SPEED;
            // VESC_MOTO_INFO[2].TARGET_eRPM = 0;   
            // VESC_Control(&VESC_MOTO_INFO[2]);
        
            break;
        }
        
        default: break;
    }

    return 0;
}


/**
 * @brief 筛球挡板控制器
 * @param 模式 OPEN or CLOSE
 * @author wnagqi
*/
void baffle_control(DOOR door)
{
    static  int8_t homeing_flag,cnt;
    switch (door)
    {
		case OPEN:                       // the func is used to get the singal 
		{								//when it get the 1, open the door
            door_satte = 1;
			HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_RESET);//气缸关闭
			if(cnt++ >= 10)
            {
			    Position_Control(&MOTO_REAL_INFO[4],120);
                homeing_flag = 0;
                cnt = 0;//利用任务的循环做粗略计时，如果使用精准计时则用TIM计时器
            }
			break;
        }
		
		case CLOSE:
        {        
            door_satte = 0;
			Position_Control(&MOTO_REAL_INFO[4],0);
            if(homeing_flag == 0)
            {
                MOTO_REAL_INFO[4].Motor_Mode = MOTO_OFF;
                MOTO_REAL_INFO[4].ANGLE = 0;//利用机械限位进行校准
                homeing_flag = 1;
            }
			if(cnt++ >= 10)
            {
                HAL_GPIO_WritePin(GPIOA,GPIO_PIN_0,GPIO_PIN_SET);   //气缸开启
                cnt = 0;
            }
			
			//float output = PID_Speed_Calculate(PID_Data * PID  ,float expect,float Encoder_Count);//
			
			break;
        }
		
		default: break;
    }
}
