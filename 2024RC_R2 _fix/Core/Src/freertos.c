/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define DEBUG_PRINTF_ENABLE 0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for ROS_32_Serial */
osThreadId_t ROS_32_SerialHandle;
const osThreadAttr_t ROS_32_Serial_attributes = {
  .name = "ROS_32_Serial",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for Motor_Control */
osThreadId_t Motor_ControlHandle;
const osThreadAttr_t Motor_Control_attributes = {
  .name = "Motor_Control",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for U8_and_VESC */
osThreadId_t U8_and_VESCHandle;
const osThreadAttr_t U8_and_VESC_attributes = {
  .name = "U8_and_VESC",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for safe_printf */
osMutexId_t safe_printfHandle;
const osMutexAttr_t safe_printf_attributes = {
  .name = "safe_printf"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
void debug_safe_printf(const char *format, ...)
{
#if DEBUG_PRINTF_ENABLE
  
  HAL_UART_MspInit(&huart5);

  osStatus xReturn;
  va_list args;
  va_start(args,format);

  xReturn = osMutexAcquire(safe_printfHandle,portMAX_DELAY);
  if(xReturn == osOK)
  {
    vprintf(format, args);
  }
  xReturn = osMutexRelease(safe_printfHandle);

#else
  HAL_UART_MspDeInit(&huart5);//为了用uart5进行串口数据传输的调试，不用时关�?
  (void)0;
#endif
}
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);
void ROS_32_Serial_Function(void *argument);
void Motor_Control_Function(void *argument);
void U8_and_VESC_Function(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of safe_printf */
  safe_printfHandle = osMutexNew(&safe_printf_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of ROS_32_Serial */
  ROS_32_SerialHandle = osThreadNew(ROS_32_Serial_Function, NULL, &ROS_32_Serial_attributes);

  /* creation of Motor_Control */
  Motor_ControlHandle = osThreadNew(Motor_Control_Function, NULL, &Motor_Control_attributes);

  /* creation of U8_and_VESC */
  U8_and_VESCHandle = osThreadNew(U8_and_VESC_Function, NULL, &U8_and_VESC_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */

    extern uint8_t test;

  /* Infinite loop */
  for(;;)
  {
    
    // comm_can_set_current(101,3); 
    /* Infinite loop */
    
    for(;;)
    {
      // VESC_MOTO_INFO[0].MOTOR_MODE = CURRENT;
      // VESC_MOTO_INFO[0].TARGET_CURRENT = 1;
      // VESC_Control(&VESC_MOTO_INFO[0]);
      
      // comm_can_set_rpm(101,1000);
      //debug_safe_printf("test_id = %d\r\n",test_id);

      debug_safe_printf("LEFT_X = %d\r\n", YaoGan_LEFT_X);
      debug_safe_printf("LEFT_Y = %d\r\n", YaoGan_LEFT_Y);
      debug_safe_printf("RIGHT_X = %d\r\n", YaoGan_RIGHT_X);
      debug_safe_printf("RIGHT_Y = %d\r\n", YaoGan_RIGHT_Y);
      // debug_safe_printf("u8 = %d\r\n",TIM10->CCR1);
      //debug_safe_printf("SWA = %d\r\n", SWA);
      //debug_safe_printf("SWB = %d\r\n", SWB);
      //debug_safe_printf("SWC = %d\r\n", SWC);
      //debug_safe_printf("SWD = %d\r\n", SWD);
      // debug_safe_printf("test = %d",test);
      // Usart_Send_Data_Test(0,0,1.2,1);
      osDelay(50);
    }
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_ROS_32_Serial_Function */
/**
* @brief Function implementing the ROS_32_Serial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_ROS_32_Serial_Function */
void ROS_32_Serial_Function(void *argument)
{
  /* USER CODE BEGIN ROS_32_Serial_Function */
  
  /* Infinite loop */
  for(;;)
  {

    //向ros上位机发送数据，控制位ROS_AUTO_CONTROL�?1时，进行自动模式，为0时为手动
    Usart_Send_Data(MOTO_REAL_INFO[0].RPM,MOTO_REAL_INFO[1].RPM,MOTO_REAL_INFO[2].RPM,MOTO_REAL_INFO[3].RPM,
                    MOTO_REAL_INFO[0].REAL_ANGLE,MOTO_REAL_INFO[1].REAL_ANGLE,MOTO_REAL_INFO[2].REAL_ANGLE,MOTO_REAL_INFO[3].REAL_ANGLE,1);
    osDelay(10);
  }
  /* USER CODE END ROS_32_Serial_Function */
}

/* USER CODE BEGIN Header_Motor_Control_Function */
/**
* @brief Function implementing the Motor_Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Motor_Control_Function */
void Motor_Control_Function(void *argument)
{
  /* USER CODE BEGIN Motor_Control_Function */

  /* Infinite loop */
  for(;;)
  {
    Robot_Wheels_Adjust();  //底盘运动解算
    Motor_Control();        //大疆电机控制
    osDelay(5);

    // Velocity_Planning_setpos(&MOTO_REAL_INFO[0],0,8000,100,2000,10,0.15,0.15);
    // Motor_Control_All(&MOTO_REAL_INFO[0], &VESC_MOTO_INFO[0], VESC_5065, VELOCITY_PLANNING_MODE, 0, 0);
  }
  /* USER CODE END Motor_Control_Function */
}

/* USER CODE BEGIN Header_U8_and_VESC_Function */
/**
* @brief Function implementing the U8_and_VESC thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_U8_and_VESC_Function */
void U8_and_VESC_Function(void *argument)
{
  /* USER CODE BEGIN U8_and_VESC_Function */
  /* Infinite loop */
  for(;;)
  {

    Robot_Control_Mode();
    // if(SWD>=950 && SWD<=1050)
    // {
    //   u8_speed_set(0,U8_PWM_PIN6);
    //   u8_speed_set(0,U8_PWM_PIN8);
    // }

    // if(SWD>=1950 && SWD<=2050)
    // {
    //   u8_speed_set(10,U8_PWM_PIN6);
    //   u8_speed_set(10,U8_PWM_PIN8);
    // }

    // if(SWD>=950 && SWD<=1050)
    // {
    //   TIM10->CCR1 = 2000;
    // }
    // if(SWD>=1950 && SWD<=2050)
    // {
    //   TIM10->CCR1 = 3000;
    // }
    // if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_0) == 0)
    // {
    //   TIM10->CCR1 = 2000;
    //   TIM13->CCR1 = 1000;
    // }
    // if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_1) == 0)
    // {
    //   TIM10->CCR1 = 3000;
    //   TIM13->CCR1 = 1500;
    // }
    // if(HAL_GPIO_ReadPin(GPIOF,GPIO_PIN_2) == 0)
    // {
    //   TIM10->CCR1 = 4000;
    //   TIM13->CCR1 = 2000;
    // }
    osDelay(5);
  }
  /* USER CODE END U8_and_VESC_Function */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

