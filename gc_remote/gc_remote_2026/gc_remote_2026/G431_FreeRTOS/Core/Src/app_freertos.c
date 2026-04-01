/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : app_freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "adc_user.h"
#include "can_user.h"
#include "uart_user.h"

#include "adc_filter.h"
#include "arm_ctrl.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */


/* USER CODE END Variables */
/* Definitions for BlueLED_Task */
osThreadId_t BlueLED_TaskHandle;
const osThreadAttr_t BlueLED_Task_attributes = {
  .name = "BlueLED_Task",
  .priority = (osPriority_t) osPriorityLow4,
  .stack_size = 128 * 4
};
/* Definitions for GetADC_Task */
osThreadId_t GetADC_TaskHandle;
const osThreadAttr_t GetADC_Task_attributes = {
  .name = "GetADC_Task",
  .priority = (osPriority_t) osPriorityNormal4,
  .stack_size = 256 * 4
};
/* Definitions for MotorCTRL_Task */
osThreadId_t MotorCTRL_TaskHandle;
const osThreadAttr_t MotorCTRL_Task_attributes = {
  .name = "MotorCTRL_Task",
  .priority = (osPriority_t) osPriorityNormal6,
  .stack_size = 256 * 4
};
/* Definitions for Serial_Task */
osThreadId_t Serial_TaskHandle;
const osThreadAttr_t Serial_Task_attributes = {
  .name = "Serial_Task",
  .priority = (osPriority_t) osPriorityNormal5,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_BlueLED_Task(void *argument);
void Start_GetADC_Task(void *argument);
void StartMotorCTRL_Task(void *argument);
void Start_Serial_Task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

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
  /* creation of BlueLED_Task */
  BlueLED_TaskHandle = osThreadNew(Start_BlueLED_Task, NULL, &BlueLED_Task_attributes);

  /* creation of GetADC_Task */
  GetADC_TaskHandle = osThreadNew(Start_GetADC_Task, NULL, &GetADC_Task_attributes);

  /* creation of MotorCTRL_Task */
  MotorCTRL_TaskHandle = osThreadNew(StartMotorCTRL_Task, NULL, &MotorCTRL_Task_attributes);

  /* creation of Serial_Task */
  Serial_TaskHandle = osThreadNew(Start_Serial_Task, NULL, &Serial_Task_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_Start_BlueLED_Task */
/**
  * @brief  Function implementing the BlueLED_Task thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_Start_BlueLED_Task */
void Start_BlueLED_Task(void *argument)
{
  /* USER CODE BEGIN Start_BlueLED_Task */
  /* Infinite loop */
  for(;;)
  {
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
    osDelay(250);//一秒闪两次
  }
		
  /* USER CODE END Start_BlueLED_Task */
}

/* USER CODE BEGIN Header_Start_GetADC_Task */
/**
* @brief Function implementing the GetADC_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_GetADC_Task */
void Start_GetADC_Task(void *argument)
{
  /* USER CODE BEGIN Start_GetADC_Task */

	//TickType_t last_wake_time = xTaskGetTickCount();	
	
	adc1_user_init();
	adc2_user_init();

  /* Infinite loop */
  for(;;)
  {
		ADC_Filter_Update();
		angle_j1 = ADC_To_Angle(ADC1_GetFiltered(0),360) + 7.428572;
		angle_j2 = ADC_To_Angle(ADC1_GetFiltered(1),360) + 30.417583;
		angle_j3 = ADC_To_Angle(ADC1_GetFiltered(2),360) - 0.842491;
	
		
		angle_j4 = ADC_To_Angle(ADC2_GetFiltered(0),360) + 4.000000;
		angle_j5 = ADC_To_Angle(ADC2_GetFiltered(1),360) + 4.80000;
		angle_j6 = ADC_To_Angle(ADC2_GetFiltered(2),360) + 2.769231;
	
		//printf("Angle_Jx = %f  %f  %f  %f  %f  %f\r\n",arm_data[0], arm_data[1], arm_data[2], arm_data[3], arm_data[4], arm_data[5]);	
    //printf("Angle_Jx = %f  %f  %f  %f  %f  %f\r\n",angle_j1, angle_j2, angle_j3, angle_j4, angle_j5, angle_j6);
	  
    osDelay(2);
	  
  }
  /* USER CODE END Start_GetADC_Task */
}

/* USER CODE BEGIN Header_StartMotorCTRL_Task */
/**
* @brief Function implementing the MotorCTRL_Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMotorCTRL_Task */
void StartMotorCTRL_Task(void *argument)
{
  /* USER CODE BEGIN StartMotorCTRL_Task */
	
	
	FDCAN_Filter_Init();
	osDelay(800);
	
static	uint8_t put_first_kuang_state2 = 0;
static	uint8_t arm_enable_state = 0;
static  uint8_t arm_hold_state = 0;
static  uint8_t first_get_state = 0;
static  uint8_t second_get_state = 0;
static  uint8_t put_first_kuang_state = 0;
static  uint8_t catch_out_state = 0;
	//	调试模式
	if(ctrl_state == disable_ctrl_state) 
	{
		FDCAN_DM_cmd_SETZERO(0x06);
		printf("motor SETZERO ok\r\n");	
	}
	
	// 正常工作使能
	if(ctrl_state == normal_ctrl_state)
	{
		FDCAN_DM_cmd_STATE(1,1);
		osDelay(5);
		FDCAN_DM_cmd_STATE(2,1);
		osDelay(5);
		FDCAN_DM_cmd_STATE(3,1);
		osDelay(5);
		
		FDCAN_DM_cmd_STATE(4,1);
		osDelay(5);
		FDCAN_DM_cmd_STATE(5,1);
		osDelay(5);
		FDCAN_DM_cmd_STATE(6,1);
		osDelay(5);
		FDCAN_DM_cmd_STATE(7,1);
		osDelay(5);
		
	}		
	
		/* Infinite loop */
		for(;;)
		{
			if(ctrl_state == normal_ctrl_state)
			{
				// arm angles for uart
				arm_data[0] = -angle_j1/2;
				arm_data[1] = motor_pos_limit(2, angle_j2);
				arm_data[2] = -motor_pos_limit(3, angle_j3);
				arm_data[3] = -motor_pos_limit(4, angle_j4);
				arm_data[4] = -motor_pos_limit(5, angle_j5);
				arm_data[5] = -motor_pos_limit(6,angle_j6);


				// arm angles for can
//				FDCAN_DM_POS_cmd_arm(0x01,arm_data[0]/57.3,2);
//				osDelay(2);
//				FDCAN_DM_POS_cmd_arm(0x02,arm_data[1]/57.3,2);
//				osDelay(2);
//				FDCAN_DM_POS_cmd_arm(0x03,arm_data[2]/57.3,4);
//				osDelay(2);
//				FDCAN_DM_POS_cmd_arm(0x04,arm_data[3]/57.3,4);
//				osDelay(2);
//				FDCAN_DM_POS_cmd_arm(0x05,arm_data[4]/57.3,4);
//				osDelay(2);
//				FDCAN_DM_POS_cmd_arm(0x06,arm_data[5]/57.3,4);
//				osDelay(2);


				
				// enable state control
				arm_enable_state = enable_state_test();
				
				if(arm_enable_state == 1)
				{
					arm_ctrl[1] = 0x01;  // 电机使能
					osDelay(2);
				}
				else if(arm_enable_state == 0)
				{
					arm_ctrl[1] = 0x00;  // 电机失能
					osDelay(2);
				}
				
				// hold state control
				arm_hold_state = hold_state_test();
				
				if(arm_hold_state == 1)
				{
					arm_ctrl[2] = 0x01;  
					osDelay(2);
				}
				else if(arm_hold_state == 0)
				{
					arm_ctrl[2] = 0x00;  
					osDelay(2);
				}
				
				// three state
				first_get_state  = get_first_test() ? 1 : 0;
				second_get_state = get_second_test() ? 1 : 0;
				put_first_kuang_state  = put_first_kuang_test() ? 1 : 0;
				put_first_kuang_state2 = put_first_kuang_test2() ? 1 : 0;
				
				
				if(first_get_state)
				{
						arm_ctrl[3] = 0x01;
							osDelay(2);
				}
				else if(second_get_state)
				{
						arm_ctrl[3] = 0x02;
							osDelay(2);
				}
				else if(put_first_kuang_state)
				{
						arm_ctrl[3] = 0x11;
							osDelay(2);
				}else if(put_first_kuang_state2)
				{
						arm_ctrl[3] = 0x66;
								osDelay(2);
				}
				else
				{
						arm_ctrl[3] = 0x00;
						osDelay(2);
				}
				
				// catch out test
				catch_out_state = catch_out_test();
				if(catch_out_state == 1)
				{
					arm_ctrl[4] = 0x01;  
					osDelay(2);
				}
				else if(catch_out_state == 0)
				{
					arm_ctrl[4] = 0x00;  
					osDelay(2);
				}
	
			}

			osDelay(2);
		}	

  /* USER CODE END StartMotorCTRL_Task */
}

/* USER CODE BEGIN Header_Start_Serial_Task */
/**
* @brief Function implementing the Serial_Task thread.
* @param argument: Not used
* @retval None
*/ 
/* USER CODE END Header_Start_Serial_Task */
void Start_Serial_Task(void *argument)
{
  /* USER CODE BEGIN Start_Serial_Task */
  /* Infinite loop */
  for(;;)
  {
		uart_transmit_arm_data();
    osDelay(34);
  }
  /* USER CODE END Start_Serial_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

