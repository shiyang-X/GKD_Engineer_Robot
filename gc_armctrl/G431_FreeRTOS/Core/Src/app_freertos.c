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
#include "adc_filter.h"
#include "can_user.h"
#include "arm_ctrl.h"
#include <string.h>
#include <math.h>

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
/* Definitions for Serial_Task */
osThreadId_t Serial_TaskHandle;
const osThreadAttr_t Serial_Task_attributes = {
  .name = "Serial_Task",
  .priority = (osPriority_t) osPriorityNormal4,
  .stack_size = 256 * 4
};
/* Definitions for MotorCTRL_Task */
osThreadId_t MotorCTRL_TaskHandle;
const osThreadAttr_t MotorCTRL_Task_attributes = {
  .name = "MotorCTRL_Task",
  .priority = (osPriority_t) osPriorityNormal5,
  .stack_size = 256 * 4
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void Start_BlueLED_Task(void *argument);
void Start_Serial_Task(void *argument);
void StartMotorCTRL_Task(void *argument);

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

  /* creation of Serial_Task */
  Serial_TaskHandle = osThreadNew(Start_Serial_Task, NULL, &Serial_Task_attributes);

  /* creation of MotorCTRL_Task */
  MotorCTRL_TaskHandle = osThreadNew(StartMotorCTRL_Task, NULL, &MotorCTRL_Task_attributes);

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
	
	uint8_t id;
	static uint16_t motor_error_count[ARM_MOTOR_ID_MAX + 1U] = {0};
  /* Infinite loop */
  for(;;)
  {
	  for (id = ARM_MOTOR_ID_MIN; id <= ARM_MOTOR_ID_MAX; id++)
	  {
		  uint8_t has_error = arm_motor_has_error(id);

		  if (has_error)
		  {
			  if (motor_error_count[id] < 100)
			  {
				  motor_error_count[id]++;
			  }

			  if (motor_error_count[id] >= 10U)
			  {
				  arm_motor_clear_error(id);
					FDCAN_DM_cmd_STATE(id, 1);
				  motor_error_count[id]++;
			  }
		  }
		  else
		  {
			  motor_error_count[id] = 0U;
		  }
	  }

	  HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_6);
    osDelay(250);
  }
	osDelay(5);
		
  /* USER CODE END Start_BlueLED_Task */
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
	extern UART_HandleTypeDef huart3;   
	
	static uint8_t rx_buffer[30];     // 数据缓存，1个帧头，24个角度数据帧（6*4），5个按键控制帧

	
  /* Infinite loop */
  for(;;)
    {
			if(HAL_UART_Receive(&huart3,rx_buffer,sizeof(rx_buffer),20)==HAL_OK){
					if(rx_buffer[0]==0xBB){
								memcpy((void*)&angle_j1, &rx_buffer[1], 4);
                memcpy((void*)&angle_j2, &rx_buffer[5], 4);
                memcpy((void*)&angle_j3, &rx_buffer[9], 4);
                memcpy((void*)&angle_j4, &rx_buffer[13], 4);
                memcpy((void*)&angle_j5, &rx_buffer[17], 4);
                memcpy((void*)&angle_j6, &rx_buffer[21], 4);
          
								clamp_state = rx_buffer[25];
								ctrl_state = rx_buffer[26];
								ctrl_mode = rx_buffer[27];
								get_kuang_num = rx_buffer[28]; 
								catch_state = rx_buffer[29];

			   } 
     }
			osDelay(2);
   }    
  /* USER CODE END Start_Serial_Task */
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
		static uint8_t last_ctrl_state = disable_ctrl_state; 
		static uint8_t last_ctrl_mode = follow_ctrl_mode;
		static uint8_t hold_target_valid = 0;
		static uint8_t get_state = 0;
	

		static float hold_angle_j1 = 0.0f;
		static float hold_angle_j2 = 0.0f;
		static float hold_angle_j3 = 0.0f;
		static float hold_angle_j4 = 0.0f;
		static float hold_angle_j5 = 0.0f;
    static float hold_angle_j6 = 0.0f;
	
	
		static float auto_first_begin_j2 = -1.37f;
		static float auto_first_begin_j3 = -1.70f;
		static float auto_first_end_j2 = -0.80f;
    static float auto_first_end_j3 = -1.15f;
	
		static uint8_t adjust_kuang = 0;
		
		FDCAN_Filter_Init();
    osDelay(800);
		
		
		/* Infinite loop */
		for(;;)
    {
        // Enable and Disable Ctrl------------------------------------
        if (ctrl_state != last_ctrl_state)
        {
            if (ctrl_state == normal_ctrl_state)
            {
							for(int n=0; n <2; n++)  // send enable twice
							{
									for(int i = 1; i <= 7; i++) 
									{
											FDCAN_DM_cmd_STATE(i, 1);
											osDelay(5);
									}
							}
            }
            else if (ctrl_state == disable_ctrl_state)
            {
							for(int n=0; n <2; n++)  // send disenable twice
							{
                for(int i = 1; i <= 7; i++) {
                    FDCAN_DM_cmd_STATE(i, 0); 
                    osDelay(5);
                }
							}
            }
            last_ctrl_state = ctrl_state;
        }

				// 夹爪控制---------------------------------------------------------
						if (clamp_state == 0x01){
							FDCAN_DM_MIT_cmd_arm(0x07, -2.3f,0, 8,0.8f, 0);
							osDelay(1);
						}	else{
							FDCAN_DM_MIT_cmd_arm(0x07, 0.0, 0,8,0.8f, 0);
							osDelay(1);
						}
				
				
        // Arm ctrl Ros mode -----------------------------------------------------
			
					if(ctrl_mode == follow_ctrl_mode) // follow mode = 0
					{   
            hold_target_valid = 0;
            if(angle_j1 != 0)
							{
								FDCAN_DM_MIT_cmd_arm(0x01, angle_j1 / 57.3f, 0,55,1.2f, 0);
								osDelay(1);
								FDCAN_DM_MIT_cmd_arm(0x02, (motor_pos_limit(2, angle_j2)) / 57.3f, 0,40,1.2f, 0);
								osDelay(1);
                FDCAN_DM_POS_cmd_arm(0x03, (motor_pos_limit(3, angle_j3)) / 57.3f, 1.5);
                osDelay(1);
                FDCAN_DM_POS_cmd_arm(0x04, angle_j4 / 57.3f, 1.5);
                osDelay(1);
                FDCAN_DM_POS_cmd_arm(0x05, angle_j5 / 57.3f, 1.5);
                osDelay(1);
                FDCAN_DM_POS_cmd_arm(0x06, angle_j6 / 57.3f, 3);
                osDelay(1);
							}
						}
					else if(ctrl_mode == hold_ctrl_mode){   // hold mode = 1
						// 记录角度
            if((last_ctrl_mode != hold_ctrl_mode) && (angle_j1 != 0))
            {
                hold_angle_j1 = angle_j1;
                hold_angle_j2 = motor_pos_limit(2, angle_j2);
                hold_angle_j3 = motor_pos_limit(3, angle_j3);
                hold_angle_j4 = angle_j4;
                hold_angle_j5 = angle_j5;
								hold_angle_j6 = angle_j6;
								hold_target_valid = 1;
								adjust_kuang = 0;
						}
						
							if(get_kuang_num == 0x00 )  // 保持
            {
								FDCAN_DM_MIT_cmd_arm(0x01, hold_angle_j1 / 57.3f, 0,55,1.2f, 0);
								osDelay(1);
								FDCAN_DM_MIT_cmd_arm(0x02,hold_angle_j2/ 57.3f, 0,40,1.2f, 0);
								osDelay(1);
						
                FDCAN_DM_POS_cmd_arm(0x03, hold_angle_j3 / 57.3f, 2);
                osDelay(1);
                FDCAN_DM_POS_cmd_arm(0x04, hold_angle_j4 / 57.3f, 2);
                osDelay(1);
                FDCAN_DM_POS_cmd_arm(0x05, hold_angle_j5 / 57.3f, 2);
                osDelay(1);
								FDCAN_DM_POS_cmd_arm(0x06, hold_angle_j6 / 57.3f, 2);
                osDelay(1);
            }
						else if(get_kuang_num == 0x01)  //取一号矿
						{
							if(catch_state == 0x00)
							{		
									auto_first_begin_j2 = -1.37f;
									auto_first_begin_j3 = -1.70f;

									FDCAN_DM_MIT_cmd_arm(0x01, 0.0f, 0, 55, 1.2f, 0);
									osDelay(1);
									FDCAN_DM_MIT_cmd_arm(0x02, auto_first_begin_j2, 0, 40, 1.2f, 0.5);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x03, auto_first_begin_j3, 2);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x04, 0.0f, 2);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x05, 0.0f, 2);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x06, 0.0f, 2);
									osDelay(1);
									}
									else if(catch_state == 0x01)
									{
											osDelay(800);
										auto_first_begin_j2 = angle_change_smooth(auto_first_begin_j2,auto_first_end_j2,0.05f);
										auto_first_begin_j3 = angle_change_smooth(auto_first_begin_j3,auto_first_end_j3,0.05f);
										
										FDCAN_DM_MIT_cmd_arm(0x01, 0.0f, 0, 55, 1.2f, 0);
										osDelay(1);
										FDCAN_DM_MIT_cmd_arm(0x02, auto_first_begin_j2, 0, 40, 1.2f, 0);
										osDelay(1);
										FDCAN_DM_POS_cmd_arm(0x03, auto_first_begin_j3, 4);
										osDelay(1);
										FDCAN_DM_POS_cmd_arm(0x04, 0.0f, 2);
										osDelay(1);
										FDCAN_DM_POS_cmd_arm(0x05, 0.0f, 2);
										osDelay(1);
										FDCAN_DM_POS_cmd_arm(0x06, 0.0f, 2);
										osDelay(1);
									}
							}
						else if(get_kuang_num == 0x11)  //兑换一号矿-全固定
						{
									FDCAN_DM_MIT_cmd_arm(0x01, 0.0f, 0, 55, 1.2f, 0);
									osDelay(1);
									FDCAN_DM_MIT_cmd_arm(0x02, -1.23f, 0, 40, 1.2f, 0.5);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x03, -1.1f, 2);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x04, 2.0f, 2);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x05, 0.0f, 2);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x06, 0.0f, 2);
									osDelay(1);
						}else if(get_kuang_num == 0x66) // 兑换1号矿，两自由度可调
							{
									FDCAN_DM_MIT_cmd_arm(0x01, 0.0f, 0, 55, 1.2f, 0);
									osDelay(1);
									FDCAN_DM_MIT_cmd_arm(0x02, (motor_pos_limit(2, angle_j2)) / 57.3f, 0, 40, 1.2f, 0.5);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x03, -1.1f, 2);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x04, (motor_pos_limit(44, angle_j4)) / 57.3f , 2);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x05, 0.0f, 2);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x06, 0.0f, 2);
									osDelay(1);
							}
						else if(get_kuang_num == 0x02)  //夹爪调整
								{
									FDCAN_DM_MIT_cmd_arm(0x01, 0.0f, 0, 55, 1.2f, 0);
									osDelay(1);
									FDCAN_DM_MIT_cmd_arm(0x02, -1.37, 0, 40, 1.2f, 0.5);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x03, -1.70, 2);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x04, 0.6f, 2);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x05, 0.0f, 2);
									osDelay(1);
									FDCAN_DM_POS_cmd_arm(0x06, 0.0f, 2);
									osDelay(1);
									if(adjust_kuang == 0){
									  osDelay(2000);
										FDCAN_DM_MIT_cmd_arm(0x07, -1.53, 0,10,0.8f, 0);
										osDelay(2000);
										FDCAN_DM_MIT_cmd_arm(0x07, -2.3, 0,10,0.8f, 0);
										osDelay(1);
										adjust_kuang =  1;
									}
								}
							 }
					
					last_ctrl_mode = ctrl_mode;
					osDelay(5); // 任务控制频率		
    }
		
		
      
  /* USER CODE END StartMotorCTRL_Task */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

