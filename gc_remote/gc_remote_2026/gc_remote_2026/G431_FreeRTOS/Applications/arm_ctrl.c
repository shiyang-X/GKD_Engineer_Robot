#include "stm32g4xx_hal.h"
#include "arm_ctrl.h"
#include "can_user.h"
#include "fdcan.h"
#include "stdio.h"
#include "cmsis_os.h"

//uint8_t ctrl_state = disable_ctrl_state;
uint8_t ctrl_state = normal_ctrl_state;

float angle_j1 = 0,angle_j2 = 0,angle_j3 = 0,angle_j4 = 0,angle_j5 = 0,angle_j6 = 0;


float motor_pos_limit(uint16_t id, float angle_jx)
{
		switch(id)
		{
//			case 1:
//				if(angle_jx < )
//				{
//				
//				}else if(angle_jx > )
//				{
//				
//				}
//				break;
			
			case 2:
				if(angle_jx < -90.0)
				{
					angle_jx = -90;
				}else if(angle_jx > -47.0)
				{
					angle_jx = -47.0;
				}
				break;
			
			case 3:
				if(angle_jx < -135.0)
				{
					angle_jx = -135.0;
				}else if(angle_jx > 135.0)
				{
					angle_jx = 135.0;
				}
				break;
			
			case 4:
				if(angle_jx < -160.0)
				{
					angle_jx = -160.0;
				}else if(angle_jx > 90.0)
				{
					angle_jx = 90.0;
				}
				break;
			
			case 5:
				if(angle_jx < -60.0)
				{
					// angle_jx = -60.0;
				}else if(angle_jx > 60.0)
				{
				  // angle_jx = 60.0;
				}
				break;
			
			case 6:
				if(angle_jx < -120)
				{
					angle_jx = -120.0;
				}else if(angle_jx > 120)
				{
					angle_jx = 120.0;
				}
				break;
		}
		
		return angle_jx;
}




// *******************************************************************************
// 拨动开关：使能使能检测
// *******************************************************************************
uint8_t enable_state_test(){
	
	uint8_t state;
	uint8_t last_state;
	last_state = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14);
	
	for(uint8_t counter = 0;counter < 6;counter++){
		osDelay(1);
		state = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_14);
		if(state != last_state ){
		  counter = 0;
			last_state = state;
		}
	}
	
	if(state == 1){
		return 1;	
	}else{
		return 0;
	}
}

// *********************************************************************************
// 拨动开关：保持位置设置
// *********************************************************************************
uint8_t hold_state_test(){
	
	uint8_t state;
	uint8_t last_state;
	last_state = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);
	
	for(uint8_t counter = 0;counter < 6;counter++){
		osDelay(1);
		state = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_13);
		if(state != last_state ){
		  counter = 0;
			last_state = state;
		}
	}
	
	if(state == 1){
		return 1;	
	}else{
		return 0;
	}
}
// ******************************************************************
// 拨动开关：1号取矿预备
// ******************************************************************
uint8_t get_first_test(){
	
	uint8_t state;
	uint8_t last_state;
	last_state = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);
	
	for(uint8_t counter = 0;counter < 6;counter++){
		osDelay(1);
		state = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_12);
		if(state != last_state ){
		  counter = 0;
			last_state = state;
		}
	}
	
	if(state == 1){
		return 1;	
	}else{
		return 0;
	}
}


// ******************************************************************
// 拨动开关：取出矿
// ******************************************************************
uint8_t catch_out_test(){
	
	uint8_t state;
	uint8_t last_state;
	last_state = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
	
	for(uint8_t counter = 0;counter < 6;counter++){
		osDelay(1);
		state = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_1);
		if(state != last_state ){
		  counter = 0;
			last_state = state;
		}
	}
	
	if(state == 1){
		return 1;	
	}else{
		return 0;
	}
}


// ******************************************************************
// 拨动开关：放一号矿
// ******************************************************************
uint8_t put_first_kuang_test(){
	
	uint8_t state;
	uint8_t last_state;
	last_state = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
	
	for(uint8_t counter = 0;counter < 6;counter++){
		osDelay(1);
		state = HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_0);
		if(state != last_state ){
		  counter = 0;
			last_state = state;
		}
	}
	
	if(state == 1){
		return 1;	
	}else{
		return 0;
	}
}


// ******************************************************************
// 拨动开关：2号取矿
// ******************************************************************
uint8_t get_second_test(){
	
	uint8_t state;
	uint8_t last_state;
	last_state = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
	
	for(uint8_t counter = 0;counter < 6;counter++){
		osDelay(1);
		state = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_6);
		if(state != last_state ){
		  counter = 0;
			last_state = state;
		}
	}
	
	if(state == 1){
		return 1;	
	}else{
		return 0;
	}
}


// ********************************************************************************
// 拨动开关：第二种放矿策略
// ********************************************************************************
uint8_t put_first_kuang_test2(){
	
	uint8_t state;
	uint8_t last_state;
	last_state = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7);
	
	for(uint8_t counter = 0;counter < 6;counter++){
		osDelay(1);
		state = HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_7);
		if(state != last_state ){
		  counter = 0;
			last_state = state;
		}
	}
	
	if(state == 1){
		return 1;	
	}else{
		return 0;
	}

}



