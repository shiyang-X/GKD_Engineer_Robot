#include "adc_user.h"
#include "stm32g4xx_hal.h"
#include "adc.h"
#include "cmsis_os.h"

uint16_t ADC1_buffer[3]={0};
uint16_t ADC2_buffer[3]={0};
 
void adc1_user_init()
{
	HAL_ADCEx_Calibration_Start(&hadc1,ADC_SINGLE_ENDED);//ADC自动校准
	osDelay(50);//延时50ms
	HAL_ADC_Start_DMA(&hadc1,(uint32_t*)ADC1_buffer,3);//启动ADC外设的DMA，内部已经默认启动了ADC外设，用户无需启动。
}


void adc2_user_init()
{
	HAL_ADCEx_Calibration_Start(&hadc2,ADC_SINGLE_ENDED);//ADC自动校准
	osDelay(50);//延时50ms
	HAL_ADC_Start_DMA(&hadc2,(uint32_t*)ADC2_buffer,3);//启动ADC外设的DMA，内部已经默认启动了ADC外设，用户无需启动。
}
