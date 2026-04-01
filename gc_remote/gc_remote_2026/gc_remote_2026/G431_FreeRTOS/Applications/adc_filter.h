#ifndef __ADC_FILTER_H__
#define __ADC_FILTER_H__

#include "stdint.h"

#define FILTER_LEN 20   // 滤波窗口长度（可改大/小）

// 为每个通道保存历史数据
static uint16_t adc1_hist[3][FILTER_LEN] = {0};
static uint16_t adc2_hist[3][FILTER_LEN] = {0};
static uint32_t adc1_sum[3] = {0};
static uint32_t adc2_sum[3] = {0};
static uint8_t  adc_index = 0;   // 当前写入位置（循环使用）

void ADC_Filter_Update(void);
uint16_t ADC1_GetFiltered(uint8_t ch);
uint16_t ADC2_GetFiltered(uint8_t ch);
float ADC_To_Angle(uint16_t adc_val,float max_angle);


#endif
