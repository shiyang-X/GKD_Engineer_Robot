#include "adc_filter.h"
#include "adc_user.h"

// 滑动均值更新函数
void ADC_Filter_Update(void)
{
    for (int ch = 0; ch < 3; ch++)
    {
        // --- ADC1 ---
        adc1_sum[ch] -= adc1_hist[ch][adc_index];   // 减去旧值
        adc1_hist[ch][adc_index] = ADC1_buffer[ch]; // 存入新值
        adc1_sum[ch] += adc1_hist[ch][adc_index];   // 加上新值

        // --- ADC2 ---
        adc2_sum[ch] -= adc2_hist[ch][adc_index];
        adc2_hist[ch][adc_index] = ADC2_buffer[ch];
        adc2_sum[ch] += adc2_hist[ch][adc_index];
    }

    // 更新下标（循环使用）
    adc_index++;
    if (adc_index >= FILTER_LEN)
        adc_index = 0;
}

// 获取平滑后的结果
uint16_t ADC1_GetFiltered(uint8_t ch)
{
    return (uint16_t)(adc1_sum[ch] / FILTER_LEN);
}

uint16_t ADC2_GetFiltered(uint8_t ch)
{
    return (uint16_t)(adc2_sum[ch] / FILTER_LEN);
}



/**
  * @brief  将ADC滤波后的数值转换成角度
  * @param  adc_val:  滤波后的ADC值 (0~4095)
  * @param  max_adc:  ADC最大值 (一般=4095)
  * @param  max_angle: 对应的最大角度 (例如 300度)
  * @retval 转换后的角度值 (float)
  */
float ADC_To_Angle(uint16_t adc_val, float max_angle)
{
    const float mid = 4095.0f / 2.0f;

    return ((float)adc_val - mid) / 4095.0f * max_angle;
}

