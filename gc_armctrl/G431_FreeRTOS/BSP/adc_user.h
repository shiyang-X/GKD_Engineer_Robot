#ifndef __ADC_USER_H__
#define __ADC_USER_H__
#include "stdio.h"
#include "stdint.h"

extern uint16_t ADC1_buffer[3];
extern uint16_t ADC2_buffer[3];

void adc1_user_init();
void adc2_user_init();

#endif
