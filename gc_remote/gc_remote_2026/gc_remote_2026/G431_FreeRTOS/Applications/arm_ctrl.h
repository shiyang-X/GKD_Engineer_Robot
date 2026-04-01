#ifndef __ARM_CTRL__H
#define __ARM_CTRL__H
#include "main.h"

extern float angle_j1,angle_j2,angle_j3,angle_j4,angle_j5,angle_j6;
extern uint8_t ctrl_state;


#define normal_ctrl_state 1
#define disable_ctrl_state 0


float motor_pos_limit(uint16_t id, float angle_jx);

uint8_t put_first_kuang_test2();
uint8_t enable_state_test();
uint8_t hold_state_test();
uint8_t get_first_test();
uint8_t get_second_test();
uint8_t put_first_kuang_test();
uint8_t catch_out_test();
#endif
