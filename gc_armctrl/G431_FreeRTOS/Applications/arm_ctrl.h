#ifndef __ARM_CTRL__H
#define __ARM_CTRL__H
#include "main.h"
#include "can_user.h"

extern volatile float angle_j1,angle_j2,angle_j3,angle_j4,angle_j5,angle_j6,clamp_j;
extern volatile uint8_t ctrl_state;
extern volatile	uint8_t clamp_state;
extern volatile uint8_t ctrl_mode;                               
extern volatile uint8_t get_kuang_num;
extern volatile uint8_t catch_state;

#define normal_ctrl_state 1
#define disable_ctrl_state 0
#define hold_ctrl_mode 1
#define follow_ctrl_mode 0

#define ARM_MOTOR_ID_MIN        1U
#define ARM_MOTOR_ID_MAX        7U
#define ARM_MOTOR_ERROR_UNKNOWN 0xFFU


float motor_pos_limit(uint16_t id, float angle_jx);
float angle_change_smooth(float begin_angle,float end_angle,float step);
void arm_motor_feedback_update(uint16_t mst_id, const uint8_t *rx_data);
uint8_t arm_motor_feedback_valid(uint16_t id);
uint8_t arm_motor_get_error_state(uint16_t id);
uint8_t arm_motor_has_error(uint16_t id);
const volatile MotorFeedback_t *arm_motor_get_feedback(uint16_t id);
HAL_StatusTypeDef arm_motor_clear_error(uint16_t id);
uint8_t arm_motor_clear_all_errors(void);

#endif
