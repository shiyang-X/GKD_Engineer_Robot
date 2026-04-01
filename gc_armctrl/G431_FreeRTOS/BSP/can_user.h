#ifndef __CAN_INITS__H
#define __CAN_INITS__H

#include "main.h"

typedef struct	//电机回报数据结构体
{
    uint16_t mst_id;   // 回报ID
    uint8_t  err;      // 错误码
    uint8_t  dev_id;   // 电机ID
    int16_t  pos;      // 位置
    int16_t  vel;      // 速度
    int16_t  torque;   // 扭矩
    uint8_t  t_mos;    // MOS温度
    uint8_t  t_rotor;  // 电机温度
} MotorFeedback_t;


void FDCAN_Filter_Init(void);

void FDCAN_DM_cmd_STATE(uint16_t id,uint8_t state);
void FDCAN_DM_cmd_SETZERO(uint16_t id);

void FDCAN_DM_POS_cmd_arm(uint16_t id, float _pos, float _vel);
void FDCAN_DM_Velocity_cmd_arm(uint16_t id, float _vel);

int float_to_uint(float x, float x_min, float x_max, int bits);
void FDCAN_DM_MIT_cmd_arm(uint16_t id, float pos, float vel,float kp,float kd, float torq);
 


#endif
