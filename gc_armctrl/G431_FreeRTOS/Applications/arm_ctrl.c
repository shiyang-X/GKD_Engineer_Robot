#include "stm32g4xx_hal.h"
#include "arm_ctrl.h"
#include "can_user.h"
#include "fdcan.h"
#include "stdio.h"

#define ARM_DM_MIT_MODE      0x000U
#define ARM_DM_POS_MODE      0x100U
#define ARM_DM_CLEAR_ERR_CMD 0xFBU

static volatile MotorFeedback_t g_arm_motor_feedback[ARM_MOTOR_ID_MAX + 1U];
static volatile uint8_t g_arm_motor_feedback_valid[ARM_MOTOR_ID_MAX + 1U];

static uint8_t arm_motor_id_is_valid(uint16_t id)
{
    return (id >= ARM_MOTOR_ID_MIN) && (id <= ARM_MOTOR_ID_MAX);
}

static uint16_t arm_motor_mode_base(uint16_t id)
{
    // This project drives joints 1/2/7 in MIT mode and 3-6 in position mode.
    switch (id)
    {
        case 1:
        case 2:
        case 7:
            return ARM_DM_MIT_MODE;

        case 3:
        case 4:
        case 5:
        case 6:
            return ARM_DM_POS_MODE;

        default:
            return ARM_DM_POS_MODE;
    }
}

static uint8_t arm_motor_is_abnormal_state(uint8_t err_state)
{
    switch (err_state)
    {
        case 0x03:
        case 0x04:
        case 0x05:
        case 0x08:
        case 0x09:
        case 0x0A:
        case 0x0B:
        case 0x0C:
        case 0x0D:
        case 0x0E:
            return 1U;

        default:
            return 0U;
    }
}



static HAL_StatusTypeDef arm_motor_send_special_cmd(uint16_t id, uint8_t cmd)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, cmd};

    if (!arm_motor_id_is_valid(id))
    {
        return HAL_ERROR;
    }

    tx_header.Identifier = arm_motor_mode_base(id) + id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = FDCAN_DLC_BYTES_8;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header.MessageMarker = 0;

    return HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &tx_header, tx_data);
}


volatile float angle_j1 = 0,angle_j2 = 0,angle_j3 = 0,angle_j4 = 0,angle_j5 = 0,angle_j6 = 0,clamp_j = 0;

volatile uint8_t ctrl_state = disable_ctrl_state; // 默认失能
//uint8_t ctrl_state = normal_ctrl_state;
volatile uint8_t ctrl_mode = follow_ctrl_mode;
//uint8_t ctrl_mode = hold_ctrl_mode;
volatile uint8_t clamp_state = 0;                               
volatile uint8_t get_kuang_num = 0;
volatile uint8_t catch_state = 0;

//  报错清理函数
void arm_motor_feedback_update(uint16_t mst_id, const uint8_t *rx_data)
{
    uint8_t dev_id;
    volatile MotorFeedback_t *feedback;

    if (rx_data == NULL)
    {
        return;
    }

    dev_id = rx_data[0] & 0x0F;
    if (!arm_motor_id_is_valid(dev_id))
    {
        return;
    }

    feedback = &g_arm_motor_feedback[dev_id];
    feedback->mst_id = mst_id;
    feedback->dev_id = dev_id;
    feedback->err = (rx_data[0] >> 4) & 0x0F;
    feedback->pos = (int16_t)(((uint16_t)rx_data[1] << 8) | rx_data[2]);
    feedback->vel = (int16_t)(((uint16_t)rx_data[3] << 4) | (rx_data[4] >> 4));
    feedback->torque = (int16_t)((((uint16_t)(rx_data[4] & 0x0F)) << 8) | rx_data[5]);
    feedback->t_mos = rx_data[6];
    feedback->t_rotor = rx_data[7];

    g_arm_motor_feedback_valid[dev_id] = 1U;
}

uint8_t arm_motor_feedback_valid(uint16_t id)
{
    if (!arm_motor_id_is_valid(id))
    {
        return 0U;
    }

    return g_arm_motor_feedback_valid[id];
}

uint8_t arm_motor_get_error_state(uint16_t id)
{
    if (!arm_motor_feedback_valid(id))
    {
        return ARM_MOTOR_ERROR_UNKNOWN;
    }

    return g_arm_motor_feedback[id].err;
}

uint8_t arm_motor_has_error(uint16_t id)
{
    uint8_t err_state = arm_motor_get_error_state(id);

     return (err_state != ARM_MOTOR_ERROR_UNKNOWN) && arm_motor_is_abnormal_state(err_state);
}

const volatile MotorFeedback_t *arm_motor_get_feedback(uint16_t id)
{
    if (!arm_motor_feedback_valid(id))
    {
        return NULL;
    }

    return &g_arm_motor_feedback[id];
}

HAL_StatusTypeDef arm_motor_clear_error(uint16_t id)
{
    HAL_StatusTypeDef status = arm_motor_send_special_cmd(id, ARM_DM_CLEAR_ERR_CMD);

    if (status != HAL_OK)
    {
        printf("Clear motor %u error failed\r\n", id);
    }

    return status;
}

uint8_t arm_motor_clear_all_errors(void)
{
    uint8_t cleared_count = 0U;
    uint16_t id;

    for (id = ARM_MOTOR_ID_MIN; id <= ARM_MOTOR_ID_MAX; id++)
    {
        if (arm_motor_has_error(id) && (arm_motor_clear_error(id) == HAL_OK))
        {
            cleared_count++;
        }
    }

    return cleared_count;
}

// 角度限位函数
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
				}else if(angle_jx > 90.0)
				{
					angle_jx = 90;
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
			
//			case 4:
//				if(angle_jx < )
//				{
//				
//				}else if(angle_jx > )
//				{
//				
//				}
//				break;
//			
//			case 5:
//				if(angle_jx < )
//				{
//				
//				}else if(angle_jx > )
//				{
//				
//				}
//				break;
//			
//			case 6:
//				if(angle_jx < )
//				{
//				
//				}else if(angle_jx > )
//				{
//				
//				}
//				break;

//			case 22:
//				if(angle_jx < )
//				{
//					angle_jx =  ;
//				}else if(angle_jx >  )
//				{
//					angle_jx =  ;
//				}
//				break;

			case 44:
				if(angle_jx < 10.0f )
				{
						angle_jx = 10.0f;
				}else if(angle_jx > 145.0)
				{
						angle_jx = 145.0;
				}
				break;


		}
		return angle_jx;
}


// 角度平滑函数
float angle_change_smooth(float begin_angle,float end_angle,float step){
    
		if(step < 0) step = -step;
    
		if(begin_angle < end_angle)
    {
        begin_angle += step;
        if(begin_angle > end_angle)
            begin_angle = end_angle;
    }
    else if(begin_angle > end_angle)
    {
        begin_angle -= step;
        if(begin_angle < end_angle)
            begin_angle = end_angle;
    }
		
		return begin_angle;
	}
