#include "stm32g4xx_hal.h"
#include "can_user.h"
#include "arm_ctrl.h"
#include "fdcan.h"
#include "stdio.h"

void FDCAN_Filter_Init(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    // �����˲���
    sFilterConfig.IdType = FDCAN_STANDARD_ID;             // ��׼ID
    sFilterConfig.FilterIndex = 0;                        // �˲������
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;         // ����ģʽ
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // ���˺����FIFO1
    sFilterConfig.FilterID1 = 0x000;                      // ��Ҫƥ���ID
    sFilterConfig.FilterID2 = 0x000;                      // ���루0��ʾ�����ˣ���������ID��

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        printf("FDCAN Filter Config Failed!\r\n");
    }

    // ���FDCAN
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) 
    {
        printf("FDCAN Start Failed!\r\n");
    }

    // ����FIFO1����Ϣ�ж�
    if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK)
    {
        printf("FDCAN Notification Failed!\r\n");
    }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    FDCAN_RxHeaderTypeDef rxHeader;
    uint8_t rxData[8];
	
    if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) == HAL_OK)
    {
        arm_motor_feedback_update(rxHeader.Identifier, rxData);
    }
    else
    {
        printf("FDCAN RX Error\r\n");
    }
}


//��������//��������//��������//��������//��������
//��������//��������//��������//��������//��������
//��������//��������//��������//��������//��������
static FDCAN_TxHeaderTypeDef DM_arm_tx_message;
static uint8_t DM_arm_can_send_data[8];

void FDCAN_DM_cmd_STATE(uint16_t id,uint8_t state)
{
    if(state > 0)
        DM_arm_can_send_data[7] = 0xFC;
    else
        DM_arm_can_send_data[7] = 0xFD;

    DM_arm_tx_message.Identifier          = 0x100 + id;
    DM_arm_tx_message.IdType              = FDCAN_STANDARD_ID;
    DM_arm_tx_message.TxFrameType         = FDCAN_DATA_FRAME;
    DM_arm_tx_message.DataLength          = FDCAN_DLC_BYTES_8;
    DM_arm_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    DM_arm_tx_message.BitRateSwitch       = FDCAN_BRS_OFF;
    DM_arm_tx_message.FDFormat            = FDCAN_CLASSIC_CAN;
    DM_arm_tx_message.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    DM_arm_tx_message.MessageMarker       = 0;

    DM_arm_can_send_data[0] = 0xFF;
    DM_arm_can_send_data[1] = 0xFF;
    DM_arm_can_send_data[2] = 0xFF;
    DM_arm_can_send_data[3] = 0xFF;
    DM_arm_can_send_data[4] = 0xFF;
    DM_arm_can_send_data[5] = 0xFF;
    DM_arm_can_send_data[6] = 0xFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &DM_arm_tx_message, DM_arm_can_send_data) != HAL_OK)
    {
        printf("FDCAN Send STATE Failed\r\n");
    }
}

void FDCAN_DM_cmd_SETZERO(uint16_t id)
{
    DM_arm_can_send_data[7] = 0xFE;

    DM_arm_tx_message.Identifier          = 0x100 + id;
    DM_arm_tx_message.IdType              = FDCAN_STANDARD_ID;
    DM_arm_tx_message.TxFrameType         = FDCAN_DATA_FRAME;
    DM_arm_tx_message.DataLength          = FDCAN_DLC_BYTES_8;
    DM_arm_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    DM_arm_tx_message.BitRateSwitch       = FDCAN_BRS_OFF;
    DM_arm_tx_message.FDFormat            = FDCAN_CLASSIC_CAN;
    DM_arm_tx_message.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    DM_arm_tx_message.MessageMarker       = 0;

    DM_arm_can_send_data[0] = 0xFF;
    DM_arm_can_send_data[1] = 0xFF;
    DM_arm_can_send_data[2] = 0xFF;
    DM_arm_can_send_data[3] = 0xFF;
    DM_arm_can_send_data[4] = 0xFF;
    DM_arm_can_send_data[5] = 0xFF;
    DM_arm_can_send_data[6] = 0xFF;

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &DM_arm_tx_message, DM_arm_can_send_data) != HAL_OK)
    {
        printf("FDCAN Send SETZERO Failed\r\n");
    }
}

// 位置速度模式控制
void FDCAN_DM_POS_cmd_arm(uint16_t id, float _pos, float _vel)
{
    uint8_t *pbuf = (uint8_t*)&_pos;
    uint8_t *vbuf = (uint8_t*)&_vel;

    DM_arm_tx_message.Identifier          = 0x100 + id;
    DM_arm_tx_message.IdType              = FDCAN_STANDARD_ID;
    DM_arm_tx_message.TxFrameType         = FDCAN_DATA_FRAME;
    DM_arm_tx_message.DataLength          = FDCAN_DLC_BYTES_8;
    DM_arm_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    DM_arm_tx_message.BitRateSwitch       = FDCAN_BRS_OFF;
    DM_arm_tx_message.FDFormat            = FDCAN_CLASSIC_CAN;
    DM_arm_tx_message.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    DM_arm_tx_message.MessageMarker       = 0;

    DM_arm_can_send_data[0] = pbuf[0];
    DM_arm_can_send_data[1] = pbuf[1];
    DM_arm_can_send_data[2] = pbuf[2];
    DM_arm_can_send_data[3] = pbuf[3];
    DM_arm_can_send_data[4] = vbuf[0];
    DM_arm_can_send_data[5] = vbuf[1];
    DM_arm_can_send_data[6] = vbuf[2];
    DM_arm_can_send_data[7] = vbuf[3];

    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &DM_arm_tx_message, DM_arm_can_send_data) != HAL_OK)
    {
        printf("FDCAN Send YAW Failed\r\n");
    }
}


// 速度控制
void FDCAN_DM_Velocity_cmd_arm(uint16_t id, float _vel)
{
    uint8_t *vbuf = (uint8_t*)&_vel;

    DM_arm_tx_message.Identifier          = 0x200 + id;
    DM_arm_tx_message.IdType              = FDCAN_STANDARD_ID;
    DM_arm_tx_message.TxFrameType         = FDCAN_DATA_FRAME;
    DM_arm_tx_message.DataLength          = FDCAN_DLC_BYTES_8;
    DM_arm_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    DM_arm_tx_message.BitRateSwitch       = FDCAN_BRS_OFF;
    DM_arm_tx_message.FDFormat            = FDCAN_CLASSIC_CAN;
    DM_arm_tx_message.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    DM_arm_tx_message.MessageMarker       = 0;

    DM_arm_can_send_data[0] = vbuf[0];
    DM_arm_can_send_data[1] = vbuf[1];
    DM_arm_can_send_data[2] = vbuf[2];
    DM_arm_can_send_data[3] = vbuf[3];


    if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &DM_arm_tx_message, DM_arm_can_send_data) != HAL_OK)
    {
        printf("FDCAN Send YAW Failed\r\n");
    }
}
/**  
2. ************************************************************************  
3. * @brief:       mit_ctrl: MIT模式下的电机控制函数  
4. * @param[in]:   hcan:           指向CAN_HandleTypeDef结构的指针，用于指定CAN总线  
* @param[in]:   motor_id:   电机ID，指定目标电机  
* @param[in]:   pos:            位置给定值  
* @param[in]:   vel:            速度给定值  
* @param[in]:   kp:             位置比例系数  
* @param[in]:   kd:             位置微分系数  
 * @param[in]:   torq:           转矩给定值  
 * @retval:      void  
 * @details:     通过CAN总线向电机发送MIT模式下的控制帧。  
 ************************************************************************ 
 **/   



int float_to_uint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x - offset) * ((float)((1<<bits)-1)) / span);
}


 void FDCAN_DM_MIT_cmd_arm(uint16_t id, float pos, float vel,float kp,float kd, float torq)   
 {   
	 
    uint16_t pos_tmp,vel_tmp,kp_tmp,kd_tmp,tor_tmp;    
        
	 
	  DM_arm_tx_message.Identifier           = 0x00 + id;   // MIT_MODE=0x00 
    DM_arm_tx_message.IdType              = FDCAN_STANDARD_ID;
    DM_arm_tx_message.TxFrameType         = FDCAN_DATA_FRAME;
    DM_arm_tx_message.DataLength          = FDCAN_DLC_BYTES_8;
    DM_arm_tx_message.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    DM_arm_tx_message.BitRateSwitch       = FDCAN_BRS_OFF;
    DM_arm_tx_message.FDFormat            = FDCAN_CLASSIC_CAN;
    DM_arm_tx_message.TxEventFifoControl  = FDCAN_NO_TX_EVENTS;
    DM_arm_tx_message.MessageMarker       = 0;
	 
	 
     //将浮点数据等比例转换成整数    
	 pos_tmp = float_to_uint(pos,  -18.85,  18.85, 16);// 一定要注意电机参数
     vel_tmp = float_to_uint(vel,  -45.0,  45.0,  12);//  
     kp_tmp  = float_to_uint(kp,   0.0, 500.0, 12);//（0.0~500.0）   
     kd_tmp  = float_to_uint(kd,   0.0, 5.0, 12);//（0.0~5.0）   
     tor_tmp = float_to_uint(torq, -10.0,  10.0,  12);//（-10.0~10.0）   
    
     DM_arm_can_send_data[0] = (pos_tmp >> 8);   
     DM_arm_can_send_data[1] = pos_tmp;   
     DM_arm_can_send_data[2] = (vel_tmp >> 4);   
     DM_arm_can_send_data[3] = ((vel_tmp&0xF)<<4)|(kp_tmp>>8);   
     DM_arm_can_send_data[4] = kp_tmp;   
     DM_arm_can_send_data[5] = (kd_tmp >> 4);   
     DM_arm_can_send_data[6] = ((kd_tmp&0xF)<<4)|(tor_tmp>>8);   
     DM_arm_can_send_data[7] = tor_tmp;   
        
     //通过can总线 发送到电机驱动器   
      if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &DM_arm_tx_message, DM_arm_can_send_data) != HAL_OK)
    {
        printf("FDCAN Send MIT Failed\r\n");
    }  



 }  
