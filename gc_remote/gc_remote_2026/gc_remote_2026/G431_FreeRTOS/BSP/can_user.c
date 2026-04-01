#include "stm32g4xx_hal.h"
#include "can_user.h"
#include "fdcan.h"
#include "stdio.h"

void FDCAN_Filter_Init(void)
{
    FDCAN_FilterTypeDef sFilterConfig;

    // 配置滤波器
    sFilterConfig.IdType = FDCAN_STANDARD_ID;             // 标准ID
    sFilterConfig.FilterIndex = 0;                        // 滤波器编号
    sFilterConfig.FilterType = FDCAN_FILTER_MASK;         // 掩码模式
    sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0; // 过滤后存入FIFO1
    sFilterConfig.FilterID1 = 0x000;                      // 需要匹配的ID
    sFilterConfig.FilterID2 = 0x000;                      // 掩码（0表示不过滤，接收所有ID）

    if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK)
    {
        printf("FDCAN Filter Config Failed!\r\n");
    }

    // 启动FDCAN
    if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) 
    {
        printf("FDCAN Start Failed!\r\n");
    }

    // 开启FIFO1新消息中断
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
        MotorFeedback_t feedback;

        // 帧ID
        feedback.mst_id = rxHeader.Identifier;

        // 数据位解析
        feedback.dev_id = rxData[0] & 0x0F;           // 低4位：ID
        feedback.err    = (rxData[0] >> 4) & 0x0F;    // 高4位：ERR

        feedback.pos = ((uint16_t)rxData[1] << 8) | rxData[2];   // POS[15:0]
        feedback.vel = ((uint16_t)rxData[3] << 4) | (rxData[4] >> 4);  // VEL[11:0]
        feedback.torque = (((uint16_t)(rxData[4] & 0x0F)) << 8) | rxData[5]; // T[11:0]

        feedback.t_mos   = rxData[6];
        feedback.t_rotor = rxData[7];

        // 打印调试
//        printf("MST_ID:0x%X DevID:%d Err:%d POS:%d VEL:%d T:%d MOS:%d Rotor:%d\r\n",
//                feedback.mst_id,
//                feedback.dev_id,
//                feedback.err,
//                feedback.pos,
//                feedback.vel,
//                feedback.torque,
//                feedback.t_mos,
//                feedback.t_rotor);

        // TODO: 你可以把 feedback 存到全局数组里，比如按 dev_id 存
    }
    else
    {
        printf("FDCAN RX Error\r\n");
    }
}


//↓达妙电机//↓达妙电机//↓达妙电机//↓达妙电机//↓达妙电机
//↓达妙电机//↓达妙电机//↓达妙电机//↓达妙电机//↓达妙电机
//↓达妙电机//↓达妙电机//↓达妙电机//↓达妙电机//↓达妙电机
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
//↑达妙电机//↑达妙电机//↑达妙电机//↑达妙电机//↑达妙电机
//↑达妙电机//↑达妙电机//↑达妙电机//↑达妙电机//↑达妙电机
//↑达妙电机//↑达妙电机//↑达妙电机//↑达妙电机//↑达妙电机
