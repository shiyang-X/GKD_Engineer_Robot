#ifndef __UART_USER__H
#define __UART_USER__H

#include "stdint.h"


extern const uint8_t CRC8_INIT;
extern const uint8_t CRC8_TAB[256];

extern const uint16_t CRC16_INIT;
extern const uint16_t W_CRC16_TAB[256];

#pragma pack(1)
typedef struct {
    uint8_t  sof;          // 固定 0xA5
	
    uint16_t data_length;  // 30
		uint8_t  seq;          // 包序号
    uint8_t  crc8;         // 帧头校验
} FrameHeader;
#pragma pack()


extern float arm_data[6];
extern uint8_t arm_ctrl[5];


void uart_transmit_arm_data();

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) ;
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength) ;

uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage, uint32_t dwLength);

#endif
