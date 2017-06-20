#ifndef DRIVER_JUDGE_H
#define DRIVER_JUDGE_H

#include "stm32f4xx.h"

#define JUDGE_BUFFER_LENGTH           256
#define JUDGE_CONTROL_FRAME_LENGTH    ((uint8_t)36)
#define JUDGE_FRAME_HEADER_LENGTH     ((uint8_t)3)
#define JUDGE_EXTRA_LENGTH            ((uint8_t)4)

#define JUDGE_FRAME_HEADER            0xA5

#ifndef JUDGE_FILE
    #define JUDGE_EXT extern
#else
    #define JUDGE_EXT
#endif

// union for format transform
typedef union {
    uint8_t U[4];
    float F;
} FormatTrans_TypeDef;

typedef struct {
    uint8_t nextDecodeOffset;

    float desiredVelocityX;
    float desiredVelocityY;
    float measuredVelocityX;
    float measuredVelocityY;
    float measuredPositionX;
    float measuredPositionY;
    float yawAnglePosition;
    float yawAngularVelocity;
} JUDGE_DecodeTypeDef;

typedef __packed struct {
  uint16_t remainLifeValue;
  float realChassisOutV;
  float realChassisOutA;
} SimulatedData_Struct;

JUDGE_EXT volatile uint8_t JUDGE_DataBuffer[JUDGE_BUFFER_LENGTH];
JUDGE_EXT volatile JUDGE_DecodeTypeDef JUDGE_Data;
JUDGE_EXT volatile uint32_t JUDGE_FrameCounter;
JUDGE_EXT volatile uint8_t JUDGE_Started, JUDGE_RemainByte;

void JUDGE_Init(void);
void JUDGE_Decode(uint32_t length);
void JUDGE_DecodeFrame(uint8_t type);

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint8_t GetCRC8(uint8_t idx, uint8_t len, uint8_t ucCRC8);
unsigned int VerifyCRC8(uint8_t idx, uint8_t len);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

#endif
