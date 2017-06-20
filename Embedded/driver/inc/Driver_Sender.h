#ifndef DRIVER_SENDER_H
#define DRIVER_SENDER_H

#include "stm32f4xx.h"

#include "Driver_Judge.h"

#ifndef SENDER_FILE
  #define SENDER_EXT extern
#else
  #define SENDER_EXT
#endif

SENDER_EXT JUDGE_DecodeTypeDef SENDER_Data;
SENDER_EXT uint8_t SENDER_FrameCounter;
SENDER_EXT uint8_t SENDER_Buffer[JUDGE_BUFFER_LENGTH];

void SENDER_Init(void);
void SENDER_UpdateData(void);
void SENDER_SendPackage(void);
void SENDER_Send(uint8_t *buffer, uint32_t length);

#endif

