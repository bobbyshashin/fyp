#define SENDER_FILE

#include "Driver_Sender.h"
#include "Driver_Dbus.h"

#include <string.h>

void SENDER_Init(void) {
  SENDER_FrameCounter = 0;
  memset(&SENDER_Data, 0, sizeof(SENDER_Data));
}

void SENDER_UpdateData(void) {
  ++SENDER_FrameCounter;
  SENDER_Data.desiredVelocityX = 12 * DBUS_Data.ch2;
  SENDER_Data.desiredVelocityY = 18 * DBUS_Data.ch1;

  SENDER_Buffer[0] = JUDGE_FRAME_HEADER;
  SENDER_Buffer[1] = JUDGE_CONTROL_FRAME_LENGTH;
  SENDER_Buffer[2] = SENDER_FrameCounter;
  SENDER_Buffer[3] = 0x01;
  float *f_buf = (float*)(SENDER_Buffer+4);
  f_buf[0] = SENDER_Data.desiredVelocityX;
  f_buf[1] = SENDER_Data.desiredVelocityY;
  f_buf[2] = SENDER_Data.measuredVelocityX;
  f_buf[3] = SENDER_Data.measuredVelocityY;
  f_buf[4] = SENDER_Data.measuredPositionX;
  f_buf[5] = SENDER_Data.measuredPositionY;
  f_buf[6] = SENDER_Data.yawAnglePosition;
  f_buf[7] = SENDER_Data.yawAngularVelocity;
}

void SENDER_SendPackage(void) {
  SENDER_UpdateData();
  SENDER_Send(SENDER_Buffer, JUDGE_CONTROL_FRAME_LENGTH);
}

void SENDER_Send(uint8_t *buffer, uint32_t length) {
  DMA_Cmd(DMA2_Stream7, DISABLE);
  DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);
  while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE)
    ;
  DMA2_Stream7->M0AR = (uint32_t)buffer;
  DMA_SetCurrDataCounter(DMA2_Stream7, length);
  DMA_Cmd(DMA2_Stream7, ENABLE);
}
