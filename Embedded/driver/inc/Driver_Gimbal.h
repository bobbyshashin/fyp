#ifndef GIMBAL_H
#define GIMBAL_H

#include "stm32f4xx.h"

#ifndef GIMBAL_FILE
    #define GIMBAL_EXT extern
#else
    #define GIMBAL_EXT
#endif // GIMBAL_FILE

#define GIMBAL_MOTOR_CNT                        4

GIMBAL_EXT volatile int32_t GimbalOutput[GIMBAL_MOTOR_CNT];
GIMBAL_EXT volatile int32_t GimbalPosOffset[GIMBAL_MOTOR_CNT];
GIMBAL_EXT volatile int32_t GimbalPosition[GIMBAL_MOTOR_CNT], GimbalLastPosition[GIMBAL_MOTOR_CNT];
GIMBAL_EXT volatile int32_t GimbalRoundCount[GIMBAL_MOTOR_CNT];
GIMBAL_EXT volatile int32_t GimbalVelocity[GIMBAL_MOTOR_CNT];
GIMBAL_EXT volatile int32_t GimbalTargetVelocity[GIMBAL_MOTOR_CNT];

void GIMBAL_Init(void);
void GIMBAL_UpdateMeasure(uint16_t motorId, uint8_t *data);
void GIMBAL_SendCmd(void);
void GIMBAL_MotorControl(uint16_t motorId);
void GIMBAL_Control(void);
void GIMBAL_SetMotion(void);
void GIMBAL_SetFree(void);

#endif // GIMBAL_H
