#ifndef EXTERNAL_VARIABLE_H
#define EXTERNAL_VARIABLE_H

#include "stm32f4xx.h"

#ifndef EXTERNAL_FILE
  #define EXTERN extern
#else
  #define EXTERN
#endif

EXTERN float chassis_vx_scale, chassis_vy_scale;
EXTERN float tx_vx, tx_vy, rx_vx, rx_vy;
EXTERN int32_t ex_Speed[4];
EXTERN int32_t ex_JudgeFrameCounter;

#endif

