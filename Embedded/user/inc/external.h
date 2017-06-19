#ifndef EXTERNAL_VARIABLE_H
#define EXTERNAL_VARIABLE_H

#include "stm32f4xx.h"

#ifndef EXTERNAL_FILE
  #define EXTERN extern
#else
  #define EXTERN
#endif

EXTERN int32_t servoAngle_L, servoAngle_R;
EXTERN int32_t foo[20];

#endif

