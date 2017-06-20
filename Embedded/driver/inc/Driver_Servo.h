#ifndef DRIVER_SERVO
#define DRIVER_SERVO

#include "stm32f4xx.h"

#define SERVO_SET_L(x)                    TIM_SetCompare1(TIM1, 1500+(x))
#define SERVO_SET_R(x)                    TIM_SetCompare2(TIM1, 1500+(x))

#endif

