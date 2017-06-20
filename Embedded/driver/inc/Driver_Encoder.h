#ifndef DRIVER_ENCODER
#define DRIVER_ENCODER

#include "stm32f4xx.h"

// 0: normal  1: reverse
#define ENCODER_DIR                             1
#define ENCODER_TIM                             TIM3

#ifndef ENCODER_FILE
    #define ENCODER_EXT extern
#else
    #define ENCODER_EXT
#endif

ENCODER_EXT int32_t ENCODER_Data;

void ENCODER_Update(void);

#endif
