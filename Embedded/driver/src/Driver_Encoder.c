#define ENCODER_FILE

#include "Driver_Encoder.h"

void ENCODER_Update(void) {
    ENCODER_Data = ENCODER_TIM->CNT - 0x7FFF;
    ENCODER_TIM->CNT = 0x7FFF;
    if (ENCODER_DIR)
        ENCODER_Data = -ENCODER_Data;
}
