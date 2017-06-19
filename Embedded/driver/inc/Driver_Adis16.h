#ifndef DRIVER_ADIS16_H
#define DRIVER_ADIS16_H

#include "stm32f4xx.h"

#ifndef ADIS16_FILE
    #define ADIS16_EXT extern
#else
    #define ADIS16_EXT
#endif

typedef struct {
    int32_t omega; // dir: clockwise
    int32_t theta, absoluteTheta;
    float temperature;

    int16_t omegaHW;
    // int32_t omegaInternal;
    // uint32_t thetaHW;
    // int16_t temperatureHW;
} ADIS16_DataTypeDef;

ADIS16_EXT volatile ADIS16_DataTypeDef ADIS16_Data;
ADIS16_EXT volatile uint8_t ADIS16_DataUpdated;

void ADIS16_Init(void);
void ADIS16_Update(void);
void ADIS16_Calibrate(uint16_t sample);

#endif
