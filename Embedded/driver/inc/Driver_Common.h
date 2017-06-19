#ifndef DRIVER_COMMON_H
#define DRIVER_COMMON_H

#include "stm32f4xx.h"

#define ABS(x) (((x)>0)?(x):(-(x)))
#define MIN(a, b) (((a)<(b))?(a):(b))
#define MAX(a, b) (((a)>(b))?(a):(b))

#define COUNT_OF(x) ((sizeof(x)/sizeof(0[x])) / ((size_t)(!(sizeof(x) % sizeof(0[x])))))

#define UNUSED(x) ((void)(x))

typedef union
{
    uint8_t U[4];
    float F;
    int32_t I;
} FormatTrans;

#endif
