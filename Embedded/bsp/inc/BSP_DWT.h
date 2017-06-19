#ifndef BSP_DWT_H
#define BSP_DWT_H

#include "stm32f4xx.h"

void BSP_DWT_InitConfig(void);
void BSP_DWT_DelayUs(uint32_t us);
void BSP_DWT_DelayMs(uint32_t ms);

#endif // DWT_H
