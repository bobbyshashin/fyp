#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "stm32f4xx.h"

void BSP_I2C_InitConfig(void);
uint8_t BSP_I2C_Read(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg);
void BSP_I2C_BurstRead(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t count);
void BSP_I2C_Write(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg, uint8_t data);
void BSP_I2C_BurstWrite(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t count);

uint8_t BSP_I2C_Probe(I2C_TypeDef *I2Cx, uint8_t addr);

#endif
