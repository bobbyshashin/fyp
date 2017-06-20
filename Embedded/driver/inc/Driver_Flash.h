#ifndef DRIVER_FLASH_H
#define DRIVER_FLASH_H

#include "stm32f4xx.h"

#ifndef FLASH_FILE
    #define FLASH_EXT extern
#else
    #define FLASH_EXT
#endif

typedef struct {
    uint32_t foo;
} FLASH_DataTypeDef;

FLASH_EXT FLASH_DataTypeDef FLASH_Data;

void FLASH_Load(void);
void FLASH_Save(void);

#endif
