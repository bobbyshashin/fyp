#ifndef MENU_H
#define MENU_H

#include "stm32f4xx.h"

typedef void (*SwitchHandler)(uint16_t pin);

typedef struct {
    char name[6];
    uint8_t type;
    union {
        int32_t i;
        float f;
    } value;

    SwitchHandler handler;
} Option_TypeDef;

typedef struct {
    Option_TypeDef *options;
    uint8_t len;
    uint8_t curr;
    
    SwitchHandler handler;
} Menu_TypeDef;

#endif
