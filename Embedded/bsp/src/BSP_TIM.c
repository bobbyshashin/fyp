#include "BSP_TIM.h"
#include "stm32f4xx.h"

/**
  * @brief  TIM Init
  * @param  void
  * @retval void
  */
void BSP_TIM_InitConfig(void) {
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_OCInitTypeDef TIM_OCInitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);

    // TIM1 (servo, 200Hz)
    TIM_TimeBaseInitStructure.TIM_ClockDivision =   TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   =   TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        =   20000-1;
    TIM_TimeBaseInitStructure.TIM_Prescaler     =   (uint32_t) (((SystemCoreClock / 1) / 1000000)-1); // 1MHz clock
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode       =   TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_Pulse        =   1500;
    TIM_OCInitStructure.TIM_OutputState  =   TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState =   TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity   =   TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  =   TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  =   TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState =   TIM_OCNIdleState_Set;
    TIM_OC1Init(TIM1, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC2Init(TIM1, &TIM_OCInitStructure);
    TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_OC3Init(TIM1, &TIM_OCInitStructure);
    TIM_OC3PreloadConfig(TIM1, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM1, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
    TIM_Cmd(TIM1, ENABLE);

    // TIM2 (brush motor, 1kHz)
    TIM_TimeBaseInitStructure.TIM_ClockDivision =   TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   =   TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        =   12000-1;
    TIM_TimeBaseInitStructure.TIM_Prescaler     =   (uint32_t) (((SystemCoreClock / 2) / 12000000)-1); // 12MHz clock
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode       =   TIM_OCMode_PWM2;
    TIM_OCInitStructure.TIM_Pulse        =   0;
    TIM_OCInitStructure.TIM_OutputState  =   TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity   =   TIM_OCPolarity_High;
    TIM_OC1Init(TIM2, &TIM_OCInitStructure);
    TIM_OC1PreloadConfig(TIM2, TIM_OCPreload_Enable);
    TIM_ARRPreloadConfig(TIM2, ENABLE);
    TIM_Cmd(TIM2, ENABLE);

    // TIM3 (encoder1)
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Falling, TIM_ICPolarity_Falling);
    TIM_Cmd(TIM3,ENABLE);
    TIM3->CNT = 0x7FFF;
        
    // TIM5 (gyro update 256Hz)
    TIM_TimeBaseInitStructure.TIM_ClockDivision =   TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   =   TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        =   125-1;
    TIM_TimeBaseInitStructure.TIM_Prescaler     =   (uint32_t) (((SystemCoreClock / 2) / 32000)-1);
    TIM_TimeBaseInit(TIM5, &TIM_TimeBaseInitStructure);

    // TIM7 (main control 1kHz)
    TIM_TimeBaseInitStructure.TIM_ClockDivision =   TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   =   TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        =   1000-1;
    TIM_TimeBaseInitStructure.TIM_Prescaler     =   (uint32_t) (((SystemCoreClock / 2) / 1000000)-1); // 1MHz clock
    TIM_TimeBaseInit(TIM7, &TIM_TimeBaseInitStructure);

    // TIM8 (buzzer PWM)
    TIM_TimeBaseInitStructure.TIM_ClockDivision =   TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode   =   TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period        =   1000-1;
    TIM_TimeBaseInitStructure.TIM_Prescaler     =   (uint32_t) (((SystemCoreClock / 1) / 1000000)-1); // 1MHz clock
    TIM_TimeBaseInit(TIM8, &TIM_TimeBaseInitStructure);

    TIM_OCInitStructure.TIM_OCMode       =   TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_Pulse        =   0;
    TIM_OCInitStructure.TIM_OutputState  =   TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_OutputNState =   TIM_OutputNState_Enable;
    TIM_OCInitStructure.TIM_OCPolarity   =   TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OCNPolarity  =   TIM_OCNPolarity_High;
    TIM_OCInitStructure.TIM_OCIdleState  =   TIM_OCIdleState_Reset;
    TIM_OCInitStructure.TIM_OCNIdleState =   TIM_OCNIdleState_Set;
    TIM_OC3Init(TIM8, &TIM_OCInitStructure);
    TIM_CtrlPWMOutputs(TIM8, ENABLE);
    TIM_Cmd(TIM8, ENABLE);
}

void BSP_TIM_Start(void) {
    // TIM5
    TIM_Cmd(TIM5, ENABLE);
    TIM_ITConfig(TIM5, TIM_IT_Update, ENABLE);
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);

    // TIM7
    TIM_Cmd(TIM7, ENABLE);
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);
}
