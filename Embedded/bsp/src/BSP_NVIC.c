#include "BSP_NVIC.h"
#include "stm32f4xx.h"

/**
  * @brief  NVIC Init
  * @param  void
  * @retval void
  */
void BSP_NVIC_InitConfig(void) {
    NVIC_InitTypeDef NVIC_InitStructure;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);

    // CAN1
    NVIC_InitStructure.NVIC_IRQChannel                      =   CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                   =   ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    =   9;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           =   0;
    NVIC_Init(&NVIC_InitStructure);
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);

    // CAN2 (Chassis)
    NVIC_InitStructure.NVIC_IRQChannel                      =   CAN2_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                   =   ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    =   11;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           =   0;
    NVIC_Init(&NVIC_InitStructure);
    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);

    // TIM7 (main control)
    NVIC_InitStructure.NVIC_IRQChannel                      =   TIM7_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                   =   ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    =   3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           =   0;
    NVIC_Init(&NVIC_InitStructure);

    // TIM5 (ADIS16 update)
    NVIC_InitStructure.NVIC_IRQChannel                      =   TIM5_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                   =   ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    =   13;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           =   0;
    NVIC_Init(&NVIC_InitStructure);

    // USART1 (DBUS)
    NVIC_InitStructure.NVIC_IRQChannel                      =   USART1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                   =   ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    =   8;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           =   0;
    NVIC_Init(&NVIC_InitStructure);

    // USART3 (Judge)
    NVIC_InitStructure.NVIC_IRQChannel                      =   USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd                   =   ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority    =   10;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority           =   0;
    NVIC_Init(&NVIC_InitStructure);
}
