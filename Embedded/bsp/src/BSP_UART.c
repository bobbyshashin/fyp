#include "BSP_UART.h"
#include "stm32f4xx.h"

/**
  * @brief  UART Init
  * @param  void
  * @retval void
  */
void BSP_UART_InitConfig(void) {
    USART_InitTypeDef USART_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

    // UART1 (DBUS)
    USART_InitStructure.USART_BaudRate              =   100000;
    USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  =   USART_Mode_Rx;
    USART_InitStructure.USART_Parity                =   USART_Parity_Even;
    USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
    USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
    USART_Init(USART1, &USART_InitStructure);
    USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    USART_Cmd(USART1, ENABLE);

    // UART3 (Judge + Data Monitor)
    USART_InitStructure.USART_BaudRate              =   115200;
    USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
    USART_InitStructure.USART_Mode                  =   USART_Mode_Rx | USART_Mode_Tx;
    USART_InitStructure.USART_Parity                =   USART_Parity_No;
    USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
    USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
    USART_Init(USART3, &USART_InitStructure);
    USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
    USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);
    USART_Cmd(USART3, ENABLE);
}
