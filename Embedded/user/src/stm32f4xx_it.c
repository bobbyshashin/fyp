/**
  ******************************************************************************
  * @file    Templates/Src/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.2.4
  * @date    06-May-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  *
  * @note    modified by ARM
  *          The modifications allow to use this file as User Code Template
  *          within the Device Family Pack.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

#define STM32F4xx_IT_FILE

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "param.h"

#include "Driver_ADIS16.h"
#include "Driver_Chassis.h"
#include "Driver_Common.h"
#include "Driver_Dbus.h"
#include "Driver_Gimbal.h"
#include "Driver_Gun.h"
#include "Driver_Judge.h"
#include "Driver_Led.h"
#include "Driver_Monitor.h"
#include "Driver_MPU6050.h"
#include "Driver_Simulator.h"

/** @addtogroup STM32F4xx_HAL_Examples
  * @{
  */

/** @addtogroup Templates
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief   This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
#ifndef RTE_CMSIS_RTOS_RTX
void SVC_Handler(void)
{
}
#endif

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
#ifndef RTE_CMSIS_RTOS_RTX
void PendSV_Handler(void)
{
}
#endif

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles UART1 interrupt request.  
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void) {
    static uint8_t dum;
    dum = USART1->DR;
    dum = USART1->SR;

    DMA_Cmd(DMA2_Stream2, DISABLE);

    if(DMA2_Stream2->NDTR == 0) {
        DBUS_Decode();
        CHASSIS_SetMotion();
    }
    
    DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2);
    while(DMA_GetCmdStatus(DMA2_Stream2) != DISABLE)
        ;
    DMA_SetCurrDataCounter(DMA2_Stream2, DBUS_BUFFER_SIZE);
    DMA_Cmd(DMA2_Stream2, ENABLE);

    UNUSED(dum);
}

/**
  * @brief  This function handles UART3 interrupt request.  
  * @param  None
  * @retval None
  */
void USART3_IRQHandler(void) {
    static uint8_t dum;
    dum = USART3->DR;
    dum = USART3->SR;

    JUDGE_Decode(DMA1_Stream1->NDTR);

    UNUSED(dum);
}

/**
  * @brief  This function handles CAN1 RX0 interrupt request.
  * @param  None
  * @retval None   
  */
#if BOARD_TYPE == BOARD_TYPE_JUDGE
typedef struct {
    uint16_t id;
    uint8_t data[8];
    uint8_t dum[6];
} Trans;
Trans Buffer[3];
// volatile uint16_t Count = 0;
#include <stdio.h>
#include <string.h>
int8_t ArmorId = 0;
void CAN1_RX0_IRQHandler(void) {
    static CanRxMsg CanRxData;
    static uint8_t isFirst[4] = {1, 1, 1, 1};
    static uint8_t id;

    CAN_Receive(CAN1, CAN_FIFO0, &CanRxData);

    switch(CanRxData.StdId) {
        case 0x150: {
            SIMULATOR_Hit();
        } break;
        case 0x240: case 0x241: case 0x242: case 0x243: {
            if (CanRxData.Data[1] == 0) {
                id = CanRxData.StdId & 0x0F;
                if (isFirst[id]) {
                    isFirst[id] = 0;
                    if (GlobalTick & 1)
                      SIMULATOR_ArmorInit(id, 0);
                    else
                      SIMULATOR_ArmorInit(id, 5);
                }
            }
        } break;
    }
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}
#else // BOARD_TYPE == BOARD_TYPE_CONTROL
void CAN1_RX0_IRQHandler(void) {
    static CanRxMsg CanRxData;

    CAN_Receive(CAN1, CAN_FIFO0, &CanRxData);

    switch(CanRxData.StdId) {
        case GIMBAL_YAW_ID: case GIMBAL_PITCH_ID: {
            GIMBAL_UpdateMeasure(CanRxData.StdId, CanRxData.Data);
        } break;
    }
    CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
}
#endif // BOARD_TYPE

/**
  * @brief  This function handles CAN2 RX0 interrupt request.
  * @param  None
  * @retval None   
  */
void CAN2_RX0_IRQHandler(void) {
    static CanRxMsg CanRxData;
    CAN_Receive(CAN2, CAN_FIFO0, &CanRxData);

    switch(CanRxData.StdId) {
        case FL_MOTOR_ID: case FR_MOTOR_ID:
        case BR_MOTOR_ID: case BL_MOTOR_ID: {
            CHASSIS_UpdateMeasure(CanRxData.StdId, CanRxData.Data);
        } break;
    }
    CAN_ITConfig(CAN2, CAN_IT_FMP0, ENABLE);
}

/**
  * @brief  This function handles TIM7 interrupt request.
  * @param  None
  * @retval None
  */
#if BOARD_TYPE == BOARD_TYPE_CONTROL
#include <external.h>
void TIM7_IRQHandler(void) {
    static uint32_t tick = 0;

    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);

    ++tick;
    ++GlobalTick;
    if (tick == 1000) {
        tick = 0;
    }

    if (tick % 500 == 0) {
        LED_Toggle();
        for (int8_t i = 0; i < 20; ++i)
          foo[i] += i;
    }

    if (tick % 20 == 0) {
        DBUS_UpdateStatus();
    }

    // MPU6050_ReadAll();

    if (DBUS_Status == kConnected) {
        CHASSIS_Control();
    }
    CHASSIS_SendCmd();
}
#else // BOARD_TYPE == BOARD_TYPE_JUDGE
int32_t Freq = 0;
void TIM7_IRQHandler(void) {
    static uint32_t tick = 0;
    static int32_t lastIdleCount = 0;

    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
    TIM_ClearFlag(TIM7, TIM_FLAG_Update);

    ++tick;
    ++GlobalTick;
    if (tick == 1000) {
        tick = 0;
        Freq = IdleCount - lastIdleCount;
        lastIdleCount = IdleCount;
    }

    if (tick % 500 == 0) {
        LED_Toggle();
        SIMULATOR_SendHeartBeat();
    }
    SIMULATOR_UpdatePower();
}
#endif// BOARD_TYPE

/**
  * @brief  This function handles TIM5 interrupt request.
  * @param  None
  * @retval None
  */
void TIM5_IRQHandler(void) {
    TIM_ClearITPendingBit(TIM5, TIM_IT_Update);
    TIM_ClearFlag(TIM5, TIM_FLAG_Update);

    ADIS16_Update();
}

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/


/**
  * @}
  */ 

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
