#include "stm32f4xx.h"
#include "Driver_Monitor.h"

void MONITOR_Send(uint8_t *buffer, uint32_t length) {
    DMA_Cmd(DMA1_Stream3, DISABLE);
    DMA_ClearFlag(DMA1_Stream3, DMA_FLAG_TCIF3);
    while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE)
        ;
    DMA1_Stream3->M0AR = (uint32_t)buffer;
    DMA_SetCurrDataCounter(DMA1_Stream3, length);
    DMA_Cmd(DMA1_Stream3, ENABLE);
}
