#include "stm32f4xx.h"
#include "BSP_SPI.h"
#include "Driver_Pinout.h"

/**
  * @brief  SPI Init
  * @param  void
  * @retval void
  */
void BSP_SPI_InitConfig(void) {
    SPI_InitTypeDef SPI_InitStructure;
    
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);
    
    // ST7735
    SPI_InitStructure.SPI_Mode              =   SPI_Mode_Master;
    SPI_InitStructure.SPI_BaudRatePrescaler =   SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_CPHA              =   SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_CPOL              =   SPI_CPOL_High;
    SPI_InitStructure.SPI_DataSize          =   SPI_DataSize_8b;
    SPI_InitStructure.SPI_Direction         =   SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_FirstBit          =   SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_NSS               =   SPI_NSS_Soft;
    SPI_Init(ST7735_SPI, &SPI_InitStructure);
    SPI_Cmd(ST7735_SPI, ENABLE);
    SPI_CalculateCRC(ST7735_SPI, DISABLE);
    SPI_SSOutputCmd(ST7735_SPI, DISABLE);

    // ADIS16
    SPI_InitStructure.SPI_Mode              =   SPI_Mode_Master;
    SPI_InitStructure.SPI_BaudRatePrescaler =   SPI_BaudRatePrescaler_32;
    SPI_InitStructure.SPI_CPHA              =   SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_CPOL              =   SPI_CPOL_High;
    SPI_InitStructure.SPI_DataSize          =   SPI_DataSize_16b;
    SPI_InitStructure.SPI_Direction         =   SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_FirstBit          =   SPI_FirstBit_MSB;
    SPI_InitStructure.SPI_NSS               =   SPI_NSS_Soft;
    SPI_Init(ADIS16_SPI, &SPI_InitStructure);
    SPI_Cmd(ADIS16_SPI, ENABLE);
    SPI_CalculateCRC(ADIS16_SPI, DISABLE);
}
