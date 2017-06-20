#include "stm32f4xx.h"
#include "BSP_GPIO.h"
#include "Driver_Pinout.h"

/**
  * @brief  GPIO Init
  * @param  void
  * @retval void 
  */
void BSP_GPIO_InitConfig(void) {
    GPIO_InitTypeDef GPIO_InitStructure;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    // Led
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   LED_PIN;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_2MHz;
    GPIO_Init(LED_PORT, &GPIO_InitStructure);
    GPIO_SetBits(LED_PORT, LED_PIN);

    // // Buzzer
    // GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    // GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    // GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_1;
    // GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    // GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    // GPIO_Init(GPIOB, &GPIO_InitStructure);
    // GPIO_PinAFConfig(GPIOB, GPIO_PinSource1, GPIO_AF_TIM8);
    // GPIO_ResetBits(GPIOB, GPIO_Pin_1);

    // 5 way switch
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_IN;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   JS_PIN;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_2MHz;
    GPIO_Init(JS_PORT, &GPIO_InitStructure);

    // CAN1 (PA11 PA12)
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   CAN1_PIN;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(CAN1_PORT, &GPIO_InitStructure);
    CAN1_AF_CONFIG();

    // CAN2 (Chassis)
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   CAN2_PIN;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(CAN2_PORT, &GPIO_InitStructure);
    CAN2_AF_CONFIG();

    // USART1 (Dbus)
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   USART1_PIN;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(USART1_PORT, &GPIO_InitStructure);
    USART1_AF_CONFIG();

    // USART3 (Judge)
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   USART3_PIN;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(USART3_PORT, &GPIO_InitStructure);
    USART3_AF_CONFIG();

    // ST7735
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   ST7735_SPI_PIN;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(ST7735_SPI_PORT, &GPIO_InitStructure);
    ST7735_SPI_AF_CONFIG();

    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_2MHz;

    GPIO_InitStructure.GPIO_Pin    =   ST7735_RST_PIN;
    GPIO_Init(ST7735_RST_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin    =   ST7735_CS_PIN;
    GPIO_Init(ST7735_CS_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin    =   ST7735_DC_PIN;
    GPIO_Init(ST7735_DC_PORT, &GPIO_InitStructure);

    // ADIS16
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   ADIS16_SPI_PIN;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(ADIS16_SPI_PORT, &GPIO_InitStructure);
    ADIS16_AF_CONFIG();

    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Pin    =   ADIS16_CS_PIN;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_2MHz;
    GPIO_Init(ADIS16_CS_PORT, &GPIO_InitStructure);
    GPIO_SetBits(ADIS16_CS_PORT, ADIS16_CS_PIN);

    // I2C1 (MPU6050)
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Pin    =   I2C1_PIN;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(I2C1_PORT, &GPIO_InitStructure);
    I2C1_AF_CONFIG();

    // ADC1 (current sensor)
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Pin    =   ADC_VOLTAGE_PIN;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(ADC_VOLTAGE_PORT, &GPIO_InitStructure);

    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_OD;
    GPIO_InitStructure.GPIO_Pin    =   ADC_CURRENT_PIN;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(ADC_CURRENT_PORT, &GPIO_InitStructure);

    // Encoder1
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_4 | GPIO_Pin_5;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    ENCODER_AF_CONFIG();

    // Brush Motor
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_0;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource0, GPIO_AF_TIM2);

    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_1;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_2MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_ResetBits(GPIOA, GPIO_Pin_1);

    // Friction Wheel
    GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
    GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
    GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10;
    GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_NOPULL;
    GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource8, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_TIM1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource10, GPIO_AF_TIM1);
}
