#include "main.h"

extern uint8_t judgeFrameDataLength;
int main(void) {
#if (BOARD_TYPE == BOARD_TYPE_CONTROL)
    CHASSIS_Init();
    /*JUDGE_Init();*/
#endif

    BSP_GPIO_InitConfig();
    BSP_CAN_InitConfig();
    BSP_TIM_InitConfig();
    BSP_UART_InitConfig();
    BSP_SPI_InitConfig();
    BSP_I2C_InitConfig();
    BSP_DMA_InitConfig();
    BSP_ADC_InitConfig();
    BSP_NVIC_InitConfig();
    BSP_DWT_InitConfig();

    ST7735_Init();
    ST7735_SetOrientation(kRevert);
    ST7735_FillColor(BLACK);
    MENU_Init();

#if (BOARD_TYPE == BOARD_TYPE_JUDGE)
    FLASH_Load();
    SIMULATOR_Init();
    SIMULATOR_CameraInit(7);
#endif

#if (BOARD_TYPE == BOARD_TYPE_CONTROL)
    /*ADIS16_Init();*/
    /*ADIS16_Calibrate(512);*/
    // MPU6050_Init();

#endif

#if (BOARD_TYPE == BOARD_TYPE_JUDGE)
    ST7735_Print(0, 1, GREEN, BLACK, "CNT");
    ST7735_Print(0, 2, GREEN, BLACK, "LEN");
    ST7735_Print(0, 3, GREEN, BLACK, "V");
    ST7735_Print(0, 4, GREEN, BLACK, "I");
    ST7735_Print(0, 5, GREEN, BLACK, "RL");
    ST7735_Print(0, 6, GREEN, BLACK, "BUF");
#endif

    BSP_TIM_Start();

    while (1) {
#if (BOARD_TYPE == BOARD_TYPE_CONTROL)
      MENU_Update();
      if (DBUS_Status == kLost) {
          CHASSIS_SetFree();
      }
#endif

#if (BOARD_TYPE == BOARD_TYPE_JUDGE)
        ST7735_Print(4, 1, GREEN, BLACK, "%d", IdleCount);
        ST7735_Print(4, 2, GREEN, BLACK, "%d", JudgeFrameLength);
        ST7735_Print(4, 3, GREEN, BLACK, "%.3f", JUDGE_Data.voltage);
        ST7735_Print(4, 4, GREEN, BLACK, "%.3f", JUDGE_Data.current);
        ST7735_Print(4, 5, GREEN, BLACK, "%d", JUDGE_Data.remainLife);
        ST7735_Print(4, 6, GREEN, BLACK, "%.3f", JUDGE_Data.remainEnergy);
#endif
    }
}
