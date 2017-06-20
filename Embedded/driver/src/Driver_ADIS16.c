#define ADIS16_FILE

#include "stm32f4xx.h"
#include "Driver_ADIS16.h"
#include "Driver_Common.h"
#include "Driver_Pinout.h"
#include "BSP_DWT.h"
#include <string.h>

/* ADIS16 register address */
#define ADIS16_FLASH              0x01
#define ADIS16_POWER              0x03
#define ADIS16_VEL                0x05
#define ADIS16_ADC                0x0B
#define ADIS16_TEMP               0x0D
#define ADIS16_ANGL               0x0F
#define ADIS16_OFF                0x15   
#define ADIS16_COMD               0x3F
#define ADIS16_SENS               0x39
#define ADIS16_SMPL               0x37

#define MIN_OMEGA                 5
#define TEMPERATURE_SCALE         0.1453f
#define TEMPERATURE_OFFSET        25.0f
#define BIAS_TEMP_COEFFICIENT     0.0051f

#define CHIP_SELECT()             GPIO_ResetBits(ADIS16_CS_PORT, ADIS16_CS_PIN)
#define CHIP_DESELECT()           GPIO_SetBits(ADIS16_CS_PORT, ADIS16_CS_PIN)

static void ADIS16_Write(uint8_t addr, uint16_t data);
static uint16_t ADIS16_Read(uint8_t addr);
static uint16_t ADIS16_SPI_IO(uint16_t data);

static int16_t ADIS16_GetOmega(void);
/*static uint16_t ADIS16_GetTheta(void);*/
/*static int16_t ADIS16_GetTemperature(void);*/

static float omegaIntegral = 0.0f;

void ADIS16_Init(void) {
    /* Data initialization */
    ADIS16_Data.omega = 0;
    ADIS16_Data.theta = 0;
    ADIS16_Data.absoluteTheta = 0;
    ADIS16_Data.temperature = 25.0f;
    ADIS16_DataUpdated = 0;

    /* Reset */
    ADIS16_Write(ADIS16_COMD, 0x0080);
    BSP_DWT_DelayMs(50);

    /* Factory Calibration Restore */
    ADIS16_Write(ADIS16_COMD, 0x0002);

    /* Filter settings */
    ADIS16_Write(ADIS16_SENS, 0x0404);
    ADIS16_Write(ADIS16_SMPL, 0x0001); // sample period: 3.906ms
    ADIS16_Write(ADIS16_COMD, 0x0008);
    BSP_DWT_DelayMs(100);
}

void ADIS16_Update(void) {
    static float delta;
    ADIS16_DataUpdated = 1;

    ADIS16_Data.omegaHW = ADIS16_GetOmega();
    ADIS16_Data.omega = -ADIS16_Data.omegaHW;

    delta = (ABS(ADIS16_Data.omega) < MIN_OMEGA) ? 0 : ADIS16_Data.omega;
    omegaIntegral += delta * (0.07326f * 0.0390625f);
    ADIS16_Data.absoluteTheta = ADIS16_Data.theta
        = (int32_t)omegaIntegral;
    ADIS16_Data.theta %= 3600;
    if (ADIS16_Data.theta < 0) ADIS16_Data.theta += 3600;
}

void ADIS16_Calibrate(uint16_t sample) {
    BSP_DWT_DelayMs(2000);

    uint16_t originalOffset = ADIS16_Read(ADIS16_OFF);
    int32_t offsetSum = 0;
    for (uint16_t i = 0; i < sample; ++i) {
        offsetSum += ADIS16_GetOmega();
        BSP_DWT_DelayMs(4);
    }

    int16_t offsetAverage = (int16_t)originalOffset-(int16_t)(offsetSum*4.0f/sample);
    uint16_t data = 0x0000U | offsetAverage;
    ADIS16_Write(ADIS16_OFF, data);
    ADIS16_Write(ADIS16_COMD, 0x0008);

    BSP_DWT_DelayMs(100);
}

/* 14 bit signed */
static int16_t ADIS16_GetOmega(void) {
    uint16_t buf = ADIS16_Read(ADIS16_VEL);

    // 14 bit to 16 bit sign extension
    if (buf & 0x2000) buf |= 0xC000;
    else buf &= 0x3FFF;

    int16_t ret = 0x0000 | buf;
    return ret;
}

/* 14 bit unsigned */
/*static uint16_t ADIS16_GetTheta(void) {*/
    /*uint16_t buf = ADIS16_Read(ADIS16_ANGL);*/
    /*return buf & 0x3FFFU;*/
/*}*/

/* 12 bit signed */
/*static int16_t ADIS16_GetTemperature(void) {*/
    /*uint16_t buf = ADIS16_Read(ADIS16_TEMP);*/
    /*if (buf & 0x0800) buf |= 0xF000;*/
    /*else buf &= 0x0FFF;*/
    /*int16_t ret = 0x0000 | buf;*/
    /*return ret;*/
/*}*/

/*
       addr-1          addr  
    data[7:0]    data[15:8]
*/
static void ADIS16_Write(uint8_t addr, uint16_t data) {
    addr = (addr & 0x3F) | 0x80;

    uint16_t cmd;

    cmd = ((uint16_t)addr << 8) | (data >> 8);
    ADIS16_SPI_IO(cmd);

    cmd = ((uint16_t)(addr-1) << 8) | (data & 0xFF);
    ADIS16_SPI_IO(cmd);
}

static uint16_t ADIS16_Read(uint8_t addr) {
    addr &= 0x3F;
    uint16_t cmd = addr << 8;

    ADIS16_SPI_IO(cmd);
    return ADIS16_SPI_IO(cmd);
}

/*
    Full duplex.
*/
static uint16_t ADIS16_SPI_IO(uint16_t data) {
    CHIP_SELECT();
    while (!SPI_I2S_GetFlagStatus(ADIS16_SPI, SPI_I2S_FLAG_TXE))
        ;
    ADIS16_SPI->DR = data;

    while (!SPI_I2S_GetFlagStatus(ADIS16_SPI, SPI_I2S_FLAG_RXNE))
        ;
    uint16_t ret = ADIS16_SPI->DR;
    CHIP_DESELECT();

    /* Minimum stall time = 9us */
    BSP_DWT_DelayUs(10);
    return ret;
}
