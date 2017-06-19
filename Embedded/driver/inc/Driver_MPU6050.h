#ifndef DRIVER_MPU6050_H
#define DRIVER_MPU6050_H

#include "stm32f4xx.h"

#ifndef MPU6050_FILE
    #define MPU6050_EXT extern
#else
    #define MPU6050_EXT
#endif

#define MPU6050_ADDR                            0xD0
#define MPU6050_I2C                             I2C1

typedef struct {
    int16_t RawAccel[3];
    int16_t RawGyro[3];
    float Temperature;

    float AccelFactor;
    float GyroFactor;
} MPU6050_DataTypeDef;

typedef enum {
    MPU6050_Accel_2G = 0x00, /*!< Range is +- 2G */
    MPU6050_Accel_4G = 0x01, /*!< Range is +- 4G */
    MPU6050_Accel_8G = 0x02, /*!< Range is +- 8G */
    MPU6050_Accel_16G = 0x03 /*!< Range is +- 16G */
} MPU6050_AccelScaleTypeDef;

typedef enum {
    MPU6050_Gyro_250s = 0x00,  /*!< Range is +- 250 degrees/s */
    MPU6050_Gyro_500s = 0x01,  /*!< Range is +- 500 degrees/s */
    MPU6050_Gyro_1000s = 0x02, /*!< Range is +- 1000 degrees/s */
    MPU6050_Gyro_2000s = 0x03  /*!< Range is +- 2000 degrees/s */
} MPU6050_GyroScaleTypeDef;

MPU6050_EXT MPU6050_DataTypeDef MPU6050_Data;
MPU6050_EXT MPU6050_AccelScaleTypeDef MPU6050_AccelScale;
MPU6050_EXT MPU6050_GyroScaleTypeDef MPU6050_GyroScale;

uint8_t MPU6050_Init(void);
void MPU6050_ReadAccel(void);
void MPU6050_ReadGyro(void);
void MPU6050_ReadTemperature(void);
void MPU6050_ReadAll(void);

#endif
