#define MPU6050_FILE

#include "BSP_I2C.h"
#include "Driver_MPU6050.h"

/* Who I am register value */
#define MPU6050_I_AM                0x68

/* MPU6050 registers */
#define MPU6050_AUX_VDDIO           0x01
#define MPU6050_SMPLRT_DIV          0x19
#define MPU6050_CONFIG              0x1A
#define MPU6050_GYRO_CONFIG         0x1B
#define MPU6050_ACCEL_CONFIG        0x1C
#define MPU6050_MOTION_THRESH       0x1F
#define MPU6050_INT_PIN_CFG         0x37
#define MPU6050_INT_ENABLE          0x38
#define MPU6050_INT_STATUS          0x3A
#define MPU6050_ACCEL_XOUT_H        0x3B
#define MPU6050_ACCEL_XOUT_L        0x3C
#define MPU6050_ACCEL_YOUT_H        0x3D
#define MPU6050_ACCEL_YOUT_L        0x3E
#define MPU6050_ACCEL_ZOUT_H        0x3F
#define MPU6050_ACCEL_ZOUT_L        0x40
#define MPU6050_TEMP_OUT_H          0x41
#define MPU6050_TEMP_OUT_L          0x42
#define MPU6050_GYRO_XOUT_H         0x43
#define MPU6050_GYRO_XOUT_L         0x44
#define MPU6050_GYRO_YOUT_H         0x45
#define MPU6050_GYRO_YOUT_L         0x46
#define MPU6050_GYRO_ZOUT_H         0x47
#define MPU6050_GYRO_ZOUT_L         0x48
#define MPU6050_MOT_DETECT_STATUS   0x61
#define MPU6050_SIGNAL_PATH_RESET   0x68
#define MPU6050_MOT_DETECT_CTRL     0x69
#define MPU6050_USER_CTRL           0x6A
#define MPU6050_PWR_MGMT_1          0x6B
#define MPU6050_PWR_MGMT_2          0x6C
#define MPU6050_FIFO_COUNTH         0x72
#define MPU6050_FIFO_COUNTL         0x73
#define MPU6050_FIFO_R_W            0x74
#define MPU6050_WHO_AM_I            0x75

/* Gyro sensitivities in °/s */
#define MPU6050_GYRO_SENS_250       131.0f
#define MPU6050_GYRO_SENS_500       65.5f
#define MPU6050_GYRO_SENS_1000      32.8f
#define MPU6050_GYRO_SENS_2000      16.4f

/* Acce sensitivities in g */
#define MPU6050_ACCEL_SENS_2         16384
#define MPU6050_ACCEL_SENS_4         8192
#define MPU6050_ACCEL_SENS_8         4096
#define MPU6050_ACCEL_SENS_16        2048

/**
  * @brief  MPU6050 Init
  * @param  void
  * @retval Init state of MPU6050, 1 for success, 0 for failure 
  */
uint8_t MPU6050_Init(void) {
    uint8_t temp;

    MPU6050_AccelScale = MPU6050_Accel_2G;
    MPU6050_GyroScale = MPU6050_Gyro_1000s;

    /* Check if device is connected */
    if (!BSP_I2C_Probe(MPU6050_I2C, MPU6050_ADDR))
        return 0;

    /* Wakeup MPU6050 */
    BSP_I2C_Write(MPU6050_I2C, MPU6050_ADDR, MPU6050_PWR_MGMT_1, 0x01);

    /* Config LPF */
    BSP_I2C_Write(MPU6050_I2C, MPU6050_ADDR, MPU6050_CONFIG, 0x03);

    /* Config sample rate */
    
    /* Config accelerometer */
    temp = BSP_I2C_Read(MPU6050_I2C, MPU6050_ADDR, MPU6050_ACCEL_CONFIG);
    temp = (temp & 0xE7) | (uint8_t)MPU6050_AccelScale << 3;
    BSP_I2C_Write(MPU6050_I2C, MPU6050_ADDR, MPU6050_ACCEL_CONFIG, temp);
    
    /* Config gyroscope */
    temp = BSP_I2C_Read(MPU6050_I2C, MPU6050_ADDR, MPU6050_GYRO_CONFIG);
    temp = (temp & 0xE7) | (uint8_t)MPU6050_GyroScale << 3;
    BSP_I2C_Write(MPU6050_I2C, MPU6050_ADDR, MPU6050_GYRO_CONFIG, temp);
    
    /* Set sensitivities for multiplying gyro and accelerometer data */
    switch (MPU6050_AccelScale) {
        case MPU6050_Accel_2G:
            MPU6050_Data.AccelFactor = 1.0f / MPU6050_ACCEL_SENS_2; 
            break;
        case MPU6050_Accel_4G:
            MPU6050_Data.AccelFactor = 1.0f / MPU6050_ACCEL_SENS_4; 
            break;
        case MPU6050_Accel_8G:
            MPU6050_Data.AccelFactor = 1.0f / MPU6050_ACCEL_SENS_8; 
            break;
        case MPU6050_Accel_16G:
            MPU6050_Data.AccelFactor = 1.0f / MPU6050_ACCEL_SENS_16; 
        default:
            break;
    }
    
    switch (MPU6050_GyroScale) {
        case MPU6050_Gyro_250s:
            MPU6050_Data.GyroFactor = 1.0f / MPU6050_GYRO_SENS_250; 
            break;
        case MPU6050_Gyro_500s:
            MPU6050_Data.GyroFactor = 1.0f / MPU6050_GYRO_SENS_500; 
            break;
        case MPU6050_Gyro_1000s:
            MPU6050_Data.GyroFactor = 1.0f / MPU6050_GYRO_SENS_1000; 
            break;
        case MPU6050_Gyro_2000s:
            MPU6050_Data.GyroFactor = 1.0f / MPU6050_GYRO_SENS_2000; 
        default:
            break;
    }

    return 1;
}

void MPU6050_ReadAccel(void) {
    static uint8_t data[6];
    
    /* Read accelerometer data */
    BSP_I2C_BurstRead(MPU6050_I2C, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, data, 6);
    
    /* Format */
    MPU6050_Data.RawAccel[0] = (int16_t)(data[0] << 8 | data[1]);    
    MPU6050_Data.RawAccel[1] = (int16_t)(data[2] << 8 | data[3]);
    MPU6050_Data.RawAccel[2] = (int16_t)(data[4] << 8 | data[5]);
}

void MPU6050_ReadGyro(void) {
    static uint8_t data[6];
    
    /* Read gyroscope data */
    BSP_I2C_BurstRead(MPU6050_I2C, MPU6050_ADDR, MPU6050_GYRO_XOUT_H, data, 6);
    
    /* Format */
    MPU6050_Data.RawGyro[0] = (int16_t)(data[0] << 8 | data[1]);    
    MPU6050_Data.RawGyro[1] = (int16_t)(data[2] << 8 | data[3]);
    MPU6050_Data.RawGyro[2] = (int16_t)(data[4] << 8 | data[5]);
}

void MPU6050_ReadTemperature(void) {
    static uint8_t data[2];
    static int16_t temp;
    
    /* Read temperature */
    BSP_I2C_BurstRead(MPU6050_I2C, MPU6050_ADDR, MPU6050_TEMP_OUT_H, data, 2);
    
    /* Format temperature */
    temp = (int16_t)(data[0] << 8 | data[1]);
    MPU6050_Data.Temperature = temp / 340.0f + 36.53f;
}

void MPU6050_ReadAll(void) {
    static uint8_t data[14];
    static int16_t temp;
    
    /* Read full raw data, 14bytes */
    BSP_I2C_BurstRead(MPU6050_I2C, MPU6050_ADDR, MPU6050_ACCEL_XOUT_H, data, 14);
    
    /* Format accelerometer data */
    MPU6050_Data.RawAccel[0] = (int16_t)(data[0] << 8 | data[1]);    
    MPU6050_Data.RawAccel[1] = (int16_t)(data[2] << 8 | data[3]);
    MPU6050_Data.RawAccel[2] = (int16_t)(data[4] << 8 | data[5]);

    /* Format temperature */
    temp = (int16_t)(data[6] << 8 | data[7]);
    MPU6050_Data.Temperature = temp / 340.0f + 36.53f;
    
    /* Format gyroscope data */
    MPU6050_Data.RawGyro[0] = (int16_t)(data[8] << 8 | data[9]);
    MPU6050_Data.RawGyro[1] = (int16_t)(data[10] << 8 | data[11]);
    MPU6050_Data.RawGyro[2] = (int16_t)(data[12] << 8 | data[13]);
}
