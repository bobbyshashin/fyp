#include "BSP_I2C.h"

#define I2C_TRANSMITTER_MODE   0
#define I2C_RECEIVER_MODE      1
#define I2C_ACK_ENABLE         1
#define I2C_ACK_DISABLE        0

// 20000 cycles
#define I2C_TIMEOUT 20000
static uint32_t I2C_Timeout;

static uint8_t BSP_I2C_Start(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t direction, uint8_t ack);
static uint8_t BSP_I2C_Stop(I2C_TypeDef *I2Cx);
static uint8_t BSP_I2C_ReadNack(I2C_TypeDef *I2Cx);
static uint8_t BSP_I2C_ReadAck(I2C_TypeDef *I2Cx);
static void BSP_I2C_WriteData(I2C_TypeDef *I2Cx, uint8_t data);

/**
  * @brief  I2C Init
  * @param  void
  * @retval void 
  */
void BSP_I2C_InitConfig(void) {
    I2C_InitTypeDef I2C_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_I2C1, ENABLE);

    // I2C1 (MPU6050)
    I2C_InitStructure.I2C_ClockSpeed          = 400000;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_Mode                = I2C_Mode_I2C;
    I2C_InitStructure.I2C_OwnAddress1         = 0x00;
    I2C_InitStructure.I2C_Ack                 = I2C_Ack_Disable;
    I2C_InitStructure.I2C_DutyCycle           = I2C_DutyCycle_2;
    I2C_Init(I2C1, &I2C_InitStructure);
    I2C_Cmd(I2C1, ENABLE);
}

/**
 * @brief  Reads single byte from slave
 * @param  *I2Cx: I2C used
 * @param  addr: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  reg: register to read from
 * @retval Data from slave
 */
uint8_t BSP_I2C_Read(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg) {
    BSP_I2C_Start(I2Cx, addr, I2C_TRANSMITTER_MODE, I2C_ACK_DISABLE);
    BSP_I2C_WriteData(I2Cx, reg);
    BSP_I2C_Start(I2Cx, addr, I2C_RECEIVER_MODE, I2C_ACK_DISABLE);
    return BSP_I2C_ReadNack(I2Cx);
}

/**
 * @brief  Reads multi bytes from slave
 * @param  *I2Cx: I2C used
 * @param  uint8_t addr: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  uint8_t reg: register to read from
 * @param  uint8_t *data: pointer to data array to store data from slave
 * @param  uint8_t count: how many bytes will be read
 * @retval None
 */
void BSP_I2C_BurstRead(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t count) {
    BSP_I2C_Start(I2Cx, addr, I2C_TRANSMITTER_MODE, I2C_ACK_ENABLE);
    BSP_I2C_WriteData(I2Cx, reg);
    BSP_I2C_Start(I2Cx, addr, I2C_RECEIVER_MODE, I2C_ACK_ENABLE);
    while (count--) {
        if (!count) {
            /* Last byte */
            *data++ = BSP_I2C_ReadNack(I2Cx);
        } else {
            *data++ = BSP_I2C_ReadAck(I2Cx);
        }
    }
}

/**
 * @brief  Writes single byte to slave
 * @param  *I2Cx: I2C used
 * @param  addr: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  reg: register to write to
 * @param  data: data to be written
 * @retval None
 */
void BSP_I2C_Write(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg, uint8_t data) {
    BSP_I2C_Start(I2Cx, addr, I2C_TRANSMITTER_MODE, I2C_ACK_DISABLE);
    BSP_I2C_WriteData(I2Cx, reg);
    BSP_I2C_WriteData(I2Cx, data);
    BSP_I2C_Stop(I2Cx);
}

/**
 * @brief  Writes multi bytes to slave
 * @param  *I2Cx: I2C used
 * @param  addr: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @param  reg: register to write to
 * @param  *data: pointer to data array to write it to slave
 * @param  count: how many bytes will be written
 * @retval None
 */
void BSP_I2C_BurstWrite(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t reg, uint8_t *data, uint16_t count) {
    BSP_I2C_Start(I2Cx, addr, I2C_TRANSMITTER_MODE, I2C_ACK_DISABLE);
    BSP_I2C_WriteData(I2Cx, reg);
    while (count--) {
        BSP_I2C_WriteData(I2Cx, *data++);
    }
    BSP_I2C_Stop(I2Cx);
}

/**
 * @brief  Checks if device is connected to I2C bus
 * @param  *I2Cx: I2C used
 * @param  addr: 7 bit slave address, left aligned, bits 7:1 are used, LSB bit is not used
 * @retval Device status:
 *            - 0: Device is not connected
 *            - 1: Device is connected
 */
uint8_t BSP_I2C_Probe(I2C_TypeDef *I2Cx, uint8_t addr) {
    uint8_t connected = 0;
    /* Try to start, function will return 0 in case device will send ACK */
    if (!BSP_I2C_Start(I2Cx, addr, I2C_TRANSMITTER_MODE, I2C_ACK_ENABLE)) {
        connected = 1;
    }
    
    /* STOP I2C */
    BSP_I2C_Stop(I2Cx);
    
    /* Return status */
    return connected;
}

/**
 * @brief  I2C Start condition
 * @param  *I2Cx: I2C used
 * @param  addr: slave address
 * @param  direction: master to slave or slave to master
 * @param  ack: ack enabled or disabled
 * @retval Start condition status (0 for success, 1 for failure)
 */
static uint8_t BSP_I2C_Start(I2C_TypeDef *I2Cx, uint8_t addr, uint8_t direction, uint8_t ack) {
    /* Generate a START condition */
    I2Cx->CR1 |= I2C_CR1_START;

    /* Wait till I2C is busy */
    I2C_Timeout = I2C_TIMEOUT;
    while (!(I2Cx->SR1 & I2C_SR1_SB))
        if (--I2C_Timeout == 0x00)
            return 1;

    /* Enable ack if we select it */
    if (ack) {
        I2Cx->CR1 |= I2C_CR1_ACK;
    }

    /* Send write/read bit */
    if (direction == I2C_TRANSMITTER_MODE) {
        /* Send address with zero last bit */
        I2Cx->DR = addr & ~I2C_OAR1_ADD0;
        
        /* Wait till finished */
        I2C_Timeout = I2C_TIMEOUT;
        while (!(I2Cx->SR1 & I2C_SR1_ADDR)) {
            if (--I2C_Timeout == 0x00) {
                return 1;
            }
        }
    }
    if (direction == I2C_RECEIVER_MODE) {
        /* Send address with 1 last bit */
        I2Cx->DR = addr | I2C_OAR1_ADD0;
        
        /* Wait till finished */
        I2C_Timeout = I2C_TIMEOUT;
        while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
            if (--I2C_Timeout == 0x00) {
                return 1;
            }
        }
    }
    
    /* Read status register to clear ADDR flag */
    I2Cx->SR2;
    
    /* Return 0, everything ok */
    return 0;
}

/**
 * @brief  Stop condition on I2C
 * @param  *I2Cx: I2C used
 * @retval Stop condition status (0 for success, 1 for failure)
 */
static uint8_t BSP_I2C_Stop(I2C_TypeDef *I2Cx) {
    /* Wait till transmitter not empty */
    I2C_Timeout = I2C_TIMEOUT;
    while (((!(I2Cx->SR1 & I2C_SR1_TXE)) || (!(I2Cx->SR1 & I2C_SR1_BTF)))) {
        if (--I2C_Timeout == 0x00) {
            return 1;
        }
    }
    
    /* Generate stop */
    I2Cx->CR1 |= I2C_CR1_STOP;
    
    /* Return 0, everything ok */
    return 0;
}

/**
 * @brief  Reads byte without ack
 * @param  *I2Cx: I2C used
 * @retval Byte from slave
 */
static uint8_t BSP_I2C_ReadNack(I2C_TypeDef *I2Cx) {
    uint8_t data;
    
    /* Disable ACK */
    I2Cx->CR1 &= ~I2C_CR1_ACK;
    
    /* Generate stop */
    I2Cx->CR1 |= I2C_CR1_STOP;
    
    /* Wait till received */
    I2C_Timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
        if (--I2C_Timeout == 0x00) {
            return 1;
        }
    }

    /* Read data */
    data = I2Cx->DR;
    
    /* Return data */
    return data;
}

/**
 * @brief  Reads byte with ack
 * @param  *I2Cx: I2C used
 * @retval Byte from slave
 */
static uint8_t BSP_I2C_ReadAck(I2C_TypeDef *I2Cx) {
    uint8_t data;
    
    /* Enable ACK */
    I2Cx->CR1 |= I2C_CR1_ACK;
    
    /* Wait till not received */
    I2C_Timeout = I2C_TIMEOUT;
    while (!I2C_CheckEvent(I2Cx, I2C_EVENT_MASTER_BYTE_RECEIVED)) {
        if (--I2C_Timeout == 0x00) {
            return 1;
        }
    }
    
    /* Read data */
    data = I2Cx->DR;
    
    /* Return data */
    return data;
}

/**
 * @brief  Writes to slave
 * @param  *I2Cx: I2C used
 * @param  data: data to be sent
 * @retval None
 */
static void BSP_I2C_WriteData(I2C_TypeDef *I2Cx, uint8_t data) {
    /* Wait till I2C is not busy anymore */
    I2C_Timeout = I2C_TIMEOUT;
    while (!(I2Cx->SR1 & I2C_SR1_TXE) && I2C_Timeout) {
        I2C_Timeout--;
    }
    
    /* Send I2C data */
    I2Cx->DR = data;
}

