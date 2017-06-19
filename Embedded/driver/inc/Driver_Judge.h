#ifndef DRIVER_JUDGE_H
#define DRIVER_JUDGE_H

#include "stm32f4xx.h"

#define JUDGE_BUFFER_LENGTH           256
#define JUDGE_INFO_FRAME_LENGTH       ((uint8_t)44)
#define JUDGE_BLOOD_FRAME_LENGTH      ((uint8_t)12)
#define JUDGE_SHOOT_FRAME_LENGTH      ((uint8_t)25)
#define JUDGE_FRAME_HEADER_LENGTH     ((uint8_t)5)
#define JUDGE_EXTRA_LENGTH            ((uint8_t)9)

#define JUDGE_FRAME_HEADER            0xA5

#ifndef JUDGE_FILE
    #define JUDGE_EXT extern
#else
    #define JUDGE_EXT
#endif

typedef __packed struct {
    uint16_t remainLifeValue;
    float realChassisOutV;
    float realChassisOutA;
} SimulatedData_Struct;

typedef __packed struct {
    uint8_t flag; // 0 invalid, 1 valid
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint32_t compass;
} tLocData;

typedef __packed struct {
    uint32_t remainTime;
    uint16_t remainLifeValue;
    float realChassisOutV;
    float realChassisOutA;
    tLocData locData;
    float remainPower;
} tGameInfo;

typedef __packed struct {
    uint8_t weakId:4;
    uint8_t way:4;
    uint16_t value;
} tRealBloodChangedData;

typedef __packed struct {
    float realBulletShootSpeed;
    float realBulletShootFreq;
    float realGolfShootSpeed;
    float realGolfShootFreq;
} tRealShootData;

typedef __packed struct {
    float data1;
    float data2;
    float data3;
} tSelfDef;

// union for format transform
typedef union {
    uint8_t U[4];
    float F;
} FormatTrans_TypeDef;

typedef struct {
    uint8_t nextDecodeOffset;

    float voltage;              // V
    float current;              // A
    uint16_t remainLife;

    float power;                // W
    float remainEnergy;         // J
    uint8_t powerUpdated;

    int8_t hitArmorId;
    uint32_t lastHitTick;
    uint16_t armorDamage;
    uint16_t speedDamage;
    uint16_t freqDamage;
    uint16_t powerDamage;
    uint16_t moduleDamage;
    uint16_t violationDamage;

    uint16_t shootNum;
    float shootSpeed;
    uint32_t lastShootTick;
} JUDGE_DecodeTypeDef;

JUDGE_EXT volatile uint8_t JUDGE_DataBuffer[JUDGE_BUFFER_LENGTH];
JUDGE_EXT volatile JUDGE_DecodeTypeDef JUDGE_Data;
JUDGE_EXT volatile uint32_t JUDGE_FrameCounter;
JUDGE_EXT volatile uint8_t JUDGE_Started, JUDGE_RemainByte;

void JUDGE_Init(void);
void JUDGE_Decode(uint32_t length);
void JUDGE_UpdatePower(void);
void JUDGE_DecodeFrame(uint8_t type);

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint8_t GetCRC8(uint8_t idx, uint8_t len, uint8_t ucCRC8);
unsigned int VerifyCRC8(uint8_t idx, uint8_t len);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);

#endif
