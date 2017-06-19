#ifndef DRIVER_SIMULATOR
#define DRIVER_SIMULATOR

#ifndef SIMULATOR_FILE
    #define SIMULATOR_EXT extern
#else
    #define SIMULATOR_EXT
#endif

typedef struct {
    float current;
    float voltage;
    float power;
    float remainEnergy;

    int16_t remainLife;
} SIMULATOR_DataTypeDef;

SIMULATOR_EXT uint16_t SIMULATOR_DataBuffer[16];
SIMULATOR_EXT float SIMULATOR_CurrentBuffer[20];
SIMULATOR_EXT float SIMULATOR_VoltageBuffer[20];
SIMULATOR_EXT SIMULATOR_DataTypeDef SIMULATOR_Data;

void SIMULATOR_Init(void);
void SIMULATOR_SendHeartBeat(void);
void SIMULATOR_CameraInit(uint8_t robotId);
void SIMULATOR_ArmorInit(uint8_t armorId, uint8_t robotId);
void SIMULATOR_UpdatePower(void);
void SIMULATOR_Hit(void);

#endif
