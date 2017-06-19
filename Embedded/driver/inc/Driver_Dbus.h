#ifndef DRIVER_DBUS_H
#define DRIVER_DBUS_H

#include "stm32f4xx.h"

#define DBUS_BUFFER_SIZE                        18U

#define KEY_V       0x4000
#define KEY_C       0x2000
#define KEY_X       0x1000
#define KEY_Z       0x0800
#define KEY_G       0x0400
#define KEY_F       0x0200
#define KEY_R       0x0100
#define KEY_E       0x0080
#define KEY_Q       0x0040
#define KEY_CTRL    0x0020
#define KEY_SHIFT   0x0010
#define KEY_D       0x0008
#define KEY_A       0x0004
#define KEY_S       0x0002
#define KEY_W       0x0001

typedef enum {
    kSwitchUp     = 1,
    kSwitchDown   = 2,
    kSwitchMiddle = 3
} DBUS_SwitchState;

/*
    DBUS decode type
*/
typedef struct {
    /*
        Range: [-660, +660]
        Init: 0
    */
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    int16_t ch4;
    
    /*
        Range: [1..3]
        1: Up  2: Down  3: Middle
    */
    DBUS_SwitchState leftSwitchState;
    DBUS_SwitchState rightSwitchState;
    
    struct {
        /*
            Range: [-32768, 32767]
            Init: 0
        */
        int16_t x;
        int16_t y;
        int16_t z;
        
        /*
            Range: [0, 1]
            0: Not pressed  1: Pressed
        */
        uint8_t press_left;
        uint8_t press_right;
        
        uint8_t jumppress_left;
        uint8_t jumppress_right;
    } mouse;
    
    struct {
        /*
            Bitmap:
                15  14  13  12  11  10  9   8   7   6   5   4   3   2   1
                V   C   X   Z   G   F   R   E   Q CTR SHT   D   A   S   W
        */
        uint16_t key_code;              // original key
        uint16_t jumpkey_code;          // key after transition
    } key;
} DBUS_DecodeTypeDef;

typedef enum {
    kLost,
    kConnected
} DBUS_StatusTypeDef;

#ifndef DBUS_FILE
    #define DBUS_EXT extern
#else
    #define DBUS_EXT
#endif // DBUS_FILE

DBUS_EXT volatile uint8_t DBUS_Buffer[DBUS_BUFFER_SIZE];
DBUS_EXT volatile DBUS_DecodeTypeDef DBUS_Data, DBUS_LastData;
DBUS_EXT volatile DBUS_StatusTypeDef DBUS_Status;
DBUS_EXT volatile uint32_t DBUS_FrameCount, DBUS_LastFrameCount;

void DBUS_Init(void);
void DBUS_Decode(void);
void DBUS_UpdateStatus(void);
uint8_t DBUS_IsKeyPressed(uint32_t key);

#endif
