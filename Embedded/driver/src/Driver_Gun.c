#define GUN_FILE

#include "Driver_Dbus.h"
#include "Driver_Encoder.h"
#include "Driver_Gun.h"
#include "PID.h"
#include "stm32f4xx_it.h"

#include <string.h>

static PID_Controller PokeSpeedController;
static PID_Controller PokeAngleController;

void GUN_Init(void) {
    memset((char*)&GUN_Data, 0, sizeof(GUN_Data));

    PID_Reset(&PokeSpeedController);
    PokeSpeedController.Kp = 100.0f;
    PokeSpeedController.Ki = 20.00f;
    PokeSpeedController.Kd = 0.00f;
    PokeSpeedController.MAX_Pout = 12000;
    PokeSpeedController.MAX_Integral = 100000;
    PokeSpeedController.MAX_PIDout = 12000;
    PokeSpeedController.MIN_PIDout = 0;
    PokeSpeedController.mode = kPositional;

    PID_Reset(&PokeAngleController);
    PokeAngleController.Kp = 0.30f;
    PokeAngleController.Ki = 0.00f;
    PokeAngleController.Kd = 0.00f;
    PokeAngleController.MAX_Pout = 12000;
    PokeAngleController.MAX_Integral = 100000;
    PokeAngleController.MAX_PIDout = 80;
    PokeAngleController.MIN_PIDout = 0;
    PokeAngleController.mode = kPositional;
}

/*
 * Called every DBUS package
 */
void GUN_SetMotion(void) {
    static char shoot = 0;
    static char jumpPress = 0, jumpRelease = 0;
    static volatile int32_t lastTick = 0;
    static volatile int32_t pressCount = 0;

    jumpPress = DBUS_Data.mouse.press_left &&
        !DBUS_LastData.mouse.press_left;
    jumpRelease = !DBUS_Data.mouse.press_left &&
        DBUS_LastData.mouse.press_left;
    if (jumpRelease) pressCount = 0;
    if (DBUS_Data.mouse.press_left) {
        ++pressCount;
    }

    shoot = jumpPress || (((pressCount & 0x000FU) == 0)&&pressCount);
    shoot = shoot && (DBUS_Data.rightSwitchState != kSwitchDown);
    shoot = shoot && (GlobalTick - lastTick > 220);
    if (shoot) {
        GUN_ShootOne();
        lastTick = GlobalTick;
    }
}

void GUN_ShootOne(void) {
    GUN_Data.pokeTargetAngle += 660;
}

void GUN_PokeControl(void) {
    GUN_Data.pokeTargetSpeed = PID_Update(&PokeAngleController,
        GUN_Data.pokeTargetAngle, GUN_Data.pokeAngle);
    GUN_PokeSpeedControl();
}

void GUN_PokeSpeedControl(void) {
    ENCODER_Update();
    GUN_Data.pokeAngle += ENCODER_Data;
    if (GUN_Data.pokeAngle > 16777216) {
        GUN_Data.pokeAngle = GUN_Data.pokeTargetAngle = 0;
    }
    GUN_Data.pokeOutput = PID_Update(&PokeSpeedController,
        GUN_Data.pokeTargetSpeed, ENCODER_Data);
    if (DBUS_Status == kLost)
        GUN_SetFree();

#if POKE_DIR == 0
    if (GUN_Data.pokeOutput >= 0) {
        GPIO_SetBits(POKE_DIR_PORT, POKE_DIR_PIN);
        POKE_SET_PWM(GUN_Data.pokeOutput);
    }
    else {
        GPIO_ResetBits(POKE_DIR_PORT, POKE_DIR_PIN);
        POKE_SET_PWM(-GUN_Data.pokeOutput);
    }
#else
    if (GUN_Data.pokeOutput >= 0) {
        GPIO_ResetBits(POKE_DIR_PORT, POKE_DIR_PIN);
        POKE_SET_PWM(GUN_Data.pokeOutput);
    }
    else {
        GPIO_SetBits(POKE_DIR_PORT, POKE_DIR_PIN);
        POKE_SET_PWM(-GUN_Data.pokeOutput);
    }
#endif
}

void GUN_SetFree(void) {
    PID_Reset(&PokeSpeedController);
    PID_Reset(&PokeAngleController);

    GUN_Data.pokeOutput = 0;
    GUN_Data.pokeTargetSpeed = 0;
    GUN_Data.pokeTargetAngle = 0;
}
