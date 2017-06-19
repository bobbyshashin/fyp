#define CHASSIS_FILE

#include "Driver_ADIS16.h"
#include "Driver_Chassis.h"
#include "Driver_Dbus.h"
#include "Driver_Judge.h"
#include "param.h"
#include <string.h>

#define _ID(x) ((x)-CHASSIS_CAN_ID_OFFSET)

static CanTxMsg ChassisCanTx;

/* 1: normal 0: reverse */
static const char MOTOR_DIR[4] = {1, 0, 0, 1};

/* motor speed pid */
static volatile uint32_t MotorFeedbackCount[4];
static volatile int16_t MotorAngle[4], MotorLastAngle[4];
static volatile int32_t MotorVelocity[4], TargetVelocity[4];
static volatile int16_t MotorVelocityBuffer[4][2];
static volatile uint8_t BufferId[4];
static volatile char MeasureUpdated[4];
static volatile int16_t MotorOutput[4];
static PID_Controller MotorController[4];

/* chassis angle pid */
static PID_Controller ChassisOmegaController;
static volatile int32_t ChassisOmegaOutput;
static volatile int32_t targetOmega;

static PID_Controller ChassisPowerController;

#define _CLEAR(x) do { memset((void*)(x), 0, sizeof(x)); } while(0)

static int32_t CHASSIS_Trim(int32_t val, int32_t lim) {
    if (val > lim) val = lim;
    if (val < -lim) val = -lim;
    return val;
}

static int32_t CHASSIS_Clamp(int32_t val, int32_t min, int32_t max) {
    if (val < min) return min;
    else if (val > max) return max;
    else return val;
}

static void CHASSIS_ClearAll(void) {
    _CLEAR(MotorAngle);
    _CLEAR(MotorLastAngle);
    _CLEAR(MotorVelocity);
    _CLEAR(MotorVelocityBuffer);
    _CLEAR(BufferId);
    _CLEAR(TargetVelocity);
    _CLEAR(MotorOutput);
    _CLEAR(MeasureUpdated);
    ChassisOmegaOutput = 0;
    targetOmega = 0;
}

void CHASSIS_Init(void) {
    ChassisCanTx.StdId = CHASSIS_MASTER_ID;
    ChassisCanTx.IDE = CAN_Id_Standard;
    ChassisCanTx.RTR = CAN_RTR_Data;
    ChassisCanTx.DLC = 8;

    /* Motor controller */
    for (uint8_t id = 0; id < 4; ++id) {
        PID_Reset(MotorController+id);
        MotorController[id].Kp = CHASSIS_KP;
        MotorController[id].Ki = CHASSIS_KI;
        MotorController[id].Kd = CHASSIS_KD;
        MotorController[id].MAX_Pout = CHASSIS_MAX_POUT;
        MotorController[id].MAX_Integral = CHASSIS_MAX_INTEGRAL;
        MotorController[id].MAX_PIDout = CHASSIS_MAX_PIDOUT;
        MotorController[id].MIN_PIDout = CHASSIS_MIN_PIDOUT;
        MotorController[id].mode = CHASSIS_PID_MODE;
    }

    /* Chassis omega controller */
    PID_Reset(&ChassisOmegaController);
    ChassisOmegaController.Kp = CHASSIS_OMEGA_KP;
    ChassisOmegaController.Ki = CHASSIS_OMEGA_KI;
    ChassisOmegaController.Kd = CHASSIS_OMEGA_KD;
    ChassisOmegaController.MAX_Pout = CHASSIS_OMEGA_MAX_POUT;
    ChassisOmegaController.MAX_Integral = CHASSIS_OMEGA_MAX_INTEGRAL;
    ChassisOmegaController.MAX_PIDout = CHASSIS_OMEGA_MAX_PIDOUT;
    ChassisOmegaController.MIN_PIDout = CHASSIS_OMEGA_MIN_PIDOUT;
    ChassisOmegaController.mode = CHASSIS_OMEGA_PID_MODE;

    /*Chassis power controller*/
    /*PID_Reset(&ChassisPowerController);*/
    /*ChassisPowerController.Kp = CHASSIS_POWER_KP;*/
    /*ChassisPowerController.Ki = CHASSIS_POWER_KI;*/
    /*ChassisPowerController.Kd = CHASSIS_POWER_KD;*/
    /*ChassisPowerController.IDecayFactor = CHASSIS_POWER_IDECAY_FACTOR;*/
    /*ChassisPowerController.MAX_Pout = CHASSIS_POWER_MAX_POUT;*/
    /*ChassisPowerController.MAX_Integral = CHASSIS_POWER_MAX_INTEGRAL;*/
    /*ChassisPowerController.MAX_PIDout = CHASSIS_POWER_MAX_PIDOUT;*/
    /*ChassisPowerController.MIN_PIDout = CHASSIS_POWER_MIN_PIDOUT;*/
    /*ChassisPowerController.mode = CHASSIS_POWER_PID_MODE;*/
    /*ChassisPowerRatio = 1.0f;*/

    CHASSIS_ClearAll();
    _CLEAR(MotorFeedbackCount);
}

void CHASSIS_UpdateMeasure(uint16_t motorId, uint8_t *data) {
    uint16_t id = _ID(motorId);
    MotorLastAngle[id] = MotorAngle[id];
    MotorAngle[id] = ((uint16_t)data[0]<<8) | ((uint16_t)data[1]);
    int16_t newVelocity = ((uint16_t)data[2]<<8) | ((uint16_t)data[3]);
    ++MotorFeedbackCount[id];

    /* overflow protection */
    if (TargetVelocity[id] > 10 && newVelocity < -3000) newVelocity += 16384;
    else if (TargetVelocity[id] < -10 && newVelocity > 3000) newVelocity -= 16384;

    MotorVelocity[id] -= MotorVelocityBuffer[id][BufferId[id]];
    MotorVelocityBuffer[id][BufferId[id]] = newVelocity;
    MotorVelocity[id] += newVelocity;
    BufferId[id] = (BufferId[id]+1)&0x1U;

    MeasureUpdated[id] = 1;
}

void CHASSIS_MotorControl(uint16_t motorId) {
    uint16_t id = _ID(motorId);
    if (!MeasureUpdated[id]) return;
    MotorOutput[id] = (int16_t)PID_Update(MotorController+id,
        TargetVelocity[id], MotorVelocity[id]/2);
    
    MeasureUpdated[id] = 0;
}

void CHASSIS_Control(void) {
    CHASSIS_MotorControl(FL_MOTOR_ID);
    CHASSIS_MotorControl(FR_MOTOR_ID);
    CHASSIS_MotorControl(BR_MOTOR_ID);
    CHASSIS_MotorControl(BL_MOTOR_ID);
}

/*
    y
    ^
    |
    0-- >x
    Rotation: CW as +ve
*/
void CHASSIS_SetMotion(void) {
    static int32_t velocityX = 0, velocityY = 0;
    static int32_t tmpVelocity[4];

    static int16_t vxData, vyData, rotData;

    /*
        DBUS right switch
        kSwitchUp: all free
        kSwitchMiddle: controller debug
        kSwitchDown: automation
    */
    if (DBUS_Data.rightSwitchState == kSwitchMiddle) {
        vxData = DBUS_Data.ch1;
        vyData = DBUS_Data.ch2;
        rotData = DBUS_Data.ch3;
    }
    else if (DBUS_Data.rightSwitchState == kSwitchDown) {
    }
    else { // DBUS_Data.leftSwitchState == kSwitchUp
        vxData = 0;
        vyData = 0;
        rotData = 0;
    }

    velocityX = 18 * vxData;
    velocityY = 12 * vyData;

    /*
        DBUS left switch
        kSwitchUp: open loop
        kSwitchMiddle: omega close loop
    */
    if (DBUS_Data.leftSwitchState == kSwitchUp) {
        ChassisOmegaOutput = 5*rotData;
    }
    // else if (DBUS_Data.leftSwitchState == kSwitchMiddle) {
    /*else {*/
        /*if (DBUS_LastData.leftSwitchState != kSwitchMiddle)*/
            /*PID_Reset(&ChassisOmegaController);*/
        /*if (ADIS16_DataUpdated) {*/
            /*ADIS16_DataUpdated = 0;*/
            /*targetOmega = 4 * rotData;*/
            
            /*ChassisOmegaOutput = (int32_t)PID_Update(&ChassisOmegaController,*/
                /*targetOmega, ADIS16_Data.omega);*/
        /*}*/
    /*}*/

    /* Mecanum wheel */
    tmpVelocity[0] = velocityY + velocityX + ChassisOmegaOutput;
    tmpVelocity[1] = velocityY - velocityX - ChassisOmegaOutput;
    tmpVelocity[2] = velocityY + velocityX - ChassisOmegaOutput;
    tmpVelocity[3] = velocityY - velocityX + ChassisOmegaOutput;

    for (uint8_t i = 0; i < 4; ++i) {
        CHASSIS_SetTargetVelocity(i+CHASSIS_CAN_ID_OFFSET, tmpVelocity[i]);
    }
}

void CHASSIS_RotationControl(void) {
    ChassisOmegaOutput = (int32_t)PID_Update(&ChassisOmegaController,
        targetOmega, ADIS16_Data.omega);
}

void CHASSIS_PowerControl(void) {
    static float reducedRatio = 0.0f;
    reducedRatio = PID_Update(&ChassisPowerController,
        CHASSIS_ENERGY, JUDGE_Data.remainEnergy);
    if (reducedRatio < 0.0f)
        reducedRatio = 0.0f;
    ChassisPowerRatio = 1.0f - reducedRatio;
    // ChassisPowerRatio = 1.0f;
}

void CHASSIS_SetTargetVelocity(uint16_t motorId, int32_t velocity) {
    uint16_t id = _ID(motorId);
    velocity = CHASSIS_Trim(velocity, MAX_TARGET_VELOCITY);
    TargetVelocity[id] = (int16_t)velocity;
    if (!MOTOR_DIR[id]) TargetVelocity[id] = -TargetVelocity[id];
}

void CHASSIS_SendCmd(void) {
    static uint8_t *data = ChassisCanTx.Data;

#ifndef USE_SIMULATED_JUDGE
    data[0] = (MotorOutput[0]&0xFF00)>>8;
    data[1] = MotorOutput[0]&0x00FF;
    data[2] = (MotorOutput[1]&0xFF00)>>8;
    data[3] = MotorOutput[1]&0x00FF;
    data[4] = (MotorOutput[2]&0xFF00)>>8;
    data[5] = MotorOutput[2]&0x00FF;
    data[6] = (MotorOutput[3]&0xFF00)>>8;
    data[7] = MotorOutput[3]&0x00FF;
#else // USE_SIMULATED_JUDGE
    if (JUDGE_Data.remainLife == 0)
        memset((char*)data, 0, sizeof(data));
    else {
        data[0] = (MotorOutput[0]&0xFF00)>>8;
        data[1] = MotorOutput[0]&0x00FF;
        data[2] = (MotorOutput[1]&0xFF00)>>8;
        data[3] = MotorOutput[1]&0x00FF;
        data[4] = (MotorOutput[2]&0xFF00)>>8;
        data[5] = MotorOutput[2]&0x00FF;
        data[6] = (MotorOutput[3]&0xFF00)>>8;
        data[7] = MotorOutput[3]&0x00FF;
    }
#endif

    CAN_Transmit(CAN2, &ChassisCanTx);
}

void CHASSIS_SetFree(void) {
    for (uint8_t i = 0; i < 4; ++i)
        PID_Reset(MotorController+i);
    PID_Reset(&ChassisOmegaController);
    CHASSIS_ClearAll();
}
