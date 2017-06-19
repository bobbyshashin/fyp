#define GIMBAL_FILE

#include "Driver_Dbus.h"
#include "Driver_Gimbal.h"
#include "PID.h"
#include "param.h"
#include <string.h>

#define _ID(x) ((x)-GIMBAL_CAN_ID_OFFSET)

#define _CLEAR(x) do { memset((void*)(x), 0, sizeof(x)); } while(0)

static CanTxMsg GimbalCanTx;

/* 1: normal 0: reverse */
static PID_Controller GimbalVelController[GIMBAL_MOTOR_CNT];
static uint16_t MeasureUpdated[GIMBAL_MOTOR_CNT];
static volatile int16_t GimbalVelocityBuffer[4][2];
static volatile uint8_t BufferId[4];
static uint32_t FrameCount[GIMBAL_MOTOR_CNT];

// static int32_t GIMBAL_Trim(int32_t val, int32_t lim) {
//     if (val > lim) val = lim;
//     if (val < -lim) val = -lim;
//     return val;
// }

static void GIMBAL_ClearAll(void) {
    _CLEAR(GimbalOutput);
    _CLEAR(GimbalVelocity);
    _CLEAR(GimbalTargetVelocity);
}

void GIMBAL_Init(void) {
    GimbalCanTx.StdId = GIMBAL_MASTER_ID;
    GimbalCanTx.IDE = CAN_Id_Standard;
    GimbalCanTx.RTR = CAN_RTR_Data;
    GimbalCanTx.DLC = 8;

    GIMBAL_ClearAll();
    _CLEAR(GimbalRoundCount);
    _CLEAR(FrameCount);
    _CLEAR(GimbalPosOffset);

    for (uint8_t i = 0; i < GIMBAL_MOTOR_CNT; ++i)
        PID_Reset(GimbalVelController+i);
    GimbalVelController[YAW].Kp = 8.00f;
    GimbalVelController[YAW].Ki = 0.55f;
    GimbalVelController[YAW].Kd = 0.80f;
    GimbalVelController[YAW].MAX_Pout = 10000;
    GimbalVelController[YAW].MAX_Integral = 25000;
    GimbalVelController[YAW].MAX_PIDout = 15000;
    GimbalVelController[YAW].MIN_PIDout = 0;
    GimbalVelController[YAW].MIN_Error = 15;
    GimbalVelController[YAW].mode = kPositional;
}

void GIMBAL_UpdateMeasure(uint16_t motorId, uint8_t *data) {
    uint16_t id = _ID(motorId);
    ++FrameCount[id];

#if defined(GIMBAL_USE_6623)
    GimbalLastPosition[id] = GimbalPosition[id];
    GimbalPosition[id] = ((uint16_t)data[0]<<8)|((uint16_t)data[1]);
    GimbalRealCurrent[id] = ((uint16_t)data[2]<<8)|((uint16_t)data[3]);
    GimbalGivenCurrent[id] = ((uint16_t)data[4]<<8)|((uint16_t)data[5]);

    GimbalVelocity[id] = GimbalPosition[id] - GimbalLastPosition[id];
    if (GimbalVelocity[id] > 5000) {
        --GimbalRoundCount[id];
        GimbalPosition[id] -= 8192;
        GimbalVelocity[id] -= 8192;
    }
    else if (GimbalVelocity[id] < -5000) {
        ++GimbalRoundCount[id];
        GimbalPosition[id] += 8192;
        GimbalVelocity[id] += 8192;
    }
#elif defined(GIMBAL_USE_3510_19)
    GimbalLastPosition[id] = GimbalPosition[id];
    GimbalPosition[id] = (((uint16_t)data[0]<<8) | ((uint16_t)data[1])) + GimbalRoundCount[id] * 8192;
    int16_t newVelocity = ((uint16_t)data[2]<<8) | ((uint16_t)data[3]);
    if (FrameCount[id] > 256)
        GimbalPosition[id] -= GimbalPosOffset[id];

    /* overflow protection */
    if (GimbalTargetVelocity[id] > 10 && newVelocity < -3000) newVelocity += 16384;
    else if (GimbalTargetVelocity[id] < -10 && newVelocity > 3000) newVelocity -= 16384;

    GimbalVelocity[id] -= GimbalVelocityBuffer[id][BufferId[id]];
    GimbalVelocityBuffer[id][BufferId[id]] = newVelocity;
    GimbalVelocity[id] += newVelocity;
    BufferId[id] = (BufferId[id]+1)&0x1U;

    /* handle round count */
    if (FrameCount[id] > 256) { // ignore initial frames
        if (GimbalPosition[id] - GimbalLastPosition[id] < -6000) {
            GimbalRoundCount[id] += 1;
            GimbalPosition[id] += 8192;
        }
        else if (GimbalPosition[id] - GimbalLastPosition[id] > 6000) {
            GimbalRoundCount[id] -= 1 ;
            GimbalPosition[id] -= 8192;
        }
    }
    else {
        GimbalPosOffset[id] += GimbalPosition[id];
        if (FrameCount[id] == 256)
            GimbalPosOffset[id] /= 256;
    }
#endif

    MeasureUpdated[id] = 1;
}

void GIMBAL_SendCmd(void) {
    static uint8_t *data = GimbalCanTx.Data;
    data[0] = (GimbalOutput[0]&0xFF00)>>8;
    data[1] = GimbalOutput[0]&0x00FF;
    data[2] = (GimbalOutput[1]&0xFF00)>>8;
    data[3] = GimbalOutput[1]&0x00FF;
#if defined(GIMBAL_USE_6623)
    /* according to datasheet */
    data[5] = (GimbalOutput[2]&0xFF00)>>8;
    data[4] = GimbalOutput[2]&0x00FF;
    data[7] = (GimbalOutput[3]&0xFF00)>>8;
    data[6] = GimbalOutput[3]&0x00FF;
#elif defined(GIMBAL_USE_3510_19)
    data[4] = (GimbalOutput[2]&0xFF00)>>8;
    data[5] = GimbalOutput[2]&0x00FF;
    data[6] = (GimbalOutput[3]&0xFF00)>>8;
    data[7] = GimbalOutput[3]&0x00FF;
#endif

    CAN_Transmit(CAN1, &GimbalCanTx);
}

void GIMBAL_MotorControl(uint16_t motorId) {
    uint16_t id = _ID(motorId);
    if (!MeasureUpdated[id]) return;
    GimbalOutput[id] = (int16_t)PID_Update(GimbalVelController+id,
        GimbalTargetVelocity[id], GimbalVelocity[id]/2);

    MeasureUpdated[id] = 0;
}

void GIMBAL_Control(void) {
    GIMBAL_MotorControl(GIMBAL_YAW_ID);
    GIMBAL_MotorControl(GIMBAL_PITCH_ID);
}

/*
    +ve direction
    YAW: CW
    PITCH:
*/
void GIMBAL_SetMotion(void) {
    GimbalTargetVelocity[0] = -DBUS_Data.ch3*3;
}

void GIMBAL_SetFree(void) {
    for (uint8_t i = 0; i < GIMBAL_MOTOR_CNT; ++i)
        PID_Reset(GimbalVelController+i);
    GIMBAL_ClearAll();
}
