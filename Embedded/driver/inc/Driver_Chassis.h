#ifndef DRIVER_CHASSIS_H
#define DRIVER_CHASSIS_H

#include "stm32f4xx.h"
#include "pid.h"

#ifndef CHASSIS_FILE
    #define CHASSIS_EXT extern
#else
    #define CHASSIS_EXT
#endif


CHASSIS_EXT volatile float ChassisPowerRatio;

/*
    Initialize controllers and internal states.
*/
void CHASSIS_Init(void);

/*
    Update motor angle & velocity.
*/
void CHASSIS_UpdateMeasure(uint16_t motorId, uint8_t *data);

/*
    Update the controller if angle/velocity updated.
*/
void CHASSIS_MotorControl(uint16_t motorId);

/*
    Update 4 controllers for wheels and send CAN command.
*/
void CHASSIS_Control(void);

/*
    Update chassis angle controller.
*/
void CHASSIS_RotationControl(void);

/*
    Set target velocity for 4 motors.
*/
void CHASSIS_SetMotion(void);

/*
    Update velocity target.
*/
void CHASSIS_SetTargetVelocity(uint16_t motorId, int32_t velocity);

/*
    Send CAN cmd.
*/
void CHASSIS_SendCmd(void);

/*
    Send zero to all chassis motors.
*/
void CHASSIS_SetFree(void);

#endif
