#ifndef PARAM_H
#define PARAM_H

/* Board type select */
#define BOARD_TYPE_CONTROL                      0
#define BOARD_TYPE_JUDGE                        1
#define BOARD_TYPE                              BOARD_TYPE_CONTROL
// #define USE_SIMULATED_JUDGE

/* CHASSIS param */
#define CHASSIS_MAX_POWER                       80
#define CHASSIS_ENERGY                          60
#define CHASSIS_USE_3510_19

#if defined(CHASSIS_USE_3510_19)
/* CAN ID */
#define CHASSIS_MASTER_ID                       0x200U
#define CHASSIS_CAN_ID_OFFSET                   0x201U
#define FL_MOTOR_ID                             0x201U
#define FR_MOTOR_ID                             0x202U
#define BR_MOTOR_ID                             0x203U
#define BL_MOTOR_ID                             0x204U

#define MAX_TARGET_VELOCITY                     8200

/* 3510 motor PID parameter */
#define CHASSIS_KP                              5.20f
#define CHASSIS_KI                              0.40f
#define CHASSIS_KD                              0.08f
#define CHASSIS_MAX_POUT                        10000
#define CHASSIS_MAX_INTEGRAL                    25000
#define CHASSIS_MAX_PIDOUT                      15000
#define CHASSIS_MIN_PIDOUT                      0
#define CHASSIS_PID_MODE                        kPositional

/* chassis angle control parameter */
#define CHASSIS_OMEGA_KP                        0.20f
#define CHASSIS_OMEGA_KI                        0.12f
#define CHASSIS_OMEGA_KD                        0.00f
#define CHASSIS_OMEGA_MAX_POUT                  5000
#define CHASSIS_OMEGA_MAX_INTEGRAL              35000
#define CHASSIS_OMEGA_MAX_PIDOUT                5300
#define CHASSIS_OMEGA_MIN_PIDOUT                5
#define CHASSIS_OMEGA_PID_MODE                  kPositional

/* chassis power control parameter */
#define CHASSIS_POWER_KP                        0.005f
#define CHASSIS_POWER_KI                        0.006f
#define CHASSIS_POWER_KD                        0.000f
#define CHASSIS_POWER_IDECAY_FACTOR             0.7f
#define CHASSIS_POWER_MAX_POUT                  100
#define CHASSIS_POWER_MAX_INTEGRAL              100
#define CHASSIS_POWER_MAX_PIDOUT                0.7
#define CHASSIS_POWER_MIN_PIDOUT                0
#define CHASSIS_POWER_PID_MODE                  kIntegralDecay

#elif defined(CHASSIS_USE_EC60)

#elif defined(CHASSIS_USE_RM35)

#endif // chassis motor parameter

/* GIMBAL param */
#define GIMBAL_USE_3510_19

#if defined(GIMBAL_USE_6623)
#define GIMBAL_MASTER_ID                        0x1FFU
#define GIMBAL_CALIB_ID                         0x3F0U
#define GIMBAL_MOTOR_CNT                        4
#define GIMBAL_CAN_ID_OFFSET                    0x205U
#define GIMBAL_YAW_ID                           0x205U
#define GIMBAL_PITCH_ID                         0x206U
#define GIMBAL_ROLL_ID                          0x207U
#define GIMBAL_RESV_ID                          0x208U
#define YAW                                     (GIMBAL_YAW_ID-GIMBAL_CAN_ID_OFFSET)
#define PITCH                                   (GIMBAL_PITCH_ID-GIMBAL_CAN_ID_OFFSET)
#define ROLL                                    (GIMBAL_ROLL_ID-GIMBAL_CAN_ID_OFFSET)
#define GIMBAL_ReSV                             (GIMBAL_RESV_ID-GIMBAL_CAN_ID_OFFSET)

#elif defined(GIMBAL_USE_3510_19)
/* CAN ID */
#define GIMBAL_MASTER_ID                        0x200U
#define GIMBAL_CAN_ID_OFFSET                    0x201U
#define GIMBAL_YAW_ID                           0x201U
#define GIMBAL_PITCH_ID                         0x202U
#define YAW                                     (GIMBAL_YAW_ID-GIMBAL_CAN_ID_OFFSET)
#define PITCH                                   (GIMBAL_PITCH_ID-GIMBAL_CAN_ID_OFFSET)

#endif

#endif // PARAM_H
