/**
  ******************************************************************************
  * @file    math_rotation.h
  * @author  Leonardo Yu Yun
  * @version V1.1.0
  * @date    20-March-2014
  * @brief   This file contains all the functions prototypes for 
  *			 rotation operations
  *         
  ******************************************************************************  
  */ 


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MATH_ROTATION_H
#define __MATH_ROTATION_H


/* Includes ------------------------------------------------------------------*/
#include "math_basic.h"


/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/  



void euler_to_DCM(matrix3f DCM, float yaw, float pitch, float roll);
void euler_to_quat(vector4f q, float yaw, float pitch, float roll);


void quat_to_DCM(matrix3f DCM, const vector4f q);
void quat_to_DCM_col_1(vector3f DCM, const vector4f q);    //x
void quat_to_DCM_col_2(vector3f DCM, const vector4f q);    //y
void quat_to_DCM_col_3(vector3f DCM, const vector4f q);    //z
void quat_to_eular(float *yaw, float *pitch, float *roll, const vector4f q);

void DCM_to_quat(vector4f q, matrix3f DCM);
void DCM_to_euler(float *yaw, float *pitch, float *roll, matrix4f DCM);

void SORA_to_quat(vector4f q, float theta_x, float theta_y, float theta_z);
void quat_to_SORA(float * theta_x, float * theta_y, float * theta_z, const vector4f q);

void axis_angle_to_quat(vector4f q, vector3f axis, float angle);

void get_tilt_quaternion_n_angle(vector4f q_tilt, const vector4f q);
void quat_decomposition_tilt_n_torsion(vector4f q_tilt, vector4f q_torsion, const vector4f q);



#endif /*__MATH_ROTATION_H */


/************************ (C) COPYRIGHT DJI ********************END OF FILE****/

