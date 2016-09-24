/**
******************************************************************************
* @file    user.h
* @author  FrankChen
* @version V0.1.0
* @date    22-May-2016
* @brief   This file provides .h file to suppose obtain_3D_pose.cpp
*
******************************************************************************
*/

typedef struct
{
    float r11,r12,r13,
          r21,r22,r23,
          r31,r32,r33;
}bit_rm;

typedef struct
{
    float r1,r2,r3;
}bit_rvec;

typedef struct
{
    float u;
    float v;
    float reliable;
    float x_w;
    float y_w;
    float z_w;

}bit_position;


typedef struct
{
    bit_position pos;
    bit_rvec     rvec;
    bit_rm       rm;
}bit_pose;










