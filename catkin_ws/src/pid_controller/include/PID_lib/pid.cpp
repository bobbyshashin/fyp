#include "pid.h"
#include <stdio.h>
#include <iostream>
using namespace std;
long long counter = 0;

PID::PID(float _Kp, float _Ki, float _Kd,
         float _integralMin, float _integralMax,
         float _ctrlMin, float _ctrlMax,
         bool _is_debug)
{
    Kp = _Kp;
    Ki = _Ki;
    Kd = _Kd;

    is_debug = _is_debug;

    alpha = 0.5;
    error = 0.0;
    error_derivative = 0;
    old_error_derivative = 0;
    setPoint = 0.0;

    //TODO: change old error to be a buffer (length 8)
    bufferCount = 0;
    for (int i=0;i<8;i++)
        errorBuffer[i] = 0;
    integral = 0;

    integralMin = _integralMin;
    integralMax = _integralMax;

    ctrlMin = _ctrlMin;
    ctrlMax = _ctrlMax;
}

float PID::update(const float currentPoint,
                  const float dt,
                  float *d_error, float *d_pval, float *d_ival, float *d_dval)
{
  
    error = setPoint - currentPoint;
    
    for (int i=7; i>0; i--)
    {
        errorBuffer[i] =  errorBuffer[i-1];
    }

    errorBuffer[0] = error;
    bufferCount++;
    if (bufferCount >= 8)
        bufferCount = 8;

    // p contrlller ------------------------------------------------------
    PVal = error * Kp;

    // d contrlller ------------------------------------------------------
    //if (bufferCount == 1)
    //    error_derivative = 0;
    //else if (bufferCount < 8)
    //    error_derivative = errorBuffer[0] - errorBuffer[1];
    //else
    //{

    // normal
    //error_derivative = errorBuffer[0] - errorBuffer[2];

    error_derivative = errorBuffer[0] + errorBuffer[1]
                      - errorBuffer[2] - errorBuffer[3];
    error_derivative /= 4.0f;

    // damp
    //error_derivative = errorBuffer[0] + errorBuffer[1] + errorBuffer[2] + errorBuffer[3]
    //                 - errorBuffer[4] - errorBuffer[5] - errorBuffer[6] - errorBuffer[7];
    //error_derivative /= 16.0f;
    //}

    // d contrlller
    DVal = (1.0f-alpha)*error_derivative + alpha*old_error_derivative;
    old_error_derivative = DVal;

    DVal = DVal / dt * Kd;

    // i contrlller ------------------------------------------------------
    integral += error * dt;
    if (integral > integralMax)
        integral = integralMax;
    if (integral < integralMin)
        integral = integralMin;
    IVal = integral * Ki;


    // PID controller
    ctrl = PVal + IVal + DVal;

    if (ctrl > ctrlMax)
        ctrl = ctrlMax;
    if (ctrl < ctrlMin)
        ctrl = ctrlMin;

    if(is_debug == true)
    {
        *d_error = error;
        *d_pval = PVal;
        *d_ival = IVal;
        *d_dval = DVal;
    }

    return ctrl;
}


float PID::update(const float currentPoint, const float dt)
{
    error  = setPoint - currentPoint;
/*
    if( counter%2 == 0 ) {
        cout << "X-axis Position Error: " << error << "    ";
        
    }
    else {
        cout << "Y-axis Position Error: " << error << endl;
    }
    counter++;
    //cout << "error: " << error << endl;
*/
    for (int i=7; i>0; i--)
    {
        errorBuffer[i] =  errorBuffer[i-1];
    }

    errorBuffer[0] = error;
    bufferCount++;
    if (bufferCount >= 8)
        bufferCount = 8;

    // p contrlller ------------------------------------------------------
    PVal = error * Kp;

    // d contrlller ------------------------------------------------------
    //if (bufferCount == 1)
    //    error_derivative = 0;
    //else if (bufferCount < 8)
    //    error_derivative = errorBuffer[0] - errorBuffer[1];
    //else
    //{

    // normal
    //error_derivative = errorBuffer[0] - errorBuffer[2];

    error_derivative = errorBuffer[0] + errorBuffer[1]
                      - errorBuffer[2] - errorBuffer[3];
    error_derivative /= 4.0f;

    // damp
    //error_derivative = errorBuffer[0] + errorBuffer[1] + errorBuffer[2] + errorBuffer[3]
    //                 - errorBuffer[4] - errorBuffer[5] - errorBuffer[6] - errorBuffer[7];
    //error_derivative /= 16.0f;
    //}

    // d contrlller
    DVal = (1.0f-alpha)*error_derivative + alpha*old_error_derivative;
    old_error_derivative = DVal;

    DVal = DVal / dt * Kd;

    // i contrlller ------------------------------------------------------
    integral += error * dt;
    if (integral > integralMax)
        integral = integralMax;
    if (integral < integralMin)
        integral = integralMin;
    IVal = integral * Ki;

    // PID controller
    ctrl = PVal + IVal + DVal;

    if (ctrl > ctrlMax)
        ctrl = ctrlMax;
    if (ctrl < ctrlMin)
        ctrl = ctrlMin;

    return ctrl;
}

void PID::set_point(float _setPoint) {

    setPoint = _setPoint;
}

void PID::set_param(float _Kp, float _Ki, float _Kd) {

    set_Kp(_Kp);
    set_Ki(_Ki);
    set_Kd(_Kd);


}
void PID::reset() {

    integral = 0;
    bufferCount = 0;
    for (int i=0;i<8;i++)
        errorBuffer[i] = 0;
    error_derivative =0;
    old_error_derivative = 0;
}

void PID::reduceIntegral(float coeffi) {

    integral *= coeffi;
}
