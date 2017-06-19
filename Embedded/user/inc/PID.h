#ifndef PID_H
#define PID_H

typedef enum {
    kLLAST = 0,
    kLAST,
    kNOW,

    kIndexCnt
} PID_IndexTypeDef;

typedef enum {
    kIncremental,
    kPositional,
    kIntegralDecay
} PID_ModeTypeDef;

// should be negative
#define PID_NO_LIMIT -1.0f

typedef struct {
    /* set by user */
    float Kp, Ki, Kd;
    float IDecayFactor;
    float MAX_Integral, MAX_Pout, MAX_PIDout, MIN_PIDout;
    float MIN_Error;
    PID_ModeTypeDef mode;

    /* updated by calling PID_Update */
    float set[kIndexCnt];
    float real[kIndexCnt];
    float err[kIndexCnt], errIntegral;
    float output;
} PID_Controller;

void PID_Reset(PID_Controller *pid);
float PID_Update(PID_Controller *pid, float target, float measure);

#endif // PID_H
