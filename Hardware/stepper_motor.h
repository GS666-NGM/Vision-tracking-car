#ifndef __STEPPER_MOTOR_H
#define __STEPPER_MOTOR_H

#include "usart.h"
#include "gpio.h"
#include "string.h"
#include "tim.h"
#include "stdlib.h"
#include "arm_math.h"
#include "arm_const_structs.h"
#include "math.h"

#define mstep   180
#define motorx    1
#define motory    0
#define CW        0
#define CCW       1
#define left      0
#define right     1
#define up        1
#define down      0
#define d         13.71
#define l         50.0f
#define change    57.297f
#define wide      20
//左-右+    上+下-   || -1  +0

typedef struct vision_track_typedef
{
    uint8_t trackstart;
    uint8_t trackflag;
    uint8_t finishxcnt;
    uint8_t finishycnt;
    float xset;
    float yset;
    float x0;
    float y0;
    float errorx;
    float errory;
}vision_track_typedef;

typedef struct calibertion_typedef
{
    uint8_t clbflag;
    uint8_t startflag;
    float a1, a2, dela;
}calibertion_typedef;

typedef struct{
    float Kp;
    float Kd;
    float setpoint;
    float lastError;
    float lastLastError;
    int16_t output;
    float error;
    uint8_t activeflag;

}PID_ControllerTypeDef;

extern char motordata[8];
extern uint8_t stmotor_rxflag;
extern char errordata[4];
extern char enablecom[6];
extern char stopcom[5];
extern char clearlocatin[4];
extern uint16_t step;
extern uint8_t finishflag;
extern vision_track_typedef vision_track;
extern PID_ControllerTypeDef vision_pidx;
extern PID_ControllerTypeDef vision_pidy;

void SetMotor_SetSpeed(uint8_t add,  int16_t speed, uint8_t  acc);
void StMotor_Returnzero(uint8_t add);
void StMotor_Setzero(uint8_t add);
void Stmotor_Active(void);
void SetMotor_SetLocation2(uint8_t add, uint16_t speed, uint8_t  acc, int16_t pwm);
void SetMotor_SetLocation1(uint8_t add, uint8_t mode, uint16_t speed, uint8_t  acc, float angle);
void Vision_Track(void);
int sign(float a);
//void Colibration(void);
float STMotor_GetAngle(uint8_t add);

    
#endif 

