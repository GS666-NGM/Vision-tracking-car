#ifndef __RESCUE_CAR_H
#define __RESCUE_CAR_H
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "stepper_motor.h"
#include "k230.h"

void Car_Init(void);
void task2(void);
void task1(void);
void task3(void);
void key_scan(void);
void lightup(void);

extern uint8_t debugflag;
extern char debugdata[5];
extern uint8_t leftflag;
extern uint8_t rightflag;
extern uint8_t shiftflag;
extern uint8_t lightflag;


#endif

