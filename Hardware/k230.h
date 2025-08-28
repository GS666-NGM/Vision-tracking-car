#ifndef __K230_H
#define __K230_H

#include "usart.h"
#include "string.h"
#include  "math.h"
#include "stdlib.h"
#include "stepper_motor.h"
#include "FreeRTOS.h"
#include "task.h"

extern char k230rxdata[30];
extern uint8_t k230rxflag;
extern char dealdata[30];

void Vision_dealdata(void);
void k230_start(void);
void k230_stop(void);


#endif
