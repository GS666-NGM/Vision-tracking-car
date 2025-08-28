#include "stepper_motor.h"

char motordata[8];
uint8_t stmotor_rxflag;
char stopcom[5] = {0x00, 0xFE, 0X98, 0X00, 0X6B};
char clearlocation[4] = {0x00, 0X0A, 0X6D, 0X6B};

uint16_t step;

uint32_t pulse[2] = {0x00000000, 0x00000004};
uint8_t lightflag;

void StMotor_Returnzero(uint8_t add)
{
    uint8_t commend[5] = {0x00, 0X9A, 0X00, 0X00, 0X6B};
    commend[0] = add;
    HAL_UART_Transmit(&motor_usart, (uint8_t*)commend, 5, 100);
}

void StMotor_Setzero(uint8_t add)
{
    uint8_t commend[5] = {0X00, 0X93, 0X88, 0X01, 0X6B};
    commend[0] = add;
    HAL_UART_Transmit(&motor_usart, (uint8_t*)commend, 5, 100);
}

char commend[3] = {0x00, 0x36, 0x6B};
float STMotor_GetAngle(uint8_t add)
{
    char rxdata[8] = {0};
    uint32_t angle;
    commend[0] = add;
    HAL_UART_Transmit(&motor_usart, (uint8_t*)commend, 3, 100);
    HAL_UART_Receive(&motor_usart, (uint8_t*)rxdata, 8, 500);
    if(rxdata[0] == 0x6B)
    {
        angle = rxdata[4]<<24 | rxdata[5]<<16 | rxdata[6]<<8 | rxdata[7];
        if(rxdata[3] == 0x01)
            return (-360.0f*angle/65536);
        else if(rxdata[3] == 0x00)
            return (360.0*angle/65536);
        else
            return -3333;
    }
    else
    {
        angle = rxdata[3]<<24 | rxdata[4]<<16 | rxdata[5]<<8 | rxdata[6];
        if(rxdata[2] == 0x01)
            return (-360.0f*angle/65536);
        else if(rxdata[2] == 0x00)
            return (360.0*angle/65536);
        else
            return -3333;
    }
    
}


void SetMotor_SetSpeed(uint8_t add,  int16_t speed, uint8_t  acc)
{
    uint8_t commend[8] = {0x00, 0xF6, 0X00, 0X00, 0X00, 0X00, 0X00, 0X6B};
    commend[0] = add;
    if(speed < 0)
        commend[2] = 0x01;
    else
        commend[2] = 0x00;
    speed = abs(speed);
    commend[3] = speed>>8;
    commend[4] =  speed;
    commend[5] = acc;
    HAL_UART_Transmit(&motor_usart, commend, 8, 100);
}

void SetMotor_SetLocation1(uint8_t add, uint8_t mode, uint16_t speed, uint8_t  acc, float angle)
{
    uint8_t commend[13] = {0x00, 0xFD, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X6B};
    uint32_t pwm;
    commend[0] = add;
    if(angle < 0)
        commend[2] = 0x01;
    else
        commend[2] = 0x00;
    commend[3] = speed>>8;
    commend[4] =  speed;
    commend[5] = acc;
    pwm = fabs(angle)*mstep/1.8; 
    commend[6] = pwm>>24;
    commend[7] = pwm>>16;
    commend[8] = pwm>>8;
    commend[9] = pwm;
    commend[10] = mode;
    HAL_UART_Transmit(&motor_usart, commend, 13, 100);
}


void SetMotor_SetLocation2(uint8_t add, uint16_t speed, uint8_t  acc, int16_t pwm)
{
    uint8_t commend[13] = {0x00, 0xFD, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X00, 0X6B};
    commend[0] = add;
    if(pwm < 0)
        commend[2] = 0x01;
    else
        commend[2] = 0x00;
    commend[3] = speed>>8;
    commend[4] =  speed;
    commend[5] = acc;
    pwm = abs(pwm);
    commend[6] = pwm>>24;
    commend[7] = pwm>>16;
    commend[8] = pwm>>8;
    commend[9] = pwm;
    HAL_UART_Transmit(&motor_usart, commend, 13, 100);
}




void Stmotor_Active(void)
{
    HAL_GPIO_WritePin(enmotor1_GPIO_Port, enmotor1_Pin, (GPIO_PinState)1);
    HAL_GPIO_WritePin(dirmotor1_GPIO_Port, dirmotor1_Pin, (GPIO_PinState)1);
    __HAL_TIM_SET_COUNTER(&htim3, 0);
    __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4, 500);
    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);
}




vision_track_typedef vision_track;

void Vision_Track(void)
{
    if(vision_track.trackstart == 1)
    {
        if(fabs(vision_track.errory) <= 4)
        {
            SetMotor_SetLocation2(0x01, 50, 100, 0);
            if(vision_track.finishycnt < 3)
                vision_track.finishycnt++;
        }
        else
        {
            if(fabs(vision_track.errory) > 30)
              SetMotor_SetLocation2(0x01, 30, 100, 30*sign(vision_track.errory));
            else if(fabs(vision_track.errory) <= 30)
              SetMotor_SetLocation2(0x01, 5, 100, 5*sign(vision_track.errory));
            vision_track.finishycnt = 0;
        }
        
        HAL_Delay(2);
        
        if(fabs(vision_track.errorx) <= 4)
        {
            SetMotor_SetLocation2(0x02, 100, 100, 0);
            if(vision_track.finishxcnt < 3)
                vision_track.finishxcnt++;
        }
        else
        {
            if(fabs(vision_track.errorx) > 30)
              SetMotor_SetLocation2(0x02, 30, 100, 30*sign(vision_track.errorx));
            else if(fabs(vision_track.errorx) <= 30)
               SetMotor_SetLocation2(0x02, 5, 100, 5*sign(vision_track.errorx));
            vision_track.finishycnt = 0;
        }
        if(vision_track.finishycnt >= 1 && vision_track.finishxcnt >= 1 )
        {
            vision_track.finishycnt = 0;
            vision_track.finishxcnt = 0;
            lightflag = 1;
        }
    }
}




PID_ControllerTypeDef vision_pidx;
PID_ControllerTypeDef vision_pidy;

void PID_Init(PID_ControllerTypeDef *pid,float kp, float kd, float setpoint) {
    pid->Kp = kp;

    pid->Kd = kd;
    pid->setpoint = setpoint;

}

void Vision_PIDX(PID_ControllerTypeDef* pid, float x0)
{
    pid->error = pid->setpoint - x0;
//    if(fabs(pid->error) <= 2)
//    {
//        pid->activeflag = 0;
//        HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_4);
//        return;
//    }
    pid->output = pid->error*pid->Kp + (pid->error - pid->lastError)*pid->Kd;
    if(pid->output < -5000) pid->output = -5000;
    else if(pid->output > 5000) pid->output = 5000;
    pid->lastError = pid->error;
    pid->activeflag = 1;
    if(pid->output<0)
        HAL_GPIO_WritePin(dirmotor2_GPIO_Port, dirmotor2_Pin, (GPIO_PinState)0);
    else
        HAL_GPIO_WritePin(dirmotor2_GPIO_Port, dirmotor2_Pin, (GPIO_PinState)1);
    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_3);
    
}

void Vision_PIDY(PID_ControllerTypeDef* pid, float y0)
{
    pid->error = pid->setpoint - y0;
//    if(fabs(pid->error) <= 2)
//    {
//        pid->activeflag = 0;
//        HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_4);
//        return;
//    }
    pid->output = pid->error*pid->Kp + (pid->error - pid->lastError)*pid->Kd;
    if(pid->output < -5000) pid->output = -5000;
    else if(pid->output > 5000) pid->output = 5000;
    pid->lastError = pid->error;
    pid->activeflag = 1;
    if(pid->output<0)
        HAL_GPIO_WritePin(dirmotor2_GPIO_Port, dirmotor2_Pin, (GPIO_PinState)0);
    else
        HAL_GPIO_WritePin(dirmotor2_GPIO_Port, dirmotor2_Pin, (GPIO_PinState)1);
    HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_4);
}

int sign(float a)
{
    if(a < 0)
        return -1;
    else
        return 1;
}


