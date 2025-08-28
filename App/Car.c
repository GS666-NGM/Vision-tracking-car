#include "Car.h"

#define CAR_START_STACK                 128
#define CAR_START_PRIORITY              1
TaskHandle_t car_start_handle;
void Car_Start(void* pv);

#define CONTROL_TASK_STACK              128*2
#define CONTROL_TASK_STACK_PRIORITY     4
TaskHandle_t control_task_handle;
void Control_Task(void* pv);

#define SHOW_TASK_STACK                 128*2
#define SHOW_TASK_STACK_PRIORITY        3
TaskHandle_t show_task_handle;
void Show_Task(void* pv);

void  Car_Init(void)
{
    xTaskCreate( (TaskFunction_t)  Car_Start,
                (char *) " Car_Start", 
                (configSTACK_DEPTH_TYPE)  CAR_START_STACK,
                (void *) NULL,
                (UBaseType_t)  CAR_START_PRIORITY,
                (TaskHandle_t *) & car_start_handle );
    vTaskStartScheduler();
}

void  Car_Start(void* pv)
{
    taskENTER_CRITICAL();
                
    xTaskCreate( (TaskFunction_t) Control_Task,
                (char *) "Control_Task", 
                (configSTACK_DEPTH_TYPE) CONTROL_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) CONTROL_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &control_task_handle );
                
    xTaskCreate( (TaskFunction_t) Show_Task,
                (char *) "Show_Task", 
                (configSTACK_DEPTH_TYPE) SHOW_TASK_STACK,
                (void *) NULL,
                (UBaseType_t) SHOW_TASK_STACK_PRIORITY,
                (TaskHandle_t *) &show_task_handle );
                
    vTaskDelete(NULL);
                
    taskEXIT_CRITICAL();
}
/*===========================================
上面是任务初始化，下面是执行的任务
//==============================================*/

char debugdata[5];
uint8_t debugflag;

uint8_t leftflag;
uint8_t rightflag;
uint8_t shiftflag;
uint8_t startflag;
uint8_t task2change;
uint8_t dirflag;

void Control_Task(void* pv)
{
//    TickType_t pxPreviousWakeTime = xTaskGetTickCount();
    while(1)
    {
        Vision_dealdata();
        Vision_Track();
//        xTaskDelayUntil(&pxPreviousWakeTime, 20);
         vTaskDelay(10);
    }
}

void Show_Task(void* pv)
{
 
    while(1)
    {
        key_scan();
//        k230_start();
        if(startflag == 1)
        {
            if(shiftflag == 1)
                task1();
            else if(shiftflag == 2)
                task2();
            else if(shiftflag == 3)
                task3();
        }
        lightup();
       vTaskDelay(20);
    }
}

void HAL_UART_AbortReceiveCpltCallback(UART_HandleTypeDef *huart)
{
   
     if(huart->Instance == USART1)
    {
        k230rxflag  = 1;
        memcpy(dealdata, k230rxdata, 15);
        memset(k230rxdata, 0, sizeof(k230rxdata));
        HAL_UART_Receive_IT(&huart1, (uint8_t*)k230rxdata, 30);
    }
}

void task1(void)
{
    
    StMotor_Returnzero(1);
    HAL_Delay(5);
    StMotor_Returnzero(2);
    HAL_Delay(5);
    char ans = 'f';
    HAL_UART_Transmit(&huart1, (uint8_t*)&ans, 1, 100);
    vision_track.trackflag = 1;
    startflag = 0;
}

void task2(void)
{
    
    StMotor_Returnzero(1);
    HAL_Delay(5);
//    StMotor_Returnzero(2);
//    HAL_Delay(5);
    char ans = 's';
    HAL_UART_Transmit(&huart1, (uint8_t*)&ans, 1, 100);
    HAL_Delay(50);
    vision_track.trackflag = 1;
    if(dirflag == 0)
    {
        leftflag =   0;
        SetMotor_SetSpeed(0x02, -9, 0);
    }

    if(dirflag == 1)
    {
        rightflag = 0;
        SetMotor_SetSpeed(0x02, 9, 0);
    }
    startflag = 0;
}

void task3(void)
{
    
}

uint8_t state1, laststate1 = 1;
uint8_t state2, laststate2 = 1;
uint8_t state3, laststate3 = 1;
void key_scan(void)
{
    state1 = HAL_GPIO_ReadPin(key1_GPIO_Port, key1_Pin);
    state2 = HAL_GPIO_ReadPin(key2_GPIO_Port, key2_Pin);
    state3 = HAL_GPIO_ReadPin(key3_GPIO_Port, key3_Pin);
    
    if(state1 == 1 && laststate1 == 0)
    {
        dirflag = 1;
    }
    if(state2 == 1 && laststate2 == 0)
    {
        startflag = 1;
    }
    if(state3 == 1 && laststate3 == 0)
    {
        shiftflag++;
    }

    laststate1 = state1;
    laststate2 = state2;
    laststate3 = state3;
}

void lightup(void)
{
    if(lightflag == 1)
        HAL_GPIO_WritePin(light_GPIO_Port, light_Pin, 0);
//    else
//        HAL_GPIO_WritePin(light_GPIO_Port, light_Pin, 1);
}

