#include "k230.h"

char k230rxdata[30];
uint8_t k230rxflag;
char dealdata[30];
uint8_t stopflag;

void Vision_dealdata(void)
{
    if(k230rxflag == 1)
    {
        k230rxflag = 0;
        
        if(vision_track.trackflag == 1)
        {
            if(dealdata[0] == 'X' && dealdata[8] == 'Y')
            {
                sscanf(dealdata, "X%6f,Y%6f", &vision_track.x0, &vision_track.y0);
                vision_track.errorx = vision_track.x0 - vision_track.xset;
                vision_track.errory = -vision_track.y0 + vision_track.yset;
                vision_track.trackstart= 1;
            }
        }
        if(dealdata[0] == 'f' && dealdata[1] =='i')
        {
            char ans[2] = "ok";
            SetMotor_SetSpeed(0x02, 0, 100);
            HAL_Delay(10);
            HAL_UART_Transmit(&huart1, (uint8_t*)ans, 2, 100);
//            vTaskDelay(10);
            
        }
    }
}

void k230_start(void)
{
    char data[6] = "start";
    HAL_UART_Transmit(&huart1, (uint8_t*)data, 5, 0x100);
}

void k230_stop(void)
{
    char data[4] = "stop";
    HAL_UART_Transmit(&huart1, (uint8_t*)data, 4, 0x100);
}

