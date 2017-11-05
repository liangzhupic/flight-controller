#include "ultrasonic.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "math.h"
#include "ahrs.h"
#include "tim.h"
#include <auto_mode.h>

//#define ultrasonicport huart3 //port of ultarsonic sensor
#define ultrasonicCaptureFreq 20 //frequence of capture distance of sensor
#define filter_count 8

#define us100
//#define ME007Y
BaseType_t ultrasonic_task;
TaskHandle_t ultrasonic_task_handle=NULL;

uint8_t ultrasonic_buff[5];
float ultrasonic_distance=0;
//int get_head = 0;
/********************************
 * Date : 2017/7/20
 * brief: Moving_Median
 *
********************************/
//=============================================================
#define MED_WIDTH_NUM 11
#define MED_FIL_ITEM  2

float med_filter_tmp[MED_FIL_ITEM][MED_WIDTH_NUM];
float med_filter_out[MED_FIL_ITEM];

unsigned char med_fil_cnt[MED_FIL_ITEM];
float Moving_Median(int item, int width_num,float in)
{
    float t;
    float tmp[MED_WIDTH_NUM];

    if(item>=MED_FIL_ITEM||width_num>=MED_WIDTH_NUM)
    {
        return 0;
    }

    else
    {
        if(++med_fil_cnt[item]>=width_num)
        {
            med_fil_cnt[item]=0;
        }

        med_filter_tmp[item][med_fil_cnt[item]]=in;

        for(int i=0;i<width_num;i++)
        {
            tmp[i]=med_filter_tmp[item][i];
        }

        //Bubble Sort
        for(int i=0;i<width_num-1;i++)
        {
            for(int j=0;(j<width_num-1-i);j++)
            {
                if(tmp[j]>tmp[j+1])
                {
                    t=tmp[j];
                    tmp[j]=tmp[j+1];
                    tmp[j+1]=t;
                }
            }
        }

        return ( tmp[(unsigned short int)width_num/2] );
    }
}
//=============================================================
void UltraSonicTaskCreate()
{

    ultrasonic_task=xTaskCreate(UltraSonicPoll,
                "ultra sonic",
                512,
                (void*)1 ,
                10,
                &ultrasonic_task_handle
                );
    if(ultrasonic_task ==!pdPASS)
    {
        vTaskDelete(ultrasonic_task_handle);
    }
}

void UltraSonicPoll()
{
    uint8_t buff[]={0x55};
    while(1)
       {
        vTaskDelay((unsigned int)(1000.0f/ultrasonicCaptureFreq));//33.33ms 30hz
#ifdef us100
        HAL_UART_Transmit_DMA(&huart2,buff,1);
        HAL_UART_Receive_DMA(&huart2,ultrasonic_buff,2);
#endif

#ifdef ME007Y
//        HAL_UART_Transmit_DMA(&huart2,buff,1);
        if(get_head)
            HAL_UART_Receive_DMA(&huart2,ultrasonic_buff,3);
        else
            HAL_UART_Receive_DMA(&huart2,ultrasonic_buff,1);
#endif
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
#ifdef us100
    if(huart->Instance==USART2)
    {
        HAL_UART_Receive_DMA(&huart2,ultrasonic_buff,2);
    }
#endif

}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==USART2)
    {
        static int counter=0;
        static float filter,T;
#ifdef ME007Y
        if(get_head)
        {
            ultrasonic.distance.raw=((ultrasonic_buff[0]<<8)+ultrasonic_buff[1])/1000.0f;
            get_head = 0;
        }
        if(ultrasonic_buff[0] == 0xff && get_head == 0)
        {
            get_head = 1;
//            HAL_UART_Receive_DMA(&huart2,ultrasonic_buff,3);
        }

#endif

#ifdef us100
            ultrasonic.distance.raw=((ultrasonic_buff[0]<<8)+ultrasonic_buff[1])/1000.0f; //convert to unit meter
#endif

            if ((ultrasonic.distance.raw<=3)&&(ultrasonic.distance.raw>=0.001)) //range 0.01m to 3.00m
            {
                filter-=filter/filter_count;
                filter+=ultrasonic.distance.raw/filter_count;

                ultrasonic.distance.real=ultrasonic.distance.raw*cos(EulerAngle.pitch)*cos(EulerAngle.roll);//transform the

                ultrasonic.time.now=__HAL_TIM_GetCounter(&htim2);
                    if(ultrasonic.time.previous>ultrasonic.time.now)
                    {
                        ultrasonic.time.interval=ultrasonic.time.now+(0xffffffff-ultrasonic.time.previous);
                    }
                ultrasonic.time.interval=ultrasonic.time.now-ultrasonic.time.previous;
                ultrasonic.time.previous=ultrasonic.time.now;
                float T=((float)ultrasonic.time.interval)/1000000; //convert to unit second
//                ultrasonic.speed=(ultrasonic.distance.real-ultrasonic.distance.previous)/T;
                ultrasonic.speed=Moving_Median(0,10,(ultrasonic.distance.real-ultrasonic.distance.previous)/T);
                if(ultrasonic.distance.real-ultrasonic.distance.previous>0.6 || ultrasonic.distance.real-ultrasonic.distance.previous< -0.6){
                    counter++;
                    if(counter<3){
                        ultrasonic.distance.real=ultrasonic.distance.previous;
                    }
                }
                else{
                    counter = 0;
                }


                ultrasonic.distance.previous=ultrasonic.distance.real;

            }
            else//error for ultrasonic
            {

            }
    }

    if(huart->Instance == USART3)
    {
        on_board_callback();
    }
}


