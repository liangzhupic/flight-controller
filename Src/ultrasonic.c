#include "ultrasonic.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "math.h"
#include "ahrs.h"
#include "tim.h"

//#define ultrasonicport huart3 //port of ultarsonic sensor
#define ultrasonicCaptureFreq 10 //frequence of capture distance of sensor
#define filter_count 10

BaseType_t ultrasonic_task;
TaskHandle_t ultrasonic_task_handle=NULL;

uint8_t ultrasonic_buff[2];
float ultrasonic_distance=0;
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
        HAL_UART_Transmit_DMA(&huart2,buff,1);
        HAL_UART_Receive_DMA(&huart2,ultrasonic_buff,2);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==USART2)
    {
        HAL_UART_Receive_DMA(&huart2,ultrasonic_buff,2);
    }
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance==USART2)
    {

        static float filter,T;
            ultrasonic.distance.raw=((ultrasonic_buff[0]<<8)+ultrasonic_buff[1])/1000.0f; //convert to unit meter
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
                ultrasonic.speed=(ultrasonic.distance.real-ultrasonic.distance.previous)/T;
                ultrasonic.distance.previous=ultrasonic.distance.real;
            }
            else//error for ultrasonic
            {

            }
    }
}


