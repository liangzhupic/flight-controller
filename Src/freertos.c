/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  *
  * Copyright (c) 2016 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     

#include "gpio.h"
#include "spi.h"
#include "mpu9250.h"
#include "mavlink.h"
#include "ahrs.h"
#include "semphr.h"
#include "tim.h"
#include "mavlink_user.h"
#include <auto_mode.h>

//#include "mavlink_user.h"

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;

/* USER CODE BEGIN Variables */

BaseType_t LED_Blinky_t;
TaskHandle_t LED_Blinky_handle=NULL;

BaseType_t InitHardware;
TaskHandle_t InitHardware_handle=NULL;
/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);

extern void MX_USB_DEVICE_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

void LED_Blinky(void* p);//on board led blinky
void Init_Hardware(void *p);//setup hardware

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
    AhrsTaskCreate();         //creater task of attitude e
    mpu9250_task_create();
    UltraSonicTaskCreate();
    automode_init();

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  LED_Blinky_t=xTaskCreate(LED_Blinky,
              "led_blinky",
              512,
              (void*)1 ,
              5,
              &LED_Blinky_handle
              );
  if(LED_Blinky_t =!pdPASS)
  {
      vTaskDelete(LED_Blinky_handle);
  }


  LED_Blinky_t=xTaskCreate(Init_Hardware,
              "init hardware",
              512,
              (void*)1 ,
              20,
              &InitHardware_handle
              );
  if(LED_Blinky_t =!pdPASS)
  {
      vTaskDelete(LED_Blinky_handle);
  }

  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {


  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Application */
void Init_Hardware(void *p)
{
  /* */

    __HAL_TIM_SET_COUNTER ( &htim2, 0);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_1);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_2);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_3);
    HAL_TIM_OC_Start_IT(&htim2, TIM_CHANNEL_4);

    HAL_TIM_Base_Start_IT(&htim6); //2khz event

    HAL_Delay(50);
    InitMpu9250();


    Gyro.offset.x=-0.913154;
    Gyro.offset.y=-0.004481;
    Gyro.offset.z=0.343141;

    Gyro.offset.x=0;
    Gyro.offset.y=0;
    Gyro.offset.z=0;
    HAL_Delay(6000);

    for(int a=1;a<=500;a++)
    {
    ReadMpuAllBloack();
    Gyro.offset.x+=(float)Gyro.x;
    Gyro.offset.y+=(float)Gyro.y;
    Gyro.offset.z+=(float)Gyro.z;
    HAL_Delay(2);
    }

    Gyro.offset.x/=500;
    Gyro.offset.y/=500;
    Gyro.offset.z/=500;

 /*   AhrsTaskCreate();         //creater task of attitude e
    mpu9250_task_create();*/

    vTaskDelete(NULL);

}

void LED_Blinky(void* p)
{

    while(1)
    {

         HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_5);
//         if(1==mpu_read_who_am_i())
//         {
           HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);
//         }

       static TimeCounter_t t;
       TimeCounter(&t,1);
       vTaskDelay(100);
       TimeCounter(&t,2);
//        HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);

       heartbeat();  //mavlink


    }
}

void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   signed char *pcTaskName)
{
    while(1)
    {
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_4,RESET);
        HAL_GPIO_WritePin(GPIOE,GPIO_PIN_5,RESET);
        HAL_Delay(100);
        HAL_GPIO_TogglePin(GPIOE,GPIO_PIN_6);
    }
}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
