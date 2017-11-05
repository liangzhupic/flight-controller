

#include "esc.h"
#include "tim.h"
#include "ppm_capure.h"

/*
 * X shape quad
 *
 *  1   2
 *   \ /
 *    #
 *   / \
 *  4   3
 *
 */
Esc_t Esc[9];//note that motor1 point to esc[1]

void Motor_Setup(void)
{
    int a;
    for(a=1;a<=9;a++)
    {
        InitEsc(&Esc[a],1000,2000,1000);
    }
    __HAL_TIM_SetCounter(&htim5,0);
    __HAL_TIM_SetCounter(&htim8,0);
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_3,1000);
    __HAL_TIM_SetCompare(&htim8,TIM_CHANNEL_4,1000);
    __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_3,1000);
    __HAL_TIM_SetCompare(&htim5,TIM_CHANNEL_4,1000);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_1,1000);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_2,1000);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_3,1000);
    __HAL_TIM_SetCompare(&htim1,TIM_CHANNEL_4,1000);
    HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim5,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start_IT(&htim8,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim8,TIM_CHANNEL_4);
    HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_1);
    HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_2);
    HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_3);
    HAL_TIM_PWM_Start_IT(&htim1,TIM_CHANNEL_4);
}

void InitEsc(Esc_t *esc, uint16_t pulse, uint16_t max,uint16_t min)
{
    esc->pulse=pulse;
    esc->max=max;
    esc->min=min;
}

void SetEsc(Esc_t *esc, uint16_t pulse)
{
   esc->pulse=pulse;
}

void EscLimit(Esc_t *esc)
{
    if(esc->pulse>esc->max)
    {
        esc->pulse=esc->max;
    }
    if(esc->pulse<esc->min)
    {
        esc->pulse=esc->min;
    }
}

void Esc_Callback_TIM5(TIM_HandleTypeDef *htim)
{
    if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3)
    {
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, Esc[3].pulse);
    }
    if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4)
    {
      __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_4, Esc[4].pulse);
    }
}


void Esc_Callback_TIM8(TIM_HandleTypeDef *htim)
{
    if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3)
    {
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, Esc[1].pulse);
    }
    if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4)
    {
      __HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_4, Esc[2].pulse);
    }
}

void Esc_Callback_TIM1(TIM_HandleTypeDef *htim)
{
    if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_1)
    {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Esc[5].pulse);
    }
    if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_2)
    {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Esc[6].pulse);
    }
    if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_3)
    {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, Esc[7].pulse);
    }
    if(htim->Channel==HAL_TIM_ACTIVE_CHANNEL_4)
    {
      __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Esc[8].pulse);
    }
}



