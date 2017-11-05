#ifndef __esc_h
#define __esc_h
#include "tim.h"


typedef struct Esc
{
     uint16_t pulse;
     uint16_t max;
     uint16_t min;

}Esc_t;


extern Esc_t Esc[9];//note that motor1 point to esc[1]

void Esc_Callback_TIM8(TIM_HandleTypeDef *htim);
void Esc_Callback_TIM5(TIM_HandleTypeDef *htim);
void Esc_Callback_TIM1(TIM_HandleTypeDef *htim);
void SetEsc(Esc_t *esc,uint16_t pulse);
void InitEsc(Esc_t *,uint16_t,uint16_t,uint16_t);
void Motor_Setup(void);
void EscLimit(Esc_t *);

#endif
