#include "ppm_capure.h"
#include "esc.h"


HAL_GPIO_EXTI_Callback(uint16_t Pin)
{
     get_ppm_value(Pin);
}

void get_ppm_value(uint16_t pin)
{
    GPIO_TypeDef *GPIO_Port;
    uint8_t ch;
    switch (pin) {
    case GPIO_PIN_0:
        GPIO_Port=GPIOB;
        ch=2;
        break;
    case GPIO_PIN_1:
        GPIO_Port=GPIOB;
        ch=3;
        break;
    case GPIO_PIN_2:
        GPIO_Port=GPIOE;
        ch=8;
        break;
    case GPIO_PIN_3:
        GPIO_Port=GPIOE;
        ch=9;
        break;
    case GPIO_PIN_6:
        GPIO_Port=GPIOC;
        ch=0;
        break;
    case GPIO_PIN_7:
        GPIO_Port=GPIOC;
        ch=1;
        break;
    case GPIO_PIN_12:
        GPIO_Port=GPIOD;
        ch=4;
        break;
    case GPIO_PIN_13:
        GPIO_Port=GPIOD;
        ch=5;
        break;
    case GPIO_PIN_14:
        GPIO_Port=GPIOD;
        ch=6;
        break;
    case GPIO_PIN_15:
        GPIO_Port=GPIOD;
        ch=7;
        break;
    default:
        break;
    }
    if(!(HAL_GPIO_ReadPin(GPIO_Port,pin))&&(ppm[ch].already==1))
    {
        ppm[ch].count.stop=__HAL_TIM_GetCounter(&htim2);
        uint32_t value=(uint32_t)((ppm[ch].count.stop-ppm[ch].count.start));
        ppm[ch].already=0;
            if((value<2200)&&(value>800))
            {
                ppm[ch].last_value=ppm[ch].value;
                ppm[ch].value=value;
                if((ch==2)&&(value<1150))
                {
                    Esc[1].pulse=(uint16_t)125;
                    Esc[2].pulse=(uint16_t)125;
                    Esc[3].pulse=(uint16_t)125;
                    Esc[4].pulse=(uint16_t)125;
                }
            }

    }
        else
        {
            GPIO_PinState a=HAL_GPIO_ReadPin(GPIO_Port,pin);
            ppm[ch].count.start=__HAL_TIM_GetCounter(&htim2);
            ppm[ch].already=1;

        }

}

