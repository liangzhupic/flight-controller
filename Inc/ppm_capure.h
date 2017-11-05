#ifndef ppm_capture_h
#define ppm_capture_h

/******************include file **********/

#include "FreeRTOS.h"
#include "task.h"
#include "mavlink.h"
#include "gpio.h"
#include "tim.h"

/**************variables declared****/
/*uint16_t ppm_map2_io[15][2]=
{
    {GPIOC, GPIO_PIN_6},   //this is ppm1
    {GPIOC, GPIO_PIN_7},
    {GPIOB, GPIO_PIN_0},
    {GPIOB, GPIO_PIN_1},
    {GPIOD, GPIO_PIN_12},
    {GPIOD, GPIO_PIN_13},
    {GPIOD, GPIO_PIN_14},
    {GPIOD, GPIO_PIN_15},
    {GPIOE, GPIO_PIN_2},
    {GPIOE, GPIO_PIN_3},
};*/
typedef struct ppm_struct
{
    uint16_t Pin;
    uint32_t value;
    uint32_t last_value;
    int8_t already;
    struct counter
    {
        uint32_t start;
        uint32_t stop;
    }count;

}ppm_struct;
ppm_struct ppm[10];

/*************function declaered**********/

void get_ppm_value(uint16_t );








#endif
