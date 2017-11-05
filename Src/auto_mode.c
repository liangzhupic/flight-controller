#include <ultrasonic.h>
#include <ppm_capure.h>
#include <auto_mode.h>
#include <FreeRTOS.h>
#include <task.h>
#include <usart.h>
#include <esc.h>

BaseType_t automode_t;
TaskHandle_t automode_handle=NULL;
BaseType_t data_reci_t;
TaskHandle_t data_reci_handle=NULL;

float auto_exp_height= 0;
int auto_mode = 0, auto_turnoff = 1;
int16_t auto_dx = 0, auto_dy = 0;
int16_t auto_dx_speed, auto_dy_speed;
int16_t dx_previous,dy_previous;
uint8_t rx_buff[10],heart_beat_c;
int found_target = 0;
int get_head = 0;
int counter = 0;
int mode_circle = 0;
int flag_moving = 0;
void automode_init(void)
{
    automode_t=xTaskCreate(automode_task,
                "automode",
                128,
                (void*)1 ,
                9,
                &automode_handle
                );
    if(automode_t =!pdPASS)
    {
        vTaskDelete(automode_handle);
    }

    data_reci_t=xTaskCreate(data_reci_task,
                "data_recive",
                128,
                (void*)1 ,
                9,
                &data_reci_handle
                );
    if(data_reci_t =!pdPASS)
    {
        vTaskDelete(data_reci_handle);
    }
}
void data_reci_task(void * p)
{
    uint8_t tx_buff[1]= {0x55};
    while(1)
    {
        if(mode_circle )
            tx_buff[0] = 0x55;
        else
            tx_buff[0] = 0x66;
        HAL_UART_Transmit(&huart3,  tx_buff, 1, 10);
//                vTaskDelay(20)
        if(get_head == 0)
            HAL_UART_Receive_IT(&huart3, rx_buff, 1);
        if(found_target)
            Esc[6].pulse = 20000;
        else
            Esc[6].pulse = 10;
        vTaskDelay(100);
    }
}

void automode_task(void *p)
{
    int counter = 0;
    while(1)
    {
    home:
        auto_turnoff = 1;
        auto_mode = 0;
        vTaskDelay(100);
        if(ppm[5].value> 1800 && ultrasonic.distance.real < 0.2 && ppm[3].value < 1200)
        {
            while(1)
            {
                auto_mode = 1;
                vTaskDelay(50);

                if(ppm[5].value < 1800)
                    goto home;  //  exit auto mode

                if(ppm[3].value < 1200)
                {
                    auto_turnoff = 1;
                }
                // auto take off
//                if( ultrasonic.distance.real < 0.2 && ppm[3].value > 1200 && ppm[3].value < 1800 )
//                {
//                    auto_exp_height = 1.1;
//                    auto_turnoff = 0;
//                }
                 //auto land mode
//                if( ultrasonic.distance.real > 0.2 && ppm[3].value > 1800 )
//                {

//                    auto_exp_height = 0;
//                    //  turn off motor when complete landing
//                    if(ultrasonic.distance.real < 0.3)
//                        counter++;
//                    else
//                        counter = 0;
//                    if(counter > 4)
//                        auto_turnoff = 1;
//                    else
//                        auto_turnoff = 0;
//                }
                if(ultrasonic.distance.real < 0.2 && ppm[6].value < 1500 )
                {
                    mode_circle = 1;
                    vTaskDelay(3*1000);
                    auto_exp_height = 1.1;
                    auto_turnoff = 0;
                    while (ultrasonic.distance.real > 0.9) {
                        vTaskDelay(100);    // wait for raising
                    }
                    vTaskDelay(20*1000);     // hover
                    auto_exp_height = 0;
                    //  turn off motor when complete landing
                    while(1)
                    {
                        if(ultrasonic.distance.real < 0.3)
                            counter++;
                        else
                            counter = 0;
                        if(counter > 4)
                        {
                            auto_turnoff = 1;
                            break;
                        }
                        else{
                            auto_turnoff = 0;
                        }
                        vTaskDelay(50);
                    }
//                     end
                    mode_circle = 0;
                }

                if(ultrasonic.distance.real < 0.2 && ppm[7].value < 1500 )
                {
                    mode_circle = 1;
                    vTaskDelay(3*1000);
                    auto_exp_height = 1.3;
                    auto_turnoff = 0;
                    while (ultrasonic.distance.real > 1.1) {
                        vTaskDelay(100);    // wait for raising
                    }
                    vTaskDelay(3*1000);
                    flag_moving = 1;
                    vTaskDelay(3*1000);
                    flag_moving =0;
                    mode_circle = 0;
                    vTaskDelay(15*1000);     // hover
                    auto_exp_height = 0;
                    //  turn off motor when complete landing
                    while(1)
                    {
                        if(ultrasonic.distance.real < 0.3)
                            counter++;
                        else
                            counter = 0;
                        if(counter > 4)
                        {
                            auto_turnoff = 1;
                            break;
                        }
                        else{
                            auto_turnoff = 0;
                        }
                        vTaskDelay(50);
                    }
//                     end
                    mode_circle = 0;
                }

                if(ultrasonic.distance.real < 0.2 && ppm[8].value < 1500 )
                {
                    mode_circle = 1;
                    vTaskDelay(3*1000);
                    auto_exp_height = 1.3;
                    auto_turnoff = 0;
                    while (ultrasonic.distance.real > 1.1) {
                        vTaskDelay(100);    // wait for raising
                    }
                    vTaskDelay(3*1000);
                    flag_moving = 1;
                    vTaskDelay(3*1000);
                    flag_moving =0;
                    mode_circle = 0;
                    vTaskDelay(75*1000);     // hover
                    auto_exp_height = 0;
                    //  turn off motor when complete landing
                    while(1)
                    {
                        if(ultrasonic.distance.real < 0.3)
                            counter++;
                        else
                            counter = 0;
                        if(counter > 4)
                        {
                            auto_turnoff = 1;
                            break;
                        }
                        else{
                            auto_turnoff = 0;
                        }
                        vTaskDelay(50);
                    }
//                     end
                    mode_circle = 0;
                }
            }
        }
    }
}

void on_board_callback(void)
{

    if(get_head){
        if( rx_buff[6] == 0xbb)
        {
            int16_t dx_raw = rx_buff[1] << 8 | rx_buff[0];
            int16_t dy_raw = rx_buff[3] << 8 | rx_buff[2];
            if(dx_raw < 640 && dx_raw > 0 && dy_raw < 480 && dy_raw > 0 )//&& rx_buff[4] != heart_beat_c)
            {
                auto_dx = auto_dx* 0.5 + 0.5* dx_raw;
                auto_dy = auto_dy* 0.5 + 0.5* dy_raw;

                auto_dx_speed = auto_dx_speed* 0.8 + 0.2 * (auto_dx - dx_previous);
                auto_dy_speed = auto_dy_speed* 0.8 + 0.2 * (auto_dy - dy_previous);

                dx_previous = auto_dx;
                dy_previous = auto_dy;
                if(rx_buff[4] == 0x55)
                {
                    found_target = 1;
//                    Esc[6].pulse = 10000;
                }
                else{
                    found_target = 0;
//                    Esc[6].pulse = 0;
                }
                heart_beat_c = rx_buff[5];

            }
            // update data

        }
        get_head = 0;
    }
    else
    {
        if(rx_buff[0] == 0xaa){
            get_head = 1;
            HAL_UART_Receive_IT(&huart3, rx_buff, 7);
        }
    }
}

