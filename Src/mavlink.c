#include "mavlink.h"
#include "usart.h"
#include "gpio.h"
#include "mavlink_user.h"
#include "mpu9250.h"
#include "ppm_capure.h"
#include "ahrs.h"
#include "ultrasonic.h"

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;


mavlink_system_t mavlink_system;
#define MAVLINK_USE_CONVENIENCE_FUNCTIONS


#include "mavlink_types.h"
#include "mavlink_helpers.h"
#include "esc.h"

// Define the system type, in this case an airplane
uint8_t system_type = MAV_TYPE_GENERIC;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight


int time,busy=0;
void test(void)
{
    time++;

    mavlink_message_t msg;
    uint8_t buf[64];
    msg.msgid = MAVLINK_MSG_ID_RAW_IMU;

//    mavlink_msg_raw_imu_send(1,time,Acc.x,Acc.y,Acc.z,Gyro.x,Gyro.y,Gyro.z,Mag.x,Mag.y,Mag.z);
//   mavlink_msg_raw_imu_pack(mavlink_system.sysid,mavlink_system.compid,&msg,time,100,101,102,103,104,105,106,107,108);
//   uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
// HAL_UART_Transmit(&huart2,buf,64,1000);
}

void heartbeat(void)
{
    mavlink_system.sysid = 42;                   ///< ID 20 for this airplane
     mavlink_system.compid = 200;
    /* The default UART header for your MCU */
//    #include "uart.h"
//    #include <mavlink/v1.0/common/mavlink.h>

    ///< The component sending the message is the IMU, it could be also a Linux process

    // Initialize the required buffers

    mavlink_message_t msg;
    uint8_t buf[128],test[6]={'a','b','c','d'};
    time++;
    // Pack the message
    mavlink_msg_heartbeat_pack(mavlink_system.sysid, mavlink_system.compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);

    // Copy the message to the send buffer
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);

    HAL_UART_Transmit(&huart1,buf,len,10);
//    vTaskDelay(1);
    mavlink_msg_raw_imu_pack(mavlink_system.sysid,mavlink_system.compid,&msg,time,Acc.x,Acc.y,Acc.z,Gyro.x,Gyro.y,Gyro.z,Mag.x,Mag.y,Mag.z);
     len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(&huart1,buf,len,1000);
    mavlink_msg_rc_channels_pack(mavlink_system.sysid,mavlink_system.compid,&msg,time,12,ppm[0].value,ppm[1].value,ppm[2].value,
            ppm[3].value,ppm[4].value,ppm[5].value,ppm[6].value,ppm[7].value,ppm[8].value,ppm[9].value,(uint16_t)(EulerAngle.pitch*57),1,2,3,Esc[4].pulse,Esc[3].pulse,Esc[2].pulse,Esc[1].pulse,95);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(&huart1,buf,len,1000);
    mavlink_msg_attitude_pack(mavlink_system.sysid,mavlink_system.compid,&msg,time,EulerAngle.roll,EulerAngle.pitch,EulerAngle.yaw,(float)gyro_count_sec,Gyro.offset.x,ultrasonic.speed);
    len = mavlink_msg_to_send_buffer(buf, &msg);
    HAL_UART_Transmit(&huart1,buf,len,1000);
//    mavlink_msg_altitude_pack(mavlink_system.sysid,mavlink_system.compid,&msg,time,)


//    mavlink_msg_attitude_pack()
//    mavlink_msg_raw_imu_pack();
//    mavlink_msg_optical_flow_pack()
}


 inline void comm_send_ch(mavlink_channel_t chan, uint8_t ch)
{
    if (chan == MAVLINK_COMM_1)
    {
        HAL_UART_Transmit(&huart2,&ch,1,10);
    }

}


