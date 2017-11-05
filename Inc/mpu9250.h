#ifndef mpu9250_H
#define mpu9250_H

/*******include file*************/
#include "spi.h"
#include "FreeRTOS.h"
#include "gpio.h"
#include "task.h"
#include "semphr.h"


/******* extern include ******/
SemaphoreHandle_t GetGyro,GetAcc,GetMag,spi_res;

extern SPI_HandleTypeDef hspi1;

extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

extern BaseType_t acc_read_t,gyro_read_t,mag_read_t;
extern TaskHandle_t acc_read_h,gyro_read_h,mag_read_h;

void mpu9250_task_create(void);
void ACC_Read(void *p);
void GYRO_Read(void *p);
void MAG_Read(void *p);
int  mpu_read_who_am_i(void);
void mpu_write_block( uint8_t, uint8_t,uint8_t );
void mpu_read_block(uint8_t reg, uint8_t *data,uint8_t len);
void mpu_akm_read(uint8_t reg, uint8_t *data, uint8_t len);
void mpu_akm_write(uint8_t reg, uint8_t data);
int  mpu_akm_ready(void);
void InitMpu9250(void);
void ReadMpuAllBloack(void);
void mpu_configure(uint8_t reg,uint8_t data);
void exponential_moving_average(float* avg,float cur,uint16_t count );

 typedef struct axis3_sensor {
    uint8_t raw_data[10];

    struct offset_raw{
        int16_t x;
        int16_t y;
        int16_t z;
    }offset_raw;

    struct offset{
        float x;
        float y;
        float z;
    }offset;

    struct moving_average
    {
        float x;
        float y;
        float z;
    }MovAver;

    int16_t x;
    int16_t y;
    int16_t z;
    int16_t update_status;
}ahrs_sensor;

extern  struct axis3_sensor Acc,Gyro,Mag;
extern  int gyro_count,gyro_count_sec;
/****mpu9250 rgesiter remap *****/
#define ACCEL_XOUT_H 0x3b
#define ACC_CONFIG   0x1c
#define ACC_CONFIG2  0x1d
#define GYRO_XOUT_H  0x43
#define GYRO_CONFIG  0x1b
#define MAGNETOMETER_XL 0x03
#define WHO_AM_I     0x75
#define READ         128
#define WRITE        0
#define AK8963_ADD   0x0c
#define I2C_SLV4_ADDR 0x31
#define I2C_SLV4_REG  0x32
#define I2C_SLV4_DO   0x33
#define I2C_SLV4_CTRL 0x34
#define I2C_SLV4_DI   0x35



#define I2C_SLV0_ADDR           0x25
#define I2C_SLV0_REG            0x26
#define I2C_SLV0_CTRL           0x27

#define I2C_SLV3_ADDR           0x2E
#define I2C_SLV3_REG            0x2F
#define I2C_SLV3_CTRL           0x30

#define I2C_SLV0_DO             0x63
#define I2C_SLV1_DO             0x64
#define I2C_SLV2_DO             0x65
#define I2C_SLV3_DO             0x66

#define EXT_SENS_DATA_00        0x49
#define I2C_MST_STATUS          0x36
#define I2C_MST_CTRL            0x24
#define USER_CTRL               0x6A


/*********************AK8963 address**************/
#define CNTL1         0x0A
#define CNTL2         0x0B
#define AK8963_ID     0x00
#define ASTC          0x0c
#define ST1           0x02

#endif
