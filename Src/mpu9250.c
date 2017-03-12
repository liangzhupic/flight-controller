 /* include file */
#include "mpu9250.h"
#include "spi.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "tim.h"

/* */
int gyro_count,gyro_count_sec;
BaseType_t acc_read_t,gyro_read_t,mag_read_t;
 struct axis3_sensor Acc,Gyro,Mag;
TaskHandle_t acc_read_h=NULL,gyro_read_h=NULL,mag_read_h=NULL;
uint8_t buffer[6];
int ok;
/* create task of the acc,gyro,mag */
void mpu9250_task_create(void)
{
 /*   xTaskCreate(ACC_Read,
                "ACC_read_sensor",
                512,
                (void *)1,
                11,
                &acc_read_h
                );
    xTaskCreate(GYRO_Read,
                "GYRO_read_sensor",
                512,
                (void *)1,
                11,
                &gyro_read_h
                );
    xTaskCreate(MAG_Read,
                "MAG_read_sensor",
                512,
                (void *)1,
                12,
                &mag_read_h
                );*/

    GetAcc=xSemaphoreCreateBinary();
    GetGyro=xSemaphoreCreateBinary();
    GetMag=xSemaphoreCreateBinary();
    spi_res=xSemaphoreCreateMutex();
}

void ReadMpuAllBloack(void)
{
    mpu_read_block(ACCEL_XOUT_H,Acc.raw_data,6);
    Acc.x=((uint16_t)Acc.raw_data[0]<<8)|Acc.raw_data[1];
    Acc.y=((uint16_t)Acc.raw_data[2]<<8)|Acc.raw_data[3];
    Acc.z=((uint16_t)Acc.raw_data[4]<<8)|Acc.raw_data[5];

    //magnetometer

 /*   if(mpu_akm_ready())
    {
   mpu_akm_read(MAGNETOMETER_XL,Mag.raw_data,7);
         if(!(Mag.raw_data[7]&0x08))
            {
               Mag.x=(Mag.raw_data[1]<<8)|Mag.raw_data[0];
               Mag.y=(Mag.raw_data[3]<<8)|Mag.raw_data[2];
               Mag.z=(Mag.raw_data[5]<<8)|Mag.raw_data[4];
            }
    }*/

      mpu_read_block(GYRO_XOUT_H,Gyro.raw_data,6);
      Gyro.x=((uint16_t)Gyro.raw_data[0]<<8)|Gyro.raw_data[1];
      Gyro.y=((uint16_t)Gyro.raw_data[2]<<8)|Gyro.raw_data[3];
      Gyro.z=((uint16_t)Gyro.raw_data[4]<<8)|Gyro.raw_data[5];

      exponential_moving_average(&Gyro.MovAver.x,Gyro.x,10);
      exponential_moving_average(&Gyro.MovAver.y,Gyro.y,10);
      exponential_moving_average(&Gyro.MovAver.z,Gyro.z,10);
        if((Gyro.x==0)&&(Gyro.y==0)&&(Gyro.z==0))
        {
            gyro_count++;
        }
}
void exponential_moving_average(float* avg,float cur,uint16_t count )
{
    *avg-=*avg/count;
    *avg+=cur/count;
}

int mpu_read_who_am_i(void)
{
    uint8_t test,tran=WHO_AM_I;

//    mpu_read_block(tran,&test,1);

    if(test==113) return 1;
    return -1;
}

void InitMpu9250(void)
{
 #define CONFIG 0x1a
    //gyroscope
    mpu_configure(GYRO_CONFIG,0x18);
    mpu_configure(CONFIG,3); //hardware low pass filter 184hz delay 2.9us
    Delayus(10);

   // MAGNETOMETER
    mpu_akm_write(CNTL2,1);

    mpu_akm_write(CNTL1,0x12);
//        mpu_akm_write(ASTC,1<<6);     //self test

    //accelater

}

void ACC_Read(void *p)
{
    while(1)
    {           static TimeCounter_t t;

        //              HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
                      TimeCounter(&t,1);
//        vTaskDelay(4);
        if(pdTRUE==xSemaphoreTake(GetAcc,portMAX_DELAY))
        {
            uint8_t tran=READ|ACCEL_XOUT_H;
            Delayus(2);
   /*          while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
             {  }

            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
            if(HAL_SPI_Transmit_DMA(&hspi1,&tran,1)!=HAL_OK)
               {
                   Error_Handler();
               }
            Acc.update_status=1;
            if(HAL_SPI_Receive_DMA(&hspi1,Acc.raw_data,6)!=HAL_OK)
             {
                 Error_Handler();
             }*/

            if(xSemaphoreTake(spi_res,50)==pdTRUE)
            {

                mpu_read_block(ACCEL_XOUT_H,Acc.raw_data,6);
                Acc.x=(Acc.raw_data[0]<<8)+Acc.raw_data[1];
                Acc.y=(Acc.raw_data[2]<<8)+Acc.raw_data[3];
                Acc.z=(Acc.raw_data[4]<<8)+Acc.raw_data[5];
              TimeCounter(&t,2);
//              gyro_count++;
                xSemaphoreGive(spi_res);
            }
       }

    }
}
void MAG_Read(void *p)
{

    while(1)
    {
       vTaskDelay(3);
        if(xSemaphoreTake(spi_res,1000)==pdTRUE)
        {
//            vTaskDelay(300000);
          /*  if(!mpu_akm_ready())
            {
                vTaskDelay(10);
            }*/
//            vTaskDelay(1);
//            mpu_read_block(ACCEL_XOUT_H,Acc.raw_data,6);
            mpu_akm_read(MAGNETOMETER_XL,Mag.raw_data,7);
            int x,y,z;
            if(!(Mag.raw_data[7]&0x08))
                {
                    Mag.x=(Mag.raw_data[1]<<8)+Mag.raw_data[0];
                    Mag.y=(Mag.raw_data[3]<<8)+Mag.raw_data[2];
                    Mag.z=(Mag.raw_data[5]<<8)+Mag.raw_data[4];
                }
            x=Mag.x;
            y=Mag.y;
            z=Mag.z;
            gyro_count++;
            xSemaphoreGive(spi_res);
        }

    }
}

void GYRO_Read(void *p)
{

    while(1)
    {

        if(pdTRUE==xSemaphoreTake(GetGyro,portMAX_DELAY))
        {
//            vTaskDelay(1);
            Delayus(2);
            uint8_t tran=READ|GYRO_XOUT_H;

        //     while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY)
          //   {
            // }

      /*      HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);

            if(HAL_SPI_Transmit_DMA(&hspi1,&tran,1)!=HAL_OK)
               {
                   Error_Handler();
               }

            Gyro.update_status=1;

            if(HAL_SPI_Receive_DMA(&hspi1,Gyro.raw_data,6)!=HAL_OK)
             {
                 Error_Handler();
             }*/
             if(xSemaphoreTake(spi_res,50)==pdTRUE)
             {
                 mpu_read_block(GYRO_XOUT_H,Gyro.raw_data,6);
                 Gyro.x=(Gyro.raw_data[0]<<8)+Gyro.raw_data[1];
                 Gyro.y=(Gyro.raw_data[2]<<8)+Gyro.raw_data[3];
                 Gyro.z=(Gyro.raw_data[4]<<8)+Gyro.raw_data[5];


                xSemaphoreGive(spi_res);
             }
        }
    }
}

void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi)
{
//    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
}

void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi)
{
int x,y,z;
    if(hspi->Instance==SPI1)
    {
        HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
        if(Acc.update_status==1)
        {

            Acc.x=(Acc.raw_data[0]<<8)+Acc.raw_data[1];
            Acc.y=(Acc.raw_data[2]<<8)+Acc.raw_data[3];
            Acc.z=(Acc.raw_data[4]<<8)+Acc.raw_data[5];
            Acc.update_status=0;
x=Acc.x;
y=Acc.y;
z=Acc.z;

        }
        if(Gyro.update_status==1)
        {

            Gyro.x=(Gyro.raw_data[0]<<8)+Gyro.raw_data[1];
            Gyro.y=(Gyro.raw_data[2]<<8)+Gyro.raw_data[3];
            Gyro.z=(Gyro.raw_data[4]<<8)+Gyro.raw_data[5];
            Gyro.update_status=0;
x=Gyro.x;
y=Gyro.y;
z=Gyro.z;

        }

    }
}


void mpu_write_block(uint8_t reg,uint8_t data,uint8_t len)
{
    uint8_t tran[1];
    while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
    tran[0]=reg|WRITE;
    tran[1]=data;
    if(HAL_SPI_Transmit(&hspi1,&tran,len,50)!=HAL_OK)
       {
           Error_Handler();
       }
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);
}


void mpu_read_block(uint8_t reg, uint8_t *data,uint8_t len)
{
    reg=reg|READ;
    while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY){}
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_RESET);
    if(HAL_SPI_Transmit(&hspi1,&reg,1,50)!=HAL_OK)
       {
           Error_Handler();
       }
    if(HAL_SPI_Receive(&hspi1,data,len,50)!=HAL_OK)
       {
           Error_Handler();
       }
  /*  if(HAL_SPI_TransmitReceive(&hspi1,&reg,data,len,50)!=HAL_OK)
    {
        Error_Handler();
    }*/
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_4,GPIO_PIN_SET);

}


void mpu_akm_read(uint8_t reg, uint8_t *data, uint8_t len)
{   Delayus(2);
    mpu_write_block(USER_CTRL,(1<<5),2);

   // vTaskDelay(1);
    Delayus(2);
    mpu_write_block(I2C_SLV0_ADDR,AK8963_ADD|READ,2);

    Delayus(2);
    mpu_write_block(I2C_SLV0_REG,reg,2);

    Delayus(2);
    mpu_write_block(I2C_SLV0_CTRL,0x80|len,2);

    Delayus(2);
    mpu_read_block(EXT_SENS_DATA_00,data,len);
}


void mpu_akm_write(uint8_t reg, uint8_t data)
{
    Delayus(50);
    mpu_write_block(USER_CTRL,(1<<5),2);

    Delayus(50);
    mpu_write_block(I2C_SLV0_ADDR,AK8963_ADD|WRITE,2);

    Delayus(50);
    mpu_write_block(I2C_SLV0_REG,reg,2);

    Delayus(50);
    mpu_write_block(I2C_SLV0_DO,data,2);

    Delayus(50);
    mpu_write_block(I2C_SLV0_CTRL,0x81,2);


}

int mpu_akm_ready(void)
{
    uint8_t data;
    mpu_akm_read(ST1,&data,1);
    if(data&0x01){
    return 1;}
    return 0;
}

void mpu_configure(uint8_t reg,uint8_t data)
{
    uint8_t data_read;
 loop:
    mpu_write_block(reg,data,2);
    mpu_read_block(reg,&data_read,1);
    if(data!=data_read)
    {
        goto loop;
    }
}
