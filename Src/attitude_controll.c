#include "pid.h"
#include "attitude_controll.h"
#include "ppm_capure.h"
#include "ahrs.h"
#include "tim.h"
#include "esc.h"
#include "task.h"
#include "semphr.h"
#include "mpu9250.h"
#include "ultrasonic.h"
#include <auto_mode.h>

float DeadZone(float v,float base, float offest,int inv);
float DeadZone_1(float x, float zone);
float HeightControll(float thr);
float To_180_degress(float x);
_pid_t att_pitch,att_roll,att_yaw,rate_pitch,rate_roll,rate_yaw;
_pid_t height_thr, rate_thr;
_pid_t opt_position_x, opt_position_y;
float EYaw_Rate;
float EYaw_Att;
float yaw_fb;
float exp_height=0.0f;
float position_x_out,position_y_out;
float dp_speed,sp_speed;
void InitAttitudePid(void)
{
/*    //rate
   InitPid(&rate_pitch,2,1,0.0,50,normol);
   InitPid(&rate_roll,2,1,0.0,50,normol);
   InitPid(&rate_yaw,3.5,1,0,50,normol);
   //stab
   InitPid(&att_pitch,7,0,0.0,15,normol);
   InitPid(&att_roll,7,0,0.0,15,normol);
*/
   //2017/08/04
   //rate
  InitPid(&rate_pitch,1.4,1,0.0,50,normol);
  InitPid(&rate_roll,1.4,1,0.0,50,normol);
  InitPid(&rate_yaw,3.5,1,0,50,normol);
  //stab
  InitPid(&att_pitch,3.5,0.01,0.0,15,normol);
  InitPid(&att_roll,3.5,0.01,0.0,15,normol);
  InitPid(&att_yaw,8,0,0.0,15,normol);
   //thr
   InitPid(&rate_thr, 54, 25, 0, 300,user_diff);
   InitPid(&height_thr,1,0,0,0,user_diff);
   // vision position
   InitPid(&opt_position_x, 0.0243, 0.0, 0.333, 5, user_diff );
   InitPid(&opt_position_y, 0.0243, 0.0, 0.333, 5, user_diff );
}

float limit(float n, float max)
{
    if(n < -max )
        n = -max;
    if(n > max)
        n = max;
    return n;
}

/**
 1       2

     x

 4       3
 motor layout

 * @brief attitudeControll
 */
float thr;
void attitudeControll(void)
{
    #define  MAX_YAW_SPEED 150

    float EPitch_Rate,ERoll_Rate;
    float EPitch_Att,ERoll_Att;
    float pit,rol,yaw;

//    SetPidParameters(&rate_pitch,((float)ppm[6].value-1000)/200,((float)ppm[7].value-1000)/500,0,50); //((float)ppm[8].value-1000)/20000
//    SetPidParameters(&rate_roll,((float)ppm[6].value-1000)/200,((float)ppm[7].value-1000)/500,0,50);
//    SetPidParameters(&rate_yaw,((float)ppm[7].value-1000)/300,0,0,0);
//    SetPidParameters(&rate_pitch,((float)ppm[6].value-1000)/400,1,0.0,50); //((float)ppm[8].value-1000)/20000
//    SetPidParameters(&rate_roll,((float)ppm[6].value-1000)/400,1,0.0,50);
//    SetPidParameters(&att_roll,((float)ppm[8].value-1000)/100,0,0,0);
//    SetPidParameters(&att_pitch,((float)ppm[8].value-1000)/100,0,0,0);
//    SetPidParameters(&rate_thr,((float)ppm[6].value-1000)/3,((float)ppm[7].value-1000)/10, ((float)ppm[8].value-1000)/3,300);
//    SetPidParameters(&height_thr,((float)ppm[8].value-1000)/250,0,0,300);
//      SetPidParameters(&opt_position_x,((float)ppm[6].value-1000)/10000,((float)ppm[7].value-1000)/100000,((float)ppm[8].value-1000)/1000,5);
//      SetPidParameters(&opt_position_y,((float)ppm[6].value-1000)/10000,((float)ppm[7].value-1000)/100000,((float)ppm[8].value-1000)/1000,5);
    if(ppm[3].value<800) //ratitude mode ~1200
    {
        EPitch_Rate=-((float)ppm[1].value-1520)/5-((float)(Gyro.x)*2000/32768-Gyro.offset.x);//
        ERoll_Rate=-((float)ppm[0].value-1520)/5-((float)(Gyro.y)*2000/32768-Gyro.offset.y);
        EYaw_Rate=-((float)ppm[4].value-1520)/4+((float)(Gyro.z)*2000/32768-Gyro.offset.z);
        thr=ppm[2].value;

    }
    else //attitude mode
    {
        EPitch_Att=-((float)ppm[1].value-1520)/10-EulerAngle.pitch*57.3;//
        ERoll_Att=-((float)ppm[0].value-1520)/10-EulerAngle.roll*57.3;


        if(auto_mode){
            float dx = (auto_dx- 320) * ultrasonic.distance.real;
            float dy = (auto_dy- 240) * ultrasonic.distance.real;
            if(flag_moving == 1)
            {
                dx = (auto_dx- 220) * ultrasonic.distance.real;
            }
            if(ultrasonic.distance.real > 0.3)
            {
                position_y_out = limit(PidCalulation(&opt_position_y, dy, -auto_dy_speed), 10.0);
                position_x_out = limit(PidCalulation(&opt_position_x, dx, -auto_dx_speed), 10.0);
            }
            else
            {
                position_x_out = position_y_out = 0;
            }
            EPitch_Att= position_y_out -EulerAngle.pitch*57.3 -((float)ppm[1].value-1520)/10;//
            ERoll_Att= position_x_out -EulerAngle.roll*57.3 -((float)ppm[0].value-1520)/10;


        }
        if(ppm[2].value>1150)
        {
            yaw_fb+=((float)DeadZone(ppm[4].value,1510,100,1)/3)*0.002f;
            if(yaw_fb>180.0f)
                   yaw_fb-=360.0f;
            if(yaw_fb<-180.0f)
                   yaw_fb+=360.0f;
        }
        else
        {
            yaw_fb= EulerAngle.yaw*57.3;
        }

        EYaw_Att= EulerAngle.yaw*57.3- yaw_fb;//exp-fb=0-fb
        if( EYaw_Att< -180 )
        {
            EYaw_Att = EulerAngle.yaw* 57.3+ 180+ 180- yaw_fb;
        }
        else if(EYaw_Att >180)
        {
            EYaw_Att = EulerAngle.yaw*57.3- 180 - yaw_fb- 180;
        }

        EYaw_Rate=PidCalulation(&att_yaw,EYaw_Att,0)+ ((float)(Gyro.z)-Gyro.offset.z)*2000/32768;

        EPitch_Rate=PidCalulation(&att_pitch,EPitch_Att,0)-((float)(Gyro.x)-Gyro.offset.x)*2000/32768;//
        ERoll_Rate=PidCalulation(&att_roll,ERoll_Att,0)-((float)(Gyro.y)-Gyro.offset.y)*2000/32768;

      // EYaw_Rate=-PidCalulation(&att_yaw,EYaw_Att,0)+((float)(Gyro.z)-Gyro.offset.z)*2000/32768;

    //    EYaw_Rate=-((float)ppm[4].value-1520)/4+((float)(Gyro.z)-Gyro.offset.z)*2000/32768;

    //    Yaw_Rate_exp=-((float)ppm[4].value-1520)/4;

    //    Yaw_Rate_exp+=Yaw_Att_out;
    //    EYaw_Rate=-Yaw_Rate_exp+((float)(Gyro.z)-Gyro.offset.z)*2000/32768;

        if(ppm[3].value> 1400 || auto_mode)
        {
            thr= HeightControll((float)ppm[2].value);
        }
        else
        {
            thr=ppm[2].value;
        }
    }


     pit=PidCalulation(&rate_pitch,EPitch_Rate,0);
     rol=PidCalulation(&rate_roll,ERoll_Rate,0);
     yaw=-PidCalulation(&rate_yaw,EYaw_Rate,0);


   // generous ppm mode
    Esc[1].pulse=(uint16_t)(thr+pit+rol+yaw);
    Esc[2].pulse=(uint16_t)(thr+pit-rol-yaw);
    Esc[3].pulse=(uint16_t)(thr-pit-rol+yaw);
    Esc[4].pulse=(uint16_t)(thr-pit+rol-yaw);
/*
   // oneshot125 mode
    Esc[1].pulse=(uint16_t)(Esc[1].pulse/8);
    Esc[2].pulse=(uint16_t)(Esc[2].pulse/8);
    Esc[3].pulse=(uint16_t)(Esc[3].pulse/8);
    Esc[4].pulse=(uint16_t)(Esc[4].pulse/8);
    */

    // normal mode 20170728
     Esc[1].pulse=(uint16_t)(Esc[1].pulse);
     Esc[2].pulse=(uint16_t)(Esc[2].pulse);
     Esc[3].pulse=(uint16_t)(Esc[3].pulse);
     Esc[4].pulse=(uint16_t)(Esc[4].pulse);

    EscLimit(&Esc[1]);
    EscLimit(&Esc[2]);
    EscLimit(&Esc[3]);
    EscLimit(&Esc[4]);

    if((ppm[2].value<1150)||(ppm[5].value<1200)|| (auto_mode && auto_turnoff))
    {

//        Esc[1].pulse=(uint16_t)125;
//        Esc[2].pulse=(uint16_t)125;
//        Esc[3].pulse=(uint16_t)125;
//        Esc[4].pulse=(uint16_t)125;

        Esc[1].pulse=(uint16_t)1000;
        Esc[2].pulse=(uint16_t)1000;
        Esc[3].pulse=(uint16_t)1000;
        Esc[4].pulse=(uint16_t)1000;

        // set zreo to integral of all pid
        rate_pitch.integral=0;
        rate_roll.integral=0;
        rate_yaw.integral=0;
        att_pitch.integral=0;
        att_roll.integral=0;
        att_yaw.integral=0;
        rate_thr.integral=0;
        opt_position_x.integral =0;
        opt_position_y.integral =0;
    }
    Esc[7].pulse= 1411 + EulerAngle.pitch* 57.8* (1000.0/90.0);
    Esc[8].pulse= 1435 - EulerAngle.roll* 57.8* (1000.0/90.0);
    if(ultrasonic.distance.real < 0.3)
        Esc[7].pulse= 1000;
}


float HeightControll(float thr)
{
#define max_speed 1.0
#define max_auto_speed 0.8
    static float thr_out= 1000;

    thr=DeadZone(thr, 1650,100,1);
    dp_speed = (thr/500.0)* max_speed;               //convert to unit meter
    sp_speed = dp_speed- ultrasonic.speed;
    if(ppm[3].value > 1800 || auto_mode )
    {
        exp_height += dp_speed*0.002f;

        if(auto_mode)
            exp_height = auto_exp_height;

        if(exp_height < 0)
            exp_height = 0;
        if(exp_height >1.5)
            exp_height = 1.5;
        float error_height = exp_height- ultrasonic.distance.real;
        if(auto_mode){
            if(error_height > max_auto_speed)
                error_height = max_auto_speed;
            if(error_height < -max_auto_speed)
                error_height = -max_auto_speed;
        }
        sp_speed = PidCalulation(&height_thr, error_height, 0)- ultrasonic.speed;
    }

    thr_out= 1590 + PidCalulation(&rate_thr,sp_speed, -acc_vertical);              // 1500 is

    if(ppm[2].value<1150)
    {
        thr_out= 1000;
        exp_height = 0;
    }
    if(thr_out<1000)
        thr_out=1000;
    if(thr_out>2000)
        thr_out=2000;
    return thr_out;
}


float DeadZone(float v,float base, float offest,int inv)
{
    if(inv==1)
    {
        if((v<=(base+offest))&&(v>=(base-offest)))
        {
            return 0;
        }
        else{
            if(v>(base+offest))
                return v-(base+offest);
            if(v<(base-offest))
                return v-(base-offest);
        }
    }
    else //error
    {
        return v-base;
    }
}

float DeadZone_1(float x,float zone)
{
    float t;
    if( x>(-zone) && x<zone )
    {
        t=0;
    }

    else
    {
        t=x;
    }

    return t;
}

float To_180_degress(float x)
{
    return (x>180?(x-360):(x<-180?(x+360):x));
}
