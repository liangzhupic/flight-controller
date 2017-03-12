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


float DeadZone(float v,float base, float offest,int inv);
float HeightControll(float );


_pid_t att_pitch,att_roll,att_yaw,rate_pitch,rate_roll,rate_yaw,rate_thr;

void InitAttitudePid(void)
{
    //rate
   InitPid(&rate_pitch,2,1,0.0,50,normol);
   InitPid(&rate_roll,2,1,0.0,50,normol);
   InitPid(&rate_yaw,3.5,1,0,50,normol);
   //stab
   InitPid(&att_pitch,7,0,0.0,15,normol);
   InitPid(&att_roll,7,0,0.0,15,normol);
   //thr
   InitPid(&rate_thr,100,0,0,0,normol);
}


/**
 1       2

     x

 4       3
 motor layout

 * @brief attitudeControll
 */
void attitudeControll(void)
{
    float EPitch_Att,ERoll_Att;
    float EPitch_Rate,ERoll_Rate,EYaw_Rate;
    float pit,rol,yaw,thr;
//    SetPidParameters(&rate_pitch,((float)ppm[6].value-1000)/200,((float)ppm[7].value-1000)/500,0,50); //((float)ppm[8].value-1000)/20000
//    SetPidParameters(&rate_roll,((float)ppm[6].value-1000)/200,((float)ppm[7].value-1000)/500,0,50);
//    SetPidParameters(&rate_yaw,((float)ppm[7].value-1000)/300,0,0,0);
//    SetPidParameters(&att_roll,((float)ppm[8].value-1000)/100,0,0,0);
//    SetPidParameters(&att_pitch,((float)ppm[8].value-1000)/100,0,0,0);
    SetPidParameters(&rate_thr,((float)ppm[6].value-1000)/30,((float)ppm[7].value-1000)/10,0,300);

if(ppm[3].value<1200) //ratitude mode
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
    EPitch_Rate=PidCalulation(&att_pitch,EPitch_Att,0)-((float)(Gyro.x)*2000/32768-Gyro.offset.x);//
    ERoll_Rate=PidCalulation(&att_roll,ERoll_Att,0)-((float)(Gyro.y)*2000/32768-Gyro.offset.y);

    EYaw_Rate=-((float)ppm[4].value-1520)/4+((float)(Gyro.z)*2000/32768-Gyro.offset.z);

    if(ppm[3].value>1800)
    {
        thr=HeightControll((float)ppm[2].value);
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

   // oneshot125 mode
    Esc[1].pulse=(uint16_t)(Esc[1].pulse/8);
    Esc[2].pulse=(uint16_t)(Esc[2].pulse/8);
    Esc[3].pulse=(uint16_t)(Esc[3].pulse/8);
    Esc[4].pulse=(uint16_t)(Esc[4].pulse/8);

    EscLimit(&Esc[1]);
    EscLimit(&Esc[2]);
    EscLimit(&Esc[3]);
    EscLimit(&Esc[4]);

    if((ppm[2].value<1150)||(ppm[5].value>1800))
    {
        Esc[1].pulse=(uint16_t)125;
        Esc[2].pulse=(uint16_t)125;
        Esc[3].pulse=(uint16_t)125;
        Esc[4].pulse=(uint16_t)125;
        // set zreo to integral of all pid
        rate_pitch.integral=0;
        rate_roll.integral=0;
        rate_yaw.integral=0;
        att_pitch.integral=0;
        att_roll.integral=0;
        att_yaw.integral=0;
        rate_thr.integral=0;
    }
}


float HeightControll(float thr)
{
     #define max_speed 6.0
   thr=DeadZone(thr,1520,75,1);
   float dp_speed=(thr/500)*max_speed;               //convert to unit meter
   float sp_speed=dp_speed-ultrasonic.speed;
   thr=PidCalulation(&rate_thr,sp_speed,0);
   thr+=1600;               // es
   return thr;
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




