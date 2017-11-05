#include "pid.h"
#include "math.h"
#include "tim.h"

void SetPidParameters(_pid_t *pid, float kp, float ki, float kd, float imax)
{
    pid->imax=imax;
    pid->kd=kd;
    pid->ki=ki;
    pid->kp=kp;

}


void InitPid(_pid_t *pid ,float kp,float ki , float kd ,float imax,int mode)
{
      pid->mode=mode;
      pid->imax=imax;
      pid->kd=kd;
      pid->ki=ki;
      pid->kp=kp;
}




float PidCalulation(_pid_t *pid,float error,float differential)
{
  pid->time.now=__HAL_TIM_GetCounter(&htim2);
  if(pid->time.previous>pid->time.now)
  {
      pid->time.interval=pid->time.now+(0xffffffff-pid->time.previous);
  }
  pid->time.interval=pid->time.now-pid->time.previous;
  pid->time.previous=pid->time.now;
  float real_dt=((float)pid->time.interval)/1000000; //convert to unit second

//   real_dt=0.002f;// 500hz

  float P= pid->kp* error;

  pid->integral += error* pid->ki* real_dt;
  float I=pid->integral;

  if(pid->integral> pid->imax)
  {
      I=pid->imax;
  }
  if((-pid->imax)> pid->integral)
  {
      I= -pid->imax;
  }
  float D;
  if(pid->mode == user_diff)
  {
      D=differential*pid->kd;
  }
  if(pid->mode == normol)
  {
      D=((error- pid->previous_error)/ real_dt)* pid->kd;
  }
  pid->previous_error = error;
  return P+ I- D;
}
