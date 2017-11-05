#ifndef __attitude_ctr_h
#define __attitude_ctr_h
#include <pid.h>


void InitAttitudePid(void);

void attitudeControll(void);

#define Limit(x,min,max) ( x<min ? min :((x>max) ? max : x))

extern float EYaw_Rate,yaw_fb,EYaw_Att;
extern float exp_height;
extern float dp_speed,sp_speed;
extern _pid_t rate_thr;
extern float position_x_out, position_y_out;
#endif
