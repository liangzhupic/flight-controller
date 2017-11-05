#ifndef _pid_h
#define _pid_h


typedef struct pid
{
    float kp;
    float ki;
    float kd;
    float imax;
    float error;
    float previous_error;
    float integral;
    struct time
    {
        long now;
        long previous;
        long interval;
    }time;
    int mode;

}_pid_t;

enum pid_mode
{
    normol =0,
    user_diff
};

void InitPid(_pid_t *,float,float,float,float,int);
void SetPidParameters(_pid_t *,float,float,float,float);
float PidCalulation(_pid_t *pid,float error,float differential);

#endif
