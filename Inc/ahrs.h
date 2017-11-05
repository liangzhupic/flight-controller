#ifndef _ahrs_h
#define _ahrs_h

#include "mpu9250.h"
#include "task.h"
#include "math.h"
#include "semphr.h"



//----------------------------------------------------------------------------------------------------
// Variable declaration
extern int instability_fix;
extern volatile float beta;				// algorithm gain
extern volatile float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame

typedef struct quaternion
{
    float q0;
    float q1;
    float q2;
    float q3;
}quaternion;
quaternion NormoliseQuaternion;

struct euler
{
    volatile float pitch;
   volatile float roll;
   volatile float yaw;
};

struct ref_g_t{
    float x;
    float y;
    float z;
};

extern struct euler EulerAngle;
extern int ahrs_count,ahrs_count_sec;
extern float acc_normol, acc_vertical;
extern struct ref_g_t ref_g;
//task

BaseType_t AHRS_t;
TaskHandle_t AHRS_h;

SemaphoreHandle_t ahrs_sem;
//---------------------------------------------------------------------------------------------------
// Function declarations

void AHRSCalulation(ahrs_sensor *, ahrs_sensor *, ahrs_sensor *,quaternion*);
void IMUCalulation(ahrs_sensor *, ahrs_sensor *,quaternion*);
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,quaternion *);

void AhrsTaskCreate(void);
void AhrsTask(void*);




#endif
