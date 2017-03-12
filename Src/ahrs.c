
// Header files

#include "ahrs.h"
#include "semphr.h"
#include "tim.h"
#include "attitude_controll.h"

int ahrs_count=0,ahrs_count_sec;

void attitude_estimation(ahrs_sensor *acc, ahrs_sensor *gyro, ahrs_sensor *mag)
{

}



//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	500.0f		// sample frequency in Hz
#define betaDef		0.1f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
volatile float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
 struct euler EulerAngle;


//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//=================================================================================================
// Functions
void AhrsTaskCreate(void)
{
  ahrs_sem=xSemaphoreCreateBinary();

  AHRS_t=xTaskCreate(AhrsTask,
              "ahrs_calulation",
              1024,
              (void*)1 ,
               14,
              &AHRS_h
              );
  if(AHRS_t==!pdPASS)
  {
      vTaskDelete(AHRS_h);
  }
}

void AhrsTask(void *p)
{


    while(1)
    {
       if(pdTRUE==xSemaphoreTake(ahrs_sem,portMAX_DELAY))
       {
           static TimeCounter_t t;

           TimeCounter(&t,1);
//           xSemaphoreGive(GetAcc);
//           xSemaphoreGive(GetGyro);
           ReadMpuAllBloack();

           IMUCalulation(&Acc,&Gyro,&NormoliseQuaternion);
//           AHRSCalulation(&Acc,&Gyro,&Mag,&NormoliseQuaternion);
           float gx=(Gyro.x*2000/32768)/57.3;
           float gy=Gyro.y*2000/32768/57.3;
           float gz=Gyro.z*2000/32768/57.3;
//           AHRSupdate(gx,gy,gz, Acc.x,Acc.y,Acc.z,Mag.x,Mag.y,Mag.z,&NormoliseQuaternion);
           GetEulerAngle(&NormoliseQuaternion);
           ahrs_count++;
           TimeCounter(&t,2);
//           HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_0);
           attitudeControll();
       }

    }
}

GetEulerAngle(quaternion * q)
{
    float sqw=q->q0*q->q0;
    float sqx=q->q1*q->q1;
    float sqy=q->q2*q->q2;
    float sqz=q->q3*q->q3;

    EulerAngle.yaw =atan2f(2.f * (q->q1*q->q2 + q->q3*q->q0), sqx - sqy - sqz + sqw);
    EulerAngle.roll = asinf(-2.f * (q->q1*q->q3 - q->q2*q->q0));
    EulerAngle.pitch = atan2f(2.f * (q->q2*q->q3 + q->q0*q->q1), -sqx - sqy + sqz + sqw);

}

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void AHRSCalulation(ahrs_sensor *acc, ahrs_sensor *gyro ,ahrs_sensor *mag,quaternion *q) {
    /*******init variables****************/
    float ax=0;//acc->x;
    float ay=0;//acc->y;
    float az=0;//acc->z;
    float gx=(float)gyro->x*2000/57.3/32768;;
    float gy=(float)gyro->y*2000/57.3/32768;;
    float gz=(float)gyro->z*2000/57.3/32768;;
    float mx=mag->x;
    float my=mag->y;
    float mz=mag->z;



    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float hx, hy;
    float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

    // Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
    if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
        IMUCalulation(acc,gyro,q);
        return;
    }

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        recipNorm = invSqrt(mx * mx + my * my + mz * mz);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;

    //update quaternion
    q->q0=q0;
    q->q1=q1;
    q->q2=q2;
    q->q3=q3;

}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void IMUCalulation(ahrs_sensor* acc,ahrs_sensor *gyro,quaternion *q) {
    /************init imu data*******/
    float ax=acc->x;
    float ay=acc->y;
    float az=acc->z;
    float gx=((float)gyro->x*2000/32768-gyro->offset.x)/57.3;
    float gy=((float)gyro->y*2000/32768-gyro->offset.y)/57.3;
    float gz=((float)gyro->z*2000/32768-gyro->offset.z)/57.3;

    float recipNorm;
    float s0, s1, s2, s3;
    float qDot1, qDot2, qDot3, qDot4;
    float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

    // Rate of change of quaternion from gyroscope
    qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
    qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
    qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
    qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

        // Normalise accelerometer measurement
        recipNorm = invSqrt(ax * ax + ay * ay + az * az);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _4q0 = 4.0f * q0;
        _4q1 = 4.0f * q1;
        _4q2 = 4.0f * q2;
        _8q1 = 8.0f * q1;
        _8q2 = 8.0f * q2;
        q0q0 = q0 * q0;
        q1q1 = q1 * q1;
        q2q2 = q2 * q2;
        q3q3 = q3 * q3;

        // Gradient decent algorithm corrective step
        s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
        s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
        s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
        s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;
        recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * (1.0f / sampleFreq);
    q1 += qDot2 * (1.0f / sampleFreq);
    q2 += qDot3 * (1.0f / sampleFreq);
    q3 += qDot4 * (1.0f / sampleFreq);

    // Normalise quaternion
    recipNorm = invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q0 *= recipNorm;
    q1 *= recipNorm;
    q2 *= recipNorm;
    q3 *= recipNorm;


    //update quaternion
    q->q0=q0;
    q->q1=q1;
    q->q2=q2;
    q->q3=q3;
}
#define halfT 0.001
float Kp=0.5;
float iq0,iq1,iq2,iq3;
void AHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz,quaternion *q)
{

  float norm;
  float hx, hy, hz, bx, bz;
  float vx, vy, vz, wx, wy, wz;
  float ex, ey, ez;

  // auxiliary variables to reduce number of repeated operations
  float q0q0 = q0*q0;
  float q0q1 = q0*q1;
  float q0q2 = q0*q2;
  float q0q3 = q0*q3;
  float q1q1 = q1*q1;
  float q1q2 = q1*q2;
  float q1q3 = q1*q3;
  float q2q2 = q2*q2;
  float q2q3 = q2*q3;
  float q3q3 = q3*q3;

  // normalise the measurements


  norm = sqrt(ax*ax + ay*ay + az*az);
  ax = ax / norm;
  ay = ay / norm;
  az = az / norm;

  norm = sqrt(mx*mx + my*my + mz*mz);
  mx = mx / norm;
  my = my / norm;
  mz = mz / norm;

  // compute reference direction of flux
  hx = 2*mx*(0.5 - q2q2 - q3q3) + 2*my*(q1q2 - q0q3) + 2*mz*(q1q3 + q0q2);
  hy = 2*mx*(q1q2 + q0q3) + 2*my*(0.5 - q1q1 - q3q3) + 2*mz*(q2q3 - q0q1);
  hz = 2*mx*(q1q3 - q0q2) + 2*my*(q2q3 + q0q1) + 2*mz*(0.5 - q1q1 - q2q2);
  bx = sqrt((hx*hx) + (hy*hy));
  bz = hz;

  // estimated direction of gravity and flux (v and w)
  vx = 2*(q1q3 - q0q2);
  vy = 2*(q0q1 + q2q3);
  vz = q0q0 - q1q1 - q2q2 + q3q3;
  wx = 2*bx*(0.5 - q2q2 - q3q3) + 2*bz*(q1q3 - q0q2);
  wy = 2*bx*(q1q2 - q0q3) + 2*bz*(q0q1 + q2q3);
  wz = 2*bx*(q0q2 + q1q3) + 2*bz*(0.5 - q1q1 - q2q2);

  // error is sum of cross product between reference direction of fields and direction measured by sensors
  ex = (ay*vz - az*vy) ;//+ (my*wz - mz*wy);
  ey = (az*vx - ax*vz) ;//+ (mz*wx - mx*wz);
  ez = (ax*vy - ay*vx) ;//+ (mx*wy - my*wx);

  // integral error scaled integral gain
  /*exInt = exInt + ex*Ki;
  eyInt = eyInt + ey*Ki;
  ezInt = ezInt + ez*Ki;*/

  // adjusted gyroscope measurements
  gx = gx + Kp*ex ;//+ exInt;
  gy = gy + Kp*ey ;//+ eyInt;
  gz = gz + Kp*ez ;//+ ezInt;

  // integrate quaternion rate and normalise
  iq0 = (-q1*gx - q2*gy - q3*gz)*halfT;
  iq1 = (q0*gx + q2*gz - q3*gy)*halfT;
  iq2 = (q0*gy - q1*gz + q3*gx)*halfT;
  iq3 = (q0*gz + q1*gy - q2*gx)*halfT;

  q0 += iq0;
  q1 += iq1;
  q2 += iq2;
  q3 += iq3;

  // normalise quaternion

  norm = sqrt(q0*q0 + q1*q1 + q2*q2 + q3*q3);
  q0 = q0 / norm;
  q1 = q1 / norm;
  q2 = q2 / norm;
  q3 = q3 / norm;

  //update quaternion
  q->q0=q0;
  q->q1=q1;
  q->q2=q2;
  q->q3=q3;

}

int instability_fix = 2;

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
    if (instability_fix == 0)
    {
        /* original code */
        float halfx = 0.5f * x;
        float y = x;
        long i = *(long*)&y;
        i = 0x5f3759df - (i>>1);
        y = *(float*)&i;
        y = y * (1.5f - (halfx * y * y));
        return y;
    }
    else if (instability_fix == 1)
    {
        /* close-to-optimal  method with low cost from http://pizer.wordpress.com/2008/10/12/fast-inverse-square-root */
        unsigned int i = 0x5F1F1412 - (*(unsigned int*)&x >> 1);
        float tmp = *(float*)&i;
        return tmp * (1.69000231f - 0.714158168f * x * tmp * tmp);
    }
    else
    {
        /* optimal but expensive method: */
        return 1.0f / sqrtf(x);
    }
}
