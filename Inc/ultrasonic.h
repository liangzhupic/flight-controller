#ifndef _ultrasonic_h
#define _ultrasonic_h

void UltraSonicTaskCreate();

void UltraSonicPoll();

extern float ultrasonic_distance;
struct ultrasonic
{
    struct distance{
        float raw;
        float real;
        float temp;
        float previous;
    }distance;
    int instance;
    float speed;
    struct time_
    {
        long now;
        long previous;
        long interval;
    }time;
}ultrasonic;

#endif
