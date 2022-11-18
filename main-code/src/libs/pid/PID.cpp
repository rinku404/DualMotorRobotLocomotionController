#include "PID.h"

#include<Arduino.h>

PID::PID(float kp_, float ki_, float kd_)
{
    kp = kp_;
    ki = ki_;
    kd = kd_;
    time_step = 0;

    error = 0;
    integral = 0;
    derivative = 0;
    output = 0;
    target = 0;

    last_error = 0;
    last_output = 0;
    last_target = 0;
    last_read = 0;
}

PID::~PID()
{

}

void PID::SetTime_step(uint32_t time_step_)
{
    time_step = time_step_;
}

uint32_t PID::GetTime_step(void)
{
    return time_step;
}

void PID::SetTarget(float target_)
{
    target = target_;
}

float PID::GetTarget(void)
{
    return target;
}

void PID::Update(uint32_t time_step, float current_read)
{
    /*
    //Reset parameters if controller revert state
    if(target * last_target < 0)
    {
        error = 0;
        derivative = 0;
        integral = 0;

        last_error = 0;
        last_output = 0;
        last_read = 0;
        
        last_target = target;
    }
    */

    error = target - current_read;
    
    if(current_read == 0) integral = 0;

    if(output < 10 && output > -10)
        integral = integral + (error * time_step);
    
    //else integral = 0;
    
    
    derivative = (error - last_error) / time_step;

    output = kp*error + ki*integral + kd*derivative;

    output /= 1000;

    if(output > 1) output = 1;
    else if(output < -1) output = -1;

    last_error = error;
    last_output = output;
    last_target = target;
    last_read = current_read;

    /*
    printf("error: %.3f ", error);
    printf("integral: %.3f ", integral);
    printf("derivative: %.3f ", derivative);
    printf("output: %.3f ", output);
    printf("\n");
    */
}

float PID::GetOutput(void)
{
    return output;
}