#ifndef PID_H
#define PID_H

#include <stdint.h>

class PID
{
private:
    float kp;
    float ki;
    float kd;

    float error;
    float integral;
    float derivative;

    float last_error;

    float target;
    float output;

    uint32_t time_step;

public: 
    PID(float kp_, float ki_, float kd_);
    ~PID();
    
    void        SetTime_step(uint32_t time_step_);
    uint32_t    GetTime_step(void);
    void        SetTarget(float target_);
    void        Update(uint32_t time_step, float current_read);
    float       GetOutput();
};

#endif