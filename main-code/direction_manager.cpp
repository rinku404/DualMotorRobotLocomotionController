#include "direction_manager.h"

#include <Arduino.h>

#include "freertos/timers.h"

#include "src/libs/pid/PID.h"

#include "motors_manager.h"
#include "imu_manager.h"
#include "espnow_manager.h"

static int dir_test_timer_id = 5;
static int reverse_timer_id = 6;
static int dir_pid_timer_id = 7;

static float max_speed = 0;
static float turn_weight = -1;

static char espnow_msg[40];

void ApplyTurnDirection(float speed, float turn_weight_)
{
    if(turn_weight_ < 0)
    {
        //SetMotorRightTargetSpeed(speed*(2*turn_weight_ + 1));
        //SetMotorLeftTargetSpeed(speed);
        SetMotorRightPWM((int)(speed*(2*turn_weight_ + 1)));
        SetMotorLeftPWM((int)speed);
    }
    else
    {
        //SetMotorRightTargetSpeed(speed);
        //SetMotorLeftTargetSpeed(speed*(-2*turn_weight_ + 1)); 
        SetMotorRightPWM((int)(speed));
        SetMotorLeftPWM((int)(speed*(-2*turn_weight_ + 1)));
    }

    //printf("%.3f, %.3f\n", GetMotorRightTargetSpeed(), GetMotorLeftTargetSpeed());
}

void DirectionTestRoutine(TimerHandle_t xTimer)
{
    IMUUpdate();
    printf("%.3f, %.3f\n", turn_weight, IMUGetYaw());    
    //printf("%.3f %.3f\n", GetMotorLeftSpeed(), GetMotorRightSpeed());
}

void ReverseDirectionTest(TimerHandle_t xTimer)
{
    turn_weight*=-1;
    ApplyTurnDirection(max_speed, turn_weight);
}

void StartDirectionTestRoutine(uint32_t data_aquisition_time, float max_speed_, uint32_t step_time)
{
    TimerHandle_t tmr_dir_test = xTimerCreate("DirectionTest", pdMS_TO_TICKS(data_aquisition_time), pdTRUE, ( void * )dir_test_timer_id, &DirectionTestRoutine);
    if( xTimerStart(tmr_dir_test, 10 ) != pdPASS ) {
        printf("Timer start error");
    }

    TimerHandle_t tmr_reverse = xTimerCreate("ReverseDirection", pdMS_TO_TICKS(step_time), pdTRUE, ( void * )reverse_timer_id, &ReverseDirectionTest);
    if( xTimerStart(tmr_reverse, 10 ) != pdPASS ) {
        printf("Timer start error");
    }

    max_speed = max_speed_;
}

//PID Direction

//static PID pid_direction(0.066642f, 0.33705f, 0.0025497f);
static PID pid_direction(0.00060007f, 0.0083758f, 1.0748e-5);  

void PIDDirectionSetTargetRotation(float target_rotation)
{
    pid_direction.SetTarget(target_rotation);
}

void PIDDirectionSetPWM(float pwm)
{
    max_speed = pwm;
}

void PIDDirectionRoutine(TimerHandle_t xTimer)
{
    IMUUpdate();
    pid_direction.Update(pid_direction.GetTime_step(), IMUGetYaw());
    ApplyTurnDirection(max_speed, pid_direction.GetOutput());
}

void StartPIDDirection(uint32_t time_step)
{
    pid_direction.SetTime_step(time_step);

    TimerHandle_t tmr_dir_pid = xTimerCreate("DirectionPID", pdMS_TO_TICKS(time_step), pdTRUE, ( void * )dir_pid_timer_id, &PIDDirectionRoutine);
    if( xTimerStart(tmr_dir_pid, 10 ) != pdPASS ) {
        printf("Timer start error");
    }
}