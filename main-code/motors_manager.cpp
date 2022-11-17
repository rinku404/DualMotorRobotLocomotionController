#include "motors_manager.h"

#include <Arduino.h>

#include "driver/ledc.h"
#include "freertos/timers.h"

#include "src/libs/ESP32Encoder/ESP32Encoder.h"

#include "src/libs/motors/motors.h"
#include "src/libs/pid/PID.h"

//id tmrs
static int id_tmr_reverse_motor_right = 1;
static int id_tmr_reverse_motor_sig_right = 2;
static int id_tmr_reverse_motor_left = 3;
static int id_tmr_reverse_motor_sig_left = 4;

typedef struct
{
    float signal;
    uint32_t data_aquisition_time;
    uint32_t step_time;
    int pwm_max_value;

}reverse_routine_t;

static ledc_timer_config_t pwm_timer;

static motor motor_right(32, 33, //int pin_pwm_left_, int pin_pwm_right_,
              26, 27, //int pin_encoder_a_, int pin_encoder_b_,
              11,     //int pulses_per_turn_,
              &pwm_timer,      //ledc_timer_config_t* pwm_timer_,
              LEDC_CHANNEL_2,  //ledc_channel_t pwm_channel_left_,
              LEDC_CHANNEL_3); //ledc_channel_t pwm_channel_right_

static motor motor_left(13, 25, //int pin_pwm_left_, int pin_pwm_right_,
              18, 19, //int pin_encoder_a_, int pin_encoder_b_,
              11,     //int pulses_per_turn_,
              &pwm_timer,      //ledc_timer_config_t* pwm_timer_,
              LEDC_CHANNEL_0,  //ledc_channel_t pwm_channel_left_,
              LEDC_CHANNEL_1); //ledc_channel_t pwm_channel_right_

void InitializeMotors(void)
{
    pwm_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    pwm_timer.timer_num = LEDC_TIMER_0;
    pwm_timer.duty_resolution = LEDC_TIMER_13_BIT;
    pwm_timer.freq_hz = 5000;

	ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));
}

//Reverse Right Motor
static reverse_routine_t reverse_routine_right;

void ReverseRightMotorRoutine(TimerHandle_t xTimer)
{
    motor_right.UpdatePWM(reverse_routine_right.pwm_max_value * reverse_routine_right.signal);
    motor_right.UpdateSpeed(reverse_routine_right.data_aquisition_time);

    printf("%.4f, %.4f\n", reverse_routine_right.signal, motor_right.GetSpeed());
}

void ReverseRightMotorChangeSignal(TimerHandle_t xTimer)
{
    reverse_routine_right.signal *= -1;
}

void StartReverseRightMotorRoutine(uint32_t data_aquisition_time, int pwm_max_value, uint32_t step_time)
{
    TimerHandle_t reverse_right_routine_tmr = xTimerCreate("ReverseRightMotorRoutine", pdMS_TO_TICKS(data_aquisition_time), pdTRUE, ( void * )id_tmr_reverse_motor_right, &ReverseRightMotorRoutine);
    if( xTimerStart(reverse_right_routine_tmr, 10 ) != pdPASS ) {
        printf("Timer start error");
    }

    TimerHandle_t reverse_right_change_sig_tmr = xTimerCreate("ReverseRightMotorChangeSig", pdMS_TO_TICKS(step_time), pdTRUE, ( void * )id_tmr_reverse_motor_sig_right, &ReverseRightMotorChangeSignal);
    if( xTimerStart(reverse_right_change_sig_tmr, 10 ) != pdPASS ) {
        printf("Timer start error");
    }

    reverse_routine_right.signal = 1;
    reverse_routine_right.data_aquisition_time = data_aquisition_time;
    reverse_routine_right.pwm_max_value = pwm_max_value;
    reverse_routine_right.step_time = step_time;
}

//Reverse Left Motor
static reverse_routine_t reverse_routine_left;

void ReverseLeftMotorRoutine(TimerHandle_t xTimer)
{
    motor_left.UpdatePWM(reverse_routine_left.pwm_max_value * reverse_routine_left.signal);
    motor_left.UpdateSpeed(reverse_routine_left.data_aquisition_time);

    printf("%.4f, %.4f\n", reverse_routine_left.signal, motor_left.GetSpeed());
}

void ReverseLeftMotorChangeSignal(TimerHandle_t xTimer)
{
    reverse_routine_left.signal *= -1;
}

void StartReverseLeftMotorRoutine(uint32_t data_aquisition_time, int pwm_max_value, uint32_t step_time)
{
    TimerHandle_t reverse_left_routine_tmr = xTimerCreate("ReverseLeftMotorRoutine", pdMS_TO_TICKS(data_aquisition_time), pdTRUE, ( void * )id_tmr_reverse_motor_left, &ReverseLeftMotorRoutine);
    if( xTimerStart(reverse_left_routine_tmr, 10 ) != pdPASS ) {
        printf("Timer start error");
    }

    TimerHandle_t reverse_left_change_sig_tmr = xTimerCreate("ReverseLeftMotorChangeSig", pdMS_TO_TICKS(step_time), pdTRUE, ( void * )id_tmr_reverse_motor_sig_left, &ReverseLeftMotorChangeSignal);
    if( xTimerStart(reverse_left_change_sig_tmr, 10 ) != pdPASS ) {
        printf("Timer start error");
    }

    reverse_routine_left.signal = 1;
    reverse_routine_left.data_aquisition_time = data_aquisition_time;
    reverse_routine_left.pwm_max_value = pwm_max_value;
    reverse_routine_left.step_time = step_time;
}

//PID
//-------Right Motor-------
static PID pid_motor_right(0.000351, 0.0132, 1.59e-6); //(float kp_, float ki_, float kd_)

void SetMotorRightTargetSpeed(float target_speed)
{
    pid_motor_right.SetTarget(target_speed);
}

void PIDRightMotorRoutine(TimerHandle_t xTimer)
{
    motor_right.UpdateSpeed(pid_motor_right.GetTime_step());

    pid_motor_right.Update(pid_motor_right.GetTime_step(), motor_right.GetSpeed());

    motor_right.UpdatePWM((int)(motor_right.GetPWMMax()*pid_motor_right.GetOutput()));

    //printf("%f\n", pid_motor_right.GetOutput()/1000);
}

void StartPIDRightMotor(uint32_t time_step)
{
    pid_motor_right.SetTime_step(time_step);

    TimerHandle_t tmr = xTimerCreate("PIDRightMotorRoutine", pdMS_TO_TICKS(time_step), pdTRUE, ( void * )id_tmr_reverse_motor_sig_right, &PIDRightMotorRoutine);
    if( xTimerStart(tmr, 10 ) != pdPASS ) {
        printf("Timer start error");
    }
}

//-------Left Motor-------
static PID pid_motor_left(0.0396, 0.0163, 2.11e-6); //(float kp_, float ki_, float kd_)

void SetMotorLeftTargetSpeed(float target_speed)
{
    pid_motor_left.SetTarget(target_speed);
}

void PIDLeftMotorRoutine(TimerHandle_t xTimer)
{
    motor_left.UpdateSpeed(pid_motor_left.GetTime_step());

    pid_motor_left.Update(pid_motor_left.GetTime_step(), motor_left.GetSpeed());

    motor_left.UpdatePWM((int)(motor_left.GetPWMMax()*pid_motor_left.GetOutput()));
}

void StartPIDLeftMotor(uint32_t time_step)
{
    pid_motor_left.SetTime_step(time_step);

    TimerHandle_t tmr = xTimerCreate("PIDLeftMotorRoutine", pdMS_TO_TICKS(time_step), pdTRUE, ( void * )id_tmr_reverse_motor_sig_left, &PIDLeftMotorRoutine);
    if( xTimerStart(tmr, 10 ) != pdPASS ) {
        printf("Timer start error");
    }
}

//
float GetMotorRightSpeed(void)
{
    return motor_right.GetSpeed();
}

float GetMotorLeftSpeed(void)
{
    return motor_left.GetSpeed();
}

float GetMotorRightTargetSpeed(void)
{
    return pid_motor_right.GetTarget();
}

float GetMotorLeftTargetSpeed(void)
{
    return pid_motor_left.GetTarget();
}

void SetMotorRightPWM(int pwm)
{
    motor_right.UpdatePWM(pwm);
}

void SetMotorLeftPWM(int pwm)
{
    motor_left.UpdatePWM(pwm);
}