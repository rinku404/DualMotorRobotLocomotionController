#ifndef MOTORS_MANAGER_H
#define MOTORS_MANAGER_H

#include <inttypes.h>

void  InitializeMotors(void);
void  StartReverseRightMotorRoutine(uint32_t data_aquisition_time, int pwm_max_value, uint32_t step_time);
void  StartReverseLeftMotorRoutine(uint32_t data_aquisition_time, int pwm_max_value, uint32_t step_time);
void  SetMotorRightTargetSpeed(float target_speed);
void  SetMotorLeftTargetSpeed(float target_speed);
void  StartPIDRightMotor(uint32_t time_step);
void  StartPIDLeftMotor(uint32_t time_step);
float GetMotorRightSpeed(void);
float GetMotorLeftSpeed(void);

#endif