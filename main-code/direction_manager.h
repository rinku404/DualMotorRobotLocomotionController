#ifndef DIRECTION_MANAGER_H
#define DIRECTION_MANAGER_H

#include <inttypes.h>

void StartDirectionTestRoutine(uint32_t data_aquisition_time, float max_speed_, uint32_t step_time);
void StartPIDDirection(uint32_t time_step);
void ApplyTurnDirection(float speed, float turn_weight_);
void PIDDirectionSetTargetRotation(float target_rotation);
void PIDDirectionSetPWM(float pwm);

#endif