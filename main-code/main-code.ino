#include <stdio.h>
#include <Arduino.h>

#include "motors_manager.h"
#include "imu_manager.h"
#include "espnow_manager.h"
#include "direction_manager.h"

static float target_speed = 3500;
static float max_pwm = 7000;

void setup()
{
    /**/
    InitializeMotors();
    
    //StartReverseRightMotorRoutine(10, 4000, 1000);
    //StartReverseLeftMotorRoutine(10, 4000, 1000);

    //SetMotorRightTargetSpeed(target_speed);
    //SetMotorLeftTargetSpeed(target_speed);

    //Coloca um delay entre os starts dos PIDs pras tarefas não ficarem se batendo
    //StartPIDRightMotor(10);
    //delay(2);
    //StartPIDLeftMotor(10);

    IMUStart();
    //ESPNowStartSender();

    //StartDirectionTestRoutine(50, target_speed, 2000);
    //StartDirectionTestRoutine(50, max_pwm, 2000);
    PIDDirectionSetTargetRotation(0);
    PIDDirectionSetPWM(7000);
    StartPIDDirection(50);
    //SetMotorLeftPWM(7000);
    //SetMotorRightPWM(7000);
    //ApplyTurnDirection(7000, 0);
    /**/

    /**/
    //ESPNowStartReceiver();
    /**/

}

void loop()
{
    //printf("%.3f, %.3f, %.3f\n", target_speed, GetMotorLeftSpeed(), GetMotorRightSpeed());
    

    //IMUUpdate();
    //printf("%.3f\n", IMUGetYaw());

    //delay(10);

    /*
    printf("%.3f\n", IMUGetYaw());

    float turn_weight = -1;

    
    for(int i = 0; i < 20; i++)
    {
        IMUUpdate();

        printf("%.4f, %.4f\n", turn_weight, IMUGetYaw());

        delay(50);
    }

    turn_weight = 1;

    for(int i = 0; i < 20; i++)
    {
        //turn_weight += 0.01f;

        IMUUpdate();

        printf("%.4f, %.4f\n", turn_weight, IMUGetYaw());

        delay(50);
    }
    */
}