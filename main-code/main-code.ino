#include <stdio.h>
#include <Arduino.h>

#include "motors_manager.h"
#include "imu_manager.h"
#include "espnow_manager.h"

void setup()
{
    // InitializeMotors();
    //StartReverseRightMotorRoutine(10, 8095, 1000);
    //StartReverseLeftMotorRoutine(10, 8095, 1000);

    // SetMotorRightTargetSpeed(3500);
    // SetMotorLeftTargetSpeed(3500);

    //Coloca um delay entre os starts dos PIDs pras tarefas não ficarem se batendo
    //StartPIDRightMotor(10);
    //delay(2);
    //StartPIDLeftMotor(10);

    IMUStart();
    //ESPNowStart();
}

void loop()
{
    // printf("%.3f, %.3f, %.3f\n", 3500.0f, GetMotorRightSpeed(), GetMotorLeftSpeed());

    IMUUpdate();
    printf("%.3f\n", IMUGetYaw());

    delay(10);

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