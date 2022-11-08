#include <stdio.h>
#include <Arduino.h>

#include "motors_manager.h"
#include "imu_manager.h"

void setup()
{
    //InitializeMotors();
    //StartReverseRightMotorRoutine(10, 8095, 1000);
    //StartReverseLeftMotorRoutine(10, 8095, 1000);

    //SetMotorRightTargetSpeed(4400);
    //SetMotorLeftTargetSpeed(4400);

    //Coloca um delay entre os starts dos PIDs pras tarefas não ficarem se batendo
    //StartPIDRightMotor(10);
    //delay(5);
    //StartPIDLeftMotor(10);

    IMUStart();

    //while(millis() < 10000)
    {
        IMUUpdate();
    }
    
}

void loop()
{
    //printf("%.3f, %.3f, %.3f\n", 4400.0f, GetMotorRightSpeed(), GetMotorLeftSpeed());

    IMUUpdate();

    printf("%.3f\n", IMUGetYaw());

    delay(50);
}