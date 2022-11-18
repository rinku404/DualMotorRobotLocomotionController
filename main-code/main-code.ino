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
    //---------Inicializa Sensores e Atuadores------
    InitializeMotors();
    IMUStart();
    //ESPNowStartSender();
    //-----------------------------------------------
    
    //---------Ensaio Motor Direito------------------
    //StartReverseRightMotorRoutine(10, 4000, 1000);
    //-----------------------------------------------

    //---------Ensaio Motor Esquerdo-----------------
    //StartReverseLeftMotorRoutine(10, 4000, 1000);
    //-----------------------------------------------

    //---------Controle Dos Motores------------------
    //SetMotorRightTargetSpeed(target_speed);
    //SetMotorLeftTargetSpeed(target_speed);

    //Coloca um delay entre os starts dos PIDs pras tarefas não ficarem se batendo
    //StartPIDRightMotor(10);
    //delay(2);
    //StartPIDLeftMotor(10);
    //-----------------------------------------------

    //--------Ensaio Direção Velocidade--------------
    //StartDirectionTestRoutine(50, target_speed, 2000);
    //-----------------------------------------------

    //--------Ensaio Direção PWM---------------------
    //StartDirectionTestRoutine(50, max_pwm, 2000);
    //-----------------------------------------------

    //--------Rotina Robô anda reto------------------
    PIDDirectionSetTargetRotation(0);
    PIDDirectionSetPWM(7000);
    StartPIDDirection(50);
    //----------------------------------------------


    //--------Robô anda sem PID --------------------
    //ApplyTurnDirection(7000, 0);
    //---------------------------------------------

}

void loop()
{
    //printf("%.3f, %.3f, %.3f, %.3f\n", target_speed, GetMotorLeftSpeed(), GetMotorRightSpeed(), 0);
    //delay(10);
}