#include <stdio.h>
#include <Arduino.h>

#include "driver/ledc.h"

#include "src/libs/ESP32Encoder/ESP32Encoder.h"

#include "motors.h"

uint32_t last_timestamp = 0;
uint32_t current_timestamp = 0;

ledc_timer_config_t pwm_timer;

motor motor_r(13, 25, //int pin_pwm_left_, int pin_pwm_right_,
              18, 19, //int pin_encoder_a_, int pin_encoder_b_,
              11,     //int pulses_per_turn_,
              &pwm_timer,      //ledc_timer_config_t* pwm_timer_,
              LEDC_CHANNEL_0,  //ledc_channel_t pwm_channel_left_,
              LEDC_CHANNEL_1); //ledc_channel_t pwm_channel_right_);
              
float sentido = 1;

void UpdateMotor_r(TimerHandle_t xTimer)
{
    current_timestamp = micros();

    motor_r.UpdatePWM(sentido*8095);
    motor_r.UpdateSpeed(current_timestamp - last_timestamp);

    //printf("%d\n", (int)(current_timestamp - last_timestamp));

    printf("%.4f, %.4f\n", 8.4*sentido, motor_r.GetSpeed());


    last_timestamp = current_timestamp;
}

void setup(){

    pwm_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    pwm_timer.timer_num = LEDC_TIMER_0;
    pwm_timer.duty_resolution = LEDC_TIMER_13_BIT;
    pwm_timer.freq_hz = 5000;

	ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));

    TimerHandle_t motor_tmr = xTimerCreate("motorTimer", pdMS_TO_TICKS(50), pdTRUE, ( void * )1, &UpdateMotor_r);
    if( xTimerStart(motor_tmr, 10 ) != pdPASS ) {
        printf("Motor Timer start error");
    }
}

void loop(){
    sentido = 1;
    delay(1000);
    sentido = -1;
    delay(1000);
}
