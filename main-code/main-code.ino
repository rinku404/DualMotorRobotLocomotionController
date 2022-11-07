#include <stdio.h>
#include <Arduino.h>

#include "driver/ledc.h"

#include "src/libs/ESP32Encoder/ESP32Encoder.h"

#include "motors.h"

ledc_timer_config_t pwm_timer;

motor motor_r(13, 25, //int pin_pwm_left_, int pin_pwm_right_,
              18, 19, //int pin_encoder_a_, int pin_encoder_b_,
              11,     //int pulses_per_turn_,
              &pwm_timer,      //ledc_timer_config_t* pwm_timer_,
              LEDC_CHANNEL_0,  //ledc_channel_t pwm_channel_left_,
              LEDC_CHANNEL_1); //ledc_channel_t pwm_channel_right_);

void setup(){

    pwm_timer.speed_mode = LEDC_LOW_SPEED_MODE;
    pwm_timer.timer_num = LEDC_TIMER_0;
    pwm_timer.duty_resolution = LEDC_TIMER_13_BIT;
    pwm_timer.freq_hz = 5000;

	ESP_ERROR_CHECK(ledc_timer_config(&pwm_timer));
}

void loop(){

    //for(int i = 0; i < 8095; i += 100)
    //{
        printf("%f\n", motor_r.GetSpeed());

        motor_r.UpdatePWM(8095);
        motor_r.UpdateSpeed(10);

        delay(10);
    //}

}
