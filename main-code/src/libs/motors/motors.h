#ifndef MOTORS_H
#define MOTORS_H

#include "esp_err.h"
#include "driver/ledc.h"

#include "../ESP32Encoder/ESP32Encoder.h"

class motor
{
private:
    ESP32Encoder encoder;
    int pin_encoder_a;
    int pin_encoder_b;
    int pulses_per_turn;

    int64_t pulses_current_loop;
    int64_t pulses_last_loop;
    
    //Precisa configurar o timer e passar o endereço dele,
    //já que vai ser usado por outros canais também
    ledc_timer_config_t* pwm_timer;
    int pwm_max_value;

    ledc_channel_config_t pwm_config_left;
    ledc_channel_t        pwm_channel_left;
    int pin_pwm_left;

    ledc_channel_config_t pwm_config_right;
    ledc_channel_t        pwm_channel_right;
    int pin_pwm_right;

    float speed;

public:
    motor(int pin_pwm_left_, int pin_pwm_right_,
          int pin_encoder_a_, int pin_encoder_b_,
          int pulses_per_turn_,
          ledc_timer_config_t* pwm_timer_,
          ledc_channel_t pwm_channel_left_,
          ledc_channel_t pwm_channel_right_);
    
    ~motor();

    float target_speed;

    void  SetupEncoder();
    void  SetupPWM();
    void  UpdatePWM(int pwm);
    void  UpdateSpeed(uint32_t deltaT);
    float GetSpeed();
    float GetPWMMax();
};



#endif