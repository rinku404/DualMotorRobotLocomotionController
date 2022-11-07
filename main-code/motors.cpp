#include "motors.h"

#include<Arduino.h>

//#include <math.h>

#define PI 3.1415926535

motor::motor(int pin_pwm_left_, int pin_pwm_right_,
             int pin_encoder_a_, int pin_encoder_b_,
             int pulses_per_turn_,
             ledc_timer_config_t* pwm_timer_,
             ledc_channel_t pwm_channel_left_,
             ledc_channel_t pwm_channel_right_)
{
    if(pwm_timer_ == NULL) return;

    pin_pwm_left = pin_pwm_left_;
    pin_pwm_right = pin_pwm_right_;
    pin_encoder_a = pin_encoder_a_;
    pin_encoder_b = pin_encoder_b_;
    pulses_per_turn = pulses_per_turn_;
    pwm_timer = pwm_timer_;
    pwm_channel_left = pwm_channel_left_;
    pwm_channel_right = pwm_channel_right_;

    //pwm_max_value = pow(2, (pwm_timer->duty_resolution)) - 1;
    pwm_max_value = 8095;

    SetupEncoder();
    SetupPWM();
}

motor::~motor()
{

}

void motor::SetupEncoder()
{
    ESP32Encoder::useInternalWeakPullResistors=UP;
	encoder.attachFullQuad(motor::pin_encoder_a, motor::pin_encoder_b);
    encoder.setFilter(1);
	encoder.clearCount();

    pulses_current_loop = 0;
    pulses_last_loop = 0;

    return;
}

void motor::SetupPWM()
{
    pwm_config_left.speed_mode = LEDC_LOW_SPEED_MODE;
    pwm_config_left.channel = pwm_channel_left;
    pwm_config_left.timer_sel = LEDC_TIMER_0;
    pwm_config_left.intr_type = LEDC_INTR_DISABLE;
    pwm_config_left.gpio_num = pin_pwm_left;
    pwm_config_left.duty = 0;
    pwm_config_left.hpoint = 0;

    ESP_ERROR_CHECK(ledc_channel_config(&(pwm_config_left)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_config_left.channel));

    pwm_config_right.speed_mode = LEDC_LOW_SPEED_MODE;
    pwm_config_right.channel = pwm_channel_right;
    pwm_config_right.timer_sel = LEDC_TIMER_0;
    pwm_config_right.intr_type = LEDC_INTR_DISABLE;
    pwm_config_right.gpio_num = pin_pwm_right;
    pwm_config_right.duty = 0;
    pwm_config_right.hpoint = 0;

    ESP_ERROR_CHECK(ledc_channel_config(&(pwm_config_right)));
    ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_config_right.channel));

    return;
}

void motor::UpdatePWM(int pwm)
{    
    if(pwm == 0)
    {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel_left, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel_left));

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel_right, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel_right));

        return;
    }
    
    if(pwm > pwm_max_value)
    {
        pwm = pwm_max_value;
    }
    else if(pwm < (pwm_max_value*-1))
    {
        pwm = (pwm_max_value*-1);
    }
    
    if(pwm > 0)
    {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel_left, (uint16_t)(pwm)));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel_left));

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel_right, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel_right));
    }
    else
    {
        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel_left, 0));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel_left));

        ESP_ERROR_CHECK(ledc_set_duty(LEDC_LOW_SPEED_MODE, pwm_channel_right, (uint16_t)(pwm*-1)));
        ESP_ERROR_CHECK(ledc_update_duty(LEDC_LOW_SPEED_MODE, pwm_channel_right));
    }

    return;
}

void motor::UpdateSpeed(uint32_t deltaT)
{
    pulses_current_loop = encoder.getCount();
    //rad/s
    speed =
        (((2*PI / (float)pulses_per_turn)*(float)(pulses_current_loop - pulses_last_loop))) / (float)(deltaT / 1.0e3);
    
    pulses_last_loop = pulses_current_loop;
}

float motor::GetSpeed()
{
    return speed;
}