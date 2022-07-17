#include "drone_motor_pwm.h"

DroneMotorPwm::DroneMotorPwm()
: pwm_{PwmOut(MOTOR_0_PWM),PwmOut(MOTOR_1_PWM),PwmOut(MOTOR_2_PWM),PwmOut(MOTOR_3_PWM),
PwmOut(MOTOR_4_PWM),PwmOut(MOTOR_5_PWM),PwmOut(MOTOR_6_PWM),PwmOut(MOTOR_7_PWM)}
{
    uint32_t period_ms_ = (1000/PWM_FREQUENCY);

    for(int i = 0; i < NUM_MOTORS; ++i){
        pwm_[i].period_ms(period_ms_);
    }

};

void DroneMotorPwm::setPWM_all(uint16_t duty[NUM_MOTORS]){
    for(int i = 0; i < NUM_MOTORS; ++i) {
        float duty_float = (float)duty[i] / 4095.0f;
        pwm_[i].write(duty_float); // automatically saturated betwee 0.0~1.0
    }
}

void DroneMotorPwm::setPWM(uint32_t num, uint16_t duty){
    if(num >= 0 && num < 8){
        float duty_float = (float)duty / 4095.0f;
        pwm_[num].write(duty_float);
    }
};