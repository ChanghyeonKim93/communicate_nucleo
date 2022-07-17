#ifndef _DRONE_MOTOR_PWM_H_
#define _DRONE_MOTOR_PWM_H_
#include "mbed.h"

#define NUM_MOTORS 8

// PWM (TIMER 3)
#define MOTOR_0_PWM PB_8  // pwm2/1
#define MOTOR_1_PWM PB_9  // pwm2/2
#define MOTOR_2_PWM PB_10 // pwm2/3
#define MOTOR_3_PWM PB_2  // pwm2/4

// PWM (TIMER 4)
#define MOTOR_4_PWM PC_6  // pwm3/1
#define MOTOR_5_PWM PB_5  // pwm3/2
#define MOTOR_6_PWM PC_8  // pwm3/3
#define MOTOR_7_PWM PC_9  // pwm3/4

#define PWM_FREQUENCY 1000 // Hz

class DroneMotorPwm {
private:
    PwmOut pwm_[8];

public:
    DroneMotorPwm();

    void setPWM_all(uint16_t duty[NUM_MOTORS]);
    void setPWM(uint32_t num, uint16_t duty);

};


#endif