/*
 * motor.c
 *
 *  Created on: 2023年9月26日
 *      Author: Universe
 */

#include "motor.h"
#include "seedline.h"
#include "pid.h"
#include "math.h"

float error;
float g_PID_Out;
float g_motor_PID_Out1;
float g_motor_PID_Out2;

//int swap;

void Motor_Init(void) {

    gpio_init(D12, GPO, 0, GPIO_PIN_CONFIG);
    gpio_init(D15, GPO, 0, GPIO_PIN_CONFIG);
//    gpio_init(D13, GPO, 1, GPIO_PIN_CONFIG);
//    gpio_init(D14, GPO, 1, GPIO_PIN_CONFIG);


    pwm_init(TIM4_PWM_MAP1_CH2_D13, 11520, 0);
    pwm_init(TIM4_PWM_MAP1_CH3_D14, 11520, 0);

}


void Motor_Set(int motor1, int motor2) // motor1左轮，motor2右轮;
{
    int motorL = 0, motorR = 0;
    //swap=swap;
    motorL = motor2;
    motorR = motor1;
    //根据参数正负 设置选择方向
    if (motorL < 0) {
        gpio_init(D12, GPO, 0, GPIO_PIN_CONFIG);

    } else gpio_init(D12, GPO, 1, GPIO_PIN_CONFIG);

    if (motorR < 0) {
        gpio_init(D15, GPO, 1, GPIO_PIN_CONFIG);

    } else gpio_init(D15, GPO, 0, GPIO_PIN_CONFIG);

    //motor1 设置左轮的转速
    if (motorL < 0) {


        if (motorL < -8000) motorL = -8000;//超过PWM幅值
        pwm_set_duty(TIM4_PWM_MAP1_CH2_D13, -motorL);

    } else {

        if (motorL > 8000) motorL = 8000;
        pwm_set_duty(TIM4_PWM_MAP1_CH2_D13, motorL);

    }

    //motor2 设置右轮的转速
    if (motorR < 0) {
        gpio_init(D15, GPO, 1, GPIO_PIN_CONFIG);
        if (motorR < -8000) motorR = -8000;//超过PWM幅值
        pwm_set_duty(TIM4_PWM_MAP1_CH3_D14, -motorR);

    } else {
        gpio_init(D15, GPO, 0, GPIO_PIN_CONFIG);
        if (motorR > 8000) motorR = 8000;
        pwm_set_duty(TIM4_PWM_MAP1_CH3_D14, motorR);


    }
}


void motorPidSetSpeed(int base1, int base2) {

    error = cal_error();
    g_PID_Out = PID_realize(&piderror, error);
    g_motor_PID_Out1 = base1 - g_PID_Out;//2000
    g_motor_PID_Out2 = base2 + g_PID_Out;//2000
    Motor_Set(g_motor_PID_Out1, g_motor_PID_Out2);

}


