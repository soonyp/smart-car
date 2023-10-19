/*
 * motor.h
 *
 *  Created on: 2023Äê9ÔÂ26ÈÕ
 *      Author: Universe
 */

#ifndef MOTOR_H_
#define MOTOR_H_

#include "zf_common_headfile.h"

extern int StraitFlag;
extern float vari;
extern float g_PID_Out;
extern float g_motor_PID_Out1;
extern float g_motor_PID_Out2;
extern float error;


void Motor_Init(void);

void Motor_Set(int motorL, int motorR);

void motorPidSetSpeed(int base1, int base2);


#endif /* MOTOR_H_ */
