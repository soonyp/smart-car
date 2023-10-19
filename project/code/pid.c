/*
 * pid.c
 *
 *  Created on: 2023Äê9ÔÂ26ÈÕ
 *      Author: Universe
 */



#include "pid.h"


tPid piderror;

void PID_init() {
    piderror.actual_val = 0.0;
    piderror.target_val = 0.00;
    piderror.err = 0.0;
    piderror.err_last = 0.0;
    piderror.err_sum = 0.0;
    piderror.Kp = 4.6;//4.6
    piderror.Ki = 0;
    piderror.Kd = 1.3;//1.3

}

float P_realize(tPid *pid, float actual_val) {
    pid->actual_val = actual_val;
    pid->err = pid->target_val - pid->actual_val;

    pid->actual_val = pid->Kp * pid->err;
    return pid->actual_val;
}

float PI_realize(tPid *pid, float actual_val) {
    pid->actual_val = actual_val;
    pid->err = pid->target_val - pid->actual_val;
    pid->err_sum += pid->err;

    pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->err_sum;

    return pid->actual_val;
}

float PID_realize(tPid *pid, float actual_val) {
    pid->actual_val = actual_val;
    pid->err = pid->target_val - pid->actual_val;
    pid->err_sum += pid->err;

    pid->actual_val = pid->Kp * pid->err + pid->Ki * pid->err_sum + pid->Kd * (pid->err - pid->err_last);

    pid->err_last = pid->err;

    return pid->actual_val;
}
