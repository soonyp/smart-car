/*
 * pid.h
 *
 *  Created on: 2023Äê9ÔÂ26ÈÕ
 *      Author: Universe
 */

#ifndef PID_H_
#define PID_H_

#include "zf_common_headfile.h"


typedef struct {
    float target_val;
    float actual_val;
    float err;
    float err_last;
    float err_sum;
    float Kp, Ki, Kd;

} tPid;

extern tPid piderror;

void PID_init(void);

float P_realize(tPid *pid, float actual_val);

float PI_realize(tPid *pid, float actual_val);

float PID_realize(tPid *pid, float actual_val);


#endif
