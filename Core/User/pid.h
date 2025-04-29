#ifndef __PID_H__
#define __PID_H__

#include "stdint.h"


void pid_init(void);
float rm_pid_f(float in);
int16_t rm_pid_d(int16_t in);
int16_t rm_pid_delta(int16_t in);

#endif // __PID_H__


