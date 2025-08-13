#ifndef __TIM_H
#define __TIM_H

#include "main.h"

#define PWM1_FREQ 50000
void TIM13_Configure(void);
void pwm1_set_duty(uint32_t duty);
extern uint32_t pwm1_min_duty;
extern uint32_t pwm1_max_duty;

void TIM1_Configure(void);

void pwm_set_config(uint32_t freq, uint32_t duty);
void pwm_set_freq(uint32_t freq);
uint32_t pwm_get_freq(void);
uint32_t pwm_get_reload(void);
uint32_t pwm_get_duty(void);
void pwm_set_freq_limt(uint32_t min, uint32_t max);
void pwm_set_dt(uint32_t dt);
void pwm_freq_increase(uint32_t step);
void pwm_freq_decrease(uint32_t step);
int pwm_duty_increase(uint32_t step);
int pwm_duty_bydt_increase(uint32_t step);
int pwm_duty_decrease(uint32_t step);
int pwm_duty_set_pid(uint32_t duty);
int pwm_duty_set_pid_dt(uint32_t duty);
int pwm_duty_bydt_decrease(uint32_t step);
void pwm_enable(uint32_t enable);

float get_capture(uint32_t num);
uint32_t timer2_get_capture(void);

#endif

