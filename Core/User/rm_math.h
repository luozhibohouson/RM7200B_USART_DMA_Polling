#ifndef __RM_MATH_H__
#define __RM_MATH_H__

#include "stdint.h"


int abs_i(int a);
float abs_f(float a);

float carmack_sqrt(float x);
float cal_rms(uint16_t *val, uint32_t len);
float get_average(float *val, uint32_t len);
float find_peak_to_peak(float *val, uint32_t len);
float get_peak_to_peak(float *data, int len);
int find_extremum(float *val, uint32_t len, float *val_maxima, float *val_minima, int *maxima_idx, int *minima_idx);
int find_extremum_minima(float *val, uint32_t len, int *val_minima, int *minima_idx);
int find_extremum_maxima(float *val, uint32_t len, int *val_maxima, int *maxima_idx);
int find_extremum_minima_i(uint32_t *val, uint32_t len, uint32_t *val_minima, int *minima_idx);

int find_maxima(float *val, uint32_t len, float *val_maxima, int *maxima_idx);
int find_minima(float *val, uint32_t len, float *val_minima, int *minima_idx);

int find_maxima_i(uint32_t *val, uint32_t len, uint32_t *val_maxima, int *maxima_idx);
int find_minima_i(uint32_t *val, uint32_t len, uint32_t *val_minima, int *minima_idx);

// 获取周期开始点和结束点index
int get_period_start_end_index(float *vol_data, int len, int *start_index, int *end_index);
// 获取电压电流有效值
float get_vol_cur_rms(float *vol_data, float *cur_data, int len, float *vol_rms, float *cur_rms);

// 计算电压电流内积
float get_dot_product(float *vol_data, float *cur_data, int len);
// 计算电压电流相位差
float get_phase_difference(float *vol_data, float *cur_data, int len);

// 3点中值滤波 (带阈值)
void median_filter_3(float *data, int len);

#endif  // __RM_MATH_H__
