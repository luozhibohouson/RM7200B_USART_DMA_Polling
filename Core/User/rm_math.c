  /* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>
#include <math.h>
#include "main.h"
#include "usart.h"
#include "rm_math.h"
#include "adc.h"
#include "tim.h"

// 绝对值
int abs_i(int a)
{
    if (a > 0)
        return a;
    return 0-a;
}

// 浮点数绝对值
float abs_f(float a)
{
    if (a > 0)
        return a;
    return 0-a;
}

// 迭代法开根号
float carmack_sqrt(float x)
{
    float xhalf = 0.5f * x;
    int i = *(int*)&x;

    i = 0x5f3759df - (i >> 1);  //
    x = *(float*)&i;
    x = x * (1.5f - xhalf * x * x);  // 迭代次数越多越精确
    x = x * (1.5f - xhalf * x * x);
    x = x * (1.5f - xhalf * x * x);
    x = x * (1.5f - xhalf * x * x);

    return 1.0f / x;
}

double calsqrt(float number)
{
    float err = 1e-5;
    double root = number;
    while (abs_f(number - root * root) > err)
    {
        root = (number / root + root) / 2.0;
    }
    return root;
}

// 任意波形有效值
float cal_rms(uint16_t *val, uint32_t len)
{
    int i = 0;
    float fx = 0;
    uint32_t sum = 0;
    uint32_t temp = 0;

    if (val == NULL || len <= 0)
        return 0;

    for (i = 0; i < len; i++) {
        temp = val[i];
        sum = sum + temp * temp;
    }
    fx = (float)sum;
    fx = fx / len;
    fx = sqrt(fx);

    return fx;
}

// 正弦波波形有效值
float cal_sin_rms(uint16_t *val, uint32_t len)
{
    int i = 0;
    float fx = 0;
    float sum0 = 0;
    uint32_t sum = 0;
    uint32_t temp = 0;

    if (val == NULL || len <= 0)
        return 0;

    for (i = 0; i < len; i++) {
        sum0 = sum0 + val[i];
    }
    sum0 = sum0 / len;

    for (i = 0; i < len; i++) {
        temp = val[i] - sum0;
        sum = sum + temp * temp;
    }
    fx = (float)sum;
    fx = fx / len;
    fx = sqrt(fx);

    return fx;
}

float get_average(float *val, uint32_t len)
{
    int i = 0;
    float sum = 0;
    if (val == NULL || len <= 0)
        return 0;

    for (i = 0; i < len; i++) {
        sum += val[i];
    }
    return sum / len;
}

float find_peak_to_peak(float *val, uint32_t len)
{
    int i = 0;
    float val_max = 0, val_min = 0;
    if (val == NULL || len <= 0)
        return 0;

    val_max = val[0];
    val_min = val[0];
    for (i = 1; i < len; i++) {
        if (val[i] > val_max)
            val_max = val[i];
        if (val[i] < val_min)
            val_min = val[i];
    }

    return val_max - val_min;
}

// 峰峰值
float get_peak_to_peak(float *data, int len)
{
    int i;
    float data_f[ADC_CH_SIZE];  // 去除平均值后的数据,近似的对称数据
    float average = 0.0;  // 平均值
    float max = 0, min = 0;  // 波峰和波谷的极值
    float max_arr[16] = {0}, min_arr[16] = {0};  // 波峰和波谷的极值数组
    float max_sum = 0, min_sum = 0;  // 波峰和波谷的极值和
    int max_cnt = 0, min_cnt = 0; // 波峰和波谷的极值个数
    int pos_cnt = 0, neg_cnt = 0;  // 正数和负数的个数
    int M = 0; // 阈值  ADC采样速率 除以 信号频率 等于 采样点数，当采样点数小于一半时，没有波峰和波谷
    float max_val;
    float min_val;    // 计算data的平均值

    if (data == NULL || len <= 0)
        return 0;

    // 计算平均值
    for (i = 0; i < len; i++) {
        average += data[i];
    }
    average = average / len;

    // 去除平均值, 近似的对称数据
    for (i = 0; i < len; i++) {
        data_f[i] = data[i] - average;
    }

    M = (adc_freq>>1) / pwm_get_freq(); // 双通道分时，实际采样率是 adc_freq / 2
    M = M / 4;  // 四分之一周期
//    printf("M:%d\r\n", M);

    // 找到波峰极大值和波谷极小值
    max = data_f[0];
    min = data_f[0];
    for (i = 1; i < len; i++)
    {
        // 判断过零
        if (data_f[i] > 0 && (data_f[i - 1] < 0)) // 从负到正, 统计前一个波的极值
        {
            // 判断neg_cnt和pos_cnt的大小，如果个数小于阈值，则不是极值
            if (neg_cnt < M)
            {
//                printf("neg_cnt = %d\n", neg_cnt);
                continue;
            }

            min_arr[min_cnt++] = min;
            max = data_f[i];
            min = data_f[i];
        }
        else if (data_f[i] < 0 && (data_f[i - 1] > 0)) // 从正到负
        {
            // 判断pos_cnt的大小，如果个数小于阈值，则不是极值
            if (pos_cnt < M)
            {
//                printf("pos_cnt = %d\n", pos_cnt);
                continue;
            }
            max_arr[max_cnt++] = max;
            max = data_f[i];
            min = data_f[i];
        }

        if (data_f[i] > 0) // 波峰
        {
            neg_cnt = 0;
            pos_cnt++;
            if (data_f[i] > max)
            {
                max = data_f[i];
            }
        }
        else // 波谷
        {
            pos_cnt = 0;
            neg_cnt++;
            if (data_f[i] < min)
            {
                min = data_f[i];
            }
        }

        if (i == (len - 1)) // 没有过零，且是最后一笔数据
        {
            if (neg_cnt >= M)
            {
                min_arr[min_cnt++]= min;
            }
            if (pos_cnt >= M)
            {
                max_arr[max_cnt++] = max;
            }
        }
    }

    // 去掉min_arr和max_arr中的最大值和最小值
    max_val = max_arr[0];
    min_val = min_arr[0];
    max_sum = max_arr[0];
    min_sum = min_arr[0];
//    printf("max_arr[%d] = %f\n", 0, max_arr[0]);
    for (i = 1; i < max_cnt; i++)
    {
//        printf("max_arr[%d] = %f\n", i, max_arr[i]);
        if (max_arr[i] > max_val)
        {
            max_val = max_arr[i];
        }
        max_sum += max_arr[i];
    }
//    printf("min_arr[%d] = %f\n", 0, min_arr[0]);
    for (i = 1; i < min_cnt; i++)
    {
//        printf("min_arr[%d] = %f\n", i, min_arr[i]);
        if (min_arr[i] < min_val)
        {
            min_val = min_arr[i];
        }
        min_sum += min_arr[i];
    }

    max_sum -= max_val;
    min_sum -= min_val;
    max_cnt--;
    min_cnt--;

//    printf("max_sum = %f, max_cnt = %d, max avg = %f\n", max_sum, max_cnt, max_sum / max_cnt);
//    printf("min_sum = %f, min_cnt = %d, min avg = %f\n", min_sum, min_cnt, min_sum / min_cnt);


    return (max_sum / max_cnt) - (min_sum / min_cnt);  // 峰峰值
}


// 获取周期开始点和结束点index
int get_period_start_end_index(float *vol_data, int len, int *start_index, int *end_index)
{
    int i = 0;
    int across_zero = 0;
    int first_index = 0;
    int last_index = 0;

    if (vol_data == NULL || len <= 0)
        return -1;
    if (start_index == NULL || end_index == NULL)
        return -1;

    // 找到第一个过零点
    for (i = 0; i < len; i++) {
        if ((vol_data[i] > 0 && vol_data[i+1] < 0) || (vol_data[i] < 0 && vol_data[i+1] > 0)) {
            first_index = i;
            if (vol_data[i] > 0 && vol_data[i+1] < 0) {  // 从正到负
                across_zero = 1;
                break;
            }
            else {  // 从负到正
                across_zero = -1;
                break;
            }
        }
    }

    // 找到最后一个过零点
    for (i = len - 1; i >= 0; i--) {
        if (across_zero == 1) {  // 如果开始电压波形从正到负，则结束电压波形从正到负,才是完整的一个周期
            if (vol_data[i] < 0 && vol_data[i-1] > 0) {  // 倒着找，找到第一个从负到正的点，才是完整的一个周期
                last_index = i;
                break;
            }
        }
        else if (across_zero == -1) {  // 如果开始电压波形从负到正，则结束电压波形从负到正,才是完整的一个周期
            if (vol_data[i] > 0 && vol_data[i-1] < 0) {  // 倒着找，找到第一个从正到负的点，才是完整的一个周期
                last_index = i;
                break;
            }
        }
    }

    *start_index = first_index;
    *end_index = last_index;
    return 0;
}

// 不分开计算的原因是电压值比较稳定，电流值比较小，误差大，在判断过零时会出错
float get_vol_cur_rms(float *vol_data, float *cur_data, int len, float *vol_rms, float *cur_rms)
{
    int i = 0;
    int count = 0;
    int first_index = 0;
    int last_index = 0;
    float vol_rmsx = 0.0;
    float cur_rmsx = 0.0;

    get_period_start_end_index(vol_data, len, &first_index, &last_index);
//    printf("first:%d, last:%d\r\n", first_index, last_index);

    // 计算有效值
    for (i = first_index; i < last_index; i++) {
        vol_rmsx += vol_data[i] * vol_data[i];
        cur_rmsx += cur_data[i] * cur_data[i];
//        printf("[%d]  vol:%.5f cur:%.5f\r\n", i, vol_data[i], cur_data[i]);
    }
    count = last_index - first_index + 1;
    vol_rmsx = vol_rmsx / (float)count;
    cur_rmsx = cur_rmsx / (float)count;
    *vol_rms = sqrt(vol_rmsx);
    *cur_rms = sqrt(cur_rmsx);

//    printf("-[%d]------------ vrms %.5f, irms %.5f, vrms_sum:%.5f irms_sum:%.5f\r\n", (last_index - first_index + 1), *vol_rms, *cur_rms, vol_rmsx, cur_rmsx);

    return 0;
}

float get_rms(float *data, int len)
{
    int i = 0;
    int first_index = 0;
    int last_index = 0;
    float rmsx = 0;

    get_period_start_end_index(data, len, &first_index, &last_index);

    // 计算有效值
    for (i = first_index; i < last_index; i++) {
        rmsx += data[i] * data[i];
    }
    rmsx = rmsx / (last_index - first_index + 1);
    return sqrt(rmsx);
}

// 计算电压电流内积
float get_dot_product(float *vol_data, float *cur_data, int len)
{
    int i = 0;
    float inner_product = 0;

    if (vol_data == NULL || cur_data == NULL || len <= 0)
        return 0.0;

    for (i = 0; i < len; i++) {
        inner_product += vol_data[i] * cur_data[i];
    }
    return inner_product;
}

// 计算电压电流相位差
float get_phase_difference(float *vol_data, float *cur_data, int len)
{
    int i = 0;
    int count = 0;
    int first_index = 0;
    int last_index = 0;
    float dot_product = 0.0;
    float vol_rms = 0.0;
    float cur_rms = 0.0;
    float phase_difference = 0.0;
    double delta_t = 0.0;
    double delta_theta = 0.0;

    if (vol_data == NULL || cur_data == NULL || len <= 0)
        return 0.0;

    // 获取周期开始和结束点index
    get_period_start_end_index(vol_data, len, &first_index, &last_index);
//    for (i = 0; i < len; i++) {
//        printf("%.5f , %.5f\r\n", vol_data[i], cur_data[i]);
//    }

    // 计算电压电流内积
//    for (i = first_index; i < last_index; i++) {
//        dot_product += vol_data[i] * cur_data[i];
//    }
    // dot_product = dot_product / (last_index - first_index + 1);

    // 计算电压电流有效值
    for (i = first_index; i < last_index; i++) {
        vol_rms += vol_data[i] * vol_data[i];
        cur_rms += cur_data[i] * cur_data[i];
        dot_product += vol_data[i] * cur_data[i];
    }
    count = last_index - first_index + 1;
    vol_rms = sqrt(vol_rms / (last_index - first_index + 1));
    cur_rms = sqrt(cur_rms / (last_index - first_index + 1));

//    vol_rms = get_rms(vol_data, len);
//    cur_rms = get_rms(cur_data, len);
    phase_difference = acos(dot_product / (vol_rms * cur_rms * count));
//    printf("%d %.5f %.5f %.5f ", count, dot_product, vol_rms, cur_rms);
    delta_t = 1.0 / adc_freq;
    delta_theta = delta_t * 2.0 * 3.14159 * pwm_get_freq();
    phase_difference = phase_difference + delta_theta;
    return phase_difference;
}

/*  查找数组中的特征极值:
 *  1. 遍历数组，比较相邻三点，找出所有的局部极小值。
 *  2. 从所有局部极小值中，筛选出值最小的一个。
 *  3. 从该最小的极小值位置开始，向后查找所有的局部极大值。
 *  4. 从找到的局部极大值中，筛选出值最大的一个，作为最终的极大值。
 找极值方法：
 *  1. 先找极小值，如果有多个极小值，则选择最小的极小值
 *  2. 找极大值，极大值依赖于极小值，找高于极小值频率的极大值的最高值？？？？ 距离极小值最近的极大值？？？？？
 */
int find_extremum(float *val, uint32_t len, float *val_maxima, float *val_minima, int *maxima_idx, int *minima_idx)
{
    int i = 0, cnt = 0;
    float val_max = 0, val_min = 0;
    int max_idx = 0, min_idx = 0;
    if (val == NULL || len <= 0)
        return 0;


    float temp0, temp1, temp2;
    float sub01, sub12;

//    for (i = 0; i < len; i++) {
//        printf("%d, %.5f\r\n", 25000 + 100 * i, val[i]);
//    }

    cnt = 0;
    temp0 = val[0];
    temp1 = (temp0 + val[1]) / 2;
//    temp1 = val[1];
    for (i = 2; i < len; i++) {
        temp2 = (temp1 + val[i]) / 2;
//        temp2 = val[i];
        sub01 = temp0 - temp1;
        sub12 = temp1 - temp2;
//        printf("%f, %f, %f\r\n", temp0, temp1, temp2);
        if (sub01 > 0 && sub12 < 0) {
            printf("min idx:%d, val:%f, sub01:%f, sub12:%f\r\n", i-1, val[i-1], sub01, sub12);
            if (cnt == 0 || (val[i-1] < val_min)) {   // 多个极小值里面找最小的极值
                val_min = val[i-1];
                min_idx = i-1;
            }
            cnt++;
        }
        temp0 = temp1;
        temp1 = temp2;
    }

    // 从极小值开始找极大值, 找极小值最近的极大值。是否需要找多个极大值中的最大值？？？？？
    cnt = 0;
    temp0 = val[min_idx];
//    temp1 = (temp0 + val[min_idx + 1]) / 2;
    temp1 = val[min_idx + 1];
    max_idx = min_idx;
    val_max = val[min_idx];
//    for (i = min_idx + 2; i < (min_idx + 12); i++) {  // 范围 极小值频率--极小值频率+1K
    for (i = min_idx + 2; i < (len); i++) {
//        temp2 = (temp1 + val[i]) / 2;
        temp2 = val[i];
        sub01 = temp0 - temp1;
        sub12 = temp1 - temp2;
        if (sub01 < 0 && sub12 > 0) {
            printf("max idx:%d, val:%f, sub01:%f, sub12:%f\r\n", i-1, val[i-1], sub01, sub12);
            if (cnt == 0 || (val[i-1] > val_max)) {  // 多个极大值时找最大的极大值
                val_max = val[i-1];
                max_idx = i-1;
            }
            cnt++;
        }
        temp0 = temp1;
        temp1 = temp2;
    }

    *val_maxima = val_max;
    *val_minima = val_min;
    *maxima_idx = max_idx;
    *minima_idx = min_idx;
    return 0;
}

/*  查找局部极大值:
 *  通过比较平滑后的相邻三点来寻找局部极大值 (点i-1 < 点i > 点i+1)。
 *  函数会返回遍历过程中找到的最后一个局部极大值。
 */
int find_extremum_maxima(float *val, uint32_t len, int *val_maxima, int *maxima_idx)
{
    int i = 0;
    float temp0, temp1, temp2;
    float sub01, sub12;
    if (val == NULL || len <= 0)
        return 0;

    printf("find_extremum_maxima\r\n");
    temp0 = val[0];
    temp1 = (temp0 + val[1]) / 2;
    for (i = 2; i < len; i++) {
        temp2 = (temp1 + val[i]) / 2;
        sub01 = temp0 - temp1;
        sub12 = temp1 - temp2;
        if (sub01 < 0 && sub12 > 0) {
            printf("idx:%d, val:%f, sub01:%f, sub12:%f\r\n", i-1, val[i-1], sub01, sub12);
            *val_maxima = val[i-1];
            *maxima_idx = i-1;
        }
        temp0 = temp1;
        temp1 = temp2;
    }
    return 0;
}

/*  查找局部极小值:
 *  通过比较平滑后的相邻三点来寻找局部极小值 (点i-1 > 点i < 点i+1)。
 *  函数会返回遍历过程中找到的最后一个局部极小值。
 */
int find_extremum_minima(float *val, uint32_t len, int *val_minima, int *minima_idx)
{
    int i = 0;
    float temp0, temp1, temp2;
    float sub01, sub12;
    if (val == NULL || len <= 0)
        return 0;

    printf("find_extremum_minima\r\n");
    temp0 = val[0];
    temp1 = (temp0 + val[1]) / 2;
    for (i = 2; i < len; i++) {
        temp2 = (temp1 + val[i]) / 2;
        sub01 = temp0 - temp1;
        sub12 = temp1 - temp2;
        if (sub01 > 0 && sub12 < 0) {
            printf("idx:%d, val:%f, sub01:%f, sub12:%f\r\n", i-1, val[i-1], sub01, sub12);
            *val_minima = val[i-1];
            *minima_idx = i-1;
        }
        temp0 = temp1;
        temp1 = temp2;
    }
    return 0;
}

int find_extremum_minima_i(uint32_t *val, uint32_t len, uint32_t *val_minima, int *minima_idx)
{
    int best_idx = -1;
    uint32_t best_value = 0;

    if (val == NULL || len < 3) {
        return -1;
    }

    // 寻找所有V型谷底数据且值最小的谷底
    for (size_t i = 1; i < len - 1; i++) {
        if (val[i-1] > val[i] && val[i] <= val[i+1]) {
            if (best_idx == -1 || val[i] < best_value) {
                best_idx = i;
                best_value = val[i];
            }
        }
    }
    if (best_idx != -1) {
        printf("1->  idx=%d value=%d\r\n", best_idx, best_value);
        *val_minima = best_value;
        *minima_idx = best_idx;
        return 1;
    }

    // 寻找峰后最低点 -> /\ 或 \ 数据
    bool peak_descent_found = false;
    for (size_t i = 2; i < len; i++) {
        bool is_descent = val[i] < val[i-1];
        bool is_after_peak = val[i-1] >= val[i-2];

        if (is_descent) {
            if (is_after_peak && !peak_descent_found) {
                peak_descent_found = true;
                best_idx = i;
                best_value = val[i];
            } else if (peak_descent_found) {
                if (val[i] < best_value) {
                    best_idx = i;
                    best_value = val[i];
                }
            }
        }
    }
    if (best_idx != -1) {
        printf("2->  idx=%d value=%d\r\n", best_idx, best_value);
        *val_minima = best_value;
        *minima_idx = best_idx;
        return 2;
    }

    // 寻找全局最小值 -> / 数据
    for (size_t i = 1; i < len; i++) {
        if (val[i-1] < val[i]) {
            if (best_idx == -1 || val[i-1] < best_value) {
                best_idx = i - 1;
                best_value = val[i-1];
            }
        }
    }
    if (best_idx != -1) {
        printf("3->  idx=%d value=%d\r\n", best_idx, best_value);
        *val_minima = best_value;
        *minima_idx = best_idx;
        return 3;
    }

    return 0;
}

/*  查找全局最大值:
 *  遍历整个浮点数数组，找出最大值及其索引。
 */
int find_maxima(float *val, uint32_t len, float *val_maxima, int *maxima_idx)
{
    int i = 0;
    float val_max = 0;
    int max_idx = 0;
    if (val == NULL || len <= 0)
        return -1;

    val_max = val[0];
    for (i = 1; i < len; i++) {
        if (val[i] > val_max) {
            val_max = val[i];
            max_idx = i;
        }
    }

    *val_maxima = val_max;
    *maxima_idx = max_idx;
    return 0;
}

/*  查找全局最大值 (无符号整型):
 *  遍历整个无符号整型数组，找出最大值及其索引。
 */
int find_maxima_i(uint32_t *val, uint32_t len, uint32_t *val_maxima, int *maxima_idx)
{
    int i = 0;
    uint32_t val_max = 0;
    int max_idx = 0;
    if (val == NULL || len <= 0)
        return -1;

    val_max = val[0];
    for (i = 1; i < len; i++) {
        if (val[i] > val_max) {
            val_max = val[i];
            max_idx = i;
        }
    }

    *val_maxima = val_max;
    *maxima_idx = max_idx;
    return 0;
}

/*  查找全局最小值:
 *  遍历整个浮点数数组，找出最小值及其索引。
 */
int find_minima(float *val, uint32_t len, float *val_minima, int *minima_idx)
{
    int i = 0;
    float val_min = 0;
    int min_idx = 0;
    if (val == NULL || len <= 0)
        return -1;

    val_min = val[0];
    for (i = 1; i < len; i++) {
        if (val[i] < val_min) {
            val_min = val[i];
            min_idx = i;
        }
    }

    *val_minima = val_min;
    *minima_idx = min_idx;
    return 0;
}

/*  查找全局最小值 (无符号整型):
 *  遍历整个无符号整型数组，找出最小值及其索引。
 */
int find_minima_i(uint32_t *val, uint32_t len, uint32_t *val_minima, int *minima_idx)
{
    int i = 0;
    uint32_t val_min = 0;
    int min_idx = 0;
    if (val == NULL || len <= 0)
        return -1;

    val_min = val[0];
    for (i = 1; i < len; i++) {
        if (val[i] < val_min) {
            val_min = val[i];
            min_idx = i;
        }
    }

    *val_minima = val_min;
    *minima_idx = min_idx;
    return 0;
}

void rm_math_test(void)
{
    ;
}

/* 3点中值滤波 (带阈值):
 * y[i] = Median(x[i-1], x[i], x[i+1])
 * 只有当 |原始值 - 中值| > 阈值 时才替换，否则保留原始值（避免削峰）
 * 原地滤波，会对原数据进行修改
 */
void median_filter_3(float *data, int len)
{
    if (data == NULL || len < 3) return;

    float prev, curr, next;
    float median;

    // 处理从索引 1 到 len-2 的数据 (首尾两点不滤波)
    for (int i = 1; i < len - 1; i++) {
        prev = data[i-1];
        curr = data[i];
        next = data[i+1];

        // 寻找 prev, curr, next 的中值
        if ((prev <= curr && curr <= next) || (next <= curr && curr <= prev))
            median = curr;
        else if ((curr <= prev && prev <= next) || (next <= prev && prev <= curr))
            median = prev;
        else
            median = next;

        data[i] = median;
    }
}

//  LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_4);
//  LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_4);
//void math_test(void)
//{
//    int i = 0;
//    volatile double rad = 0.0;
//    volatile double y = 125.125;
//    volatile double x = 35.35;
//    volatile double y_x = 0.0;
//
//    volatile float rad_f = 0;
//    volatile float y_f = 125.125;
//    volatile float x_f = 35.35;
//    volatile float y_x_f = 0.0;
//
//    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_9);
//    sys_delayms(1000);
//    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_9);
//    sys_delayms(1000);
//    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_9);
//    for (i = 0; i < 1000; i++) {  // 357ms
//        y_x = i / x;
//        rad = atan(y_x);
//    }
//    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_9);
//    printf("rad:%f\r\n", rad);
//    sys_delayms(500);
//
//    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_9);
//    for (i = 0; i < 1000; i++) {  // 356ms
//        rad = atan2(y, x);
//    }
//    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_9);
//    sys_delayms(500);
//
//    LL_GPIO_ResetOutputPin(GPIOB, LL_GPIO_PIN_9);
//    for (i = 0; i < 1000; i++) {  // 30ms
//        rad_f = atan2f(y_f, x_f);
//    }
//    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_9);
//    while(1);
//}










