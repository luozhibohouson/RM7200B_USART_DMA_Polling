  /* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <ctype.h>
#include <stdlib.h>

#include "main.h"
#include "usart.h"
#include "tim.h"
#include "adc.h"
#include "gpio.h"
#include "rm_math.h"
#include "controller.h"
// #include "air_flowmeter.h"
#include "magic_cool.h"
// #include "oled.h"
#include "pid.h"
// #include "dac.h"
#include "rm_fft.h"



#if RM_MAGIC_COOL

#define     KEY_VOL_CFG     1  // 按键设置电压

/*****************************************************************/


float voltage_gain = 11.8;
float impedance[64] = {0};
float phase[64] = {0};
uint32_t freq_pwr[64] = {0};

/*******************************************************************/
/*******************************************************************/
/*******************************************************************/
// 气泵
uint8_t  magic_cool_channel = 0;
uint8_t  magic_cool_mode = 0;     // 0:暂停 1:运行阻抗谱 2:不追频运行 3:追频运行 4: 按键调占空比
uint8_t  magic_cool_adj_dir = 0;  // 0:频率减小   1:频率增加
uint32_t magic_cool_ipp = 0;
uint32_t magic_cool_ph_min = 0;
uint32_t magic_cool_freqstart = 22500, magic_cool_freqstop = 29000;
uint32_t magic_cool_runfreq = 25000;
uint32_t magic_cool_freq_step = 100;
uint32_t magic_cool_target_vol = 70;
uint32_t magic_cool_key_count = 0;

uint32_t magic_cool_pwr_max = 0;

uint16_t pwm1_duty_out = HSI_VALUE/PWM1_FREQ*20/33;
uint16_t pwm1_duty_limit_min = HSI_VALUE/PWM1_FREQ*4/33; //最小为0.4V - 145
uint16_t pwm1_duty_limit_max = HSI_VALUE/PWM1_FREQ*30/33; //最大为3.0V - 1090

float magic_cool_ph_proxth = 0;

int manual = 0;
uint32_t mode2_tick = 0;
/*******************************************************************/
uint8_t mc_key1_last = 1, mc_key2_last = 1, mc_key3_last = 1;  // KEY
/*******************************************************************/
/*******************************************************************/
/*******************************************************************/


void magic_cool_key_scan(uint8_t key1, uint8_t key2, uint8_t key3)
{
    int freq_stepx = 20;
//     if (mc_key1_last != key1) {
// #if KEY_VOL_CFG
//         magic_cool_target_vol -= 5; // 电压减小
//         if (magic_cool_target_vol < 20)
//             magic_cool_target_vol = 20;
//         printf("set vol %d\r\n", magic_cool_target_vol);
//         magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 100);  // 电压闭环
// #else
//         manual = 1;
//         pwm_freq_decrease(freq_stepx);  // 频率增加
//         magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 100);  // 电压闭环    ;
// #endif
//     }
    if (mc_key2_last != key2) {
#if KEY_VOL_CFG

#else
        if (manual == 1) {
            manual = 0;
            return;
        }
#endif
        if (magic_cool_mode == 0) {
            magic_cool_mode = 1;
        } else {
            magic_cool_mode = 0;
            pwm_enable(DISABLE);
        }
    }
//     if (mc_key3_last != key3) {
// #if KEY_VOL_CFG
//         magic_cool_target_vol += 5; // 电压增加
//         if (magic_cool_target_vol > 60)
//             magic_cool_target_vol = 60;
//         printf("set vol %d\r\n", magic_cool_target_vol);
//         magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 100);  // 电压闭环    ;
// #else
//         manual = 1;
//         pwm_freq_increase(freq_stepx);  // 频率减小
//         magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 100);  // 电压闭环
// #endif
//    }
}

void magic_cool_set_adcfreq(void)
{
    adc_set_conv_freq(ADC_15MHZ_CONV_FREQ_1MHZ);  // 同时采样电压电流，
}

// 设置频率范围
void magic_cool_set_limt(uint32_t freq_min, uint32_t freq_max)
{
    pwm_set_freq_limt(freq_min, freq_max);
    magic_cool_freqstart = freq_min;
    magic_cool_freqstop = freq_max;
}

// 电压闭环PWM占空比方式
int magic_cool_voltage_closeloop_duty(uint32_t vol_target, uint32_t vol_err, uint32_t timeout)
{
    int ret = 0, count = 0;
    int i = 0, n = 0;
    int16_t duty = 0;
    int16_t vpp = 0;
    int16_t vpp_sum = 0;
    int16_t vol_targetx = vol_target * voltage_gain;  // 目标电压对应ADC值
    int16_t vol_errx = vol_err * voltage_gain;  // 误差范围对应ADC值
    int16_t voltage_err = 0;    // 当前误差

    for (i = 0; i < timeout; i++) {    // 调整次数
        vpp_sum = 0;
        for (n = 0; n < 10; n++) {
//            adc_output_conv(128);
            adc_voltage_get_vpp(128);  // 电压闭环，使用简单计算，提高反馈速度
            vpp_sum += adc_vpp;
        }
        vpp = vpp_sum / 10;
        voltage_err = (vol_targetx - vpp);
        duty = rm_pid_d(voltage_err);
        printf("err:%d, vpp:%d, vol_errx:%d, out:%d\r\n", voltage_err, vpp, vol_errx, duty);
        if (abs_i(voltage_err) < vol_errx) {  // 电压小于误差范围认为电压稳定
            count++;
            if (count > 10)    // 连续获取电压10次都在误差范围就认为电压稳定，退出
                return 0;
            continue;
        }
        count = 0;

        if (duty < 0) {
            duty = 0;
        }
        ret = pwm_duty_set_pid(duty);  // 设置PWM占空比
//        ret = pwm_duty_set_pid_dt(channel, duty);  // 设置PWM死区占空比

        if (ret == 1) {
//            printf("duty out of range\r\n");
            return 1;
        }
    }

    return 0;
}

// 电压闭环DCDC方式
int magic_cool_voltage_closeloop_dcdc(uint32_t vol_target, uint32_t vol_err, uint32_t timeout)
{
    int ret = 0, count = 0;
    int i = 0, n = 0;
    int16_t pid_delta = 0;
    int16_t vpp = 0;
    int32_t vpp_sum = 0;
    int16_t vol_targetx = vol_target * voltage_gain;  // 目标电压对应ADC值
    int16_t vol_errx = vol_err * voltage_gain;  // 误差范围对应ADC值
    int16_t voltage_err = 0;    // 当前误差

    for (i = 0; i < timeout; i++) {    // 调整次数
        vpp_sum = 0;
        for (n = 0; n < 10; n++) {
//            adc_output_conv(128);
            adc_voltage_get_vpp(128);  // 电压闭环，使用简单计算，提高反馈速度
            vpp_sum += adc_vpp;  // 获取单次计算的峰峰值
        }
        vpp = vpp_sum / 10;
        voltage_err = (vol_targetx - vpp);

//        printf("err:%d, vpp:%d, vol_errx:%d, out:%d, delta:%d\r\n", voltage_err, vpp, vol_errx, pwm1_duty_out, pid_delta);
        if (abs_i(voltage_err) < vol_errx) {  // 电压小于误差范围认为电压稳定
            count++;
            if (count > 10)    // 连续获取电压10次都在误差范围就认为电压稳定，退出
                return 0;
            continue;
        }
        count = 0;

//        pid_delta = rm_pid_delta(voltage_err);
        pid_delta = abs_i(voltage_err) / 5;
        if (voltage_err > 0) {     // 目标值大于实际值, 没有达到目标值
            pwm1_duty_out -= pid_delta;
        } else {                   // 目标值小于实际值，超过目标值
            pwm1_duty_out += pid_delta;
        }

        if (pwm1_duty_out < pwm1_duty_limit_min) {
            pwm1_duty_out = pwm1_duty_limit_min;
            ret = 1;
        } else if (pwm1_duty_out > pwm1_duty_limit_max) {
            pwm1_duty_out = pwm1_duty_limit_max;
            ret = 1;
        }
//        printf("pwm1_duty_out:%d\r\n", pwm1_duty_out);
        pwm1_set_duty(pwm1_duty_out);
        sys_delayms(50);

        if (ret == 1) {
//            printf("pwm1_duty_out of range\r\n");
            return 1;
        }
    }

    return 0;
}

// 电压闭环
// 1. 单端的时候，由于LC谐振，电压在不同频率下波动较大，可以通过调整PWM占空比升降压，也可以通过调整DCDC ref电压升降压
// 2. 差分模式的时候，由于LC谐振，调压只能通过调整DCDC ref电压升降压，PWM占空比固定为50%，不能变。
int magic_cool_voltage_closeloop(uint32_t vol_target, uint32_t vol_err, uint32_t timeout)
{
#ifndef MAGIC_COOL_DIFF
    int ret = 0;
    int loop = 5;
    while(loop--) {
        ret = magic_cool_voltage_closeloop_duty(vol_target, vol_err, timeout);
        if (ret)
            magic_cool_voltage_closeloop_dcdc(vol_target, vol_err, timeout);
        else
            break;
    }
#else
    magic_cool_voltage_closeloop_dcdc(vol_target, vol_err, timeout);
#endif
    return 0;
}

// 找Vpp，Ipp方式：
// 1. 采样后，找最大值，最小值， 计算峰峰值 peak_to_peak = max - min
// 2. 采样后，找多个周期的极大值，求平均值，作为最大值，找多个周期的极小值，求平均值，作为最小值，计算峰峰值 peak_to_peak = max - min

// 阻抗谱：
// 1. 采样多次电压Vpp， 电流Ipp，取平均值， 计算阻抗Z = Vpp / Ipp
// 2. 采样一次电压电流，通过积分计算rms电压电流，计算阻抗Z = Vrms / Irms

// 相位
// 1. 采样后，通过FFT计算相位
// 2. 采样后，通过积分计算rms电压电流，通过电压电流内积计算cos(phase) , 通过acos(cos(phase))计算相位
// 3. 直接通过电压电流过零比较器获取相位。


int magic_cool_calc_impedance(uint32_t start_freq, uint32_t stop_freq, uint32_t step_freq)
{
    int ret = 0, index = 0, i;
    int flow = 0;
    int freq = 0;
    float phasex = 0;
    float ipp = 0.0, vpp = 0.0;
    float irms = 0.0, vrms = 0.0;
    float zx = 0.0;
    float hvol = 0.0, hcur = 0.0, lcur = 0.0;

    set_power_enable(ENABLE);  // 高电压
    pwm_enable(ENABLE);

    printf("magic_cool_calc_impedance imp\r\n");
    for (freq = start_freq; freq <= stop_freq; ) {
        pwm_set_freq(freq);
        magic_cool_voltage_closeloop(magic_cool_target_vol, 3, 100);// 电压闭环
        sys_delayms(20);
        magic_cool_voltage_closeloop(magic_cool_target_vol, 3, 100);// 电压闭环
        printf("freq: %d ", freq);
        sys_delayms(200);
#if MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_VPP
        ipp = 0.0; vpp = 0.0;
        for (i = 0; i < 50; i++) {
            sys_delayms(2);
            // ADC采样，同时采样电压电流
            adc_output_conv(128);
            vpp += adc_vpp;  // 电压峰峰值
            ipp += adc_ipp;  // 电流峰峰值
        }
        zx = vpp / ipp;  // Z 的模
        printf("Vpp: %.5f Ipp: %.5f Z: %.5f ", vpp, ipp, zx);
#elif MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_RMS
        irms = 0.0;
        vrms = 0.0;
        for (i = 0; i < 50; i++) {
            sys_delayms(2);
            // ADC采样，同时采样电压电流
            adc_output_conv(128);
            vrms += adc_vrms;  // 电压有效值
            irms += adc_irms;  // 电流有效值
//            printf("vrms:%.5f irms:%.5f\r\n", adc_vrms, adc_irms);
        }
        zx = vrms / irms;  // Z 的模
        printf("Vrms: %.5f Irms: %.5f Z: %.5f Dac: %d ", vrms, irms, zx, pwm1_duty_out);
#else  // 其他方式，待定

#endif
        impedance[index] = zx;

#if MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_FFT
        // ADC数据处理，FFT
        adc_output_conv(128);
        phasex = ProcessADCData(adc_voltage_data, adc_current_data);
        printf("fft phase: %.5f ", phasex);
#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_DOT
        // ADC数据处理，点积
        adc_output_conv(128);
        phasex = get_phase_difference(adc_voltage_data, adc_current_data, 128);
        printf("dot phase: %.5f ", phasex);
#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_CAPTURE
        // PWM 输入捕获
        adc_output_conv(128);
        set_compare_dcoffset(128);
        sys_delayms(100);
        phasex = get_capture(8);
        printf("cap phase: %.5f ", phasex);
#else  // 其他方式，待定
        phasex = 0;
#endif
        phase[index] = phasex;

#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_ALL
        // 低电流
        adc_hvli_input_conv(128);
        hvol = adc_dc_hvol_avg;
        lcur = adc_dc_lcur_avg;
//        freq_pwr[index] = hvol * lcur;
        printf("hvol: %.2f lcur: %.2f ", hvol, lcur);

        // 高电流
        adc_hv_input_conv(128);
        hvol = adc_dc_hvol_avg;
        hcur = adc_dc_hcur_avg;
        freq_pwr[index] = hvol * hcur;
        printf("hvol: %.2f hcur: %.2f ", hvol, hcur);

#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
        // 低电流
        adc_hvli_input_conv(128);
        hvol = adc_dc_hvol_avg;
        lcur = adc_dc_lcur_avg;
        freq_pwr[index] = hvol * lcur;
        printf("hvol: %.2f lcur: %.2f ", hvol, lcur);
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
        // 高电流
        adc_hv_input_conv(128);
        hvol = adc_dc_hvol_avg;
        hcur = adc_dc_hcur_avg;
        freq_pwr[index] = hvol * hcur;
        printf("hvol: %.2f hcur: %.2f ", hvol, hcur);
#endif

#if USE_AIR_FLOWMETER
        // 获取空气流量
        sys_delayms(200);
        flow = get_air_flow();
        printf("flow: %d ", flow);
#endif
//#if MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_VPP
//        printf("%d, %.5f, %.5f, %.5f, %.5f, %d\r\n", freq, vpp, ipp, zx, phasex, flow);
//#elif MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_RMS
//        printf("%d, %.5f, %.5f, %.5f, %.5f, %d\r\n", freq, vrms, irms, zx, phasex, flow);
//#else

//#endif
        printf("\r\n");
        freq += step_freq;
        index++;
    }
    printf("magic_cool_calc_impedance imp end \r\n\r\n");
    ret = index;
    return ret;
}

void magic_cool_run_impedance(void)
{
    int len, i, cnt;
    int idx_max, idx_min;
    int freq_max, freq_min, gold_freq;
    int pwr_max_idx;
    int phase_max_idx;
    uint32_t hvol = 0, hcur = 0, lcur = 0;
    float val_max, val_min;
    float phasex = 0, phase_sum = 0, phase_max;
    float cur = 0.0;
    float cur_sum = 0.0;
    float flow = 0.0;


    pwm_enable(ENABLE);
    pwm_set_freq(25000);
    adc_output_conv(128);

    len = magic_cool_calc_impedance(magic_cool_freqstart, magic_cool_freqstop, 100);  // 阻抗/频率谱
    sys_delayms(200);

    /*  找极值方法：
     *  1. 先找极小值，如果有多个极小值，则选择最小的极小值
     *  2. 找极大值，极大值依赖于极小值，找高于极小值频率的极大值的最高值？？？？ 距离极小值最近的极大值？？？？？
     */
    find_extremum(impedance, len, &val_max, &val_min, &idx_max, &idx_min);  // 找极值
//    find_extremum_minima(impedance, len, &val_min, &idx_min);  // 找极小值
//    find_extremum_maxima(impedance, len, &val_max, &idx_max);  // 找极大值

    // 频率处理
    freq_min = magic_cool_freqstart + 100 * idx_min;  // 极小值对应频率
    freq_max = magic_cool_freqstart + 100 * idx_max;  // 极大值对应频率
//    freq_min = freq_max;
#if MAGIC_COOL_TRACK_DEFAULT == MAGIC_COOL_TRACK_IMPEDANCE
    gold_freq = freq_min;  // 阻抗最小点追频
    printf("min freq:%d, max freq:%d, gold freq:%d\r\n", freq_min, freq_max, gold_freq);
#elif MAGIC_COOL_TRACK_DEFAULT == MAGIC_COOL_TRACK_PHASE
// 相位追频
    // 频率区间测数据
//    freq_min = freq_max - 200;
//    freq_max = freq_max + 200;
    int temp = (freq_max - freq_min) / 50;
    for (i = 0; i < temp; i++) {   // 对谐振点和反谐振点中间的电流，相位重新扫描，作为设置阈值的基础
        pwm_set_freq(freq_min + i * 50);  // 步进50Hz
        magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 100);  // 电压闭环调整，电压误差±2V
        sys_delayms(200);  // 等待2ms，让运行平稳
#if MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_FFT
        // ADC采样，同时采样电压电流
        adc_output_conv(128);
        // ADC数据处理，FFT求相位差
        phasex = ProcessADCData(adc_voltage_data, adc_current_data);
#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_DOT
        // ADC采样，同时采样电压电流
        adc_output_conv(128);
        // ADC数据处理，点积求相位差
        phasex = get_phase_difference(adc_voltage_data, adc_current_data, 128);
#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_CAPTURE
        // 过零比较器求相位差
        adc_output_conv(128);
        set_compare_dcoffset(128);
        sys_delayms(100);
        phasex = get_capture(8);
#else    // 其他方式，不使用相位追频
        phasex = 0;
#endif
        phase_sum += phasex;
        phase[i] = phasex;
#if USE_AIR_FLOWMETER
        // 获取空气流量
        sys_delayms(1500);
        flow = get_air_flow();
#endif
        printf("freq:%d, phase:%.5f flow: %.5f\r\n", freq_min + i * 50, phasex, flow);
    }
    find_maxima(phase, temp, &phase_max, &phase_max_idx);  // 找相位最大值

    phase_sum = 0;
    cnt = 0;
    for (i = phase_max_idx - 2; i < phase_max_idx + 2; i++) {
        if (i < 0)
            continue;
        phase_sum += phase[i];
        cnt++;
        printf("%d %.5f\r\n", i, phase[i]);
    }
    gold_freq = freq_min + phase_max_idx * 50;  // 相位最大值对应的频率，相位差最小值
    magic_cool_ph_proxth = phase_max - (phase_max - (phase_sum / cnt)) / 2.0; // // 取平均相位时间和最小相位的中间值作为相位阈值

#elif MAGIC_COOL_TRACK_DEFAULT == MAGIC_COOL_TRACK_CURRENT  // TODO: 电流追频方法未测试，待定

    find_maxima_i(freq_pwr, len, &magic_cool_pwr_max, &pwr_max_idx);  // 找最大值
    freq_min = magic_cool_freqstart + 100 * pwr_max_idx - 250;
//    freq_max = magic_cool_freqstart + 100 * pwr_max_idx + 250;
    pwr_max_idx = 0;
    freq_pwr[pwr_max_idx] = 0;
    for (i = 0; i < 10; i++) {  // 扫描前后250Hz 共500Hz范围 步进50Hz，10个点
        pwm_set_freq(freq_min + i * 50);
        magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 100);  // 电压闭环调整，电压误差±2V
        for (cnt = 0; cnt < 10; cnt++) {
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
            // 低电流
            adc_hvli_input_conv(128);
            hvol = adc_dc_hvol_avg;
            lcur = adc_dc_lcur_avg;
            freq_pwr[i] += hvol * lcur;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
            // 高电流
            adc_hv_input_conv(128);
            hvol = adc_dc_hvol_avg;
            hcur = adc_dc_hcur_avg;
            freq_pwr[i] += hvol * hcur;

#endif
            sys_delayms(10); // 错开一段时间

        }
        freq_pwr[i] = freq_pwr[i] / 10;

        if (freq_pwr[pwr_max_idx] < freq_pwr[i]) {
            pwr_max_idx = i;
        }
        printf("%d %d\r\n", freq_min + i * 50, freq_pwr[i]);
    }
    magic_cool_pwr_max = freq_pwr[pwr_max_idx];

    gold_freq = freq_min + 50 * pwr_max_idx;
//    printf("freq:%d, idx:%d, pwr:%d\r\n", gold_freq, pwr_max_idx, magic_cool_pwr_max);
#else  // 其他方式, 待定
    gold_freq = freq_min;
#endif

    magic_cool_runfreq = gold_freq; // 初始频率为相位最大值对应的频率
    printf("freq:%d, phase proxth:%f,  pwr max :%d\r\n", magic_cool_runfreq, magic_cool_ph_proxth, magic_cool_pwr_max);

    // 设置输出频率
    pwm_set_freq(magic_cool_runfreq); // 阻抗谱计算最优频率
    magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 100);// 电压闭环
    sys_delayms(200);
}

#if MAGIC_COOL_TRACK_DEFAULT == MAGIC_COOL_TRACK_PHASE   // 相位追频

#if MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_FFT     // fft 追频

#if 0
// 频率朝向调整，检测相位，缓慢移动频率，直到相位差最小或次数达到最大
void magic_cool_freq_track_phase_fft(void)
{
    int loop = 5;
    int i = 0, count = 5;
    int freq;
    int freq_stepx = 20;
    int vpp;
    float ph1, ph2;

    // 获取相位，
    ph1 = 0.0;
    for (i = 0; i < count; i++) {
        adc_output_conv(128);
        // ADC数据处理，FFT
        ph1 += ProcessADCData(adc_voltage_data, adc_current_data);
    }
    ph1 = ph1 / count;
    if (ph1 < magic_cool_ph_proxth)
    {
        while(loop--) {
            if (magic_cool_adj_dir) {
                pwm_freq_increase(freq_stepx);  // 频率增加
            } else {
                pwm_freq_decrease(freq_stepx);  // 频率减小
            }
            magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 50);
            sys_delayms(100);
            ph2 = 0.0;
            for (i = 0; i < count; i++) {
                // ADC采样，同时采样电压电流
                adc_output_conv(128);
                // ADC数据处理，FFT
                ph2 += ProcessADCData(adc_voltage_data, adc_current_data);
            }
            ph2 = ph2 / count;
            vpp = adc_vpp;
            freq = pwm_get_freq();
            printf("vpp:%d freq:%d ph2:%f\r\n", vpp, freq, ph2);

//            if (ph2 > magic_cool_ph_proxth)
//                return;

            if (ph2 < ph1) { // 与上次数据作对比，判断调整方向是否正确，如果不正确则改变调整方向
                magic_cool_adj_dir = !magic_cool_adj_dir;
                freq_stepx = freq_stepx + 0;
            }

            // 反方向调
            if (magic_cool_adj_dir) {
                pwm_freq_increase(freq_stepx);  // 频率增加
            } else {
                pwm_freq_decrease(freq_stepx);  // 频率减小
            }
            magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 50);// 电压闭环
            sys_delayms(100);
            ph1 = 0.0;
            for (i = 0; i < count; i++) {
                adc_output_conv(128);
                // ADC数据处理，FFT
                ph1 += ProcessADCData(adc_voltage_data, adc_current_data);
            }
            ph1 = ph1 / count;
            vpp = adc_vpp;
            freq = pwm_get_freq();
            printf("vpp:%d freq:%d ph1:%f\r\n", vpp, freq, ph1);

//            if (ph1 > magic_cool_ph_proxth)
//                return;

        }
    }
}

#else

// 三次不同频率采样，分别是当前频率，当前频率+20Hz，当前频率-20Hz，
// 计算相位差，如果当前频率相位差最小，退出，
// 如果当前频率+20Hz相位差最小，则调整频率+20Hz，
// 如果当前频率-20Hz相位差最小，则调整频率-20Hz，
// 循环直到相位差最小或次数达到最大
void magic_cool_freq_track_phase_fft(void)
{
    int loop = 5;
    int i = 0, count = 5;
    int freq;
    int freq_stepx = 20;
    int vpp;
    int freq0, freq1, freq2;
    float ph0, ph1, ph2;
    float zx0, zx1, zx2;
    float dx = 0.005;

    // 获取当前频率相位，
    ph1 = 0.0;
    for (i = 0; i < count; i++) {
        adc_output_conv(128);
        // ADC数据处理，FFT
        ph1 += ProcessADCData(adc_voltage_data, adc_current_data);
        zx1 += adc_vrms / adc_irms;
    }
    zx1 = zx1 / count;
    ph1 = ph1 / count;

    if (ph1 < magic_cool_ph_proxth)
    {
        while(loop--) {
            ph1 = 0.0;
            zx1 = 0.0;
            for (i = 0; i < count; i++) {
                adc_output_conv(128);
                // ADC数据处理，FFT
                ph1 += ProcessADCData(adc_voltage_data, adc_current_data);  // 获取当前频率相位
                zx1 += adc_vrms / adc_irms;
            }
            zx1 = zx1 / count;
            ph1 = ph1 / count;
            vpp = adc_vpp;
            freq1 = pwm_get_freq();
            printf("vpp:%d freq:%d ph1:%.5f zx1:%.5f\r\n", vpp, freq1, ph1, zx1);

            pwm_freq_increase(freq_stepx);  // 频率增加
            magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 100);
            sys_delayms(500);
            ph2 = 0.0;
            zx2 = 0.0;
            for (i = 0; i < count; i++) {
                // ADC采样，同时采样电压电流
                adc_output_conv(128);
                // ADC数据处理，FFT
                ph2 += ProcessADCData(adc_voltage_data, adc_current_data);  // 获取当前频率+20Hz相位
                zx2 += adc_vrms / adc_irms;
            }
            zx2 = zx2 / count;
            ph2 = ph2 / count;

            vpp = adc_vpp;
            freq2 = pwm_get_freq();
            printf("vpp:%d freq:%d ph2:%.5f zx2:%.5f\r\n", vpp, freq2, ph2, zx2);

//            // 反方向调
//            if (magic_cool_adj_dir) {
//                pwm_freq_increase(freq_stepx);  // 频率增加
//            } else {
//                pwm_freq_decrease(freq_stepx);  // 频率减小
//            }
            pwm_freq_decrease(freq_stepx + freq_stepx);  // 频率减小
            magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 100);// 电压闭环
            sys_delayms(500);
            ph0 = 0.0;
            zx0 = 0.0;
            for (i = 0; i < count; i++) {
                adc_output_conv(128);
                // ADC数据处理，FFT
                ph0 += ProcessADCData(adc_voltage_data, adc_current_data);  // 获取当前频率-20Hz相位
                zx0 += adc_vrms / adc_irms;
            }
            zx0 = zx0 / count;
            ph0 = ph0 / count;
            vpp = adc_vpp;
            freq0 = pwm_get_freq();
            printf("vpp:%d freq:%d ph0:%.5f zx0:%.5f\r\n", vpp, freq0, ph0, zx0);

            if ((ph1 - ph0) > dx && (ph1 - ph2) > dx) {  // 当前频率相位差最小
                pwm_set_freq(freq1);
                magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 100);// 电压闭环
                break;
            }
            if ((ph0 - ph1) > dx && (ph1 - ph2) > dx) {  // 当前频率-20Hz相位差最小
                pwm_set_freq(freq0);
                magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 100);// 电压闭环
                continue;
            }
            if ((ph2 - ph1) > dx && (ph1 - ph0) > dx) {  // 当前频率+20Hz相位差最小
                pwm_set_freq(freq2);
                magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 100);// 电压闭环
                continue;
            }
        }
    }
}
#endif

#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_DOT
// 点积追频
void magic_cool_freq_track_phase_dot(void)
{
    int loop = 5;
    int freq;
    int freq_stepx = 20;
    float ph1, ph2;
    int vpp;

    // 获取相位，获取到的数据是绝对值，在追频的时候追返回相位的最小值

    adc_output_conv(128);
    // ADC数据处理，点积求相位差
    ph1 = get_phase_difference(adc_voltage_data, adc_current_data, 128);
    if (ph1 < magic_cool_ph_proxth)
    {
        while(loop--) {
            if (magic_cool_adj_dir) {
                pwm_freq_increase(freq_stepx);  // 频率增加
            } else {
                pwm_freq_decrease(freq_stepx);  // 频率减小
            }
            magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 50);
            sys_delayms(100);
            // ADC采样，同时采样电压电流
            adc_output_conv(128);
            vpp = adc_vpp;
            // ADC数据处理，点积求相位差
            ph2 = get_phase_difference(adc_voltage_data, adc_current_data, 128);
            freq = pwm_get_freq();
            printf("vpp:%d freq:%d ph2:%f\r\n", vpp, freq, ph2);

            if (ph2 > magic_cool_ph_proxth)
                return;

            if (ph2 < ph1) { // 与上次数据作对比，判断调整方向是否正确，如果不正确则改变调整方向
                magic_cool_adj_dir = !magic_cool_adj_dir;
                freq_stepx = freq_stepx + 0;
            }

            // 反方向调
            if (magic_cool_adj_dir) {
                pwm_freq_increase(freq_stepx);  // 频率增加
            } else {
                pwm_freq_decrease(freq_stepx);  // 频率减小
            }
            magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 50);// 电压闭环
            sys_delayms(100);
            adc_output_conv(128);
            vpp = adc_vpp;
            // ADC数据处理，点积求相位差
            ph1 = get_phase_difference(adc_voltage_data, adc_current_data, 128);
            freq = pwm_get_freq();
            printf("vpp:%d freq:%d ph1:%f\r\n", vpp, freq, ph1);

            if (ph1 > magic_cool_ph_proxth)
                return;

        }
    }

}

#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_CAPTURE
// 过零比较器追频
void magic_cool_freq_track_phase_capture(void)
{
    int loop = 5;
    int freq;
    int freq_stepx = 20;
    float ph1, ph2;
    int vpp;

    // 获取相位，
    adc_voltage_current_get(128);
    set_compare_dcoffset(128);
    sys_delayms(150);
    ph1 = get_capture(8);

    if (ph1 > magic_cool_ph_proxth)
    {
        while(loop--) {
            if (magic_cool_adj_dir) {
                pwm_freq_increase(freq_stepx);  // 频率增加
            } else {
                pwm_freq_decrease(freq_stepx);  // 频率减小
            }
            magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 50);
            sys_delayms(100);
            adc_voltage_current_get(128);
            vpp = find_peak_to_peak(adc_voltage_data, 128);
            set_compare_dcoffset(128);
            sys_delayms(150);
            ph2 = get_capture(8);
            freq = pwm_get_freq();
            printf("vpp:%d freq:%d ph2:%f\r\n", vpp, freq, ph2);

            if (ph2 < magic_cool_ph_proxth)
                return;

            if (ph2 > ph1) { // 与上次数据作对比，判断调整方向是否正确，如果不正确则改变调整方向
                magic_cool_adj_dir = !magic_cool_adj_dir;
                freq_stepx = freq_stepx + 0;
            }

            // 反方向调
            if (magic_cool_adj_dir) {
                pwm_freq_increase(freq_stepx);  // 频率增加
            } else {
                pwm_freq_decrease(freq_stepx);  // 频率减小
            }
            magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 50);// 电压闭环
            sys_delayms(100);
            adc_voltage_current_get(128);
            vpp = find_peak_to_peak(adc_voltage_data, 128);
            set_compare_dcoffset(128);
            sys_delayms(150);
            ph1 = get_capture(8);
            freq = pwm_get_freq();
            printf("vpp:%d freq:%d ph1:%f\r\n", vpp, freq, ph1);

            if (ph1 < magic_cool_ph_proxth)
                return;

        }
    }
}

#else  // 其他方式，待定

#endif

void magic_cool_freq_track_phase(void)
{
#if MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_FFT
    magic_cool_freq_track_phase_fft();
#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_DOT
    magic_cool_freq_track_phase_dot();
#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_CAPTURE
    magic_cool_freq_track_phase_capture();
#else  // 其他方式，待定

#endif
}

#elif MAGIC_COOL_TRACK_DEFAULT == MAGIC_COOL_TRACK_IMPEDANCE
// 阻抗追频
void magic_cool_freq_track_imp()
{
    int loop = 2;
    int freq1,freq2,freq3;
    int i;
    int loop_time = 25;
    int freq_stepx = 20;
    float vpp, ipp;
    float vrms, irms, vrms_sum, irms_sum;
    float imp1, imp2, imp3;

__again:
    magic_cool_voltage_closeloop_dcdc( magic_cool_target_vol, 2, 100);// 电压闭环
    sys_delayms(100);
#if MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_VPP
    // ADC数据处理，
    vpp = 0; ipp = 0;
    for (i = 0; i < loop_time; i++) {
        // ADC采样，同时采样电压电流
        adc_output_conv(128);
        vpp += adc_vpp;
        ipp += adc_ipp;
        sys_delayms(10);
    }
    imp1 = vpp / ipp;
    freq1 = pwm_get_freq();
    printf("vpp:%f freq1:%d imp1:%f \r\n", vpp, freq1, imp1);
#elif MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_RMS
    for (i = 0; i < loop_time; i++) {
        sys_delayms(10);
        // ADC采样，同时采样电压电流
        adc_output_conv(128);  // 需要处理好电压电流的中心对称
        get_vol_cur_rms(adc_voltage_data, adc_current_data, 128, &vrms, &irms);
        vrms_sum += vrms;
        irms_sum += irms;
    }
    imp1 = vrms_sum / irms_sum;
    freq1 = pwm_get_freq();
    printf("vrms:%f freq1:%d imp1:%f \r\n", vrms_sum, freq1, imp1);
#else  // 其他方式，待定

#endif

    pwm_freq_increase(freq_stepx);  // 频率增加
    magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
    sys_delayms(100);
#if MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_VPP
    // ADC数据处理，
    vpp = 0, ipp = 0;
    for (i = 0; i < loop_time; i++) {
        // ADC采样，同时采样电压电流
        adc_output_conv(128);
        vpp += adc_vpp;
        ipp += adc_ipp;
        sys_delayms(10);
    }
    imp2 = vpp / ipp;
    freq2 = pwm_get_freq();
    printf("vpp:%f freq2:%d imp2:%f\r\n", vpp, freq2, imp2);
#elif MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_RMS
    for (i = 0; i < loop_time; i++) {
        sys_delayms(10);
        // ADC采样，同时采样电压电流
        adc_output_conv(128);  // 需要处理好电压电流的中心对称
        get_vol_cur_rms(adc_voltage_data, adc_current_data, 128, &vrms, &irms);
        vrms_sum += vrms;
        irms_sum += irms;
    }
    imp2 = vrms_sum / irms_sum;
    freq2 = pwm_get_freq();
    printf("vrms:%f freq2:%d imp2:%f\r\n", vrms_sum, freq2, imp2);
#else  // 其他方式，待定

#endif

    pwm_freq_decrease(freq_stepx + freq_stepx);  // 频率减小
    magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
    sys_delayms(100);

#if MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_VPP
    // ADC数据处理，
    vpp = 0; ipp = 0;
    for (i = 0; i < loop_time; i++) {
        // ADC采样，同时采样电压电流
        adc_output_conv(128);
        vpp += adc_vpp;
        ipp += adc_ipp;
        sys_delayms(10);
    }
    imp3 = vpp / ipp;
    freq3 = pwm_get_freq();
    printf("vpp:%f freq3:%d imp3:%f\r\n", vpp, freq3, imp3);
#elif MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_RMS
    for (i = 0; i < loop_time; i++) {
        sys_delayms(10);
        // ADC采样，同时采样电压电流
        adc_output_conv(128);  // 需要处理好电压电流的中心对称
        get_vol_cur_rms(adc_voltage_data, adc_current_data, 128, &vrms, &irms);
        vrms_sum += vrms;
        irms_sum += irms;
    }
    imp3 = vrms_sum / irms_sum;
    freq3 = pwm_get_freq();
    printf("vrms:%f freq3:%d imp3:%f\r\n", vrms_sum, freq3, imp3);
#else  // 其他方式，待定

#endif

    if (imp1 < imp2 && imp1 < imp3) {
        pwm_set_freq(freq1);
        magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
        return;
    } else {  // 查找最小点
        if (imp3 < imp1 && imp3 < imp2) {
            if (abs_f(imp1 - imp3) > IMPEDANCE_THRESHOLD && abs_f(imp2 - imp3) > IMPEDANCE_THRESHOLD) {
                pwm_set_freq(freq3);
                magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
            }
        } else if (imp2 < imp1 && imp2 < imp3) {
            if (abs_f(imp1 - imp2) > IMPEDANCE_THRESHOLD && abs_f(imp3 - imp2) > IMPEDANCE_THRESHOLD) {
                pwm_set_freq(freq2);
                magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
            }
        }
        if (loop > 0) {
            loop--;
            sys_delayms(100);
            goto __again;
        }
    }

}

#elif MAGIC_COOL_TRACK_DEFAULT == MAGIC_COOL_TRACK_CURRENT
// 电流追频, 找功率最大点
#if 0
// 固定最大值方式
void magic_cool_freq_track_current(void)
{
    int loop = 2;
    int freq0,freq1,freq2;
    int i,j;
    int n = 5;
    int loop_time = 5;
    int freq_stepx = 20;
    int pwr0, pwr1, pwr2;
    int pwrx = 0;
    uint32_t dc_vol, dc_cur;
    int pwr_proxth = 9000;


    for (i = 0; i < n; i++) {
        sys_delayms(10);
        // ADC采样，同时采样电压电流
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW  // 直流高压输入电压,低端电流
        adc_hvli_input_conv(128);
        dc_vol = adc_dc_hvol_avg;
        dc_cur = adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH  // 直流高压输入电压,高端电流
        adc_hv_input_conv(128);
        dc_vol = adc_dc_hvol_avg;
        dc_cur = adc_dc_hcur_avg;
#else  // 其他方式，待定

#endif
        pwrx += dc_vol * dc_cur;
    }
    pwrx = pwrx / n;
    freq0 = pwm_get_freq();
    printf("freq: %d pwr:%d\r\n", freq0, pwrx);

    printf("absx: %d\r\n", abs_i(magic_cool_pwr_max - pwrx));
    if (abs_i(magic_cool_pwr_max - pwrx) < pwr_proxth) {
        return;
    }

    for (i = 0; i < loop_time; i++) {
        // 获取当前频率
        freq1 = pwm_get_freq();
        // 获取当前频率+20Hz频率
        freq2 = freq1 + freq_stepx;
        // 获取当前频率-20Hz频率
        freq0 = freq1 - freq_stepx;

        pwm_set_freq(freq1);  // 设置当前频率
        magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
        sys_delayms(100);
        pwr1 = 0;
        for (j = 0; j < n; j++) {
            sys_delayms(10);
            // ADC采样，同时采样电压电流
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW  // 直流高压输入电压,低端电流
            adc_hvli_input_conv(128);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH  // 直流高压输入电压,高端电流
            adc_hv_input_conv(128);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_hcur_avg;
#else  // 其他方式，待定

#endif
            pwr1 += dc_vol * dc_cur;
        }
        pwr1 = pwr1 / n;
        printf("freq1: %d pwr1:%d\r\n", freq1, pwr1);

        pwm_set_freq(freq0);  // 设置当前频率-20Hz
        magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
        sys_delayms(100);
        pwr0 = 0;
        for (j = 0; j < n; j++) {
            sys_delayms(10);
            // ADC采样，同时采样电压电流
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW  // 直流高压输入电压,低端电流
            adc_hvli_input_conv(128);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH  // 直流高压输入电压,高端电流
            adc_hv_input_conv(128);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_hcur_avg;
#else  // 其他方式，待定

#endif
            pwr0 += dc_vol * dc_cur;
        }
        pwr0 = pwr0 / n;
        printf("freq0: %d pwr0:%d\r\n", freq0, pwr0);

        pwm_set_freq(freq2);  // 设置当前频率+20Hz
        magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
        sys_delayms(100);
        pwr2 = 0;
        for (j = 0; j < n; j++) {
            sys_delayms(10);
            // ADC采样，同时采样电压电流
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW  // 直流高压输入电压,低端电流
            adc_hvli_input_conv(128);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH  // 直流高压输入电压,高端电流
            adc_hv_input_conv(128);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_hcur_avg;
#else  // 其他方式，待定
#endif
            pwr2 += dc_vol * dc_cur;
        }
        pwr2 = pwr2 / n;
        printf("freq2: %d pwr2:%d\r\n", freq2, pwr2);

        if (((pwr0 - pwr1) > pwr_proxth) && ((pwr0 - pwr2) > pwr_proxth)) {
            pwm_set_freq(freq0);
            printf("set freq0:%d\r\n", freq0);
            magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
            continue;
        } else if (((pwr1 - pwr0) > pwr_proxth) && ((pwr1 - pwr2) > pwr_proxth)) {
            pwm_set_freq(freq1);
            printf("set freq1:%d\r\n", freq1);
            magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
            break;
        } else if (((pwr2 - pwr0) > pwr_proxth) && ((pwr2 - pwr1) > pwr_proxth)) {
            pwm_set_freq(freq2);
            printf("set freq2:%d\r\n", freq2);
            magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
            continue;
        } else {
            pwm_set_freq(freq1);
        }

    }
}

#else
// 动态更新最大值方式
void magic_cool_freq_track_current(void)
{
    uint32_t temp = 0;
    int freq0,freq1,freq2;
    int i,j;
    int n = 5;
    int loop_time = 5;
    int freq_stepx = 20;
    int pwr0, pwr1, pwr2;
    int pwrx = 0;
    uint32_t dc_vol, dc_cur;
    int pwr_proxth = 9000;

    static uint32_t scan_tick = 0;
    uint32_t index = 0;
    uint32_t pwr_arr[32] = {0};

#if 0
    if (get_systick() > scan_tick) {
        scan_tick = get_systick() + 120000;    // 2min = 2 * 60s * 1000
        freq1 = pwm_get_freq();
        j = 0;
        for (temp = freq1 - 500; temp < (freq1 + 500); temp += 50) {  // ±500Hz 扫频
            pwm_set_freq(temp);  // 设置当前频率
            magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
            sys_delayms(200);
            for (i = 0; i < n; i++) {
                sys_delayms(50);
                // ADC采样，同时采样电压电流
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW  // 直流高压输入电压,低端电流
                adc_hvli_input_conv(128);
                dc_vol = adc_dc_hvol_avg;
                dc_cur = adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH  // 直流高压输入电压,高端电流
                adc_hv_input_conv(128);
                dc_vol = adc_dc_hvol_avg;
                dc_cur = adc_dc_hcur_avg;
#else  // 其他方式，待定

#endif
                pwrx += dc_vol * dc_cur;
            }
            pwrx = pwrx / n;
            pwr_arr[j++] = pwrx;
            printf("--scan freq:%d pwr:%d\r\n", temp, pwrx);
        }


        magic_cool_pwr_max = pwr_arr[0];
        for (i = 0; i < j; i++) {
            if (magic_cool_pwr_max < pwr_arr[i]) {
                magic_cool_pwr_max = pwr_arr[i];
                temp = i;
            }
        }
        temp = freq1 - 500 + temp * 50;
        printf("--scan max freq:%d pwr:%d\r\n", temp, magic_cool_pwr_max);
        pwm_set_freq(temp);  // 设置当前频率
        magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
        sys_delayms(200);
    }
#endif

    for (i = 0; i < n; i++) {
        sys_delayms(10);
        // ADC采样，同时采样电压电流
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW  // 直流高压输入电压,低端电流
        adc_hvli_input_conv(128);
        dc_vol = adc_dc_hvol_avg;
        dc_cur = adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH  // 直流高压输入电压,高端电流
        adc_hv_input_conv(128);
        dc_vol = adc_dc_hvol_avg;
        dc_cur = adc_dc_hcur_avg;
#else  // 其他方式，待定

#endif
        pwrx += dc_vol * dc_cur;
    }
    pwrx = pwrx / n;
    freq0 = pwm_get_freq();
    printf("freq: %d pwr:%d\r\n", freq0, pwrx);

    printf("absx: %d\r\n", abs_i(magic_cool_pwr_max - pwrx));
    if (abs_i(magic_cool_pwr_max - pwrx) < pwr_proxth) {
        return;
    }

    for (i = 0; i < loop_time; i++) {
        // 获取当前频率
        freq1 = pwm_get_freq();
        // 获取当前频率+20Hz频率
        freq2 = freq1 + freq_stepx;
        // 获取当前频率-20Hz频率
        freq0 = freq1 - freq_stepx;

        pwm_set_freq(freq1);  // 设置当前频率
        magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
        sys_delayms(200);
        pwr1 = 0;
        for (j = 0; j < n; j++) {
            sys_delayms(10);
            // ADC采样，同时采样电压电流
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW  // 直流高压输入电压,低端电流
            adc_hvli_input_conv(128);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH  // 直流高压输入电压,高端电流
            adc_hv_input_conv(128);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_hcur_avg;
#else  // 其他方式，待定

#endif
            pwr1 += dc_vol * dc_cur;
        }
        pwr1 = pwr1 / n;
        printf("freq1: %d pwr1:%d\r\n", freq1, pwr1);

        pwm_set_freq(freq0);  // 设置当前频率-20Hz
        magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
        sys_delayms(200);
        pwr0 = 0;
        for (j = 0; j < n; j++) {
            sys_delayms(10);
            // ADC采样，同时采样电压电流
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW  // 直流高压输入电压,低端电流
            adc_hvli_input_conv(128);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH  // 直流高压输入电压,高端电流
            adc_hv_input_conv(128);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_hcur_avg;
#else  // 其他方式，待定

#endif
            pwr0 += dc_vol * dc_cur;
        }
        pwr0 = pwr0 / n;
        printf("freq0: %d pwr0:%d\r\n", freq0, pwr0);

        pwm_set_freq(freq2);  // 设置当前频率+20Hz
        magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
        sys_delayms(200);
        pwr2 = 0;
        for (j = 0; j < n; j++) {
            sys_delayms(10);
            // ADC采样，同时采样电压电流
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW  // 直流高压输入电压,低端电流
            adc_hvli_input_conv(128);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH  // 直流高压输入电压,高端电流
            adc_hv_input_conv(128);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_hcur_avg;
#else  // 其他方式，待定
#endif
            pwr2 += dc_vol * dc_cur;
        }
        pwr2 = pwr2 / n;
        printf("freq2: %d pwr2:%d\r\n", freq2, pwr2);

        if (((pwr0 - pwr1) > pwr_proxth) && ((pwr0 - pwr2) > pwr_proxth)) {
            pwm_set_freq(freq0);
            printf("set freq0:%d\r\n", freq0);
            magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
            continue;
        } else if (((pwr1 - pwr0) > pwr_proxth) && ((pwr1 - pwr2) > pwr_proxth)) {
            pwm_set_freq(freq1);
            printf("set freq1:%d\r\n", freq1);
            magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
            break;
        } else if (((pwr2 - pwr0) > pwr_proxth) && ((pwr2 - pwr1) > pwr_proxth)) {
            pwm_set_freq(freq2);
            printf("set freq2:%d\r\n", freq2);
            magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 2, 100);// 电压闭环
            continue;
        } else {
            pwm_set_freq(freq1);
        }

    }
}
#endif

#else  // 其他方式, 待定

#endif


void magic_cool_freq_track(void)
{
    static int tick_ph = 50, tick_cur = 100, tick_vol = 1500, tick_imp = 50;
    int feedback_tick = 15000;

#if KEY_VOL_CFG

#else
    if (manual == 1)
        return;
#endif

#if MAGIC_COOL_TRACK_DEFAULT == MAGIC_COOL_TRACK_PHASE
   // 相位最小，
   if (get_systick() >= tick_ph) {
       magic_cool_freq_track_phase();
       tick_ph = get_systick() + feedback_tick;
   }
#elif MAGIC_COOL_TRACK_DEFAULT == MAGIC_COOL_TRACK_IMPEDANCE
    // 阻抗最小
    if (get_systick() >= tick_imp) {
        magic_cool_freq_track_imp();
        tick_imp = get_systick() + feedback_tick;
    }
#elif MAGIC_COOL_TRACK_DEFAULT == MAGIC_COOL_TRACK_CURRENT
    // 电流
    if (get_systick() >= tick_cur) {
        magic_cool_freq_track_current();
        tick_cur = get_systick() + feedback_tick;
    }
#else  // 其他方式, 待定

#endif

    if (get_systick() >= tick_vol) {
        magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 50);// 电压闭环
        tick_vol = get_systick() + 500;
    }
}

void magic_cool_set_target_vol(uint32_t vol)
{
    magic_cool_target_vol = vol;
}

void magic_cool_config_vol(uint32_t opt)
{
    if (opt == 1) {
        magic_cool_target_vol -= 5;
    } else if (opt == 2) {
        magic_cool_target_vol += 5;
    }

    if (magic_cool_target_vol > 120) {
        magic_cool_target_vol = 120;
    } else if (magic_cool_target_vol < 70) {
        magic_cool_target_vol = 70;
    }
    printf("set voltage %d V\r\n", magic_cool_target_vol);
}

void magic_cool_config_freq(uint32_t opt)
{
    if (opt == 2)
        pwm_freq_increase(50);
    else if (opt == 1)
        pwm_freq_decrease(50);
}

void magic_cool_set_mode(uint32_t mode)
{
    magic_cool_mode = mode;
    if (mode == 2) {
        mode2_tick = get_systick();
    }
}

void magic_cool_mode2(void)
{
    if (mode2_tick > get_systick()) {  // 500ms更新一次
        return;
    }

    int i = 0;
    int flow = 0;
    int freq = 0;
    float vpp, ipp;
    float imp = 0.0;
    float phase = 0.0;
    float hvol = 0.0, hcur = 0.0, lcur = 0.0, power;

//    printf("systick:%d\r\n", get_systick());

//    sys_delayms(500);
    mode2_tick = get_systick() + 2000;
    // ADC采样，同时采样电压电流
    adc_output_conv(128);
    vpp = adc_vpp;
    ipp = adc_ipp;
    vpp = vpp / voltage_gain;  // 转成真实电压

#if MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_VPP
    imp = vpp / ipp;
#elif MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_RMS
    imp = adc_vrms / adc_irms;
#else  // 其他方式，待定
#endif

#if MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_FFT
    phase = ProcessADCData(adc_voltage_data, adc_current_data);
#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_DOT
    phase = get_phase_difference(adc_voltage_data, adc_current_data, 128);
#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_CAPTURE
    set_compare_dcoffset(128);
    sys_delayms(100);
    phase = get_capture(8);
#else  // 其他方式，待定
#endif

//  for (i = 0; i < 128; i++) {  // 打印电压电流
//      printf("%f %f\r\n", adc_voltage_data[i], adc_current_data[i]);
//  }
#if USE_AIR_FLOWMETER
    flow = get_air_flow();
#endif

    freq = pwm_get_freq();
// 功率计算
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
        // 低电流
        adc_hvli_input_conv(128);
        hvol = adc_dc_hvol_avg;
        lcur = adc_dc_lcur_avg;
        power = hvol * lcur * 0.000885102;
        printf("freq:%d, vpp:%0.2f, ipp:%.2f, imp: %.3f phase: %.3f hvol: %.2f lcur: %.2f power:%.2f flow:%d\r\n", freq, vpp, ipp, imp, phase, hvol, lcur, power, flow);
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
        // 高电流
        adc_hv_input_conv(128);
        hvol = adc_dc_hvol_avg;
        hcur = adc_dc_hcur_avg;
        power = hvol * hcur * 0.000885102;
        printf("freq:%d, vpp:%0.2f, ipp:%.2f, imp: %.3f phase: %.3f hvol: %.2f hcur: %.2f power:%.2f flow:%d\r\n", freq, vpp, ipp, imp, phase, hvol, hcur, power, flow);
#endif
}

void magic_cool_test(void)
{
    magic_cool_run_impedance();
}


void magic_cool_config(void)
{
#if USE_AIR_FLOWMETER
    air_flowmeter_init();
//    air_flowmeter_test();
#endif
    sys_delayms(200);
    pid_init();
    magic_cool_set_adcfreq();
    magic_cool_set_limt(25000, 29000);
    pwm_set_config(26500, 50);  // KHz  50%占空比
    pwm_enable(DISABLE);
    set_power_enable(ENABLE);
    magic_cool_set_target_vol(50);   // 设置运行电压
    pwm1_set_duty(pwm1_duty_out);  // 设置DAC输出DCDC
    // set_dac_output(2, 1024);  // cur offset
    magic_cool_mode = 0;
}


void magic_cool_run(void)
{
//    int i;
//    int vppf, ippf;
    if (magic_cool_mode == 0) {
        return;
    } else if (magic_cool_mode == 1) {  // 校准,阻抗谱
        magic_cool_set_target_vol(50);   // 设置运行电压
        magic_cool_run_impedance();
//        pwm_set_freq(ch, 28000);
        magic_cool_mode = 3;
        magic_cool_set_target_vol(50);   // 设置运行电压
//        oled_show_ui();
    } else if (magic_cool_mode == 2) {  // 不追频运行，打印电压电流流量
        magic_cool_mode2();
        magic_cool_voltage_closeloop(magic_cool_target_vol, 3, 100);  // 电压闭环;
    } else if (magic_cool_mode == 3) {  // 追频运行
        magic_cool_freq_track();
        magic_cool_mode2();
    } else if (magic_cool_mode == 4) {  // 只电压闭环
        magic_cool_voltage_closeloop(magic_cool_target_vol, 3, 100);// 电压闭环    ;
    } else if (magic_cool_mode == 5) {  // 运行test
        magic_cool_test();
//        magic_cool_mode = 0;
    }
}



#endif




