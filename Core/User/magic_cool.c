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
#include "opa.h"
#include "flash_ops.h"

#include "usart1.h"
#ifndef ENABLE_QUERY_CMD
  #define ENABLE_QUERY_CMD 0
#endif

#if RM_MAGIC_COOL

// 重定义差分和PID宏定义
#undef MAGIC_COOL_DIFF
#undef MAGIC_COOL_PID
#if PWM_DRIVER_METHOD == PWM_DIFFERENTIAL_DRIVE
    #define MAGIC_COOL_DIFF 1
    #define MAGIC_COOL_PID  0
#else
    #define MAGIC_COOL_DIFF 0
    #define MAGIC_COOL_PID  1
#endif

// 没有定义PER_ENABLE时，默认关闭
#ifndef ENABLE_PER
#define ENABLE_PER  0
#endif

#ifndef ENABLE_WATER_INTRUSION
#define ENABLE_WATER_INTRUSION 0
#endif

/*****************************************************************/
#if ENABLE_QUERY_CMD
typedef struct {
    uint32_t vol;
    uint32_t cur;
} vol_cur_data_t;

vol_cur_data_t max_vol_cur_data, current_vol_cur_data;
#endif

uint8_t current_flow_level = FLOW_LEVEL_100_PERCENT;  // 当前流量档位
protocol_fault_t fault_status = FAULT_NORMAL;  // 当前故障状态
protocol_fault_t fault_vol_status = FAULT_NORMAL;  // 当前过压故障状态
protocol_fault_t fault_cur_status = FAULT_NORMAL;  // 当前过流故障状态
bool reset_time_flag = false;

#if ENABLE_KEY_VOL_CFG
volatile bool led_always_on = 0;
#endif
bool scan_freq_enable = false;
bool first_scan_freq = false;
bool reset_pwr_proxth_flag = false;
#if ENABLE_WATER_INTRUSION
bool reset_water_intrusion_flag = false;
#endif
uint16_t magic_cool_vpp = 0;

static uint32_t tick_cur = 100;
static uint16_t feedback_tick = 20000;


int32_t pwm1_duty_out = PWM1_MIN_POWER_DUTY;
uint16_t pwm1_duty_limit_min = PWM1_MAX_POWER_DUTY;
uint16_t pwm1_duty_limit_max = PWM1_MIN_POWER_DUTY;
/*****************************************************************/
float voltage_gain = MAGIC_COOL_VOLTAGE_GAIN;
float voltage_offset = MAGIC_COOL_VOLTAGE_OFFSET;
#if MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_VPP || \
    MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_RMS
float impedance[64] = {0};
#endif

#if MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_FFT || \
    MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_DOT || \
    MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_CAPTURE
float phase[64] = {0};
#endif

#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW || \
    MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH ||\
    MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_ALL
// uint32_t freq_pwr[64] = {0};
uint32_t freq_pwr[25] = {0};
#endif

/*******************************************************************/
/*******************************************************************/
/*******************************************************************/
// 气泵
uint8_t  magic_cool_mode = 0;     // 0:暂停 1:运行阻抗谱 2:不追频运行 3:追频运行 4: 按键调占空比
#if MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_FFT || \
    MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_DOT || \
    MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_CAPTURE
uint8_t  magic_cool_adj_dir = 0;  // 0:频率减小   1:频率增加
float magic_cool_ph_proxth = 0;
#endif
uint32_t magic_cool_freqstart = 22500, magic_cool_freqstop = 29000;
uint32_t magic_cool_runfreq = 25000;
uint32_t magic_cool_target_vol = VOL_TARGET;
#if ENABLE_KEY_VOL_CFG || ENABLE_USART
uint32_t adjust_target_vol = VOL_TARGET;
#endif

uint32_t magic_cool_pwr_max = 0;

uint32_t mode2_tick = 0;
/*******************************************************************/
/*******************************************************************/
/*******************************************************************/
/*******************************************************************/
#if ENABLE_QUERY_CMD
uint32_t get_max_vol(void)
{
    return max_vol_cur_data.vol;
}

uint32_t get_max_cur(void)
{
    return max_vol_cur_data.cur;
}

uint32_t get_current_vol(void)
{
    return current_vol_cur_data.vol;
}

uint32_t get_current_cur(void)
{
    return current_vol_cur_data.cur;
}
#endif

void magic_cool_set_limt(uint32_t freq_min, uint32_t freq_max);
static void dcdc_power_control(uint8_t enable);

// ==================== 架构优化：类型定义和函数声明 ====================
// ADC采样数据结构
typedef struct {
    uint32_t voltage;
    uint32_t current;
} adc_sample_t;

// 功率测量配置
typedef struct {
    uint32_t freq;
    uint32_t target_vol;
    uint32_t vol_err;
    uint8_t sample_count;
    uint16_t delay_before_ms;
    FunctionalState check_overvoltage;
} power_measure_config_t;

// 扫频配置
typedef struct {
    uint32_t start_freq;
    uint32_t stop_freq;
    uint16_t freq_step;
    uint32_t target_vol;
    uint32_t vol_err;
    uint8_t sample_count;
    bool verbose_print;       // true: 详细打印(--scan格式), false: 简单打印
} scan_config_t;

// 扫频结果
typedef struct {
    uint32_t powers[15];
    uint8_t count;
    uint8_t max_index;
    uint32_t max_power;
    bool all_zero;
} scan_result_t;

// 函数声明
static inline adc_sample_t adc_sample_once(void);
static inline uint32_t calculate_power(adc_sample_t sample);
static uint32_t measure_power_with_config(const power_measure_config_t *config);
static uint32_t measure_power_simple(uint32_t freq, uint8_t n);
static uint32_t measure_power_force(uint32_t freq, uint8_t n);
static scan_result_t scan_frequency_range(const scan_config_t *cfg);
static uint32_t get_freq_from_scan_index(uint32_t start_freq, uint16_t step, uint8_t index);

void close_all_output(void)
{
    magic_cool_mode = 0;
    pwm_enable(DISABLE);
    dcdc_power_control(DISABLE);
}

static void is_over_voltage(uint16_t vpp, FunctionalState over_voltage_check_enable)
{
    protocol_fault_t ret = FAULT_NORMAL;

    if( magic_cool_mode == 0 ) {
        return;
    }

    float vol = (float)((vpp+voltage_offset)/voltage_gain);
    // printf("vol: %.2f\r\n", vol);

    // 若追频时，电压超过最大电压，则认为过压停止运行
    if( vol >= VOL_TARGET_MAX ) {
        // 扫频时，过压不关输出
        if( over_voltage_check_enable == ENABLE ) {
            close_all_output();
        }
        // 需要报过压故障
        ret = FAULT_OVER_VOLTAGE;
    }
    // 认为是调档失败
#if ENABLE_KEY_VOL_CFG || ENABLE_USART
    else if( vol >= (adjust_target_vol + 5) )
#else
    else if( vol >= (VOL_TARGET + 5) )
#endif
    {
        ret = FAULT_SETTING_FAILED;
    } else {
        switch( fault_vol_status ) {
            case FAULT_OVER_VOLTAGE:
                if( vol > (VOL_TARGET_MAX - 3) ) {
                    ret = FAULT_OVER_VOLTAGE;
                }
                break;
            case FAULT_SETTING_FAILED:
            #if ENABLE_KEY_VOL_CFG || ENABLE_USART
                if( vol >= (adjust_target_vol + 3) )
            #else
                if( vol >= (VOL_TARGET + 3) )
            #endif
                {
                    ret = FAULT_SETTING_FAILED;
                }
                break;
            default:
                break;
        }
    }

    if( ret != fault_vol_status ) {
        printf("vol status: %d\r\n", ret);
        fault_vol_status = ret;
    }
}

static void is_over_current(uint16_t cur, FunctionalState not_load_check_enable)
{
    protocol_fault_t ret = FAULT_NORMAL;

    if( magic_cool_mode == 0 ) {
        return;
    }

    // 每颗IC的运放偏置电流的AD都不一样，需要重新加回偏置电流
    if( cur > CURRENT_ADC_MAX ) {
        close_all_output();
        ret = FAULT_OVER_CURRENT;
    }
#if CURRENT_ADC_MIN > 0
    else if( cur < CURRENT_ADC_MIN ) {
        if( not_load_check_enable == ENABLE ) {
            // close_all_output();
            ret = FAULT_NOT_LOAD;
        }
    }
#endif
    else {
        if( fault_cur_status == FAULT_NOT_LOAD ) {
            if( cur < CURRENT_ADC_MIN + 20 ) {
                ret = FAULT_NOT_LOAD;
            }
        }
    }

    if( ret != fault_cur_status ) {
        printf("cur status: %d\r\n", ret);
        fault_cur_status = ret;
    }
}

static void check_fault_status(void)
{
    static uint32_t fault_status_tick = 0;
    static protocol_fault_t last_fault_status = FAULT_NORMAL;

    typedef struct {
        protocol_fault_t *status_var;
        protocol_fault_t fault_code;
        uint32_t         delay_ms; // 0表示立即触发，>0表示需要持续的时长
    } FaultPriority;

    const FaultPriority priority_table[] = {
        {&fault_cur_status, FAULT_OVER_CURRENT,   0},      // 过流，立即触发
        {&fault_vol_status, FAULT_OVER_VOLTAGE,   0},      // 过压，立即触发
        {&fault_cur_status, FAULT_NOT_LOAD,       5000},   // 空载，持续5秒触发
        {&fault_vol_status, FAULT_SETTING_FAILED, 2000},   // 调档失败，持续2秒触发
    };

    #define NUM_FAULTS  (sizeof(priority_table) / sizeof(priority_table[0]))
    static uint32_t fault_start_times[NUM_FAULTS] = {0};

    if( reset_time_flag ) {
        memset(fault_start_times, 0, sizeof(fault_start_times));
        reset_time_flag = false;
    }

    protocol_fault_t current_highest_fault = FAULT_NORMAL;

    // 遍历所有可能的故障
    for (uint8_t i = 0; i < NUM_FAULTS; i++) {
        // 检查当前故障的条件是否满足
        if (*(priority_table[i].status_var) == priority_table[i].fault_code) {
            if (priority_table[i].delay_ms == 0) {
                // 延时为0，立即确认故障
                current_highest_fault = priority_table[i].fault_code;
                break; // 已找到最高优先级的故障，跳出循环
            } else {
                // 需要延时确认
                if (fault_start_times[i] == 0) {
                    // 第一次检测到，启动计时器
                    fault_start_times[i] = get_systick();
                } else if (get_systick() - fault_start_times[i] >= priority_table[i].delay_ms) {
                    // 持续时间已满足，确认故障
                    current_highest_fault = priority_table[i].fault_code;
                    break; // 已找到最高优先级的故障，跳出循环
                }
                // 否则，延时未到，继续检查下一个（优先级更低的）故障
            }
        } else {
            // 故障条件不满足，重置对应的计时器
            fault_start_times[i] = 0;
        }
    }

    fault_status = current_highest_fault;

    // 只有在最终确认的故障状态变化时才上报
    if (last_fault_status != fault_status) {
        last_fault_status = fault_status;
        if (fault_status != FAULT_NORMAL) {
            fault_report_active(fault_status);
            if(fault_status == FAULT_OVER_VOLTAGE || \
                fault_status == FAULT_OVER_CURRENT || \
                fault_status == FAULT_NOT_LOAD ) {
                    close_all_output();
            }
            printf("fault_status: %d\r\n", fault_status);
        } else {
            printf("fault_status is normal\r\n");
        }
    }
}

static void dcdc_power_control(uint8_t enable)
{
    if( enable ) {
        GPIO_WriteBit(GPIOB, GPIO_Pin_5 | GPIO_Pin_6, Bit_SET);
    } else {
        GPIO_WriteBit(GPIOB, GPIO_Pin_5 | GPIO_Pin_6, Bit_RESET);
    }
}

#if ENABLE_KEY_VOL_CFG
void magic_cool_led_control(uint32_t tick)
{
    if( magic_cool_mode == 0 ) {
        GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);
        return;
    }

    if( led_always_on ) {
        GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);
        return;
    }

    // 闪烁组：每次闪烁100ms亮+100ms灭；完成后空窗2s再复始
    uint8_t flash_count = 0;

    // 根据电压值确定闪烁次数（40→2，35→3，30→4，25→5，20→6）
    switch( adjust_target_vol ) {
    #if Magic_Cool_Customer == AK_Anker
        case VOL_TARGET: flash_count = 1; break;
        // case 35: flash_count = 2; break;
        case VOL_TARGET_1: flash_count = 2; break;
        case VOL_TARGET_2: flash_count = 3; break;
        // case 20: flash_count = 3; break;
    #else
        case VOL_TARGET: flash_count = 1; break;
        case VOL_TARGET_90P: flash_count = 2; break;
        case VOL_TARGET_80P: flash_count = 3; break;
        case VOL_TARGET_70P: flash_count = 4; break;
        case VOL_TARGET_60P: flash_count = 5; break;
        case VOL_TARGET_50P: flash_count = 6; break;
    #endif
        default: flash_count = 0; break;
    }

    if( flash_count == 0 ) {
        GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);
        return;
    }

    const uint32_t single_blink_ms = 500;   // 100ms亮 + 100ms灭
    const uint32_t gap_ms = 2000;           // 组间隔2s
    const uint32_t group_ms = (uint32_t)flash_count * single_blink_ms;
    const uint32_t period_ms = group_ms + gap_ms;

    uint32_t time_in_period = tick % period_ms;

    if( time_in_period < group_ms ) {
        // 处于闪烁组内
        uint32_t t_in_blink = time_in_period % single_blink_ms;
        if( t_in_blink < (single_blink_ms>>1) ) {
            GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_SET);   // 亮100ms
        } else {
            GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET); // 灭100ms
        }
    } else {
        // 组间隔期（2s全灭）
        GPIO_WriteBit(GPIOB, GPIO_Pin_5, Bit_RESET);
    }
}
#endif

void magic_cool_key_scan(uint8_t key1, uint8_t key2, uint8_t key3)
{
    if (key2 == 0x01) {
        if (magic_cool_mode == 0) {
            magic_cool_mode = 1;
        #if ENABLE_KEY_VOL_CFG
            adjust_target_vol = VOL_TARGET;
        #endif
        } else {
        #if ENABLE_KEY_VOL_CFG
            #if Magic_Cool_Customer == AK_Anker
                if( adjust_target_vol == VOL_TARGET ) {
                    adjust_target_vol = VOL_TARGET_1;
                } else if( adjust_target_vol == VOL_TARGET_1 ) {
                    adjust_target_vol = VOL_TARGET_2;
                } else {
                    adjust_target_vol = VOL_TARGET;
                }
            #else
                // 使用表格驱动法优化电压档位切换
                const uint8_t voltage_levels[] = {
                    VOL_TARGET,
                    VOL_TARGET_90P,
                    VOL_TARGET_80P,
                    VOL_TARGET_70P,
                    VOL_TARGET_60P,
                    VOL_TARGET_50P,
                };
                const uint8_t num_levels = sizeof(voltage_levels) / sizeof(voltage_levels[0]);
                static uint8_t current_level_index = 0;

                // 找到当前电压对应的索引，以防外部直接修改 adjust_target_vol
                for (uint8_t i = 0; i < num_levels; i++) {
                    if (adjust_target_vol == voltage_levels[i]) {
                        current_level_index = i;
                        break;
                    }
                }

                // 切换到下一个档位，并循环
                current_level_index = (current_level_index + 1) % num_levels;
                adjust_target_vol = voltage_levels[current_level_index];
            #endif
        #else
            magic_cool_mode = 0;
        #endif
        }
    } else if (key2 == 0x02) {
        magic_cool_mode = 0;
        close_all_output();
    }
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

#if !MAGIC_COOL_DIFF && MAGIC_COOL_PID
// 电压闭环PWM占空比方式
int magic_cool_voltage_closeloop_duty(uint32_t vol_target, uint32_t vol_err, uint32_t timeout)
{
    int ret = 0, count = 0;
    int i = 0, n = 0;
    int16_t duty = 0;
    int16_t vpp = 0;
    int32_t vpp_sum = 0;
    int16_t vol_targetx = vol_target * voltage_gain - voltage_offset;  // 目标电压对应ADC值
    int16_t vol_errx = vol_err * voltage_gain - voltage_offset;  // 误差范围对应ADC值
    int16_t voltage_err = 0;    // 当前误差

    for (i = 0; i < timeout; i++) {    // 调整次数
        vpp_sum = 0;
        for (n = 0; n < 10; n++) {
//            adc_output_conv(ADC_CH_SIZE);
            adc_voltage_get_vpp(ADC_CH_SIZE);  // 电压闭环，使用简单计算，提高反馈速度
            vpp_sum += adc_vpp;
        }
        vpp = vpp_sum / 10;
        voltage_err = (vol_targetx - vpp);
        // duty = rm_pid_d(voltage_err);
        duty = rm_pid_delta(voltage_err);
        // printf("duty -- err:%d, vpp:%d %.2f, vol_errx:%d, out:%d\r\n", voltage_err, vpp, (float)((vpp+voltage_offset) / voltage_gain), vol_errx, duty);
        if (abs_i(voltage_err) < (vol_errx+5)) {  // 电压小于误差范围认为电压稳定
            count++;
            if (count > 10) {    // 连续获取电压10次都在误差范围就认为电压稳定，退出
                magic_cool_vpp = vpp;
                return 0;
            }
            continue;
        }
        count = 0;

        // if (duty < 0) {
        //     duty = 0;
        // }
        // 增量式PID
        extern uint32_t tim1_duty;
        duty += tim1_duty;
        ret = pwm_duty_set_pid(duty);  // 设置PWM占空比
//        ret = pwm_duty_set_pid_dt(channel, duty);  // 设置PWM死区占空比
        // printf("duty:%d cnt:%d\r\n", pwm_get_duty(), i);

        if (ret == 1) {
//            printf("duty out of range\r\n");
            magic_cool_vpp = vpp;
            return 1;
        }
    }
    return 1;
}
#endif

// 电压闭环DCDC方式
int magic_cool_voltage_closeloop_dcdc(uint32_t vol_target, uint32_t vol_err, uint32_t timeout)
{
    int ret = 0, count = 0;
    int i = 0, n = 0;
    int16_t pid_delta = 0;
    int16_t vpp = 0;
    int32_t vpp_sum = 0;
    int16_t vol_targetx = vol_target * voltage_gain - voltage_offset;  // 目标电压对应ADC值
    int16_t vol_errx = vol_err * voltage_gain - voltage_offset;  // 误差范围对应ADC值
    int16_t voltage_err = 0;    // 当前误差

    for (i = 0; i < timeout; i++) {    // 调整次数
        vpp_sum = 0;
        for (n = 0; n < 10; n++) {
//            adc_output_conv(ADC_CH_SIZE);
            adc_voltage_get_vpp(ADC_CH_SIZE);  // 电压闭环，使用简单计算，提高反馈速度
            vpp_sum += adc_vpp;  // 获取单次计算的峰峰值
        }
        vpp = vpp_sum / 10;
        voltage_err = (vol_targetx - vpp);

        // printf("err:%d, vpp:%d, vol_errx:%d, pwm1_duty_out:%d\r\n", voltage_err, vpp, vol_errx, pwm1_duty_out);
        if (abs_i(voltage_err) < vol_errx) {  // 电压小于误差范围认为电压稳定
            count++;
            if (count > 10) {   // 连续获取电压10次都在误差范围就认为电压稳定，退出
                magic_cool_vpp = vpp;
                return 0;
            }
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

        // if( pid_delta <= 1  ) {
        //     // 防止过度循环，加快系统运算时间
        //     ret = 1;
        // }
        // printf("pwm1_duty_out:%d, pid_delta:%d\r\n", pwm1_duty_out, pid_delta);
        pwm1_set_duty((uint32_t)pwm1_duty_out);
        sys_delayms(50);

        if (ret == 1) {
            // printf("pwm1_duty_out of range\r\n");
            magic_cool_vpp = vpp;
            return 1;
        }
    }

    magic_cool_vpp = vpp;
    return 1;
}

// 电压闭环
// 1. 单端的时候，由于LC谐振，电压在不同频率下波动较大，可以通过调整PWM占空比升降压，也可以通过调整DCDC ref电压升降压
// 2. 差分模式的时候，由于LC谐振，调压只能通过调整DCDC ref电压升降压，PWM占空比固定为50%，不能变。
int magic_cool_voltage_closeloop(uint32_t vol_target, uint32_t vol_err, uint32_t timeout, FunctionalState over_voltage_check_enable)
{
#if !MAGIC_COOL_DIFF && MAGIC_COOL_PID
    int ret = 0;
    int loop = 5;
    while(loop--) {
        ret = magic_cool_voltage_closeloop_duty(vol_target, vol_err, timeout);
        if (ret) {
            magic_cool_voltage_closeloop_dcdc(vol_target, vol_err, timeout);
        } else {
            break;
        }
    }

    // magic_cool_voltage_closeloop_duty(vol_target, vol_err, timeout);
#else
    magic_cool_voltage_closeloop_dcdc(vol_target, vol_err, timeout);
    is_over_voltage(magic_cool_vpp, over_voltage_check_enable);
#if ENABLE_QUERY_CMD
    float vpp = ((magic_cool_vpp+voltage_offset)/voltage_gain) * 1000;
    if( max_vol_cur_data.vol < vpp ) {
        max_vol_cur_data.vol = vpp;
    }
    current_vol_cur_data.vol = vpp;
#endif
#endif
    return 0;
}

void magic_cool_calc_current(FunctionalState not_load_check_enable)
{
    adc_sample_t sample = adc_sample_once();
    is_over_current((uint16_t)sample.current, not_load_check_enable);
#if ENABLE_QUERY_CMD
    float cur = CUR_CAL(sample.current) * 1000; //转化为uA
    if( max_vol_cur_data.cur < cur ) {
        max_vol_cur_data.cur = cur;
    }
    current_vol_cur_data.cur = cur;
#endif
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

int magic_cool_scan_power_profile(uint32_t start_freq, uint32_t stop_freq, uint32_t step_freq)
{
#if PWM_DRIVER_METHOD == PWM_DIFFERENTIAL_DRIVE
    uint32_t target_vpp = VOL_TARGET;
#else
    #if Magic_Cool_Customer == AK_Anker
        uint32_t target_vpp = VOL_TARGET_1;
    #else
        uint32_t target_vpp = VOL_TARGET_70P;
    #endif
#endif

    int ret = 0, index = 0, i;
    int freq = 0;

    printf("magic cool scan power profile\r\n");
    for (freq = start_freq; freq <= stop_freq; ) {
        pwm_set_freq(freq);
        magic_cool_voltage_closeloop(target_vpp, 2, 100, DISABLE);// 电压闭环
        if( fault_vol_status == FAULT_NORMAL ) {
            sys_delayms(20);
            magic_cool_voltage_closeloop(target_vpp, 2, 100, DISABLE);// 电压闭环
            if( fault_vol_status == FAULT_NORMAL ) {
                sys_delayms(200);
            }
        }

        freq_pwr[index] = pwm1_duty_out;

        printf("freq: %d vpp:%.1f dac: %.2f pwm1:%d \r\n", freq, (float)(magic_cool_vpp+voltage_offset)/voltage_gain, (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), freq_pwr[index]);

        freq += step_freq;
        index++;

        if( !magic_cool_mode ) {
            break;
        }
    }
    printf("magic cool scan power profile end\r\n");
    ret = index;
    return ret;
}

int magic_cool_calc_impedance(uint32_t start_freq, uint32_t stop_freq, uint32_t step_freq)
{
#if PWM_DRIVER_METHOD == PWM_DIFFERENTIAL_DRIVE
    uint32_t target_vpp = VOL_TARGET;
#else
    #if Magic_Cool_Customer == AK_Anker
        uint32_t target_vpp = VOL_TARGET_1;
    #else
        uint32_t target_vpp = VOL_TARGET_70P;
    #endif
#endif
    int ret = 0, index = 0, i;
    int flow = 0;
    int freq = 0;
    float phasex = 0;
    float ipp = 0.0, vpp = 0.0;
    float irms = 0.0, vrms = 0.0;
    float zx = 0.0;
    float hvol = 0.0, hcur = 0.0, lcur = 0.0;

    printf("magic_cool_calc_impedance imp\r\n");
    for (freq = start_freq; freq <= stop_freq; ) {
        pwm_set_freq(freq);
        magic_cool_voltage_closeloop(target_vpp, 2, 100, DISABLE);// 电压闭环
        if( fault_vol_status == FAULT_NORMAL ) {
            sys_delayms(20);
            magic_cool_voltage_closeloop(target_vpp, 2, 100, DISABLE);// 电压闭环
            if( fault_vol_status == FAULT_NORMAL ) {
                sys_delayms(200);
            }
        }
#if MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_VPP
        ipp = 0.0; vpp = 0.0;
        for (i = 0; i < 50; i++) {
            sys_delayms(2);
            // ADC采样，同时采样电压电流
            adc_output_conv(ADC_CH_SIZE);
            vpp += adc_vpp;  // 电压峰峰值
            ipp += adc_ipp;  // 电流峰峰值
        }
        zx = vpp / ipp;  // Z 的模
        printf("Vpp: %.5f Ipp: %.5f Z: %.5f ", vpp, ipp, zx);
        impedance[index] = zx;
#elif MAGIC_COOL_IMPEDANCE_DEFAULT == MAGIC_COOL_IMPEDANCE_RMS
        irms = 0.0;
        vrms = 0.0;
        for (i = 0; i < 50; i++) {
            sys_delayms(2);
            // ADC采样，同时采样电压电流
            adc_output_conv(ADC_CH_SIZE);
            vrms += adc_vrms;  // 电压有效值
            irms += adc_irms;  // 电流有效值
//            printf("vrms:%.5f irms:%.5f\r\n", adc_vrms, adc_irms);
        }
        zx = vrms / irms;  // Z 的模
        printf("Vrms: %.5f Irms: %.5f Z: %.5f Dac: %.2f ", vrms, irms, zx, (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)));
        impedance[index] = zx;
#else  // 其他方式，待定

#endif

#if MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_FFT
        // ADC数据处理，FFT
        adc_output_conv(ADC_CH_SIZE);
        phasex = ProcessADCData(adc_voltage_data, adc_current_data);
        printf("fft phase: %.5f ", phasex);
        phase[index] = phasex;
#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_DOT
        // ADC数据处理，点积
        adc_output_conv(ADC_CH_SIZE);
        phasex = get_phase_difference(adc_voltage_data, adc_current_data, 128);
        printf("dot phase: %.5f ", phasex);
        phase[index] = phasex;
#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_CAPTURE
        // PWM 输入捕获
        adc_output_conv(ADC_CH_SIZE);
        set_compare_dcoffset(128);
        sys_delayms(100);
        phasex = get_capture(8);
        printf("cap phase: %.5f ", phasex);
        phase[index] = phasex;
#else  // 其他方式，待定
        phasex = 0;
#endif

#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_ALL
        // 低电流
        adc_hvli_input_conv(ADC_CH_SIZE);
        hvol = adc_dc_hvol_avg;
        lcur = adc_dc_lcur_avg;
//        freq_pwr[index] = hvol * lcur;
        printf("hvol: %.2f lcur: %.2f ", hvol, lcur);

        // 高电流
        adc_hv_input_conv(ADC_CH_SIZE);
        hvol = adc_dc_hvol_avg;
        hcur = adc_dc_hcur_avg;
        freq_pwr[index] = hvol * hcur;
        printf("hvol: %.2f hcur: %.2f ", hvol, hcur);

#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
        // 低电流
        magic_cool_calc_current(DISABLE);
        hvol = adc_dc_hvol_avg;
        lcur = adc_dc_lcur_avg;
        // 电压无故障，对应频率则进行功率比较
        if( fault_vol_status == FAULT_NORMAL ) {
            freq_pwr[index] = hvol * lcur;
        } else {
            freq_pwr[index] = 0;
        }
        printf("freq: %d vpp:%.1f duty:%d hvol: %.2f lcur: %.2f pwr: %d dac: %.2f \r\n", freq, (float)(magic_cool_vpp+voltage_offset)/voltage_gain, pwm_get_duty(), hvol, lcur, freq_pwr[index], (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)));
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
        // 高电流
        adc_hv_input_conv(ADC_CH_SIZE);
        hvol = adc_dc_hvol_avg;
        hcur = adc_dc_hcur_avg;
        freq_pwr[index] = hvol * hcur;
        printf("freq: %d vpp:%.1f duty:%d hvol: %.2f hcur: %.2f pwr: %d dac: %.2f ", freq, (float)vpp, pwm_get_duty(), hvol, hcur, freq_pwr[index], (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)));
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
        freq += step_freq;
        index++;

        if( !magic_cool_mode ) {
            break;
        }
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

    OPA_Enable();
    // 从低频率开始，防止过冲烧坏气泵
    // 恢复默认dac
    pwm1_duty_out = PWM1_MIN_POWER_DUTY;
    pwm1_set_duty(pwm1_duty_out);
    set_power_enable(ENABLE);
    sys_delayms(2);
    dcdc_power_control(ENABLE);
    sys_delayms(10);    //NOTE:增加延时,防止短时间电压过冲
    // 升压稳定后再开H桥PWM
    pwm_set_freq(magic_cool_freqstart);
    pwm_enable(ENABLE);
    // sys_delayms(100);
    // adc_output_conv(ADC_CH_SIZE);

    // 重置错误标志
    fault_status = FAULT_NORMAL;
    fault_vol_status = FAULT_NORMAL;
    fault_cur_status = FAULT_NORMAL;
    reset_time_flag = 1;

    // 重置历史电压/电流最大值
#if ENABLE_QUERY_CMD
    max_vol_cur_data.vol = 0;
    max_vol_cur_data.cur = 0;
#endif

#if ENABLE_KEY_VOL_CFG
    led_always_on = 1;
#endif

    // NOTE: 范围从20K~30KHz扫描，找到接近最优频率的整数倍频率
    // if( magic_cool_freqstart == 20000 && magic_cool_freqstop == FREQ_MAX ) {
    if((FREQ_MAX-FREQ_MIN) >= 3000) {
        magic_cool_freqstart = FREQ_MIN;
        magic_cool_freqstop = FREQ_MAX;
        len = magic_cool_scan_power_profile(magic_cool_freqstart, magic_cool_freqstop, 1000);
        uint32_t val;
        if( find_extremum_minima_i(freq_pwr, len, (uint32_t *)&val, &idx_min) == -1 ) {
            magic_cool_mode = 0;
            printf("fault freq, stop work\r\n");
        }
        if( !magic_cool_mode ) {
            return;
        }
        // 寻找连续的最小值区间
        int idx_start = idx_min;
        int idx_end = idx_min;

        // 向前寻找
        while (idx_start > 0 && freq_pwr[idx_start - 1] == val) {
            idx_start--;
        }
        // 向后寻找
        while (idx_end < (len - 1) && freq_pwr[idx_end + 1] == val) {
            idx_end++;
        }

        uint32_t freq_start_found = magic_cool_freqstart + 1000 * idx_start;
        uint32_t freq_end_found = magic_cool_freqstart + 1000 * idx_end;

        printf("min freq range:%d-%d, min val:%d\r\n", freq_start_found, freq_end_found, (int)val);

        magic_cool_freqstart = freq_start_found - 1000;
        magic_cool_freqstop = freq_end_found + 500;
        printf("magic_cool_freqstart:%d, magic_cool_freqstop:%d\r\n", magic_cool_freqstart, magic_cool_freqstop);
    }

    uint32_t step_freq = 200;
    len = magic_cool_calc_impedance(magic_cool_freqstart, magic_cool_freqstop, step_freq);  // 阻抗/频率谱
    sys_delayms(200);

    /*  找极值方法：
     *  1. 先找极小值，如果有多个极小值，则选择最小的极小值
     *  2. 找极大值，极大值依赖于极小值，找高于极小值频率的极大值的最高值？？？？ 距离极小值最近的极大值？？？？？
     */
    // find_extremum(impedance, len, &val_max, &val_min, &idx_max, &idx_min);  // 找极值
//    find_extremum_minima(impedance, len, &val_min, &idx_min);  // 找极小值
//    find_extremum_maxima(impedance, len, &val_max, &idx_max);  // 找极大值

    // 频率处理
    // freq_min = magic_cool_freqstart + 100 * idx_min;  // 极小值对应频率
    // freq_max = magic_cool_freqstart + 100 * idx_max;  // 极大值对应频率
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
        adc_output_conv(ADC_CH_SIZE);
        // ADC数据处理，FFT求相位差
        phasex = ProcessADCData(adc_voltage_data, adc_current_data);
#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_DOT
        // ADC采样，同时采样电压电流
        adc_output_conv(ADC_CH_SIZE);
        // ADC数据处理，点积求相位差
        phasex = get_phase_difference(adc_voltage_data, adc_current_data, 128);
#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_CAPTURE
        // 过零比较器求相位差
        adc_output_conv(ADC_CH_SIZE);
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

    magic_cool_runfreq = gold_freq; // 初始频率为相位最大值对应的频率
    printf("freq:%d, phase proxth:%f\r\n", magic_cool_runfreq, magic_cool_ph_proxth);

#elif MAGIC_COOL_TRACK_DEFAULT == MAGIC_COOL_TRACK_CURRENT
    if( !magic_cool_mode ) {
        return;
    }
    find_maxima_i(freq_pwr, len, &magic_cool_pwr_max, &pwr_max_idx);  // 找最大值
#if 1
    #define SCAN_FREQ_STEP   50  //20 // 50
    #define SCAN_FREQ_RANGE  150  //60 // 100

#if PWM_DRIVER_METHOD == PWM_DIFFERENTIAL_DRIVE
    uint32_t target_vpp = VOL_TARGET;
#else
    #if Magic_Cool_Customer == AK_Anker
        uint32_t target_vpp = VOL_TARGET_1;
    #else
        uint32_t target_vpp = VOL_TARGET_70P;
    #endif
#endif

    freq_min = magic_cool_freqstart + step_freq * pwr_max_idx - SCAN_FREQ_RANGE;
    freq_max = magic_cool_freqstart + step_freq * pwr_max_idx + SCAN_FREQ_RANGE;

    // 使用扫频抽象层 - DRY原则
    scan_config_t impedance_scan_cfg = {
        .start_freq = freq_min,
        .stop_freq = freq_max,
        .freq_step = SCAN_FREQ_STEP,
        .target_vol = target_vpp,
        .vol_err = 2,
        .sample_count = 10,
        .verbose_print = false  // 阻抗扫描使用简单格式：freq pwr
    };

    scan_result_t impedance_result = scan_frequency_range(&impedance_scan_cfg);

    if (!magic_cool_mode) {
        return;
    }

    // 复制结果到freq_pwr数组（保持兼容性）
    for (i = 0; i < impedance_result.count && i < (sizeof(freq_pwr) / sizeof(freq_pwr[0])); i++) {
        freq_pwr[i] = impedance_result.powers[i];
    }

    pwr_max_idx = impedance_result.max_index;
    gold_freq = get_freq_from_scan_index(freq_min, SCAN_FREQ_STEP, pwr_max_idx);
#endif

    magic_cool_pwr_max = freq_pwr[pwr_max_idx];

    magic_cool_runfreq = gold_freq; // 初始频率为相位最大值对应的频率
    printf("freq:%d, pwr max :%d\r\n", magic_cool_runfreq, magic_cool_pwr_max);

#else  // 其他方式, 待定
    gold_freq = FREQ_MIN;
    printf("min freq:%d, max freq:%d, gold freq:%d\r\n", freq_min, freq_max, gold_freq);
#endif
    // 设置输出频率
    pwm_set_freq(magic_cool_runfreq); // 阻抗谱计算最优频率
    magic_cool_voltage_closeloop(target_vpp, 2, 100, ENABLE);// 电压闭环
    sys_delayms(200);
#if ENABLE_KEY_VOL_CFG
    led_always_on = 0;
#endif

    scan_freq_enable = false;
    first_scan_freq = true;

    reset_pwr_proxth_flag = true;
#if ENABLE_WATER_INTRUSION
    reset_water_intrusion_flag = true;
#endif
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
        adc_output_conv(ADC_CH_SIZE);
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
                adc_output_conv(ADC_CH_SIZE);
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
                adc_output_conv(ADC_CH_SIZE);
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
        adc_output_conv(ADC_CH_SIZE);
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
                adc_output_conv(ADC_CH_SIZE);
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
                adc_output_conv(ADC_CH_SIZE);
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
                adc_output_conv(ADC_CH_SIZE);
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

    adc_output_conv(ADC_CH_SIZE);
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
            adc_output_conv(ADC_CH_SIZE);
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
            adc_output_conv(ADC_CH_SIZE);
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
        adc_output_conv(ADC_CH_SIZE);
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
        adc_output_conv(ADC_CH_SIZE);  // 需要处理好电压电流的中心对称
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
        adc_output_conv(ADC_CH_SIZE);
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
        adc_output_conv(ADC_CH_SIZE);  // 需要处理好电压电流的中心对称
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
        adc_output_conv(ADC_CH_SIZE);
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
        adc_output_conv(ADC_CH_SIZE);  // 需要处理好电压电流的中心对称
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
        adc_hvli_input_conv(ADC_CH_SIZE);
        dc_vol = adc_dc_hvol_avg;
        dc_cur = adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH  // 直流高压输入电压,高端电流
        adc_hv_input_conv(ADC_CH_SIZE);
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
            adc_hvli_input_conv(ADC_CH_SIZE);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH  // 直流高压输入电压,高端电流
            adc_hv_input_conv(ADC_CH_SIZE);
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
            adc_hvli_input_conv(ADC_CH_SIZE);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH  // 直流高压输入电压,高端电流
            adc_hv_input_conv(ADC_CH_SIZE);
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
            adc_hvli_input_conv(ADC_CH_SIZE);
            dc_vol = adc_dc_hvol_avg;
            dc_cur = adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH  // 直流高压输入电压,高端电流
            adc_hv_input_conv(ADC_CH_SIZE);
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
// ==================== 底层ADC采样抽象层 ====================
// 策略模式：ADC采样接口（实现）
static inline adc_sample_t adc_sample_once(void)
{
    adc_sample_t sample = {0};
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
    adc_hvli_input_conv(ADC_CH_SIZE);
    sample.voltage = adc_dc_hvol_avg;
    sample.current = adc_dc_lcur_avg;
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
    adc_hv_input_conv(ADC_CH_SIZE);
    sample.voltage = adc_dc_hvol_avg;
    sample.current = adc_dc_hcur_avg;
#endif
    return sample;
}

// 功率计算：单一职责（实现）
static inline uint32_t calculate_power(adc_sample_t sample)
{
    return sample.voltage * sample.current;
}

// ==================== 功率测量抽象层 ====================
// 功率测量核心函数：依赖倒置原则（依赖配置抽象）（实现）
static uint32_t measure_power_with_config(const power_measure_config_t *config)
{
    uint32_t power_sum = 0;

    // 设置频率和电压闭环
    pwm_set_freq(config->freq);
    magic_cool_voltage_closeloop(config->target_vol, config->vol_err, 100, config->check_overvoltage);

    // 可选延时
    if (config->delay_before_ms > 0) {
        sys_delayms(config->delay_before_ms);
    }

    // 采样并累加功率
    for (uint8_t i = 0; i < config->sample_count; i++) {
        sys_delayms(10);
        adc_sample_t sample = adc_sample_once();
        power_sum += calculate_power(sample);
    }

    return power_sum / config->sample_count;
}

// 便捷函数：接口隔离原则（实现）
static uint32_t measure_power_simple(uint32_t freq, uint8_t n)
{
    power_measure_config_t config = {
        .freq = freq,
        .target_vol = magic_cool_target_vol,
        .vol_err = 1,
        .sample_count = n,
        .delay_before_ms = 200,
        .check_overvoltage = ENABLE
    };

    // 先执行电压闭环，然后检查故障状态
    pwm_set_freq(freq);
    magic_cool_voltage_closeloop(config.target_vol, config.vol_err, 100, config.check_overvoltage);

    if (fault_vol_status == FAULT_NORMAL) {
        sys_delayms(200);
        return measure_power_with_config(&config);
    }
    return 0;
}

// 强制测量功率（不检查故障状态）- 用于特殊场景如 all_zero
static uint32_t measure_power_force(uint32_t freq, uint8_t n)
{
    power_measure_config_t config = {
        .freq = freq,
        .target_vol = magic_cool_target_vol,
        .vol_err = 1,
        .sample_count = n,
        .delay_before_ms = 200,
        .check_overvoltage = DISABLE  // 强制测量时不检查过压
    };

    // 直接测量，无视故障状态
    return measure_power_with_config(&config);
}

uint32_t get_current_pwr(int freq, uint8_t n)
{
    return measure_power_simple(freq, n);
}

// ==================== 扫频抽象层 ====================
// 扫频核心函数：单一职责 + DRY原则（实现）
static scan_result_t scan_frequency_range(const scan_config_t *cfg)
{
    scan_result_t result = {0};
    result.all_zero = true;
    uint32_t dc_vol, dc_cur, pwrx;

    for (uint32_t freq = cfg->start_freq; freq <= cfg->stop_freq; freq += cfg->freq_step) {
        // 设置频率和电压闭环
        pwm_set_freq(freq);
        magic_cool_voltage_closeloop(cfg->target_vol, cfg->vol_err, 100, DISABLE);

        // 根据故障状态决定是否测量功率
        if (fault_vol_status == FAULT_NORMAL) {
            sys_delayms(200);
            pwrx = 0;

            // 采样并累加功率
            for (uint8_t i = 0; i < cfg->sample_count; i++) {
                sys_delayms(10);
                adc_sample_t sample = adc_sample_once();
                pwrx += calculate_power(sample);
            }

            pwrx = pwrx / cfg->sample_count;
            result.all_zero = false;
        } else {
            pwrx = 0;
        }

        // 存储功率并更新最大值
        if (result.count < (sizeof(result.powers) / sizeof(result.powers[0]))) {
            result.powers[result.count] = pwrx;

            if (pwrx > result.max_power) {
                result.max_power = pwrx;
                result.max_index = result.count;
            }

            result.count++;
        }

        // 根据配置选择打印格式
        if (cfg->verbose_print) {
            // 详细格式（用于追频）
            printf("--scan freq:%d vpp:%.2f pwr:%d\r\n", freq,
                   (float)((magic_cool_vpp+voltage_offset)/voltage_gain), pwrx);
        } else {
            // 简单格式（用于阻抗扫描）
            printf("%d %d\r\n", freq, pwrx);
        }

        // 检查是否需要退出
        if (!magic_cool_mode) {
            break;
        }
    }

    return result;
}

// 计算扫描结果的频率（实现）
static uint32_t get_freq_from_scan_index(uint32_t start_freq, uint16_t step, uint8_t index)
{
    return start_freq + index * step;
}

// ==================== 频率追踪逻辑层 ====================

#if ENABLE_PER
    #define PWR_PROXTH      6
    #define PWR_PROXTH_MIN  4  //NOTE: 改百分比阈值后,需要重新测试高温环境是否正常 -> 测试OK
    #define PWR_PROXTH_MAX  15
#else
    #define PWR_PROXTH      ((int)POWER_PROXTH(30))
    #define PWR_PROXTH_MIN  ((int)POWER_PROXTH(15))
    #define PWR_PROXTH_MAX  ((int)POWER_PROXTH(60))
#endif

#define FREQ_NORMAL_STEP   40
#define FREQ_NORMAL_RANGE  160
#define FREQ_HIGH_TEMP_STEP   50
#define FREQ_HIGH_TEMP_RANGE  250

// 追踪状态上下文
typedef struct {
    uint16_t freq_step;
    uint16_t freq_range;
    int pwr_proxth;
    uint8_t pwr_proxth_reset_cnt;

    // Stuck detection
    uint8_t pwr0_stuck_cnt;
    uint8_t pwr2_stuck_cnt;
    uint8_t pwr0_max_cnt;
    uint8_t pwr2_max_cnt;

    // Timer state
    int last_freq;
    uint32_t countdown_xmin;
    uint32_t long_time_same_freq_tick;
    uint32_t long_time_same_freq_interval;
    bool long_time_same_freq_scan_pending;
} TrackingState;

static TrackingState track_state = {
    .freq_step = FREQ_NORMAL_STEP,
    .freq_range = FREQ_NORMAL_RANGE,
    .pwr_proxth = PWR_PROXTH,
    .long_time_same_freq_interval = (10*60*1000)
};

static void track_reset_state(void)
{
    track_state.freq_step = FREQ_NORMAL_STEP;
    track_state.freq_range = FREQ_NORMAL_RANGE;
    track_state.pwr_proxth = PWR_PROXTH;
    feedback_tick = 20000;
    track_state.pwr_proxth_reset_cnt = 0;

    track_state.countdown_xmin = get_systick() + (3*60*1000); // 3分钟
    track_state.long_time_same_freq_tick = get_systick();
    track_state.long_time_same_freq_interval = (10*60*1000);
    track_state.long_time_same_freq_scan_pending = false;
}

static void track_check_timers(void)
{
    // 1. 3分钟倒计时检查
    if( (get_systick() >= track_state.countdown_xmin) && (track_state.countdown_xmin != 0) ) {
        track_state.countdown_xmin = 0;
        scan_freq_enable = true;
        track_state.long_time_same_freq_tick = get_systick();
        printf("scan enable:%d. countdown 3min\r\n", __LINE__);
    }

    // 2. 长时间同频检查
    if( track_state.last_freq == pwm_get_freq() ) {
        if( track_state.long_time_same_freq_scan_pending ) {
            if( track_state.long_time_same_freq_interval < (30*60*1000) ) {
                track_state.long_time_same_freq_interval += (10*60*1000);
                if( track_state.long_time_same_freq_interval > (30*60*1000) ) {
                    track_state.long_time_same_freq_interval = (30*60*1000);
                }
            }
            track_state.long_time_same_freq_scan_pending = false;
        }

        uint32_t elapsed = get_systick() - track_state.long_time_same_freq_tick;
        if( elapsed >= track_state.long_time_same_freq_interval ) {
            track_state.long_time_same_freq_tick = get_systick();
            scan_freq_enable = true;
            track_state.long_time_same_freq_scan_pending = true;
            printf("scan enable:%d. long time same freq\r\n", __LINE__);
        } else {
            printf("same freq time:%d interval:%d \r\n", ((get_systick() - track_state.long_time_same_freq_tick)/1000), (track_state.long_time_same_freq_interval/1000));
        }
    } else {
        track_state.last_freq = pwm_get_freq();
        track_state.long_time_same_freq_tick = get_systick();
        track_state.long_time_same_freq_interval = (10*60*1000);
        track_state.long_time_same_freq_scan_pending = false;
    }
}

static void track_perform_scan(uint8_t n)
{
    #if ENABLE_KEY_VOL_CFG
        led_always_on = 1;
    #endif

    int freq1 = pwm_get_freq();
    scan_config_t scan_cfg = {
        .start_freq = freq1 - track_state.freq_range,
        .stop_freq = freq1 + track_state.freq_range,
        .freq_step = track_state.freq_step,
        .target_vol = magic_cool_target_vol,
        .vol_err = 1,
        .sample_count = n,
        .verbose_print = true
    };

    scan_result_t result = scan_frequency_range(&scan_cfg);

    if (!magic_cool_mode) return;
#if ENABLE_KEY_VOL_CFG || ENABLE_USART
    if( adjust_target_vol != magic_cool_target_vol ) return;
#endif
    if (result.all_zero) {
        printf("--all_pwr_zero, test freq_start and freq_stop only\r\n");
        uint32_t pwr_start = measure_power_force(scan_cfg.start_freq, n);
        uint32_t pwr_stop = measure_power_force(scan_cfg.stop_freq, n);

        uint32_t best_freq = (pwr_start > pwr_stop) ? scan_cfg.start_freq : scan_cfg.stop_freq;
        pwm_set_freq(best_freq);
        magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, DISABLE);
    } else {
        uint32_t best_freq = get_freq_from_scan_index(scan_cfg.start_freq, track_state.freq_step, result.max_index);
        magic_cool_pwr_max = result.max_power;

        pwm_set_freq(best_freq);
        magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, DISABLE);

        printf("--scan max freq:%d pwr:%d vpp:%.2f dac:%.2f\r\n", best_freq, magic_cool_pwr_max,
            (float)((magic_cool_vpp+voltage_offset)/voltage_gain),
            (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)));

        // 边界检查：如果在边界，需要重新扫频（通过外部循环控制）
        if (best_freq == scan_cfg.start_freq ||
            best_freq == (scan_cfg.start_freq + track_state.freq_step) ||
            best_freq == (scan_cfg.stop_freq - track_state.freq_step) ||
            best_freq == scan_cfg.stop_freq) {
            printf("scan enable:%d. freq at boundary\r\n", __LINE__);
            scan_freq_enable = true; // 触发下一次循环继续扫
            return;
        }
    }

    #if ENABLE_KEY_VOL_CFG
        led_always_on = 0;
    #endif
    scan_freq_enable = false;
    #if ENABLE_WATER_INTRUSION
        reset_water_intrusion_flag = true;
    #endif
}

static bool track_check_power_stability(uint8_t n)
{
    // 测量当前功率
    magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, DISABLE);

    uint32_t pwrx = 0;
    for (int i = 0; i < n; i++) {
        sys_delayms(10);
        adc_sample_t sample = adc_sample_once();
        pwrx += calculate_power(sample);
    }
    pwrx /= n;
    printf("freq: %d pwr:%d\r\n", pwm_get_freq(), pwrx);

    uint32_t diff = abs_i(magic_cool_pwr_max - pwrx);

    #if ENABLE_PER
        uint8_t percent = (uint8_t)(diff*100/magic_cool_pwr_max);
        printf("absx: %d percent: %d\r\n", diff, percent);
        bool is_stable = (percent < track_state.pwr_proxth);
    #else
        printf("absx: %d diff:%.2f\r\n", diff, (float)(diff*(CURRENT_COEFFICIENT * VOL_H_COEFFICIENT)));
        bool is_stable = (diff < track_state.pwr_proxth);
    #endif

    if (is_stable) {
        if( feedback_tick == 5000 ) {
            if( ++track_state.pwr_proxth_reset_cnt >= 10 ) {
                track_state.freq_step = FREQ_NORMAL_STEP;
                track_state.freq_range = FREQ_NORMAL_RANGE;
                track_state.pwr_proxth = PWR_PROXTH;
                feedback_tick = 20000;
                track_state.pwr_proxth_reset_cnt = 0;
                printf("reset pwr proxth. feedback_tick:%d\r\n", feedback_tick);
            }
        }
        return true; // 功率正常（误差在阈值内），无需调整
    } else {
        track_state.pwr_proxth_reset_cnt = 0;
        #if ENABLE_PER
            bool huge_stability = (percent >= PWR_PROXTH_MAX);
        #else
            bool huge_stability = (diff >= PWR_PROXTH_MAX);
        #endif

        if (huge_stability) {
            scan_freq_enable = true;
            track_state.freq_step = FREQ_HIGH_TEMP_STEP;
            track_state.freq_range = FREQ_HIGH_TEMP_RANGE;
            track_state.pwr_proxth = PWR_PROXTH_MIN;
            feedback_tick = 5000;
            printf("scan enable:%d. feedback_tick:%d\r\n", __LINE__, feedback_tick);
            return true; // 触发扫频，跳过P&O
        }
    }
    return false; // 功率下降但未到极大值，进入P&O
}

static void track_perturb_observe(uint8_t n)
{
    track_state.pwr0_stuck_cnt = 0;
    track_state.pwr2_stuck_cnt = 0;
    track_state.pwr0_max_cnt = 0;
    track_state.pwr2_max_cnt = 0;

    for (int i = 0; i < 4; i++) { // loop_time = 4
        int freq1 = pwm_get_freq();
        int freq2 = freq1 + 20;
        int freq0 = freq1 - 20;

        uint32_t pwr1 = measure_power_simple(freq1, n);
        printf("freq1: %d pwr1:%d vpp:%.2f dac:%.2f duty:%d\r\n", freq1, pwr1, (float)((magic_cool_vpp+voltage_offset)/voltage_gain), (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), pwm_get_duty());

        uint32_t pwr0 = measure_power_simple(freq0, n);
        printf("freq0: %d pwr0:%d vpp:%.2f dac:%.2f duty:%d\r\n", freq0, pwr0, (float)((magic_cool_vpp+voltage_offset)/voltage_gain), (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), pwm_get_duty());

        uint32_t pwr2 = measure_power_simple(freq2, n);
        printf("freq2: %d pwr2:%d vpp:%.2f dac:%.2f duty:%d\r\n", freq2, pwr2, (float)((magic_cool_vpp+voltage_offset)/voltage_gain), (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), pwm_get_duty());

        if (!magic_cool_mode) return;

        if (pwr1 == 0 && pwr0 == 0 && pwr2 == 0) {
            scan_freq_enable = true;
            printf("scan enable:%d. all pwr is 0\r\n", __LINE__);
            return;
        }

        #if ENABLE_PER
            int tmp = (magic_cool_pwr_max * track_state.pwr_proxth / 100 ) >> 1;
        #else
            int tmp = track_state.pwr_proxth >> 1;
        #endif

        if ((pwr0 > pwr1 + tmp) && (pwr0 > pwr2 + tmp)) {
            if( track_state.pwr0_max_cnt == 0 ) {
                track_state.pwr0_max_cnt++;
                track_state.pwr2_max_cnt = 0;
                pwm_set_freq(freq1);
            } else {
                track_state.pwr0_max_cnt = 0;
                pwm_set_freq(freq0);
                printf("set freq0:%d \r\n", freq0);
                magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, ENABLE);
                magic_cool_pwr_max = pwr0;
            }
        } else if ((pwr1 > pwr0 + tmp) && (pwr1 > pwr2 + tmp)) {
            pwm_set_freq(freq1);
            printf("set freq1:%d \r\n", freq1);
            magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, ENABLE);
            if( pwr1 > magic_cool_pwr_max ) magic_cool_pwr_max = pwr1;
            break;
        } else if ((pwr2 > pwr0 + tmp) && (pwr2 > pwr1 + tmp)) {
            if( track_state.pwr2_max_cnt == 0 ) {
                track_state.pwr2_max_cnt++;
                track_state.pwr0_max_cnt = 0;
                pwm_set_freq(freq1);
            } else {
                track_state.pwr2_max_cnt = 0;
                pwm_set_freq(freq2);
                printf("set freq2:%d \r\n", freq2);
                magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, ENABLE);
                magic_cool_pwr_max = pwr2;
            }
        } else {
            pwm_set_freq(freq1);
            track_state.pwr0_max_cnt = 0;
            track_state.pwr2_max_cnt = 0;

            if( feedback_tick == 5000 ) {
                #if ENABLE_PER
                    int tmp_stuck = (magic_cool_pwr_max * track_state.pwr_proxth / 100 ) >> 2;
                #else
                    int tmp_stuck = track_state.pwr_proxth >> 2;
                #endif

                if( (pwr0 > pwr1 + tmp_stuck && pwr0 > pwr2 + tmp_stuck) ) {
                    track_state.pwr2_stuck_cnt = 0;
                    track_state.pwr0_stuck_cnt++;
                } else if ( (pwr2 > pwr1 + tmp_stuck && pwr2 > pwr0 + tmp_stuck) ) {
                    track_state.pwr0_stuck_cnt = 0;
                    track_state.pwr2_stuck_cnt++;
                } else {
                    track_state.pwr0_stuck_cnt = 0;
                    track_state.pwr2_stuck_cnt = 0;
                }

                if (track_state.pwr0_stuck_cnt >= 3 || track_state.pwr2_stuck_cnt >= 3) {
                    track_state.pwr0_stuck_cnt = 0;
                    track_state.pwr2_stuck_cnt = 0;
                    scan_freq_enable = true;
                    printf("scan enable:%d. high temp stuck\r\n", __LINE__);
                    break;
                }
            }
        }
    }
}

// 动态更新最大值方式
void magic_cool_freq_track_current(void)
{
    uint8_t n = 5;

    if( reset_pwr_proxth_flag ) {
        reset_pwr_proxth_flag = false;
        track_reset_state();
    } else {
        track_check_timers();
    }

    // 循环直到不需要扫频
    while (scan_freq_enable) {
        track_perform_scan(n);
        if (!magic_cool_mode) return;
    }

    // 检查功率稳定性
    bool skip_po = track_check_power_stability(n);
    if (skip_po) return;

    // 追频
    track_perturb_observe(n);
}
#endif

#else  // 其他方式, 待定

#endif


void magic_cool_freq_track(void)
{
    static int tick_ph = 50, tick_vol = 1500, tick_imp = 50;
#if ENABLE_KEY_VOL_CFG || ENABLE_USART
    if( adjust_target_vol != magic_cool_target_vol ) {
        // if( adjust_target_vol > magic_cool_target_vol )
        { // 电压从低->高才进行小范围扫频 --- X
          // 经测试，目标电压变化后必须扫频找到合适频率才能守住电压，否则电压会飘高
            if( adjust_target_vol != VOL_TARGET || first_scan_freq == false ) {
                scan_freq_enable = true;
            }

            first_scan_freq = false;
        }
        magic_cool_target_vol = adjust_target_vol;
        magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 50, ENABLE);// 电压闭环
        sys_delayms(10);
    } else {
        first_scan_freq = false;
    }
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
    if (get_systick() >= tick_cur || scan_freq_enable) {
        magic_cool_freq_track_current();
        tick_cur = get_systick() + feedback_tick;
        // scan_freq_enable = false;
    }
#else  // 其他方式, 待定

#endif

    if (get_systick() >= tick_vol) {
        magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, ENABLE);// 电压闭环
        tick_vol = get_systick() + 500;
    }
}

void magic_cool_set_target_vol(uint32_t vol)
{
    magic_cool_target_vol = vol;
}

// 串口发送指令调整流量 -> 调整电压
void magic_cool_set_target_vol_by_flow(uint8_t flow_level)
{
    uint32_t target_vol = 0;
#if ENABLE_USART && Magic_Cool_Customer != AK_Anker
    switch (flow_level) {
        case FLOW_LEVEL_90_PERCENT: target_vol = VOL_TARGET_90P; break;
        case FLOW_LEVEL_80_PERCENT: target_vol = VOL_TARGET_80P; break;
        case FLOW_LEVEL_70_PERCENT: target_vol = VOL_TARGET_70P; break;
        case FLOW_LEVEL_60_PERCENT: target_vol = VOL_TARGET_60P; break;
        case FLOW_LEVEL_50_PERCENT: target_vol = VOL_TARGET_50P; break;
        default: target_vol = VOL_TARGET; break;
    }

    if( target_vol != adjust_target_vol ) {
        adjust_target_vol = target_vol;
    }

    if( !magic_cool_mode ) {
        magic_cool_mode = 1;
    }
#endif
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

#if ENABLE_WATER_INTRUSION
void magic_cool_check_water_intrusion(uint32_t power)
{
    #define FILTER_WINDOW           10
    #define POWER_CHANGE_THRESHOLD  25   // 突变阈值 25%
    #define INTRUSION_DETECT_COUNT  5    // 连续检测次数

    static uint32_t power_filter[FILTER_WINDOW] = {0};
    static uint8_t  filter_index = 0;
    static uint8_t  filter_count = 0;
    static uint8_t  water_intrusion_cnt = 0;
    static uint32_t last_avg_power = 0; // 这是一个"干净"的参考值

    if(reset_water_intrusion_flag) {
        reset_water_intrusion_flag = false;
        water_intrusion_cnt = 0;
        last_avg_power = power;
        filter_index = 0;
        filter_count = 0;
        // 初始化滤波器，避免刚启动时均值为0导致的误判
        for(uint8_t k = 0; k < FILTER_WINDOW; k++) power_filter[k] = power;
        filter_count = FILTER_WINDOW;
        return;
    }

    if(!magic_cool_mode) {
        return;
    }

    bool is_abnormal = false;
    uint32_t power_diff = 0;
    uint8_t diff_percent = 0;

    if(last_avg_power > 0) {
        power_diff = abs_i((int) (power - last_avg_power));

        // 计算动态阈值：百分比阈值 + 绝对底噪保护
        // 如果 last_avg_power 很小，只用百分比会误判，必须保证 diff 超过一个最小物理量
        uint32_t dynamic_threshold = (last_avg_power * POWER_CHANGE_THRESHOLD) / 100;

        // 底噪保护，防止误判，必须超过5000（防止高温误判进水）
        if(power_diff > dynamic_threshold && power_diff > 5000) {
            is_abnormal = true;
            diff_percent = (uint8_t) (power_diff * 100 / last_avg_power);
        }
    }

    if(is_abnormal) {
    // 功率突变 (疑似进水)
        water_intrusion_cnt++;

        printf("[WARN] Power Surge! Cur:%d Ref:%d Diff:%d (%d%%) Cnt:%d\r\n",
            power, last_avg_power, power_diff, diff_percent, water_intrusion_cnt);

        tick_cur = get_systick() + feedback_tick;

        if(water_intrusion_cnt >= INTRUSION_DETECT_COUNT) {
            printf("[ALARM] Water intrusion CONFIRMED! Shutting down magic cool.\r\n");
            magic_cool_mode = 0;
        }

        // 关键点：检测到异常时，直接返回，绝对不要更新滤波器！
        // 这样 last_avg_power 保持在"进水前"的正常水平，
        // 下一次进来的 power 依然是异常值，依然会触发阈值，从而实现连续计数。
        return;
    }
    else {
     // [正常分支]：无突变 (或者是缓慢的热漂移)
     // 即使有轻微波动，只要没超过阈值，我们都认为是合法的，允许更新基准值
        if(water_intrusion_cnt > 0) {
            printf("[INFO] Power recovered or stable. Reset cnt.\r\n");
            water_intrusion_cnt = 0;
        }
    }

    // --- 4. 更新滤波器 (仅在数据正常时执行) ---
    // 这种滑动平均能自动适应"热漂移"，因为热漂移是缓慢变化的，
    // 每次变化都在 25% 以内，基准值会跟着慢慢变。
    power_filter[filter_index] = power;
    filter_index++;
    if(filter_index >= FILTER_WINDOW) {
        filter_index = 0;
    }

    if(filter_count < FILTER_WINDOW) {
        filter_count++;
    }

    // 计算新的移动平均值作为下一次的基准
    uint32_t sum = 0;
    for(uint8_t i = 0; i < filter_count; i++) {
        sum += power_filter[i];
    }
    last_avg_power = sum / filter_count;
}
#endif

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
#if !ENABLE_WATER_INTRUSION
    mode2_tick = get_systick() + 2000;
#else
    mode2_tick = get_systick() + 100;
#endif
    // ADC采样，同时采样电压电流
    adc_output_conv(ADC_CH_SIZE);
    vpp = adc_vpp;
    ipp = adc_ipp;
    vpp = (vpp + voltage_offset) / voltage_gain;  // 转成真实电压

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
        adc_sample_t sample = adc_sample_once();
        hvol = sample.voltage;
        lcur = sample.current;

        // 保持过流检测逻辑
        is_over_current((uint16_t)lcur, ENABLE);

        // 系数计算: hvol和lcur为adc值，将该值转换为电压电流后简化计算就能得到一个系数
        // power = POWER_CAL(hvol, lcur);
    #if ENABLE_WATER_INTRUSION
        static uint8_t tick_2s;
        // if( ++tick_2s >= 5*4 )
        {
            tick_2s = 0;
            printf("freq:%d, vpp:%0.2f, duty:%ld, hvol: %.2f lcur: %.2f power:%.2f flow:%d dac:%.2f pwr:%d\r\n", freq, vpp, pwm_get_duty(), hvol, lcur, POWER_CAL(hvol, lcur), flow, (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), (int)(hvol*lcur));
        }

        magic_cool_check_water_intrusion((uint32_t)(hvol*lcur));
    #else
        printf("freq:%d, vpp:%0.2f, duty:%ld, hvol: %.2f lcur: %.2f power:%.2f flow:%d dac:%.2f pwr:%d\r\n", freq, vpp, pwm_get_duty(), hvol, lcur, POWER_CAL(hvol, lcur), flow, (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), (int)(hvol*lcur));
    #endif

#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
        // 高电流
        adc_sample_t sample = adc_sample_once();
        hvol = sample.voltage;
        hcur = sample.current;

        power = hvol * hcur * 8.843037 / 1E4; // 0.0008843037
        printf("freq:%d, vpp:%0.2f, ipp:%.2f, imp: %.3f phase: %.3f hvol: %.2f hcur: %.2f power:%.2f flow:%d\r\n", freq, vpp, ipp, imp, phase, hvol, hcur, power, flow);
#endif
}

void maigc_cool_test_vpp(void)
{
    static uint16_t cnt;
    if( cnt == 0 ) {
        pwm_enable(ENABLE);
        pwm_set_config(24700, 30);
        set_power_enable(ENABLE);
        // cnt = 1000;
    }
    sys_delayms(1000);
    // for(; cnt<1000; cnt++) {
    //     adc_output_conv(ADC_CH_SIZE);
    //     printf("%.2f,", find_peak_to_peak(adc_voltage_data, 128));
    // }
    if( cnt == 1000 ) {
        // printf("\r\n");
        // adc_hvli_input_conv(ADC_CH_SIZE);
        // for(int i = 0; i < 128; i++) {
        //     printf("%.2f \r\n", adc_voltage_data[i]);
        // }
        // printf("---------\r\n");
        // for(int i = 0; i < 128; i++) {
        //     printf("%.2f \r\n", adc_current_data[i]);
        // }
        // for(int i = 0; i < 128; i++) {
        //     printf("%d, %d\r\n", adc_voltage_data[i], adc_current_data[i]);
        // }
        sys_delayms(100);
        cnt = 1001;
        // pwm_enable(DISABLE);
    }
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

// 零点校准
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
    adc_hvli_input_conv(ADC_CH_SIZE);
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
    adc_hv_input_conv(ADC_CH_SIZE);
#endif

#if MAGIC_COOL_PID
    pid_init();
#endif
    magic_cool_set_adcfreq();
    magic_cool_set_limt(FREQ_MIN, FREQ_MAX);
#if !MAGIC_COOL_DIFF && MAGIC_COOL_PID
    pwm_set_config(FREQ_MIN, 10);  // KHz  10%占空比
#else
    pwm_set_config(FREQ_MIN, 50);  // KHz  50%占空比
#endif
    pwm_enable(DISABLE);
    pwm1_duty_out = PWM1_MIN_POWER_DUTY;
    pwm1_set_duty(pwm1_duty_out);  // 设置DAC输出DCDC
    set_power_enable(ENABLE);
    magic_cool_mode = 0;

    printf("freq_start:%d, freq_stop:%d, vol_target:%d customer:%d\r\n", FREQ_MIN, FREQ_MAX, VOL_TARGET, Magic_Cool_Customer);
}

void magic_cool_run(void)
{
    static uint32_t next_calibration_tick = 0;

    if (magic_cool_mode == 0) {
        // maigc_cool_test_vpp();
        #if 1
            pwm_enable(DISABLE);
            pwm1_duty_out = PWM1_MIN_POWER_DUTY;
            pwm1_set_duty(pwm1_duty_out);  // 设置DAC输出DCDC
            set_power_enable(ENABLE);
            OPA_Disable();
            dcdc_power_control(DISABLE);
        #endif

        // 每3秒执行一次零点校准
        if (get_systick() >= next_calibration_tick) {
            #if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
                adc_hvli_input_conv(ADC_CH_SIZE);
            #elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
                adc_hv_input_conv(ADC_CH_SIZE);
            #endif
            next_calibration_tick = get_systick() + 3000; // 设置下一次校准时间
        }

        return;
    } else if (magic_cool_mode == 1) {  // 校准,阻抗谱
        magic_cool_mode = 3;
        magic_cool_run_impedance();
    } else if (magic_cool_mode == 2) {  // 不追频运行，打印电压电流流量
        magic_cool_mode2();
        magic_cool_voltage_closeloop(magic_cool_target_vol, 3, 100, ENABLE);  // 电压闭环;
    } else if (magic_cool_mode == 3) {  // 追频运行
        magic_cool_freq_track();
        magic_cool_mode2();
    } else if (magic_cool_mode == 4) {  // 只电压闭环
        magic_cool_voltage_closeloop(magic_cool_target_vol, 3, 100, ENABLE);// 电压闭环    ;
    } else if (magic_cool_mode == 5) {  // 运行test
        magic_cool_test();
    }

    check_fault_status();
}



#endif




