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

/*****************************************************************/
#if ENABLE_QUERY_CMD
typedef struct {
    uint32_t vol;
    uint32_t cur;
} vol_cur_data_t;

vol_cur_data_t max_vol_cur_data, current_vol_cur_data;
#endif

typedef struct {
    // uint16_t over_vol_freq[20];
    uint16_t normal_vol_freq_start;
    uint16_t normal_vol_freq_end;
    uint16_t normal_work_freq;
    uint16_t crc16_check;
}_flow_freq_cfg_t;

_flow_freq_cfg_t flow_freq_cfg;

uint8_t current_flow_level = FLOW_LEVEL_100_PERCENT;  // 当前流量档位
protocol_fault_t fault_status = FAULT_NORMAL;  // 当前故障状态
protocol_fault_t fault_vol_status = FAULT_NORMAL;  // 当前过压故障状态
protocol_fault_t fault_cur_status = FAULT_NORMAL;  // 当前过流故障状态
bool reset_time_flag = false;

#if KEY_VOL_CFG
volatile uint8_t led_always_on = 0;
#endif
bool scan_freq_enable = false;
bool first_scan_freq = false;
bool reset_pwr_proxth_flag = false;
uint16_t magic_cool_vpp = 0;

static int feedback_tick = 20000; // 15000;


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
uint32_t freq_pwr[64] = {0};
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
#if KEY_VOL_CFG || ENABLE_USART
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
#if KEY_VOL_CFG || ENABLE_USART
    else if( vol >= (adjust_target_vol + 5) ) {
#else
    else if( vol >= (VOL_TARGET + 5) ) {
#endif
        ret = FAULT_SETTING_FAILED;
    } else {
        switch( fault_vol_status ) {
            case FAULT_OVER_VOLTAGE:
                if( vol > (VOL_TARGET_MAX - 3) ) {
                    ret = FAULT_OVER_VOLTAGE;
                }
                break;
            case FAULT_SETTING_FAILED:
            #if KEY_VOL_CFG || ENABLE_USART
                if( vol >= (adjust_target_vol + 3) ) {
            #else
                if( vol >= (VOL_TARGET + 3) ) {
            #endif
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
        {&fault_vol_status, FAULT_SETTING_FAILED, 1000},   // 调档失败，持续1秒触发
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
            if( fault_status == FAULT_OVER_VOLTAGE || \
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

#define  FLOW_FREQ_CFG_ADDR  (PARAM_START_ADDR - FLASH_PAGE_SIZE)
static void flow_freq_cfg_write(uint16_t gold_freq)
{
    uint8_t need_to_write = 0;
    _flow_freq_cfg_t tmp_cfg;

    if( !magic_cool_mode ) {
        return;
    }

    // printf("%s: gd: %d,nr: %d,st: %d,end: %d\r\n", __func__, gold_freq, flow_freq_cfg.normal_work_freq, flow_freq_cfg.normal_vol_freq_start, flow_freq_cfg.normal_vol_freq_end);

    // 若normal_work_freq超出范围（表明是刚烧录程序）则直接获取gold_freq并写入flash
    // 若gold_freq已超出保存的normal_work_freq±300Hz范围，则不写入并且重新大范围扫频
    if( (flow_freq_cfg.normal_work_freq < 20000 || flow_freq_cfg.normal_work_freq > 30000) || \
        (gold_freq >= (flow_freq_cfg.normal_work_freq-200) && gold_freq <= (flow_freq_cfg.normal_work_freq+200)) ) {

        // 表示前面的数据有问题
        if( gold_freq < 20000 || gold_freq > 30000 ) {
            goto re_calc;
        }

        if( flow_freq_cfg.normal_work_freq < 20000 || flow_freq_cfg.normal_work_freq > 30000 ) {
            need_to_write = 1;
        }

        flow_freq_cfg.normal_work_freq = ((gold_freq+5)/100)*100; //四舍五入并且取百位整数
        flow_freq_cfg.normal_vol_freq_start = flow_freq_cfg.normal_work_freq - 300;
        flow_freq_cfg.normal_vol_freq_end = flow_freq_cfg.normal_work_freq + 300;
        // NOTE: 修复手动关闭or手动发送指令关闭时，扫频没有从normal_vol_freq_start和normal_vol_freq_end开始
        magic_cool_set_limt(flow_freq_cfg.normal_vol_freq_start, flow_freq_cfg.normal_vol_freq_end);
        printf("%s: Success\r\n", __func__);
    } else {
        printf("%s: Fail. wk_fq:%d, st_fq:%d, end_fq:%d\r\n", __func__, flow_freq_cfg.normal_work_freq, flow_freq_cfg.normal_vol_freq_start, flow_freq_cfg.normal_vol_freq_end);
re_calc:
        flow_freq_cfg.normal_vol_freq_start = FREQ_MIN;
        flow_freq_cfg.normal_vol_freq_end = FREQ_MAX;
        flow_freq_cfg.normal_work_freq = 0;
        magic_cool_set_limt(FREQ_MIN, FREQ_MAX);
        magic_cool_mode = 1;
        return;
    }

    if( !need_to_write ) {
        printf("%s: no need to write\r\n", __func__);
        return;
    }

    // 最后的crc16_check不参与校验
    flow_freq_cfg.crc16_check = crc16((uint8_t*)&flow_freq_cfg, (sizeof(flow_freq_cfg)-2));

    uint8_t retry_cnt = 3;
    while( retry_cnt-- ) {
        flash_erase_page((uint16_t)(FLOW_FREQ_CFG_ADDR / FLASH_PAGE_SIZE));
        flash_write_halfword(FLOW_FREQ_CFG_ADDR, (uint16_t*)&flow_freq_cfg, sizeof(flow_freq_cfg));

        flash_read_bytes(FLOW_FREQ_CFG_ADDR, (uint8_t*)&tmp_cfg, sizeof(tmp_cfg));
        if( strncmp((char*)&tmp_cfg, (char*)&flow_freq_cfg, sizeof(flow_freq_cfg)) == 0 ) {
            printf("write flow freq cfg success\r\n");
            break;
        } else {
            printf("write flow freq cfg failed, retry: %d\r\n", retry_cnt);
        }
    }
}

static void flow_freq_cfg_init(void)
{
    uint16_t crc16_check = 0;

    flash_read_bytes(FLOW_FREQ_CFG_ADDR, (uint8_t*)&flow_freq_cfg, sizeof(flow_freq_cfg));

    crc16_check = crc16((uint8_t*)&flow_freq_cfg, (sizeof(flow_freq_cfg)-2));
    if( flow_freq_cfg.crc16_check == crc16_check ) {
        printf("flow freq cfg crc16 check success\r\n");
        return;
    } else {
        printf("flow freq cfg crc16 check failed\r\n");
    }

    flow_freq_cfg.normal_vol_freq_start = FREQ_MIN;
    flow_freq_cfg.normal_vol_freq_end = FREQ_MAX;
}

static void dcdc_power_control(uint8_t enable)
{
    if( enable ) {
        GPIO_WriteBit(GPIOB, GPIO_Pin_5 | GPIO_Pin_6, Bit_SET);
    } else {
        GPIO_WriteBit(GPIOB, GPIO_Pin_5 | GPIO_Pin_6, Bit_RESET);
    }
}

#if KEY_VOL_CFG
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
        case 40: flash_count = 1; break;
        // case 35: flash_count = 2; break;
        case 30: flash_count = 2; break;
        case 25: flash_count = 3; break;
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
        #if KEY_VOL_CFG
            adjust_target_vol = VOL_TARGET;
        #endif
        } else {
        #if KEY_VOL_CFG
            #if Magic_Cool_Customer == AK_Anker
                if( adjust_target_vol == 40 ) {
                    adjust_target_vol = 30;
                } else if( adjust_target_vol == 30 ) {
                    adjust_target_vol = 25;
                } else {
                    adjust_target_vol = 40;
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
//            adc_output_conv(128);
            adc_voltage_get_vpp(128);  // 电压闭环，使用简单计算，提高反馈速度
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
//            adc_output_conv(128);
            adc_voltage_get_vpp(128);  // 电压闭环，使用简单计算，提高反馈速度
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
    adc_hvli_input_conv(128);
    is_over_current((uint16_t)adc_dc_lcur_avg, not_load_check_enable);
#if ENABLE_QUERY_CMD
    float cur = CUR_CAL(adc_dc_lcur_avg) * 1000; //转化为uA
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
    uint32_t target_vpp = VOL_TARGET_70P;
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
    uint32_t target_vpp = VOL_TARGET_70P;
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
            adc_output_conv(128);
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
            adc_output_conv(128);
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
        adc_output_conv(128);
        phasex = ProcessADCData(adc_voltage_data, adc_current_data);
        printf("fft phase: %.5f ", phasex);
        phase[index] = phasex;
#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_DOT
        // ADC数据处理，点积
        adc_output_conv(128);
        phasex = get_phase_difference(adc_voltage_data, adc_current_data, 128);
        printf("dot phase: %.5f ", phasex);
        phase[index] = phasex;
#elif MAGIC_COOL_PHASE_DEFAULT == MAGIC_COOL_PHASE_CAPTURE
        // PWM 输入捕获
        adc_output_conv(128);
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
        adc_hv_input_conv(128);
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
    // adc_output_conv(128);

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

#if KEY_VOL_CFG
    led_always_on = 1;
#endif
    // TODO: 范围从20K~30KHz扫描，找到接近最优频率的整数倍频率
    if( magic_cool_freqstart == FREQ_MIN && magic_cool_freqstop == FREQ_MAX ) {
        len = magic_cool_scan_power_profile(magic_cool_freqstart, magic_cool_freqstop, 1000);
        uint32_t val;
        find_extremum_minima_i(freq_pwr, len, (uint32_t *)&val, &idx_min);
        if( !magic_cool_mode ) {
            return;
        }
        gold_freq = magic_cool_freqstart + 1000 * idx_min;
        printf("min freq:%d, min val:%d\r\n", gold_freq, (int)val);
        if( gold_freq < 20000 || gold_freq > 30000 ) {
            magic_cool_mode = 0;
            printf("fault freq, stop work\r\n");
            return;
        } else {
            magic_cool_freqstart = gold_freq - 1000;
            magic_cool_freqstop = gold_freq + 500;
            printf("magic_cool_freqstart:%d, magic_cool_freqstop:%d\r\n", magic_cool_freqstart, magic_cool_freqstop);
        }
    }

    len = magic_cool_calc_impedance(magic_cool_freqstart, magic_cool_freqstop, 100);  // 阻抗/频率谱
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
    uint32_t target_vpp = VOL_TARGET_70P;
#endif

    freq_min = magic_cool_freqstart + 100 * pwr_max_idx - SCAN_FREQ_RANGE;
//    freq_max = magic_cool_freqstart + 100 * pwr_max_idx + 250;
    pwr_max_idx = 0;
    freq_pwr[pwr_max_idx] = 0;
    for (i = 0; i < SCAN_FREQ_RANGE*2/SCAN_FREQ_STEP; i++) {  // 扫描前后250Hz 共500Hz范围 步进50Hz，10个点
        pwm_set_freq(freq_min + i * SCAN_FREQ_STEP);
        magic_cool_voltage_closeloop(target_vpp, 2, 100, DISABLE);  // 电压闭环调整，电压误差±2V
        if( fault_vol_status == FAULT_NORMAL ) {
            for (cnt = 0; cnt < 10; cnt++) {
                sys_delayms(10); // 错开一段时间
    #if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
                // 低电流
                magic_cool_calc_current(DISABLE);
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
            }
            freq_pwr[i] = freq_pwr[i] / 10;
        } else {
            freq_pwr[i] = 0;
        }

        if (freq_pwr[pwr_max_idx] < freq_pwr[i]) {
            pwr_max_idx = i;
        }
        printf("%d %d\r\n", freq_min + i * SCAN_FREQ_STEP, freq_pwr[i]);

        if( !magic_cool_mode ) {
            return;
        }
    }
    gold_freq = freq_min + SCAN_FREQ_STEP * pwr_max_idx;
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
#if KEY_VOL_CFG
    led_always_on = 0;
#endif

    flow_freq_cfg_write(magic_cool_runfreq);

    scan_freq_enable = false;
    first_scan_freq = true;

    reset_pwr_proxth_flag = true;
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
uint32_t get_current_pwr(int freq, uint8_t n)
{
    uint32_t pwr;

    pwm_set_freq(freq);  // 设置当前频率
    // magic_cool_voltage_closeloop_dcdc(magic_cool_target_vol, 1, 100);// 电压闭环
    // sys_delayms(50);
    magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 100, ENABLE);// 电压闭环
    sys_delayms(200);
    pwr = 0;
    for (uint8_t j = 0; j < n; j++) {
        sys_delayms(10);
        // ADC采样，同时采样电压电流
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW  // 直流高压输入电压,低端电流
        magic_cool_calc_current(ENABLE);
        pwr += (uint32_t)(adc_dc_hvol_avg * adc_dc_lcur_avg);
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH  // 直流高压输入电压,高端电流
        adc_hv_input_conv(128);
        pwr += (uint32_t)(adc_dc_hvol_avg * adc_dc_hcur_avg);
#else  // 其他方式，待定

#endif

    }
    pwr = pwr / n;

    return pwr;
}

// 动态更新最大值方式
void magic_cool_freq_track_current(void)
{
    uint32_t temp = 0;
    int freq0,freq1,freq2;
    int i,j;
    uint8_t n = 5;
    uint8_t loop_time = 4;
    uint8_t freq_stepx = 20;
    int pwr0, pwr1, pwr2;
    uint32_t pwrx = 0;
    uint32_t dc_vol, dc_cur;

    #define PWR_PROXTH      ((int)POWER_PROXTH(30))
    #define PWR_PROXTH_MIN  ((int)POWER_PROXTH(15))
    #define PWR_PROXTH_MAX  ((int)POWER_PROXTH(70))

    static int pwr_proxth = PWR_PROXTH; //10mW // 11308; //(10mW / 0.0008843037)

    static uint8_t pwr0_max_cnt = 0, pwr1_max_cnt = 0, pwr2_max_cnt = 0;
    static uint8_t pwr_proxth_reset_cnt = 0;
    static uint8_t pwr0_stuck_cnt;
    static uint8_t pwr2_stuck_cnt;
    uint32_t pwr_arr[15] = {0};

#if 1
    #define FREQ_NORMAL_STEP   40   //50
    #define FREQ_NORMAL_RANGE  160  //200

    #define FREQ_HIGH_TEMP_STEP   50
    #define FREQ_HIGH_TEMP_RANGE  250

    static uint16_t freq_step = FREQ_NORMAL_STEP;
    static uint16_t freq_range = FREQ_NORMAL_RANGE;
    static int last_freq;
    static uint32_t countdown_xmin, long_time_same_freq_tick;

    if( reset_pwr_proxth_flag ) {
        reset_pwr_proxth_flag = false;

        freq_step = FREQ_NORMAL_STEP;
        freq_range = FREQ_NORMAL_RANGE;
        pwr_proxth = PWR_PROXTH;
        feedback_tick = 20000;
        pwr_proxth_reset_cnt = 0;

        countdown_xmin = get_systick() + (3*60*1000); // 3分钟
    }
#if 1 // TODO:待定
    else {
        // 气泵启动一定时间后，开启小范围扫频 -> 快速找到最佳工作点
        if( (get_systick() >= countdown_xmin) && (countdown_xmin != 0) ) {
            countdown_xmin = 0;
            scan_freq_enable = true;
            printf("scan enable:%d. countdown 3min\r\n", __LINE__);
        }
    }
#endif

    // 长时间同一频率运行，进行小范围扫频确认是否当前为最佳频率
    if( last_freq == pwm_get_freq() ) {
        if( (get_systick() - long_time_same_freq_tick) >= (10*60*1000) ) { // 10分钟
            long_time_same_freq_tick = get_systick();
            scan_freq_enable = true;
            printf("scan enable:%d. long time same freq\r\n", __LINE__);
        }
        //DEBUG:
        else {
            printf("same freq time:%d \r\n", ((get_systick() - long_time_same_freq_tick)/1000));
        }
    } else {
        last_freq = pwm_get_freq();
        long_time_same_freq_tick = get_systick();
    }

    if( scan_freq_enable ) {
        #if KEY_VOL_CFG
            led_always_on = 1;
        #endif
            freq1 = pwm_get_freq();
            j = 0;
            pwrx = 0;
            for (temp = freq1 - freq_range; temp <= (freq1 + freq_range); temp += freq_step) {  // ±500Hz 扫频
                pwm_set_freq(temp);  // 设置当前频率
                magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, DISABLE);// 电压闭环
                sys_delayms(200);
                if( fault_vol_status == FAULT_NORMAL ) {
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
                    if( j < (sizeof(pwr_arr)/sizeof(pwr_arr[0])) ) {
                        pwr_arr[j++] = pwrx;
                    }
                } else {
                    if( j < (sizeof(pwr_arr)/sizeof(pwr_arr[0])) ) {
                        pwr_arr[j++] = 0;
                    }
                }
                printf("--scan freq:%d vpp:%.2f pwr:%d\r\n", temp, (float)((magic_cool_vpp+voltage_offset)/voltage_gain), pwrx);
                pwrx = 0;

                if( !magic_cool_mode ) {
                    return;
                }
            }
        #if 1
            magic_cool_pwr_max = pwr_arr[0];
            temp = 0;  // 初始化temp为0，对应pwr_arr[0]的索引
            for (i = 0; i < j; i++) {
                if (magic_cool_pwr_max < pwr_arr[i]) {
                    magic_cool_pwr_max = pwr_arr[i];
                    temp = i;
                }
            }
        #else
            uint32_t pwr_max, pwr_second_max;
            int8_t pwr_max_index, pwr_second_max_index, current_index;

            // 用前两个元素初始化
            if (pwr_arr[0] >= pwr_arr[1]) {
                pwr_max = pwr_arr[0];
                pwr_second_max = pwr_arr[1];
                pwr_max_index = 0;
                pwr_second_max_index = 1;
            } else {
                pwr_max = pwr_arr[1];
                pwr_second_max = pwr_arr[0];
                pwr_max_index = 1;
                pwr_second_max_index = 0;
            }

            temp = 0;
            // 从第三个元素开始比较
            for (i = 2; i < j; i++) {
                if (pwr_arr[i] > pwr_max) {
                    // 新的最大值，原最大值变成第二大
                    pwr_second_max = pwr_max;
                    pwr_second_max_index = pwr_max_index;
                    pwr_max = pwr_arr[i];
                    pwr_max_index = i;
                    temp = i;
                } else if (pwr_arr[i] > pwr_second_max) {
                    // 新的第二大值
                    pwr_second_max = pwr_arr[i];
                    pwr_second_max_index = i;
                }
            }

            current_index = j / 2;
            // 使用最大功率值
            int temp1 = abs_i(pwr_max_index - current_index);
            int temp2 = abs_i(pwr_second_max_index - current_index);
            // 如果最大功率点比次大功率点更偏离中心，并且功率值优势不明显（例如，小于5%），才考虑使用次大值
            if( (temp1 > temp2) && (abs_i(temp1 - temp2) > 2) && (pwr_max < pwr_second_max * 1.05f) ) {
                magic_cool_pwr_max = pwr_second_max;
                temp = pwr_second_max_index;
            } else {
                magic_cool_pwr_max = pwr_max;
                temp = pwr_max_index;
            }

            printf("pwr_max:%d, pwr_second_max:%d, \r\n", pwr_max, pwr_second_max);

        #endif

            temp = freq1 - freq_range + temp * freq_step;
            printf("--scan max freq:%d pwr:%d vpp:%.2f dac:%.2f\r\n", temp, magic_cool_pwr_max, (float)((magic_cool_vpp+voltage_offset)/voltage_gain), (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)));
            pwm_set_freq(temp);  // 设置当前频率
            magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, DISABLE);// 电压闭环
            // sys_delayms(200);
        #if KEY_VOL_CFG
            led_always_on = 0;
        #endif
            scan_freq_enable = false;
            return;
        }
#else
    scan_freq_enable = false;
#endif

    // DEBUG: 测试该增加该语句，对比较PWR差值抖动是否有帮助
    magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, DISABLE);// 电压闭环

    for (i = 0; i < n; i++) {
        sys_delayms(10);
        // ADC采样，同时采样电压电流
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW  // 直流高压输入电压,低端电流
        magic_cool_calc_current(ENABLE);
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
    printf("freq: %d pwr:%d\r\n", pwm_get_freq(), pwrx);

    temp = abs_i(magic_cool_pwr_max - pwrx);
    printf("absx: %d\r\n", temp);
    if (temp < pwr_proxth) {
        if( feedback_tick == 5000 ) {
            if( ++pwr_proxth_reset_cnt >= 10 ) {
                freq_step = FREQ_NORMAL_STEP;
                freq_range = FREQ_NORMAL_RANGE;
                pwr_proxth = PWR_PROXTH;
                feedback_tick = 20000;
                pwr_proxth_reset_cnt = 0;

                printf("reset pwr proxth. feedback_tick:%d\r\n", feedback_tick);
            } else {
                printf("pwr_proxth_reset_cnt:%d\r\n", pwr_proxth_reset_cnt);
            }
        }

        return;
    } else {
        pwr_proxth_reset_cnt = 0;

        if( temp >= PWR_PROXTH_MAX ) {
            pwr_proxth_reset_cnt = 0;
            scan_freq_enable = true;
            freq_step = FREQ_HIGH_TEMP_STEP;
            freq_range = FREQ_HIGH_TEMP_RANGE;
            pwr_proxth = PWR_PROXTH_MIN;
            feedback_tick = 5000;
            printf("scan enable:%d. feedback_tick:%d\r\n", __LINE__, feedback_tick);
            return;
        }
    }

    pwr0_stuck_cnt = 0;
    pwr2_stuck_cnt = 0;
    pwr0_max_cnt = 0;
    // pwr1_max_cnt = 0;
    pwr2_max_cnt = 0;
    for (i = 0; i < loop_time; i++) {
        // 获取当前频率
        freq1 = pwm_get_freq();
        // 获取当前频率+20Hz频率
        freq2 = freq1 + freq_stepx;
        // 获取当前频率-20Hz频率
        freq0 = freq1 - freq_stepx;

        pwr1 = get_current_pwr(freq1, n);
        printf("freq1: %d pwr1:%d vpp:%.2f dac:%.2f duty:%d\r\n", freq1, pwr1, (float)((magic_cool_vpp+voltage_offset)/voltage_gain), (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), pwm_get_duty());

        pwr0 = get_current_pwr(freq0, n);
        printf("freq0: %d pwr0:%d vpp:%.2f dac:%.2f duty:%d\r\n", freq0, pwr0, (float)((magic_cool_vpp+voltage_offset)/voltage_gain), (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), pwm_get_duty());

        pwr2 = get_current_pwr(freq2, n);
        printf("freq2: %d pwr2:%d vpp:%.2f dac:%.2f duty:%d\r\n", freq2, pwr2, (float)((magic_cool_vpp+voltage_offset)/voltage_gain), (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), pwm_get_duty());

        if( !magic_cool_mode ) {
            return;
        }

        int tmp = pwr_proxth >> 1; //NOTE: 不同频率间阈值超过一半则认为有明显差异
        if ((pwr0 > pwr1 + tmp) && (pwr0 > pwr2 + tmp)) {
            pwr1_max_cnt = 0;
            if( pwr0_max_cnt == 0 ) {
                pwr0_max_cnt++;
                pwr2_max_cnt = 0;
                pwm_set_freq(freq1);
            } else {
                pwr0_max_cnt = 0;
                pwm_set_freq(freq0);
                printf("set freq0:%d \r\n", freq0);
                magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, ENABLE);// 电压闭环
                magic_cool_pwr_max = pwr0;
                printf("pwr max:%d\r\n", magic_cool_pwr_max);
            }
        } else if ((pwr1 > pwr0 + tmp) && (pwr1 > pwr2 + tmp)) {
            pwm_set_freq(freq1);
            printf("set freq1:%d \r\n", freq1);
            magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, ENABLE);// 电压闭环
            // NOTE: 高温实验后，若一直触发freq1为最大频率，但是总体频率又比一开始pwr max小，刷新后就导致无法追到最优频率了
            // 增加连续触发追频，但是每次都是Freq1为最大，则进行小范围扫频。打印小范围扫频具体是哪个行为触发的
            // if( ++pwr1_max_cnt >= 5 ) {
            //     pwr1_max_cnt = 0;
            //     scan_freq_enable = true;
            //     printf("scan freq enable:%d\r\n", __LINE__);
            // }
            // else {
                if( pwr1 > magic_cool_pwr_max ) {
                    magic_cool_pwr_max = pwr1;
                }
                printf("pwr max:%d\r\n", magic_cool_pwr_max);
            // }
            break;
        } else if ((pwr2 > pwr0 + tmp) && (pwr2 > pwr1 + tmp)) {
            pwr1_max_cnt = 0;
            if( pwr2_max_cnt == 0 ) {
                pwr2_max_cnt++;
                pwr0_max_cnt = 0;
                pwm_set_freq(freq1);
            } else {
                pwr2_max_cnt = 0;
                pwm_set_freq(freq2);
                printf("set freq2:%d \r\n", freq2);
                magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, ENABLE);// 电压闭环
                magic_cool_pwr_max = pwr2;
                printf("pwr max:%d\r\n", magic_cool_pwr_max);
            }
        } else {
            pwm_set_freq(freq1);
            pwr0_max_cnt = 0;
            pwr1_max_cnt = 0;
            pwr2_max_cnt = 0;

            if( feedback_tick == 5000 ) {
                // 高温时，可能每个频率功率相差不是很大，减小判断阈值
                int tmp = pwr_proxth >> 2; //防止抖动
                if( (pwr0 > pwr1 + tmp && pwr0 > pwr2 + tmp) ) {
                    pwr2_stuck_cnt = 0;
                    pwr0_stuck_cnt++;
                } else if ( (pwr2 > pwr1 + tmp && pwr2 > pwr0 + tmp) ) {
                    pwr0_stuck_cnt = 0;
                    pwr2_stuck_cnt++;
                } else {
                    pwr0_stuck_cnt = 0;
                    pwr2_stuck_cnt = 0;
                }

                if (pwr0_stuck_cnt >= 3 || pwr2_stuck_cnt >= 3) {
                    pwr0_stuck_cnt = 0;
                    pwr2_stuck_cnt = 0;
                    scan_freq_enable = true;
                    printf("scan enable:%d. high temp stuck\r\n", __LINE__);
                    break;
                }
            }
        }

        if( !magic_cool_mode ) {
            return;
        }
    }
}
#endif

#else  // 其他方式, 待定

#endif


void magic_cool_freq_track(void)
{
    static int tick_ph = 50, tick_cur = 100, tick_vol = 1500, tick_imp = 50;
#if KEY_VOL_CFG || ENABLE_USART
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
    switch (flow_level) {
        case FLOW_LEVEL_90_PERCENT: target_vol = VOL_TARGET_90P; break;
        case FLOW_LEVEL_80_PERCENT: target_vol = VOL_TARGET_80P; break;
        case FLOW_LEVEL_70_PERCENT: target_vol = VOL_TARGET_70P; break;
        case FLOW_LEVEL_60_PERCENT: target_vol = VOL_TARGET_60P; break;
        case FLOW_LEVEL_50_PERCENT: target_vol = VOL_TARGET_50P; break;
        default: target_vol = VOL_TARGET; break;
    }
#if ENABLE_USART
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
        magic_cool_calc_current(ENABLE);
        hvol = adc_dc_hvol_avg;
        lcur = adc_dc_lcur_avg;
        // 系数计算: hvol和lcur为adc值，将该值转换为电压电流后简化计算就能得到一个系数
        // power = POWER_CAL(hvol, lcur);
        printf("freq:%d, vpp:%0.2f, duty:%ld, hvol: %.2f lcur: %.2f power:%.2f flow:%d dac:%.2f pwr:%d\r\n", freq, vpp, pwm_get_duty(), hvol, lcur, POWER_CAL(hvol, lcur), flow, (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), (int)(hvol*lcur));
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
        // 高电流
        adc_hv_input_conv(128);
        hvol = adc_dc_hvol_avg;
        hcur = adc_dc_hcur_avg;
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
    //     adc_output_conv(128);
    //     printf("%.2f,", find_peak_to_peak(adc_voltage_data, 128));
    // }
    if( cnt == 1000 ) {
        // printf("\r\n");
        // adc_hvli_input_conv(128);
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

    flow_freq_cfg_init();

// 零点校准
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
    adc_hvli_input_conv(128);
#elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
    adc_hv_input_conv(128);
#endif

#if MAGIC_COOL_PID
    pid_init();
#endif
    magic_cool_set_adcfreq();
    magic_cool_set_limt(flow_freq_cfg.normal_vol_freq_start, flow_freq_cfg.normal_vol_freq_end);
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

    printf("freq_start:%d, freq_stop:%d, vol_target:%d\r\n", FREQ_MIN, FREQ_MAX, VOL_TARGET);
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
                adc_hvli_input_conv(128);
            #elif MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH
                adc_hv_input_conv(128);
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




