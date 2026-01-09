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

/* 延时宏：用于快速响应休眠指令 */
#define DELAY_MS_OR_RETURN(ms, ret_val) do { if(sys_delayms(ms)) return (ret_val); } while(0)
#define DELAY_MS_OR_RETURN_VOID(ms)     do { if(sys_delayms(ms)) return; } while(0)

/*****************************************************************/
#if ENABLE_QUERY_CMD
typedef struct {
    uint32_t vol;
    uint32_t cur;
} vol_cur_data_t;

vol_cur_data_t max_vol_cur_data, current_vol_cur_data;
#endif

#if ENABLE_WRITE_FREQ
typedef struct {
    // uint16_t over_vol_freq[20];
    uint32_t normal_vol_freq_start;
    uint32_t normal_vol_freq_end;
    uint32_t normal_work_freq;
    uint16_t reserved:15;
    bool have_been_write_freq:1;
    uint16_t crc16_check;
}_flow_freq_cfg_t;

_flow_freq_cfg_t flow_freq_cfg;

static volatile bool pending_write_freq_to_flash = false;
#endif

bool stop_scan_freq = false;

bool stop_delay_ms = false;

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
#if defined(ENABLE_HIGH_TEMP_SCAN) && (ENABLE_HIGH_TEMP_SCAN == 1)
bool enable_high_temp_scan = false;
#endif
#if ENABLE_WATER_INTRUSION
bool reset_water_intrusion_flag = false;
#endif
uint16_t magic_cool_vpp = 0;

static uint32_t tick_cur = 100;
static uint16_t feedback_tick = 1000;


int32_t pwm1_duty_out = PWM1_MIN_POWER_DUTY;
uint16_t pwm1_duty_limit_min = PWM1_MAX_POWER_DUTY;
uint16_t pwm1_duty_limit_max = PWM1_MIN_POWER_DUTY;
/*****************************************************************/
float voltage_gain = MAGIC_COOL_VOLTAGE_GAIN;
float voltage_offset = MAGIC_COOL_VOLTAGE_OFFSET;

#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW || \
    MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_HIGH ||\
    MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_ALL
uint32_t freq_pwr[25] = {0};
#endif

/*******************************************************************/
/*******************************************************************/
/*******************************************************************/
// 气泵
uint8_t  magic_cool_mode = 0;     // 0:暂停 1:运行阻抗谱 2:不追频运行 3:追频运行 4: 按键调占空比
uint32_t magic_cool_freqstart = 22500, magic_cool_freqstop = 29000;
uint32_t magic_cool_runfreq = 25000;
uint32_t magic_cool_target_vol = VOL_TARGET;
#if ENABLE_KEY_VOL_CFG || defined(ENABLE_USART)
uint32_t adjust_target_vol = VOL_TARGET; //扫频后以2档启动
#endif

uint32_t magic_cool_pwr_max = 0;

uint32_t mode2_tick = 0;

// TODO: 切档一定时间内，无视PWM变化
uint32_t ignore_pwm_change_tick = 0;
uint32_t debug_tick = 0;
/*******************************************************************/
/*******************************************************************/
/*******************************************************************/
/*******************************************************************/
#if (!defined(ENABLE_PRINTF)) || (defined(ENABLE_USART))
// DEBUG
static uint32_t same_freq_work_time = 0;
static uint32_t total_work_time = 0;
static uint32_t work_freq = 0;
void reset_work_time_status( void )
{
    same_freq_work_time = get_systick(); // 1ms为单位
    total_work_time = get_systick();
    work_freq = pwm_get_freq();
}

void printf_work_time( void )
{
    if(magic_cool_mode == 0)
        return;

    uint32_t now = get_systick();

    /* ---------- total work time：每跨过一个10s桶打印一次 ---------- */
    static uint32_t last_total_bucket = 0U; // 0,1,2,... 对应 0~9s,10~19s,...
    uint32_t tick_total = now - total_work_time;
    uint32_t cur_total_bucket = tick_total / 10000U;

    if(cur_total_bucket != last_total_bucket)
    {
        last_total_bucket = cur_total_bucket;

        uint32_t total_sec = tick_total / 1000U;
        uint32_t hh = total_sec / 3600U;
        uint32_t mm = (total_sec % 3600U) / 60U;
        uint32_t ss = total_sec % 60U;

        printf( "[DEBUG PRINTF] TOTAL WORK TIME: %02lu:%02lu:%02lu\r\n", ( unsigned long ) hh, ( unsigned long ) mm,
            ( unsigned long ) ss );
    }

    /* ---------- same freq time：稳定区间内，每跨过一个10s桶打印一次 ---------- */
    uint32_t freq = pwm_get_freq();
    int32_t df = ( int32_t ) freq - ( int32_t ) work_freq;

    if((df >= -20) && (df <= 20))
    {
        static uint32_t last_same_bucket = 0U;
        uint32_t tick_same = now - same_freq_work_time;
        uint32_t cur_same_bucket = tick_same / 10000U;

        if(cur_same_bucket != last_same_bucket)
        {
            last_same_bucket = cur_same_bucket;

            uint32_t total_sec = tick_same / 1000U;
            uint32_t hh = total_sec / 3600U;
            uint32_t mm = (total_sec % 3600U) / 60U;
            uint32_t ss = total_sec % 60U;

            printf( "[DEBUG PRINTF] SAME FREQ TIME:%02lu:%02lu:%02lu\r\n", ( unsigned long ) hh, ( unsigned long ) mm,
                ( unsigned long ) ss );
        }
    }
    else
    {
        same_freq_work_time = now;
        work_freq = freq;
    }
}
#endif

void set_stop_delay_ms(bool value)
{
    stop_delay_ms = value;
}

bool get_stop_delay_ms(void)
{
    return stop_delay_ms;
}

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
void write_final_freq_to_flash(void);
void maigc_cool_test_vpp(void);
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

uint8_t get_magic_cool_mode(void)
{
    return magic_cool_mode;
}

bool is_target_vol_change(void)
{
#if ENABLE_KEY_VOL_CFG || defined(ENABLE_USART)
    if ( adjust_target_vol != magic_cool_target_vol ) {
        return true;
    }
#endif
    return false;
}

void close_all_output(void)
{
    magic_cool_mode = 0;
    pwm_enable(DISABLE);
    dcdc_power_control(DISABLE);
}

static void is_over_voltage(uint16_t vpp, FunctionalState over_voltage_check_enable)
{
    protocol_fault_t ret = FAULT_NORMAL;

    if( magic_cool_mode == 0 || stop_scan_freq ) {
        return;
    }

    float vol = (float)((vpp+voltage_offset)/voltage_gain);
    // printf("vol: %.2f\r\n", vol);

    // 若追频时，电压超过最大电压，则认为过压停止运行
    if( vol >= VOL_TARGET_MAX ) {
        // 扫频时，过压不关输出
        if( over_voltage_check_enable == ENABLE ) {
            close_all_output();
            // 需要报过压故障
            ret = FAULT_OVER_VOLTAGE;
        }
    }
    // 认为是调档失败
#if ENABLE_KEY_VOL_CFG || defined(ENABLE_USART)
    else if( vol >= (magic_cool_target_vol + 5) )
#else
    else if( vol >= (VOL_TARGET + 5) )
#endif
    {
        ret = FAULT_SETTING_FAILED;
        // scan_freq_enable = true;
        // printf("scan enable:%d. vol setting failed\r\n", __LINE__);
    } else {
        switch( fault_vol_status ) {
            case FAULT_SETTING_FAILED:
            #if ENABLE_KEY_VOL_CFG || defined(ENABLE_USART)
                if( vol >= (magic_cool_target_vol + 3) )
            #else
                if( vol >= (VOL_TARGET + 3) )
            #endif
                {
                    ret = FAULT_SETTING_FAILED;
                    // scan_freq_enable = true;
                    // printf("scan enable:%d. vol setting failed\r\n", __LINE__);
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

    if( magic_cool_mode == 0 || stop_scan_freq ) {
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

#if ENABLE_WRITE_FREQ
#define  FLOW_FREQ_CFG_ADDR  (PARAM_START_ADDR - FLASH_PAGE_SIZE)

#define CFG_RECORD_SIZE     sizeof(_flow_freq_cfg_t)  // 每条记录大小（字节）
#define CFG_MAX_RECORDS     (FLASH_PAGE_SIZE / CFG_RECORD_SIZE)  // 1K可存64条

// 检查一个槽位是否为空（全0xFF）
static bool is_slot_empty(uint32_t addr)
{
    uint32_t data[CFG_RECORD_SIZE / 4];
    flash_read_bytes(addr, (uint8_t*)data, CFG_RECORD_SIZE);
    for(int i = 0; i < CFG_RECORD_SIZE / 4; i++) {
        if(data[i] != 0xFFFFFFFF) return false;
    }
    return true;
}

// 查找第一个空闲槽位的索引，返回-1表示已满
static int find_free_slot(void)
{
    for(int i = 0; i < CFG_MAX_RECORDS; i++) {
        uint32_t addr = FLOW_FREQ_CFG_ADDR + i * CFG_RECORD_SIZE;
        if(is_slot_empty(addr)) {
            return i;
        }
    }
    return -1;  // 所有槽位都已使用
}

// 从后往前查找最新的有效记录
static int find_latest_valid_record(void)
{
    for(int i = CFG_MAX_RECORDS - 1; i >= 0; i--) {
        uint32_t addr = FLOW_FREQ_CFG_ADDR + i * CFG_RECORD_SIZE;
        if(!is_slot_empty(addr)) {
            _flow_freq_cfg_t tmp;
            flash_read_bytes(addr, (uint8_t*)&tmp, sizeof(tmp));
            // CRC校验
            uint16_t calc_crc = crc16((uint8_t*)&tmp, sizeof(tmp) - 2);
            if(calc_crc == tmp.crc16_check) {
                return i;  // 找到有效记录
            }
        }
    }
    return -1;  // 没有有效记录
}

void flow_freq_write_to_flash(_flow_freq_cfg_t *cfg)
{
    // 最后的crc16_check不参与校验
    cfg->crc16_check = crc16((uint8_t*)cfg, (sizeof(*cfg)-2));

    int free_slot = find_free_slot();

    if(free_slot < 0) {
        // 所有槽位已满，需要擦除
        flash_erase_page((uint16_t)(FLOW_FREQ_CFG_ADDR / FLASH_PAGE_SIZE));
        free_slot = 0;
    }

    uint32_t write_addr = FLOW_FREQ_CFG_ADDR + free_slot * CFG_RECORD_SIZE;

    uint8_t retry_cnt = 3;
    while( retry_cnt-- ) {
        // flash_erase_page((uint16_t)(FLOW_FREQ_CFG_ADDR / FLASH_PAGE_SIZE));
        flash_write_word(write_addr, (uint32_t*)cfg, sizeof(*cfg));

        _flow_freq_cfg_t tmp_cfg;
        flash_read_bytes(write_addr, (uint8_t*)&tmp_cfg, sizeof(tmp_cfg));
        if( memcmp((uint8_t*)&tmp_cfg, (uint8_t*)cfg, sizeof(*cfg)) == 0 ) {
            printf("write flow freq cfg success, slot: %d\r\n", free_slot);
            return;
        }
    }

    printf("write flow freq cfg failed, retry: %d\r\n", retry_cnt);
    // return;
}

// 读取配置（自动找最新有效记录）
bool flow_freq_read_from_flash(_flow_freq_cfg_t *cfg)
{
    int valid_idx = find_latest_valid_record();
    if(valid_idx < 0) {
        printf("no valid flow freq cfg\r\n");
        return false;  // 没有有效数据
    }

    uint32_t addr = FLOW_FREQ_CFG_ADDR + valid_idx * CFG_RECORD_SIZE;
    flash_read_bytes(addr, (uint8_t*)cfg, sizeof(*cfg));
    printf("\r\nread flow freq cfg success, slot: %d\r\n", valid_idx);
    return true;
}

void erase_flow_freq_cfg(void)
{
    flash_erase_page((uint16_t)(FLOW_FREQ_CFG_ADDR / FLASH_PAGE_SIZE));
}


static void flow_freq_cfg_write(uint16_t gold_freq, uint16_t freq_min, uint16_t freq_max)
{
    static uint8_t restart_scan_cnt;
    static bool restart_scan_freq_flag;

    uint8_t need_to_write = 0;
    // _flow_freq_cfg_t tmp_cfg;

    if( magic_cool_mode == 0 || flow_freq_cfg.have_been_write_freq ) {
        return;
    }

    // printf("%s: gd: %d,nr: %d,st: %d,end: %d\r\n", __func__, gold_freq, flow_freq_cfg.normal_work_freq, flow_freq_cfg.normal_vol_freq_start, flow_freq_cfg.normal_vol_freq_end);

    // if( gold_freq == freq_min || gold_freq == freq_max ) {
    //     if( ++restart_scan_cnt >= 2 ) {
    //         restart_scan_cnt = 0;
    //         restart_scan_freq_flag = false;
    //     } else {
    //         restart_scan_freq_flag = true;
    //     }
    // } else {
    //     restart_scan_cnt = 0;
    //     restart_scan_freq_flag = false;
    // }

    // 若normal_work_freq超出范围（表明是刚烧录程序）则直接获取gold_freq并写入flash
    // 若gold_freq已超出保存的normal_work_freq±200Hz范围，则不写入并且重新大范围扫频
    // bool freq_out_of_valid_range = (flow_freq_cfg.normal_work_freq < 20000 ||
    //                                 flow_freq_cfg.normal_work_freq > FREQ_MAX);
    // bool gold_within_tolerance = (gold_freq >= flow_freq_cfg.normal_work_freq - 200 &&
    //                                 gold_freq <= flow_freq_cfg.normal_work_freq + 200);

    // if(!restart_scan_freq_flag && (freq_out_of_valid_range || gold_within_tolerance)) {
    // if( !restart_scan_freq_flag ) {
        // 表示前面的数据有问题
        if( gold_freq < 20000 || gold_freq > FREQ_MAX ) {
            flow_freq_cfg.normal_vol_freq_start = FREQ_MIN;
            flow_freq_cfg.normal_vol_freq_end = FREQ_MAX;
            flow_freq_cfg.normal_work_freq = 0;
            magic_cool_set_limt(FREQ_MIN, FREQ_MAX);
            magic_cool_mode = 1;
            return;
        }

        if( flow_freq_cfg.normal_work_freq < 20000 || flow_freq_cfg.normal_work_freq > FREQ_MAX ) {
            need_to_write = 1;
        }
    #if 0
        flow_freq_cfg.normal_work_freq = ((gold_freq+5)/100)*100; //四舍五入并且取百位整数
        flow_freq_cfg.normal_vol_freq_start = flow_freq_cfg.normal_work_freq - 300;
        flow_freq_cfg.normal_vol_freq_end = flow_freq_cfg.normal_work_freq + 300;
    #else
        flow_freq_cfg.normal_work_freq = gold_freq;
        flow_freq_cfg.normal_vol_freq_start = flow_freq_cfg.normal_work_freq;
        flow_freq_cfg.normal_vol_freq_end = flow_freq_cfg.normal_work_freq;
    #endif
        flow_freq_cfg.have_been_write_freq = true;
        // NOTE: 修复手动关闭or手动发送指令关闭时，扫频没有从normal_vol_freq_start和normal_vol_freq_end开始
        magic_cool_set_limt(flow_freq_cfg.normal_vol_freq_start, flow_freq_cfg.normal_vol_freq_end);
        printf("[sys:%d] %s: Success.flag:%d\r\n", get_systick() - debug_tick, __func__, flow_freq_cfg.have_been_write_freq);
        debug_tick = get_systick();
    // } else {
    //     printf("%s: Fail. wk_fq:%d, st_fq:%d, end_fq:%d\r\n", __func__, flow_freq_cfg.normal_work_freq, flow_freq_cfg.normal_vol_freq_start, flow_freq_cfg.normal_vol_freq_end);
    //     flow_freq_cfg.normal_vol_freq_start = FREQ_MIN;
    //     flow_freq_cfg.normal_vol_freq_end = FREQ_MAX;
    //     flow_freq_cfg.normal_work_freq = 0;
    //     magic_cool_set_limt(FREQ_MIN, FREQ_MAX);
    //     magic_cool_mode = 1;
    //     return;
    // }

    if( !need_to_write ) {
        printf("[sys:%d] %s: no need to write\r\n", get_systick() - debug_tick, __func__);
        debug_tick = get_systick();
        return;
    }

    flow_freq_write_to_flash(&flow_freq_cfg);
}

static void flow_freq_cfg_init(void)
{
    uint16_t crc16_check = 0;

    // flash_read_bytes(FLOW_FREQ_CFG_ADDR, (uint8_t*)&flow_freq_cfg, sizeof(flow_freq_cfg));

    if( !flow_freq_read_from_flash(&flow_freq_cfg) ) {
        flow_freq_cfg.normal_vol_freq_start = FREQ_MIN;
        flow_freq_cfg.normal_vol_freq_end = FREQ_MAX;
        flow_freq_cfg.have_been_write_freq = false;
        return;
    }

    crc16_check = crc16((uint8_t*)&flow_freq_cfg, (sizeof(flow_freq_cfg)-2));
    if( flow_freq_cfg.crc16_check == crc16_check ) {
        // flow_freq_cfg.have_been_write_freq = true;
        magic_cool_set_limt(flow_freq_cfg.normal_work_freq, flow_freq_cfg.normal_work_freq);
        printf("flow freq cfg crc16 check success.flag:%d\r\n", flow_freq_cfg.have_been_write_freq);
        return;
    } else {
        printf("flow freq cfg crc16 check failed crc16_check:%04x %04x\r\n", crc16_check, flow_freq_cfg.crc16_check);
    }

    flow_freq_cfg.normal_vol_freq_start = FREQ_MIN;
    flow_freq_cfg.normal_vol_freq_end = FREQ_MAX;
    flow_freq_cfg.have_been_write_freq = false;
}

//TODO: 只有按键关闭输出或发deepsleep指令才会保存数据,需要注意高温环境下不能保存
void write_final_freq_to_flash(void)
{
    // ============ 验证0：扫频状态检查 ============
    if (scan_freq_enable) {
        printf("scanning freq, skip write\r\n");
        return;
    }

    // ============ 验证1：首次扫频检查 ============（新增）
    if (first_scan_freq) {
        printf("first scan not complete, skip write\r\n");
        return;
    }

#if defined(ENABLE_HIGH_TEMP_SCAN) && (ENABLE_HIGH_TEMP_SCAN == 1)
    // ============ 验证2：高温环境检查 ============（新增）
    if (enable_high_temp_scan) {
        printf("high temp detected, skip write\r\n");
        return;
    }
#endif

    uint32_t freq = pwm_get_freq();

    // 必须校准过才能保存
    if( flow_freq_cfg.have_been_write_freq == false ) {
        printf("have not calc\r\n");
        return;
    }

    // ============ 验证3：基本频率范围检查 ============
    if (freq < FREQ_MIN || freq > FREQ_MAX) {
        printf("freq out of range, skip: %d\r\n", freq);
        return;
    }

    // ============ 验证4：故障状态检查 ============
    if (fault_status != FAULT_NORMAL) {
        printf("fault detected, skip write\r\n");
        return;
    }

    // ============ 验证5：过压/过流即时检查 ============
    if (fault_vol_status != FAULT_NORMAL || fault_cur_status != FAULT_NORMAL) {
        printf("vol/cur fault detected, skip write\r\n");
        return;
    }

    // flash_read_bytes(FLOW_FREQ_CFG_ADDR, (uint8_t*)&flow_freq_cfg, sizeof(flow_freq_cfg));
    flow_freq_read_from_flash(&flow_freq_cfg);

    // ============ 验证3：与已校准频率的偏差检查 ============
    // 如果已有有效的校准频率，新频率不能偏离太多
    #define FREQ_SAVE_TOLERANCE  300  // 容许偏差±300Hz（可根据实际调整到200-500Hz）

    // 检查已保存的频率是否有效（在合理的工作范围内，比如25k-29k）
    bool has_valid_calibration = ( flow_freq_cfg.normal_work_freq >= FREQ_MIN &&
                                   flow_freq_cfg.normal_work_freq <= FREQ_MAX );

    if (has_valid_calibration) {
        int32_t freq_diff = (int32_t)freq - (int32_t)flow_freq_cfg.normal_work_freq;
        if (freq_diff < 0) freq_diff = -freq_diff;  // abs

        if (freq_diff > FREQ_SAVE_TOLERANCE) {
            // 偏差过大，可能是异常状态（高温漂移、故障等）
            printf("freq drift too large: saved=%d, current=%d, diff=%d\r\n",
                    flow_freq_cfg.normal_work_freq, freq, freq_diff);
            return;
        } else if( freq_diff < 50 ) {
            // 偏差过小，不更新
            printf("freq drift too small: saved=%d, current=%d, diff=%d\r\n",
                    flow_freq_cfg.normal_work_freq, freq, freq_diff);
            return;
        }
    }

    if( freq != flow_freq_cfg.normal_work_freq ) {
        flow_freq_cfg.normal_work_freq = freq;
        flow_freq_cfg.normal_vol_freq_start = freq;
        flow_freq_cfg.normal_vol_freq_end = freq;
        flow_freq_cfg.have_been_write_freq = true;

        flow_freq_write_to_flash(&flow_freq_cfg);
    }
}

#endif

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
        #ifdef VOL_TARGET_90P
            case VOL_TARGET_90P: flash_count = 2; break;
        #endif
        #ifdef VOL_TARGET_80P
            case VOL_TARGET_80P: flash_count = 3; break;
        #endif
        #ifdef VOL_TARGET_70P
            case VOL_TARGET_70P: flash_count = 4; break;
        #endif
        #ifdef VOL_TARGET_60P
            case VOL_TARGET_60P: flash_count = 5; break;
        #endif
        #ifdef VOL_TARGET_50P
            case VOL_TARGET_50P: flash_count = 6; break;
        #endif
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
                #ifdef VOL_TARGET_90P
                    VOL_TARGET_90P,
                #endif
                #ifdef VOL_TARGET_80P
                    VOL_TARGET_80P,
                #endif
                #ifdef VOL_TARGET_70P
                    VOL_TARGET_70P,
                #endif
                #ifdef VOL_TARGET_60P
                    VOL_TARGET_60P,
                #endif
                #ifdef VOL_TARGET_50P
                    VOL_TARGET_50P,
                #endif
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
            close_all_output();
            #if ENABLE_WRITE_FREQ
                pending_write_freq_to_flash = true;  // 标志位延迟写入，避免在中断中执行 Flash 操作
            #endif
        #endif
        }
    } else if (key2 == 0x02) {
        close_all_output();
    #if ENABLE_WRITE_FREQ
        pending_write_freq_to_flash = true;  // 标志位延迟写入，避免在中断中执行 Flash 操作
    #endif
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
            adc_voltage_get_vpp(ADC_CH_SIZE);  // 电压闭环，使用简单计算，提高反馈速度
            vpp_sum += adc_vpp;  // 获取单次计算的峰峰值
        }
        vpp = vpp_sum / 10;
        voltage_err = (vol_targetx - vpp);

        // printf("err:%d, vpp:%d, vol_errx:%d, pwm1_duty_out:%d\r\n", voltage_err, vpp, vol_errx, pwm1_duty_out);
        if (abs_i(voltage_err) < vol_errx) {  // 电压小于误差范围认为电压稳定
            count++;
            if (count > 5) {   // 连续获取电压10次都在误差范围就认为电压稳定，退出
                magic_cool_vpp = vpp;
                return 0;
            }
            continue;
        }
        count = 0;

//        pid_delta = rm_pid_delta(voltage_err);
        // pid_delta = abs_i(voltage_err) / 5;
        pid_delta = abs_i(voltage_err) / 4;
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

        magic_cool_vpp = vpp;

        if (ret == 1) {
            // printf("pwm1_duty_out of range\r\n");
            magic_cool_vpp = vpp;
            return 1;
        }

    #if ENABLE_KEY_VOL_CFG || defined(ENABLE_USART)
        if( (first_scan_freq == false) && (adjust_target_vol != magic_cool_target_vol) )
        {
            stop_scan_freq = true;
            printf("stop scan freq\r\n");
            return 1;
        }
    #endif

        DELAY_MS_OR_RETURN(20, 1);
    }

    return 1;
}

// 电压闭环
// 1. 单端的时候，由于LC谐振，电压在不同频率下波动较大，可以通过调整PWM占空比升降压，也可以通过调整DCDC ref电压升降压
// 2. 差分模式的时候，由于LC谐振，调压只能通过调整DCDC ref电压升降压，PWM占空比固定为50%，不能变。
int magic_cool_voltage_closeloop(uint32_t vol_target, uint32_t vol_err, uint32_t timeout, FunctionalState over_voltage_check_enable)
{
    magic_cool_voltage_closeloop_dcdc(vol_target, vol_err, timeout);
    if( over_voltage_check_enable ) {
        is_over_voltage(magic_cool_vpp, over_voltage_check_enable);
    }
#if ENABLE_QUERY_CMD
    float vpp = ((magic_cool_vpp+voltage_offset)/voltage_gain) * 1000;
    if( max_vol_cur_data.vol < vpp ) {
        max_vol_cur_data.vol = vpp;
    }
    current_vol_cur_data.vol = vpp;
#endif
    return 0;
}

adc_sample_t magic_cool_calc_current(FunctionalState not_load_check_enable)
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
    return sample;
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
    uint32_t target_vpp = VOL_TARGET;
#if PWM_DRIVER_METHOD == PWM_DIFFERENTIAL_DRIVE
    #if ENABLE_WRITE_FREQ
        #if ENABLE_KEY_VOL_CFG || defined(ENABLE_USART)
            target_vpp = (!flow_freq_cfg.have_been_write_freq) ? VOL_TARGET : adjust_target_vol;
        #endif
    #endif
#else
    #if Magic_Cool_Customer == AK_Anker
        target_vpp = VOL_TARGET_1;
    #else
        target_vpp = VOL_TARGET_70P;
    #endif
#endif

    int ret = 0, index = 0;
    int freq = 0;
    uint16_t vpp = 0;

    uint32_t delay_tick = get_systick();

    printf("[sys:%d] magic cool scan power profile\r\n", get_systick() - debug_tick);
    debug_tick = get_systick();

    for (freq = start_freq; freq <= stop_freq; ) {
        pwm_set_freq(freq);
        magic_cool_voltage_closeloop(target_vpp, 2, 100, DISABLE);// 电压闭环
        if( fault_vol_status == FAULT_NORMAL ) {
            DELAY_MS_OR_RETURN(20, 1);
            magic_cool_voltage_closeloop(target_vpp, 2, 100, DISABLE);// 电压闭环
            if( fault_vol_status == FAULT_NORMAL ) {
                DELAY_MS_OR_RETURN(100, 1);
            }
        }

        vpp = (uint16_t)((magic_cool_vpp+voltage_offset)/voltage_gain);
        if( vpp < target_vpp - 5 ) {
            printf("[ERROR] freq: %d vpp: %d pwm1:%d\r\n", freq, vpp, pwm1_duty_out);
            pwm1_duty_out = PWM1_MIN_POWER_DUTY;
            // 恢复上一个正常工作的频率
            pwm_set_freq(freq-step_freq);
            magic_cool_voltage_closeloop(target_vpp, 2, 100, DISABLE);// 电压闭环
            break;
        }

        freq_pwr[index] = pwm1_duty_out;

        printf("[sys:%d] freq: %d vpp:%.1f dac: %.2f pwm1:%d \r\n", get_systick() - debug_tick, freq, (float)(magic_cool_vpp+voltage_offset)/voltage_gain, (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), freq_pwr[index]);
        debug_tick = get_systick();

        freq += step_freq;
        index++;

        if( magic_cool_mode == 0 ) {
            break;
        }
    }
    printf("[sys:%d] magic cool scan power profile end\r\n", get_systick() - debug_tick);
    debug_tick = get_systick();

    ret = index;
    return ret;
}

int magic_cool_calc_impedance(uint32_t start_freq, uint32_t stop_freq, uint32_t step_freq)
{
    uint32_t target_vpp = VOL_TARGET;
#if PWM_DRIVER_METHOD == PWM_DIFFERENTIAL_DRIVE
    #if ENABLE_WRITE_FREQ
        #if ENABLE_KEY_VOL_CFG || defined(ENABLE_USART)
            target_vpp = (!flow_freq_cfg.have_been_write_freq) ? VOL_TARGET : adjust_target_vol;
        #endif
    #endif
#else
    #if Magic_Cool_Customer == AK_Anker
        target_vpp = VOL_TARGET_1;
    #else
        target_vpp = VOL_TARGET_70P;
    #endif
#endif
    int ret = 0, index = 0;
    int flow = 0;
    int freq = 0;

    printf("[sys:%d] magic_cool_calc_impedance imp\r\n", get_systick() - debug_tick);
    debug_tick = get_systick();

    for (freq = start_freq; freq <= stop_freq; ) {
        pwm_set_freq(freq);
        magic_cool_voltage_closeloop(target_vpp, 2, 100, DISABLE);// 电压闭环

#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
        // 电压无故障，对应频率则进行功率比较
        if( fault_vol_status == FAULT_NORMAL ) {
            DELAY_MS_OR_RETURN(100, 1);
            // 低电流
            adc_sample_t sample = magic_cool_calc_current(DISABLE);
            freq_pwr[index] = calculate_power(sample);
        } else {
            freq_pwr[index] = 0;
        }
        printf("[sys:%d] freq: %d vpp:%.1f duty:%d hvol: %.2f lcur: %.2f pwr: %d dac: %.2f \r\n", get_systick() - debug_tick, freq, (float)(magic_cool_vpp+voltage_offset)/voltage_gain, pwm_get_duty(), adc_dc_hvol_avg, adc_dc_lcur_avg, freq_pwr[index], (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)));
        debug_tick = get_systick();
#endif

#if USE_AIR_FLOWMETER
        // 获取空气流量
        sys_delayms(200);
        flow = get_air_flow();
        printf("flow: %d ", flow);
#endif
        freq += step_freq;
        index++;

        if( magic_cool_mode == 0 ) {
            break;
        }
    }
    printf("[sys:%d] magic_cool_calc_impedance imp end \r\n\r\n", get_systick() - debug_tick);
    debug_tick = get_systick();
    ret = index;
    return ret;
}

void magic_cool_run_impedance(void)
{
    int len, i;
    int idx_min;
    int freq_max, freq_min, gold_freq;
    int pwr_max_idx;
    uint32_t step_freq;
#if PWM_DRIVER_METHOD == PWM_DIFFERENTIAL_DRIVE
    uint32_t target_vpp = VOL_TARGET;
#else
    #if Magic_Cool_Customer == AK_Anker
        uint32_t target_vpp = VOL_TARGET_1;
    #else
        uint32_t target_vpp = VOL_TARGET_70P;
    #endif
#endif
#if ENABLE_WRITE_FREQ
    flow_freq_cfg_init();
    #if ENABLE_KEY_VOL_CFG || defined(ENABLE_USART)
        target_vpp = (!flow_freq_cfg.have_been_write_freq) ? VOL_TARGET : adjust_target_vol;
    #endif
#endif

    OPA_Enable();
    // 从低频率开始，防止过冲烧坏气泵
    // 恢复默认dac
    pwm1_duty_out = PWM1_MIN_POWER_DUTY;
    pwm1_set_duty(pwm1_duty_out);
    set_power_enable(ENABLE);
    DELAY_MS_OR_RETURN_VOID(2);
    dcdc_power_control(ENABLE);
    DELAY_MS_OR_RETURN_VOID(10); //NOTE:增加延时,防止短时间电压过冲
    // 升压稳定后再开H桥PWM
    pwm_set_freq(magic_cool_freqstart);
    pwm_enable(ENABLE);

    debug_tick = 0;

    // 重置错误标志
    fault_status = FAULT_NORMAL;
    fault_vol_status = FAULT_NORMAL;
    fault_cur_status = FAULT_NORMAL;

    first_scan_freq = true;

    // 重置历史电压/电流最大值
#if ENABLE_QUERY_CMD
    max_vol_cur_data.vol = 0;
    max_vol_cur_data.cur = 0;
#endif

#if ENABLE_KEY_VOL_CFG
    led_always_on = 1;
#endif

    // NOTE: 范围从20K~30KHz扫描，找到接近最优频率的整数倍频率
#if ENABLE_WRITE_FREQ
    if((FREQ_MAX-FREQ_MIN) >= 3000 && !flow_freq_cfg.have_been_write_freq)
#else
    if((FREQ_MAX-FREQ_MIN) >= 3000)
#endif
    {
        // step_freq = 500; // 有种新陶瓷需要500Hz步进才能找到谐振频率
        step_freq = 1000;
        magic_cool_freqstart = FREQ_MIN;
        magic_cool_freqstop = FREQ_MAX;
        len = magic_cool_scan_power_profile(magic_cool_freqstart, magic_cool_freqstop, step_freq);
        uint32_t val;
        if( find_extremum_minima_i(freq_pwr, len, (uint32_t *)&val, &idx_min) == -1 ) {
            close_all_output();
            printf("fault freq, stop work\r\n");
        }
        if( magic_cool_mode == 0 ) {
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

        uint32_t freq_start_found = magic_cool_freqstart + step_freq * idx_start;
        uint32_t freq_end_found = magic_cool_freqstart + step_freq * idx_end;

        printf("[sys:%d] min freq range:%d-%d, min val:%d\r\n", get_systick() - debug_tick, freq_start_found, freq_end_found, (int)val);
        debug_tick = get_systick();

        magic_cool_freqstart = freq_start_found - 1000;
        // magic_cool_freqstop = freq_end_found + 500;
        magic_cool_freqstop = freq_end_found;
        printf("[sys:%d] magic_cool_freqstart:%d, magic_cool_freqstop:%d\r\n", get_systick() - debug_tick, magic_cool_freqstart, magic_cool_freqstop);
        debug_tick = get_systick();
    }

    step_freq = 200;
    len = magic_cool_calc_impedance(magic_cool_freqstart, magic_cool_freqstop, step_freq);  // 阻抗/频率谱
    DELAY_MS_OR_RETURN_VOID(100);

#if MAGIC_COOL_TRACK_DEFAULT == MAGIC_COOL_TRACK_CURRENT
    if( magic_cool_mode == 0 ) {
        return;
    }
    find_maxima_i(freq_pwr, len, &magic_cool_pwr_max, &pwr_max_idx);  // 找最大值

    #define SCAN_FREQ_STEP   50  //20 // 50
    #define SCAN_FREQ_RANGE  150  //60 // 100
#if ENABLE_WRITE_FREQ
    if( !flow_freq_cfg.have_been_write_freq )
#endif
    {
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

        if (magic_cool_mode == 0) {
            return;
        }

        // 复制结果到freq_pwr数组（保持兼容性）
        for (i = 0; i < impedance_result.count && i < (sizeof(freq_pwr) / sizeof(freq_pwr[0])); i++) {
            freq_pwr[i] = impedance_result.powers[i];
        }

        pwr_max_idx = impedance_result.max_index;
        magic_cool_pwr_max = freq_pwr[pwr_max_idx];
        gold_freq = get_freq_from_scan_index(freq_min, SCAN_FREQ_STEP, pwr_max_idx);
    }
#if ENABLE_WRITE_FREQ
    else {
        gold_freq = flow_freq_cfg.normal_work_freq;
    }
#endif

    magic_cool_runfreq = gold_freq; // 初始频率为相位最大值对应的频率
    printf("[sys:%d] freq:%d, pwr max :%d\r\n", get_systick() - debug_tick, magic_cool_runfreq, magic_cool_pwr_max);
    debug_tick = get_systick();

#else  // 其他方式, 待定
    gold_freq = FREQ_MIN;
    printf("min freq:%d, max freq:%d, gold freq:%d\r\n", freq_min, freq_max, gold_freq);
#endif
    // 设置输出频率
    pwm_set_freq(magic_cool_runfreq); // 阻抗谱计算最优频率
    magic_cool_voltage_closeloop(target_vpp, 2, 100, DISABLE);// 电压闭环
    // sys_delayms(100);
#if ENABLE_KEY_VOL_CFG
    led_always_on = 0;
#endif

    scan_freq_enable = false;

#if ENABLE_WRITE_FREQ
    flow_freq_cfg_write(magic_cool_runfreq, freq_min, freq_max);
#endif

#if defined(ENABLE_HIGH_TEMP_SCAN) && (ENABLE_HIGH_TEMP_SCAN == 1)
    enable_high_temp_scan = false;
#endif

    reset_pwr_proxth_flag = true;
    reset_time_flag = true;
#if ENABLE_WATER_INTRUSION
    reset_water_intrusion_flag = true;
#endif

#if (!defined(ENABLE_PRINTF)) || (defined(ENABLE_USART))
    reset_work_time_status();
#endif
}

#if MAGIC_COOL_TRACK_DEFAULT == MAGIC_COOL_TRACK_CURRENT
// 电流追频, 找功率最大点
// ==================== 底层ADC采样抽象层 ====================
// 策略模式：ADC采样接口（实现）
static inline adc_sample_t adc_sample_once(void)
{
    adc_sample_t sample = {0};
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
    adc_hvli_input_conv(ADC_CH_SIZE);
    sample.voltage = adc_dc_hvol_avg;
    sample.current = adc_dc_lcur_avg;
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
        DELAY_MS_OR_RETURN(config->delay_before_ms, 0);
    }

    // 采样并累加功率
    for (uint8_t i = 0; i < config->sample_count; i++) {
        DELAY_MS_OR_RETURN(10, 0);
        adc_sample_t sample = magic_cool_calc_current(ENABLE);
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
        .delay_before_ms = 100,
        .check_overvoltage = ENABLE
    };

    if ( is_target_vol_change() ) {
        return 0;
    }

    // 先执行电压闭环，然后检查故障状态
    pwm_set_freq(freq);
    magic_cool_voltage_closeloop(config.target_vol, config.vol_err, 100, config.check_overvoltage);

    if (fault_vol_status == FAULT_NORMAL) {
        DELAY_MS_OR_RETURN(config.delay_before_ms, 0);
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
        .delay_before_ms = 100,
        .check_overvoltage = DISABLE  // 强制测量时不检查过压
    };

    // 直接测量，无视故障状态
    return measure_power_with_config(&config);
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
            DELAY_MS_OR_RETURN(100, result);
            pwrx = 0;

            // 采样并累加功率
            for (uint8_t i = 0; i < cfg->sample_count; i++) {
                DELAY_MS_OR_RETURN(10, result);
                adc_sample_t sample = magic_cool_calc_current(ENABLE);
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
            printf("--scan freq:%d vpp:%.2f hvol:%.2f lcur:%.2f pwr:%d\r\n", freq,
                   (float)((magic_cool_vpp+voltage_offset)/voltage_gain), adc_dc_hvol_avg, adc_dc_lcur_avg, pwrx);
        } else {
            // 简单格式（用于阻抗扫描）
            printf("[sys:%d] %d %d\r\n", get_systick() - debug_tick, freq, pwrx);
            debug_tick = get_systick();
        }

        // 检查是否需要退出
        if (magic_cool_mode == 0) {
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
    #define PWR_PROXTH      3  // 6
    #define PWR_PROXTH_MIN  2
    #define PWR_PROXTH_MAX  10 //15
#else
    #define PWR_PROXTH      ((int)POWER_PROXTH(30))
    #define PWR_PROXTH_MIN  ((int)POWER_PROXTH(15))
    #define PWR_PROXTH_MAX  ((int)POWER_PROXTH(60))
#endif

//TODO: 若最后确实需要使用小范围扫频，可尝试缩小范围，多次调频
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

    uint8_t pwr_diff_cnt;
    uint8_t pwr_high_diff_cnt;

    bool is_huge_stability_scan;

    // Stuck detection
    uint8_t pwr0_max_cnt;
    uint8_t pwr2_max_cnt;

#if defined(ENABLE_HIGH_TEMP_SCAN) && (ENABLE_HIGH_TEMP_SCAN == 1)
    // Timer state
    int last_freq;
    uint32_t countdown_xmin;
    uint32_t long_time_same_freq_tick;
    uint32_t long_time_same_freq_interval;
    bool long_time_same_freq_scan_pending;
#endif

    // status
    uint32_t normal_pwr_max;

    uint8_t perturb_step;
    uint8_t perturb_cnt;
} TrackingState;

static TrackingState track_state;

// 初始化track_state
static void track_reset_state(void)
{
    track_state.freq_step = FREQ_NORMAL_STEP;
    track_state.freq_range = FREQ_NORMAL_RANGE;
    track_state.pwr_proxth = PWR_PROXTH;
    feedback_tick = 1000;
    track_state.pwr_proxth_reset_cnt = 0;

    track_state.perturb_step = 20;
    track_state.perturb_cnt = 3;
    track_state.normal_pwr_max = magic_cool_pwr_max;

#if defined(ENABLE_HIGH_TEMP_SCAN) && (ENABLE_HIGH_TEMP_SCAN == 1)
    track_state.countdown_xmin = get_systick() + (3*60*1000); // 3分钟
    track_state.long_time_same_freq_tick = get_systick();
    track_state.long_time_same_freq_interval = (10*60*1000);
    track_state.long_time_same_freq_scan_pending = false;
#endif
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

    scan_freq_enable = false;

    if (!magic_cool_mode) return;
#if ENABLE_KEY_VOL_CFG || defined(ENABLE_USART)
    if( adjust_target_vol != magic_cool_target_vol ) return;
#endif
    if (result.all_zero) {
        printf("--all_pwr_zero, test freq_start and freq_stop only\r\n");
        uint32_t pwr_start = measure_power_force(scan_cfg.start_freq, n);
        uint32_t pwr_stop = measure_power_force(scan_cfg.stop_freq, n);

        uint32_t best_freq = (pwr_start > pwr_stop) ? scan_cfg.start_freq : scan_cfg.stop_freq;
        pwm_set_freq(best_freq);
        magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, DISABLE);
        printf("scan enable:%d. all_zero, continue scan\r\n", __LINE__);
        scan_freq_enable = true;  // 继续循环扫频
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
        }
    }

    #if ENABLE_KEY_VOL_CFG
        led_always_on = 0;
    #endif
    #if ENABLE_WATER_INTRUSION
        reset_water_intrusion_flag = true;
    #endif
}

//TODO: 可改成若功率变化过大，则增大P&O步长，恢复后则按正常步长P&O
static bool track_check_power_stability(uint8_t n)
{
    adc_sample_t sample = {0};
    // 测量当前功率
    magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 50, ENABLE);
    DELAY_MS_OR_RETURN(50, false);
    magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 50, ENABLE);

    uint32_t pwrx = 0;
    for (int i = 0; i < n; i++) {
        DELAY_MS_OR_RETURN(10, false);
        sample = magic_cool_calc_current(ENABLE);
        pwrx += calculate_power(sample);
    }
    pwrx /= n;
    // printf("freq: %d pwr:%d\r\n", pwm_get_freq(), pwrx);

    printf("freq:%d, vpp:%0.2f, duty:%ld, hvol: %d lcur: %d power:%.2f dac:%.2f pwr:%d\r\n", pwm_get_freq(), \
    (float)((magic_cool_vpp+voltage_offset)/voltage_gain), \
    pwm_get_duty(), sample.voltage, sample.current, \
    (float)POWER_CAL(sample.voltage, sample.current), \
    (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), \
    pwrx);

    // 一定时间内进行功率更新
    if( get_systick() <= ignore_pwm_change_tick ) {
        magic_cool_pwr_max = pwrx;
        track_state.pwr_diff_cnt = 0;
        printf("max pwm: %d \r\n", magic_cool_pwr_max);
        return true;
    }

    uint32_t diff = abs_i(magic_cool_pwr_max - pwrx);

    #if ENABLE_PER
        uint8_t percent = (uint8_t)(diff*100/magic_cool_pwr_max);
        printf("absx: %d percent: %d\r\n", diff, percent);
        bool is_stable = (percent < track_state.pwr_proxth);
    #else
        printf("absx: %d diff:%.2f\r\n", diff, (float)(diff*(CURRENT_COEFFICIENT * VOL_H_COEFFICIENT)));
        bool is_stable = (diff < track_state.pwr_proxth);
    #endif

    bool is_pwr_back_to_normal = (pwrx >= (track_state.normal_pwr_max * 97 / 100));

    if( track_state.is_huge_stability_scan && is_pwr_back_to_normal ) {
        if( ++track_state.pwr_proxth_reset_cnt >= 10 ) // 高温连续10次检测都恢复到正常功率值，则恢复正常P&O参数
        {
            track_state.freq_step = FREQ_NORMAL_STEP;
            track_state.freq_range = FREQ_NORMAL_RANGE;
            track_state.pwr_proxth = PWR_PROXTH;
            // feedback_tick = 1000;
            track_state.is_huge_stability_scan = false;
            track_state.pwr_proxth_reset_cnt = 0;
            track_state.perturb_step = 20;
            track_state.perturb_cnt = 3;
            // scan_freq_enable = false;
            printf("reset pwr proxth. perturb_step:%d is_huge_stability_scan:%d\r\n", track_state.perturb_step, track_state.is_huge_stability_scan);
        }
    } else {
        track_state.pwr_proxth_reset_cnt = 0;
    }

    if (is_stable) {
        track_state.pwr_diff_cnt = 0; // 重置P&O计数器
        return true; // 功率正常（误差在阈值内），无需调整
    } else {
        #if ENABLE_PER
            bool huge_stability = (percent >= PWR_PROXTH_MAX);
        #else
            bool huge_stability = (diff >= PWR_PROXTH_MAX);
        #endif

        if (huge_stability && track_state.is_huge_stability_scan == false) {
            if( track_state.pwr_high_diff_cnt < 2 ) {
                track_state.pwr_high_diff_cnt++;
                return true; // 跳过P&O
            } else {
                track_state.pwr_high_diff_cnt = 0;
            }
            // scan_freq_enable = true;
            track_state.freq_step = FREQ_HIGH_TEMP_STEP;
            track_state.freq_range = FREQ_HIGH_TEMP_RANGE;
            track_state.pwr_proxth = PWR_PROXTH_MIN;
            // feedback_tick = 0;
            track_state.is_huge_stability_scan = true;
        #if defined(ENABLE_HIGH_TEMP_SCAN) && (ENABLE_HIGH_TEMP_SCAN == 1)
            enable_high_temp_scan = true;
        #endif
            track_state.perturb_step = 50;
            track_state.perturb_cnt = 0;
            track_state.normal_pwr_max = magic_cool_pwr_max;
            printf("perturb_step:%d is_huge_stability_scan:%d normal_pwr_max:%d\r\n", track_state.perturb_step, track_state.is_huge_stability_scan, track_state.normal_pwr_max);
            // return true; // 跳过P&O
        } else {
            track_state.pwr_high_diff_cnt = 0;

            track_state.pwr_diff_cnt++;
            //需要连续N次PWM检测超过阈值，才进行P&O，否则跳过
            if( track_state.pwr_diff_cnt >= track_state.perturb_cnt ) {
                track_state.pwr_diff_cnt = 0;
            } else {
                printf("pwr_diff_cnt: %d\r\n", track_state.pwr_diff_cnt);
                return true; // 跳过P&O
            }
        }
    }
    return false; // 进入P&O
}

static void track_perturb_observe(uint8_t n)
{
    track_state.pwr0_max_cnt = 0;
    track_state.pwr2_max_cnt = 0;

    // for (int i = 0; i < 2; i++)
    { // loop_time = 4
        int freq1 = pwm_get_freq();
        int freq2 = freq1 + track_state.perturb_step;
        int freq0 = freq1 - track_state.perturb_step;

        uint32_t pwr1 = measure_power_simple(freq1, n);
        printf("freq1: %d pwr1:%d vpp:%.2f dac:%.2f duty:%d\r\n", freq1, pwr1, (float)((magic_cool_vpp+voltage_offset)/voltage_gain), (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), pwm_get_duty());

        uint32_t pwr0 = measure_power_simple(freq0, n);
        printf("freq0: %d pwr0:%d vpp:%.2f dac:%.2f duty:%d\r\n", freq0, pwr0, (float)((magic_cool_vpp+voltage_offset)/voltage_gain), (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), pwm_get_duty());

        uint32_t pwr2 = measure_power_simple(freq2, n);
        printf("freq2: %d pwr2:%d vpp:%.2f dac:%.2f duty:%d\r\n", freq2, pwr2, (float)((magic_cool_vpp+voltage_offset)/voltage_gain), (float)(pwm1_duty_out*MCU_VDD_GAIN/(HSI_VALUE/PWM1_FREQ)), pwm_get_duty());

        if (magic_cool_mode == 0 || is_target_vol_change() ) return;

    #if 1 //TODO:待定是否添加
        // 代表电压过压,调档或高温恢复常温过程会出现
        uint16_t step = 100;
        while( !pwr0 && !pwr1 && !pwr2 )
        {
            freq2 = freq1 + step;
            freq0 = freq1 - step;

            pwr0 = measure_power_simple(freq0, n);
            pwr2 = measure_power_simple(freq2, n);

            printf("step:%d pwr0:%d pwr1:%d pwr2:%d\r\n", step, pwr0, pwr1, pwr2);
            step += 50;

            if( magic_cool_mode == 0 || stop_scan_freq )
            {
                return;
            }

            if( pwr0 > pwr2 ) {
                pwm_set_freq(freq0);
                magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, ENABLE);
                return;
            } else if( pwr2 > pwr0 ) {
                pwm_set_freq(freq2);
                magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, ENABLE);
                return;
            }
        }
    #endif

        #if ENABLE_PER
            int tmp = (magic_cool_pwr_max * track_state.pwr_proxth / 100 ) >> 1;
        #else
            int tmp = track_state.pwr_proxth >> 1;
        #endif

        if ((pwr0 > pwr1 + tmp) && (pwr0 > pwr2 + tmp)) {
            // if( track_state.pwr0_max_cnt == 0 ) {
            //     track_state.pwr0_max_cnt++;
            //     track_state.pwr2_max_cnt = 0;
            //     pwm_set_freq(freq1);
            // } else {
                track_state.pwr0_max_cnt = 0;
                pwm_set_freq(freq0);
                printf("set freq0:%d \r\n", freq0);
                magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, ENABLE);
                magic_cool_pwr_max = pwr0;
            // }
        } else if ((pwr1 > pwr0 + tmp) && (pwr1 > pwr2 + tmp)) {
            pwm_set_freq(freq1);
            printf("set freq1:%d \r\n", freq1);
            magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, ENABLE);
            if( pwr1 > magic_cool_pwr_max )
                magic_cool_pwr_max = pwr1;
            // break;
        } else if ((pwr2 > pwr0 + tmp) && (pwr2 > pwr1 + tmp)) {
            // if( track_state.pwr2_max_cnt == 0 ) {
            //     track_state.pwr2_max_cnt++;
            //     track_state.pwr0_max_cnt = 0;
            //     pwm_set_freq(freq1);
            // } else {
                track_state.pwr2_max_cnt = 0;
                pwm_set_freq(freq2);
                printf("set freq2:%d \r\n", freq2);
                magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, ENABLE);
                magic_cool_pwr_max = pwr2;
            // }
        } else {
            pwm_set_freq(freq1);
            track_state.pwr0_max_cnt = 0;
            track_state.pwr2_max_cnt = 0;
        }
    }
}

// 动态更新最大值方式
void magic_cool_freq_track_current(void)
{
    uint8_t n = 5;

    if( reset_pwr_proxth_flag ) {
        track_reset_state();
        reset_pwr_proxth_flag = false;
    }

    // 循环直到不需要扫频
    // while (scan_freq_enable) {
    //     track_perform_scan(n);
    // }

    // 检查功率稳定性
    bool skip_po = track_check_power_stability(n);
    //TODO: 调档失败直接进入P&O
    if (skip_po && fault_vol_status!=FAULT_SETTING_FAILED) return;

    // 追频
    track_perturb_observe(n);
}


#endif

void magic_cool_vpp_change(void)
{
#if ENABLE_KEY_VOL_CFG || defined(ENABLE_USART)
    while( (adjust_target_vol != magic_cool_target_vol) || (first_scan_freq == true) ) {

        if( adjust_target_vol != VOL_TARGET || first_scan_freq == false ) {
            // TODO: 切档后取消小范围扫频
            // scan_freq_enable = true;
        }

        if( (magic_cool_target_vol != adjust_target_vol) || (first_scan_freq == true) ) {
            if( first_scan_freq == true ) {
                // // 启动1分钟内不进行追频比较
                // feedback_tick = 60000;
                // tick_cur = get_systick() + feedback_tick;

                // 每次启动后30s内，无视PWM变化
                // ignore_pwm_change_tick = get_systick() + 30000;
            } else {
                // 每次切档后5s内，无视PWM变化
                ignore_pwm_change_tick = get_systick() + 5000;
            }
            magic_cool_target_vol = adjust_target_vol;
        }

        first_scan_freq = false;

        // magic_cool_voltage_closeloop(magic_cool_target_vol, 2, 50, ENABLE);// 电压闭环
        // sys_delayms(10);

        //TODO: 切档可以在此再次优化
        uint32_t pwr = 0;
        magic_cool_voltage_closeloop(magic_cool_target_vol, 1, 100, ENABLE);// 电压闭环

        if( stop_scan_freq ) {
            stop_scan_freq = false;
            continue;
        }

        if (fault_vol_status == FAULT_NORMAL) {
            for (uint8_t i = 0; i < 5; i++) {
                DELAY_MS_OR_RETURN_VOID(10);
                adc_sample_t sample = magic_cool_calc_current(ENABLE);
                pwr += calculate_power(sample);
            }
            pwr = pwr / 5;
        }

        if( pwr != 0 ) {
            magic_cool_pwr_max = pwr;
        }
        printf("freq:%d vpp:%.2f max pwr:%d\r\n", pwm_get_freq(), (float)(magic_cool_vpp+voltage_offset)/voltage_gain, magic_cool_pwr_max);

        scan_freq_enable = false;
        printf("scan freq disable:%d\r\n", __LINE__);

        // 重置进水状态，防止切换档位误报进水
    #if ENABLE_WATER_INTRUSION
        reset_water_intrusion_flag = true;
    #endif
    }
#elif ENABLE_WRITE_FREQ
    if( first_scan_freq == true )
    {
        ignore_pwm_change_tick = get_systick() + 5000;
    }
#endif
    first_scan_freq = false;
}

void magic_cool_freq_track(void)
{
    magic_cool_vpp_change();
#if MAGIC_COOL_TRACK_DEFAULT == MAGIC_COOL_TRACK_CURRENT
    // 电流
    if (get_systick() >= tick_cur || scan_freq_enable) {
        magic_cool_freq_track_current();
        tick_cur = get_systick() + feedback_tick;
        // scan_freq_enable = false;
    }
#endif

#if 0
    maigc_cool_test_vpp();
#endif
}

void magic_cool_set_target_vol(uint32_t vol)
{
    magic_cool_target_vol = vol;
}

// 串口发送指令调整流量 -> 调整电压
void magic_cool_set_target_vol_by_flow(uint8_t flow_level)
{
    uint32_t target_vol = 0;
#if defined(ENABLE_USART) && Magic_Cool_Customer != AK_Anker
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

    if( magic_cool_mode == 0 ) {
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

    if(magic_cool_mode == 0) {
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
            close_all_output();
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

void maigc_cool_test_vpp(void)
{
    static uint16_t cnt;

    if( get_systick() < 3*60*1000 || cnt >= 1001 ) {
        return;
    }

    // sys_delayms(1000);
    for(; cnt<1000; cnt++) {
        adc_output_conv(ADC_CH_SIZE);
        printf("%.2f,", find_peak_to_peak(adc_voltage_data, (uint32_t)ADC_CH_SIZE));
    }
    if( cnt == 1000 ) {
        printf("\r\n");
        // adc_hvli_input_conv(ADC_CH_SIZE);
        for(int i = 0; i < ADC_CH_SIZE; i++) {
            printf("%.2f \r\n", adc_voltage_data[i]);
        }
        printf("---------\r\n");
        for(int i = 0; i < ADC_CH_SIZE; i++) {
            printf("%.2f \r\n", adc_current_data[i]);
        }
        // for(int i = 0; i < ADC_CH_SIZE; i++) {
        //     printf("%.2f, %.2f\r\n", adc_voltage_data[i], adc_current_data[i]);
        // }
        sys_delayms(100);
        cnt = 1001;
        close_all_output();
    }
}

void magic_cool_config(void)
{
#if USE_AIR_FLOWMETER
    air_flowmeter_init();
//    air_flowmeter_test();
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

// 零点校准
#if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
    adc_hvli_input_conv(ADC_CH_SIZE);
#endif

#if ENABLE_WRITE_FREQ
    flow_freq_cfg_init();
#endif

    printf("freq_start:%d, freq_stop:%d, vol_target:%d customer:%d\r\n", FREQ_MIN, FREQ_MAX, VOL_TARGET, Magic_Cool_Customer);
}

void magic_cool_idle(void)
{
    static uint32_t next_calibration_tick = 0;

    pwm_enable(DISABLE);
    pwm1_duty_out = PWM1_MIN_POWER_DUTY;
    pwm1_set_duty(pwm1_duty_out);  // 设置DAC输出DCDC
    set_power_enable(ENABLE);
    OPA_Disable();
    dcdc_power_control(DISABLE);

    // // 每3秒执行一次零点校准
    // if (get_systick() >= next_calibration_tick) {
    //     #if MAGIC_COOL_DC_CURRENT_DEFAULT == MAGIC_COOL_DC_CURRENT_LOW
    //         adc_hvli_input_conv(ADC_CH_SIZE);
    //     #endif
    //     next_calibration_tick = get_systick() + 3000; // 设置下一次校准时间
    // }
}

void magic_cool_run(void)
{
#if ENABLE_WRITE_FREQ
    // 在主循环中执行 Flash 写入（避免在中断中阻塞）
    if (pending_write_freq_to_flash) {
        pending_write_freq_to_flash = false;
        write_final_freq_to_flash();
    }
#endif

    switch (magic_cool_mode) {
        case 0:
            magic_cool_idle();
            break;
        case 1:
            magic_cool_mode = 3;
            magic_cool_run_impedance();
            break;
        case 3:
            magic_cool_freq_track();
            break;
        default:
            break;
    }

    check_fault_status();

#if (!defined(ENABLE_PRINTF)) || (defined(ENABLE_USART))
    printf_work_time();
#endif
}

#endif


