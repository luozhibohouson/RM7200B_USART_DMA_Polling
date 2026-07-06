#ifndef __DEFINE_GLUE_TEST_H
#define __DEFINE_GLUE_TEST_H

#include "define.h"
#include "magic_cool.h"
#if Magic_Cool_Customer == Glue_Test

// =================== 流量目标设定 ===================
// 修改说明：原压电泵交流驱动目标 50V 改为 DC-DC 直流输出 3.4V
#define     VOL_TARGET      3.4f  // DC-DC 直流输出电压目标 3.4V
#define     VOL_TARGET_90P  3.23f // 90% 电压 3.23V
#define     VOL_TARGET_80P  3.06f // 80% 电压 3.06V
#define     VOL_TARGET_70P  2.89f // 70% 电压 2.89V
#define     VOL_TARGET_60P  2.72f // 60% 电压 2.72V
#define     VOL_TARGET_50P  2.55f // 50% 电压 2.55V

// 最大电压保护（超过 3.8V 停止输出）
#define     VOL_TARGET_MAX    3.8f  // 最大保护电压 3.8V
// ================================================

// =================== 功能开关 ===================
#define     ENABLE_PER             1  // 百分比阈值设定 or 固定阈值设定
#define     ENABLE_WATER_INTRUSION 0  // 进水检测开关
#define     ENABLE_KEY_VOL_CFG     0  // 按键调整电压
#define     ENABLE_WRITE_FREQ      0  // 写频开关
#define     ENABLE_QUERY_CMD       0  // 查询指令开关，需要开启串口通讯才能使用 -- 仅在内部测试时使用，客户版本禁止开启该宏
#define     ENABLE_PUMP_STATUS_CMD 0  // 气泵状态查询开关
// ================================================

// =================== 气泵频率设定 ===================
#if 0
#define FREQ_MIN                   19000//4000//31000//1000//19000////19000//16080//13000//4500//10000//4500//4920//3000//20000第一个 // 25500
#define FREQ_MAX                   24000//12000//8000//35000//30000//24000////24000//22000//20080//17000//14000//5300//8000//60000第一个//30000 // 27500
#elif 0
#define FREQ_MIN                   20000//4000//31000//1000//19000////19000//16080//13000//4500//10000//4500//4920//3000//20000第一个 // 25500
#define FREQ_MAX                   40000//15000//12000//8000//35000//30000//24000////24000//22000//20080//17000//14000//5300//8000//60000第一个//30000 // 27500
#elif 0
#define FREQ_MIN                   15000//4000//31000//1000//19000////19000//16080//13000//4500//10000//4500//4920//3000//20000第一个 // 25500
#define FREQ_MAX                   19000//12000//8000//35000//30000//24000////24000//22000//20080//17000//14000//5300//8000//60000第一个//30000 // 27500
#elif 0
#define FREQ_MIN                   32000//11000//10000//20000//1000//19000////19000//16080//13000//4500//10000//4500//4920//3000//20000第一个 // 25500
#define FREQ_MAX                   37000//33000//15000//12000//25000//30000//24000////24000//22000//20080//17000//14000//5300//8000//60000第一个//30000 // 27500
#elif 0
#define FREQ_MIN                   20800//11000//10000//20000//1000//19000////19000//16080//13000//4500//10000//4500//4920//3000//20000第一个 // 25500
#define FREQ_MAX                   22800//33000//15000//12000//25000//30000//24000////24000//22000//20080//17000//14000//5300//8000//60000第一个//30000 // 27500
#elif 0
#define FREQ_MIN                   34200//11000//10000//20000//1000//19000////19000//16080//13000//4500//10000//4500//4920//3000//20000第一个 // 25500
#define FREQ_MAX                   37200//33000//15000//12000//25000//30000//24000////24000//22000//20080//17000//14000//5300//8000//60000第一个//30000 // 27500
#elif 1
#define FREQ_MIN                   gScanFreqMin //30000//11000//10000//20000//1000//19000////19000//16080//13000//4500//10000//4500//4920//3000//20000第一个 // 25500
#define FREQ_MAX                   gScanFreqMax //34000//33000//15000//12000//25000//30000//24000////24000//22000//20080//17000//14000//5300//8000//60000第一个//30000 // 27500
#endif// ================================================

// =================== 驱动方式设定 ===================
#define PWM_DRIVER_METHOD         PWM_DIFFERENTIAL_DRIVE
// ================================================

// =================== 供电选择设定 ===================
#define MCU_VDD                    MCU_VDD_3V3
#if MCU_VDD == MCU_VDD_3V3
  #define MCU_VDD_GAIN             3.3
  #define MCU_VDD_GAIN_10X         33
  #define MCU_VDD_MAX_GAIN_10X     33
#if 0
  #define MCU_VDD_MIN_GAIN_10X     0
#else // 未修改DC升压反馈电阻,最小只能到0.2V
  #define MCU_VDD_MIN_GAIN_10X     2
#endif
#elif MCU_VDD == MCU_VDD_3V0
  #define MCU_VDD_GAIN             3.0
  // 硬件调整电阻后，DAC驱动电压最大只能到3V,最小是0V。原来是3.3V和0.2V
  #define MCU_VDD_GAIN_10X         30
  #define MCU_VDD_MAX_GAIN_10X     30
#if 0
  #define MCU_VDD_MIN_GAIN_10X     0
#else // 未修改DC升压反馈电阻,最小只能到0.2V
  #define MCU_VDD_MIN_GAIN_10X     2
#endif
#endif
// ================================================

// =================== 电压增益与偏移设定 ===================
// 修改说明：原压电泵交流 50V 改为 DC-DC 直流 3.4V 输出
// 分压电阻：110K:22K = 1:6
// 增益计算：ADC 计数 = V_target × (4096 / 3.3) × 6 = V_target × 206.8
#if MCU_VDD == MCU_VDD_3V3
  #if PWM_DRIVER_METHOD == PWM_DIFFERENTIAL_DRIVE
    #define MAGIC_COOL_VOLTAGE_GAIN  206.8  // DC-DC 直流 3.4V 输出增益
  #else
    #define MAGIC_COOL_VOLTAGE_GAIN  206.8  // DC-DC 直流 3.4V 输出增益
  #endif
#elif MCU_VDD == MCU_VDD_3V0
  #define MAGIC_COOL_VOLTAGE_GAIN   206.8  // DC-DC 直流 3.4V 输出增益
#endif

// 直流信号无偏移
#define MAGIC_COOL_VOLTAGE_OFFSET   0
// ================================================

// =================== DAC升压配置设定 ===================
#define PWM1_MIN_POWER_DUTY         ((HSI_VALUE/PWM1_FREQ*MCU_VDD_MAX_GAIN_10X/MCU_VDD_GAIN_10X))
#define PWM1_MAX_POWER_DUTY         ((HSI_VALUE/PWM1_FREQ*MCU_VDD_MIN_GAIN_10X/MCU_VDD_GAIN_10X))
// ================================================

// =================== 电流计算参数设定 ===================
#define CURRENT_ADC_MAX_TEST        137 //10mA -- 过流测试使用
#define CURRENT_ADC_MAX             (4000 - adc_dc_lcur_offset) // (2 * 4096 * 11 / MCU_VDD_GAIN_10X) //0.2A * 4096 * 11 / 3.3 = 2730
#define CURRENT_ADC_MIN             DISABLE // 15 //DISABLE 为0则关闭空载检测
// ================================================

// =================== 功率计算参数设定 ===================
// VOCurP 电流检测公式：I = (VOCurP - 1V) / 510
// 其中：1V 为偏置电压，510 = 100Ω×5.1(运放增益)
#if MCU_VDD == MCU_VDD_3V3
  #define VOL_H_COEFFICIENT      (3.3*6/4096)        // 0.00483398... 电压系数 (分压比 1:6)
#elif MCU_VDD == MCU_VDD_3V0
  #define VOL_H_COEFFICIENT      (3.0*6/4096)        // 0.00439453... 电压系数 (分压比 1:6)
#endif

// 电流计算宏（VOCurP 高端总电流检测）
#define CUR_CAL(cur_ad)           ((cur_ad * 3.3 / 4096 - 0*3.3/4096) / 510)  // 单位：A (安培)
// 计算过程：ADC 值 → 电压 (×3.3/4096) → 减偏置 (-1V) → 除以增益 (/510)

// 电压计算宏
#define VOL_H_CAL(vol_ad)         (vol_ad *(3.3/4096)/0.167f)//(vol_ad *(0.653f/925)/0.191f)  // 单位：V (伏特)

// 功率计算宏
#define POWER_CAL(vol_ad, cur_ad) (VOL_H_CAL(vol_ad) * CUR_CAL(cur_ad))  // 单位：W (瓦特)
#define POWER_PROXTH(pwr)         (pwr / (VOL_H_COEFFICIENT * 3.3 / 4096 / 510))  // 功率→ADC 阈值

#define POWER_AMP(pwr)         1000*(pwr*0.00482*(3.3 / 4096)/510)  // ADC阈值→功率
// ================================================

#endif
#endif
