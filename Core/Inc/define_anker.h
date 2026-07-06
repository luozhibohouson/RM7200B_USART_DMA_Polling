#ifndef __DEFINE_ANKER_H
#define __DEFINE_ANKER_H

#include "define.h"

#if Magic_Cool_Customer == AK_Anker

// =================== 电压设定 ===================
#define     VOL_TARGET      50 // 30 // 40 // 40  // 流量目标
#define     VOL_TARGET_1    40 // 28 // 30 // 50
#define     VOL_TARGET_2    30 // 25 // 25 // 60

// 最大电压，超过停止输出
#define     VOL_TARGET_MAX    (VOL_TARGET_2+30)
// ================================================

// =================== 功能开关 ===================
#define     ENABLE_PER             1  // 百分比阈值设定 or 固定阈值设定
#define     ENABLE_WATER_INTRUSION 0  // 进水检测开关
#define     ENABLE_KEY_VOL_CFG     1  // 按键调整电压
// ================================================

// =================== 气泵频率设定 ===================
#define FREQ_MIN                   20000
#define FREQ_MAX                   25000
// ================================================

// =================== 驱动方式设定 ===================
#define PWM_DRIVER_METHOD          PWM_DIFFERENTIAL_DRIVE
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
#if MCU_VDD == MCU_VDD_3V3
  #if PWM_DRIVER_METHOD == PWM_DIFFERENTIAL_DRIVE
    #define MAGIC_COOL_VOLTAGE_GAIN  11.8
  #else
    #define MAGIC_COOL_VOLTAGE_GAIN  13.79
  #endif
#elif MCU_VDD == MCU_VDD_3V0
  #define MAGIC_COOL_VOLTAGE_GAIN   12.98
#endif

#if PWM_DRIVER_METHOD == PWM_DIFFERENTIAL_DRIVE
  #define MAGIC_COOL_VOLTAGE_OFFSET 0
#else
  #define MAGIC_COOL_VOLTAGE_OFFSET 12.46
#endif
// ================================================

// =================== DAC升压配置设定 ===================
#define PWM1_MIN_POWER_DUTY         ((HSI_VALUE/PWM1_FREQ*MCU_VDD_MAX_GAIN_10X/MCU_VDD_GAIN_10X))
#define PWM1_MAX_POWER_DUTY         ((HSI_VALUE/PWM1_FREQ*MCU_VDD_MIN_GAIN_10X/MCU_VDD_GAIN_10X))
// ================================================

// =================== 电流计算参数设定 ===================
#define CURRENT_ADC_MAX_TEST        137 //10mA -- 过流测试使用
#define CURRENT_ADC_MAX             (4000 - adc_dc_lcur_offset) // (2 * 4096 * 11 / MCU_VDD_GAIN_10X) //0.2A * 4096 * 11 / 3.3 = 2730
#define CURRENT_ADC_MIN             15 // 15 //DISABLE 为0则关闭空载检测
// ================================================

// =================== 功率计算参数设定 ===================
#if MCU_VDD == MCU_VDD_3V3
  #define CURRENT_COEFFICIENT        (0.07324)
  #define VOL_H_COEFFICIENT          (0.01207)
  // 0.07324 * 0.01207 = 0.0008843037
#elif MCU_VDD == MCU_VDD_3V0
  #define CURRENT_COEFFICIENT        (0.06658)
  #define VOL_H_COEFFICIENT          (0.01098)
  // 0.06658 * 0.01098 = 0.0007310484
#endif

#define CUR_CAL(cur_ad)             (cur_ad * CURRENT_COEFFICIENT) // (3.3*AD/4096/11)*1000  已放大1000倍 -> 单位为mA
#define VOL_H_CAL(vol_ad)           (vol_ad * VOL_H_COEFFICIENT)   // 3.3*AD/4096/0.0667
#define POWER_CAL(vol_ad, cur_ad)   (VOL_H_CAL(vol_ad) * CUR_CAL(cur_ad))
#define POWER_PROXTH(pwr)           (pwr / (CURRENT_COEFFICIENT * VOL_H_COEFFICIENT))
// ================================================
#endif

#endif
