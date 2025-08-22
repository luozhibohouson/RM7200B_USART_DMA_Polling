#ifndef __DEFINE_CY_H
#define __DEFINE_CY_H

// 传音配置（占位，按原文件最小映射）

#if 0
    #define     VOL_TARGET      50  // 流量目标
    #define     VOL_TARGET_90P  47
    #define     VOL_TARGET_80P  44
    #define     VOL_TARGET_70P  41
    #define     VOL_TARGET_60P  38
    #define     VOL_TARGET_50P  35
#else
    #define     VOL_TARGET      40 //40  // 流量目标
    #define     VOL_TARGET_90P  37 //37
    #define     VOL_TARGET_80P  34 //35
    #define     VOL_TARGET_70P  31 //33
    #define     VOL_TARGET_60P  28 //30
    #define     VOL_TARGET_50P  25 //27

    // 最大电压，超过停止输出
    #define     VOL_TARGET_MAX  53 //(VOL_TARGET+20)
#endif

// 泵频
// #define PUMP_FREQ                  25200
#define FREQ_MIN                   23600 // (PUMP_FREQ - 300)
#define FREQ_MAX                   26300 // (PUMP_FREQ + 300)

// 硬件版本
#define HW_V1_0                    "1"
// #define HW_V1_1                    "2"
#define HARDWARE_VERSION           HW_V1_0

// 软件版本
#define APP_VERSION                "12" //V1.2

// 驱动方式 - 差分/单端驱动
#define PWM_DRIVER_METHOD          PWM_DIFFERENTIAL_DRIVE

// 供电选择
#define MCU_VDD                    MCU_VDD_3V0
#if MCU_VDD == MCU_VDD_3V3
  #define MCU_VDD_GAIN             3.3
  #define MCU_VDD_GAIN_10X         33
#if 1
  #define MCU_VDD_MAX_GAIN_10X     30
  #define MCU_VDD_MIN_GAIN_10X     0
#else // 未修改DC升压反馈电阻,最小只能到0.2V
  #define MCU_VDD_MAX_GAIN_10X     33
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

// 电压增益与偏移
#if MCU_VDD == MCU_VDD_3V3
  #if PWM_DRIVER_METHOD == PWM_DIFFERENTIAL_DRIVE
    #define MAGIC_COOL_VOLTAGE_GAIN  11.8
  #else
    #define MAGIC_COOL_VOLTAGE_GAIN  13.79
  #endif
#elif MCU_VDD == MCU_VDD_3V0
  #define MAGIC_COOL_VOLTAGE_GAIN  12.98
#endif

#if PWM_DRIVER_METHOD == PWM_DIFFERENTIAL_DRIVE
  #define MAGIC_COOL_VOLTAGE_OFFSET 0
#else
  #define MAGIC_COOL_VOLTAGE_OFFSET 12.46
#endif

// DAC升压配置
#define PWM1_MIN_POWER_DUTY         ((HSI_VALUE/PWM1_FREQ*MCU_VDD_MAX_GAIN_10X/MCU_VDD_GAIN_10X))
#define PWM1_MAX_POWER_DUTY         ((HSI_VALUE/PWM1_FREQ*MCU_VDD_MIN_GAIN_10X/MCU_VDD_GAIN_10X))

// 电流计算参数
#define CURRENT_ADC_MAX             4000 // 不好计算，直接取ADC的最大值


// 功能开关
// 按键调整电压
#define KEY_VOL_CFG                 0

// 打印开关
#define ENABLE_PRINTF               0

// 串口通讯开关
#define ENABLE_USART                1

#endif

